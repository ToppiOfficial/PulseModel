// dmx.cpp - data model, guid/type helpers, header parsing and codec dispatch.

#include "dmx/dmx.h"

#include <cctype>
#include <cstdio>
#include <cstring>
#include <fstream>

namespace pulse::dmx {

// --- Guid ------------------------------------------------------------------
std::string GuidToString(const Guid& g) {
    static const char* hex = "0123456789abcdef";
    std::string s;
    s.reserve(36);
    for (int i = 0; i < 16; ++i) {
        if (i == 4 || i == 6 || i == 8 || i == 10) s += '-';
        s += hex[g[i] >> 4];
        s += hex[g[i] & 0xF];
    }
    return s;
}

static int HexVal(char c) {
    if (c >= '0' && c <= '9') return c - '0';
    if (c >= 'a' && c <= 'f') return c - 'a' + 10;
    if (c >= 'A' && c <= 'F') return c - 'A' + 10;
    return -1;
}

bool GuidFromString(const std::string& s, Guid& out) {
    int n = 0;
    int hi = -1;
    for (char c : s) {
        if (c == '-') continue;
        int v = HexVal(c);
        if (v < 0) return false;
        if (hi < 0) {
            hi = v;
        } else {
            if (n >= 16) return false;
            out[n++] = static_cast<uint8_t>((hi << 4) | v);
            hi = -1;
        }
    }
    return n == 16 && hi < 0;
}

// --- Type helpers ----------------------------------------------------------
bool IsArrayType(AttrType t) {
    return t >= AttrType::ElementArray && t <= AttrType::MatrixArray;
}

AttrType ScalarOfArray(AttrType t) {
    if (!IsArrayType(t)) return t;
    int delta = static_cast<int>(AttrType::ElementArray) - static_cast<int>(AttrType::Element);
    return static_cast<AttrType>(static_cast<int>(t) - delta);
}

// --- Element accessors -----------------------------------------------------
const Attribute* Element::Get(const std::string& n) const {
    for (const auto& a : attributes)
        if (a.name == n) return &a;
    return nullptr;
}

template <typename T>
static const T* GetScalar(const Element& e, const std::string& n) {
    const Attribute* a = e.Get(n);
    if (!a) return nullptr;
    return std::get_if<T>(&a->value);
}

ElementPtr Element::GetElement(const std::string& n) const {
    const Attribute* a = Get(n);
    if (!a) return nullptr;
    if (auto p = std::get_if<ElementPtr>(&a->value)) return *p;
    return nullptr;
}

const std::string* Element::GetString(const std::string& n) const {
    return GetScalar<std::string>(*this, n);
}

int32_t Element::GetInt(const std::string& n, int32_t fallback) const {
    if (auto p = GetScalar<int32_t>(*this, n)) return *p;
    return fallback;
}

float Element::GetFloat(const std::string& n, float fallback) const {
    if (auto p = GetScalar<float>(*this, n)) return *p;
    return fallback;
}

bool Element::GetBool(const std::string& n, bool fallback) const {
    if (auto p = GetScalar<bool>(*this, n)) return *p;
    return fallback;
}

Vector3 Element::GetVector3(const std::string& n, Vector3 fallback) const {
    if (auto p = GetScalar<Vector3>(*this, n)) return *p;
    return fallback;
}

const std::vector<ElementPtr>* Element::GetElementArray(const std::string& n) const {
    return GetScalar<std::vector<ElementPtr>>(*this, n);
}
const std::vector<int32_t>* Element::GetIntArray(const std::string& n) const {
    return GetScalar<std::vector<int32_t>>(*this, n);
}
const std::vector<float>* Element::GetFloatArray(const std::string& n) const {
    return GetScalar<std::vector<float>>(*this, n);
}
const std::vector<std::string>* Element::GetStringArray(const std::string& n) const {
    return GetScalar<std::vector<std::string>>(*this, n);
}
const std::vector<Vector3>* Element::GetVector3Array(const std::string& n) const {
    return GetScalar<std::vector<Vector3>>(*this, n);
}
const std::vector<Vector2>* Element::GetVector2Array(const std::string& n) const {
    return GetScalar<std::vector<Vector2>>(*this, n);
}
const std::vector<Quaternion>* Element::GetQuaternionArray(const std::string& n) const {
    return GetScalar<std::vector<Quaternion>>(*this, n);
}

// --- Datamodel -------------------------------------------------------------
Element* Datamodel::CreateElement(const Guid& id, const std::string& className) {
    auto e = std::make_unique<Element>();
    e->id = id;
    e->className = className;
    Element* raw = e.get();
    elements.push_back(std::move(e));
    // A null/zero guid is used for anonymous elements; don't index those.
    Guid zero{};
    if (id != zero) by_id_[id] = raw;
    return raw;
}

Element* Datamodel::FindById(const Guid& id) const {
    auto it = by_id_.find(id);
    return it == by_id_.end() ? nullptr : it->second;
}

void Datamodel::IndexElement(Element* e, const Guid& id) {
    e->id = id;
    Guid zero{};
    if (id != zero) by_id_[id] = e;
}

// --- Header parsing + dispatch --------------------------------------------
// Header: <!-- dmx encoding <enc> <encver> format <fmt> <fmtver> -->
static bool ParseHeader(const char* data, size_t size, Datamodel& dm,
                        size_t& body_offset, std::string* err) {
    // Find end of the first line.
    size_t nl = 0;
    while (nl < size && data[nl] != '\n') ++nl;
    std::string line(data, nl);
    body_offset = (nl < size) ? nl + 1 : size;

    // Tokenize on whitespace; expect: <!-- dmx encoding E EV format F FV -->
    char enc[64] = {0}, fmt[64] = {0};
    int encv = 0, fmtv = 0;
    // sscanf tolerates the surrounding "<!--" / "-->".
    int got = std::sscanf(line.c_str(),
                          " <!-- dmx encoding %63s %d format %63s %d -->",
                          enc, &encv, fmt, &fmtv);
    if (got != 4) {
        if (err) *err = "not a DMX file (bad header): " + line;
        return false;
    }
    dm.encoding = enc;
    dm.encoding_version = encv;
    dm.format = fmt;
    dm.format_version = fmtv;
    return true;
}

std::unique_ptr<Datamodel> Datamodel::LoadFromMemory(const uint8_t* data,
                                                     size_t size,
                                                     std::string* err) {
    auto dm = std::make_unique<Datamodel>();
    size_t body = 0;
    if (!ParseHeader(reinterpret_cast<const char*>(data), size, *dm, body, err))
        return nullptr;

    bool ok = false;
    if (dm->encoding == "keyvalues2" || dm->encoding == "keyvalues2_flat") {
        ok = ParseKeyValues2(*dm, reinterpret_cast<const char*>(data) + body,
                             size - body, err);
    } else if (dm->encoding == "binary") {
        ok = ParseBinary(*dm, data + body, size - body, err);
    } else {
        if (err) *err = "unsupported DMX encoding: " + dm->encoding;
        return nullptr;
    }
    if (!ok) return nullptr;
    return dm;
}

std::unique_ptr<Datamodel> Datamodel::Load(const std::string& path,
                                           std::string* err) {
    std::ifstream f(path, std::ios::binary | std::ios::ate);
    if (!f) {
        if (err) *err = "cannot open file: " + path;
        return nullptr;
    }
    std::streamsize n = f.tellg();
    f.seekg(0);
    // Uninitialized buffer: read() overwrites it whole, so skip the zero-fill a
    // sized vector would do (the source file can be hundreds of MB).
    size_t sz = static_cast<size_t>(n);
    std::unique_ptr<uint8_t[]> buf(new uint8_t[sz]);
    if (n > 0 && !f.read(reinterpret_cast<char*>(buf.get()), n)) {
        if (err) *err = "read error: " + path;
        return nullptr;
    }
    return LoadFromMemory(buf.get(), sz, err);
}

} // namespace pulse::dmx

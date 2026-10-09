// dmx.cpp - data model, guid/type helpers, header parsing and codec dispatch.

#include "dmx/dmx.h"

#include <cctype>
#include <cstdio>
#include <cstring>
#include <filesystem>
#include <fstream>
#include <random>
#include <unordered_set>

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

// --- Writing ---------------------------------------------------------------
Guid NewGuid() {
    static thread_local std::mt19937_64 rng(std::random_device{}());
    Guid id{};
    for (auto& byte : id) byte = static_cast<uint8_t>(rng());
    id[6] = static_cast<uint8_t>((id[6] & 0x0F) | 0x40);
    id[8] = static_cast<uint8_t>((id[8] & 0x3F) | 0x80);
    return id;
}

// AttrValue's alternatives are declared in AttrType order, so a well-formed
// attribute has value.index() == type.
static bool CheckWritable(const Datamodel& dm, std::string* err) {
    auto fail = [&](const std::string& m) {
        if (err) *err = "dmx write: " + m;
        return false;
    };
    if (!dm.root || dm.elements.empty() || dm.elements.front().get() != dm.root)
        return fail("the root must be the first element");
    std::unordered_set<const Element*> owned;
    for (const auto& e : dm.elements) owned.insert(e.get());
    auto inside = [&](ElementPtr e) { return !e || owned.count(e) != 0; };
    for (const auto& e : dm.elements) {
        for (const Attribute& a : e->attributes) {
            const std::string where = e->className + "." + a.name;
            if (!a.semantic.empty()) return fail(where + " has custom type " + a.semantic);
            if (a.type == AttrType::Unknown || a.value.index() != static_cast<size_t>(a.type))
                return fail(where + " value does not match its type");
            if (auto p = std::get_if<ElementPtr>(&a.value); p && !inside(*p))
                return fail(where + " references an element outside the document");
            if (auto v = std::get_if<std::vector<ElementPtr>>(&a.value))
                for (ElementPtr p : *v)
                    if (!inside(p)) return fail(where + " references an element outside the document");
        }
    }
    return true;
}

bool Save(const Datamodel& dm, std::vector<uint8_t>& out, std::string* err) {
    if (!CheckWritable(dm, err)) return false;
    std::string text;
    bool ok = false;
    if (dm.encoding == "binary") {
        ok = WriteBinary(dm, text, err);
    } else if (dm.encoding == "keyvalues2" || dm.encoding == "keyvalues2_flat") {
        ok = WriteKeyValues2(dm, text, err);
    } else if (err) {
        *err = "unsupported DMX encoding: " + dm.encoding;
    }
    if (!ok) return false;
    out.assign(text.begin(), text.end());
    return true;
}

bool Save(const Datamodel& dm, const std::string& path, std::string* err) {
    std::vector<uint8_t> bytes;
    if (!Save(dm, bytes, err)) return false;
    std::ofstream f(std::filesystem::u8path(path), std::ios::binary);
    if (f) f.write(reinterpret_cast<const char*>(bytes.data()), static_cast<std::streamsize>(bytes.size()));
    if (!f) {
        if (err) *err = "cannot write file: " + path;
        return false;
    }
    return true;
}

// --- Builder ---------------------------------------------------------------
Builder::Builder(const std::string& format, int formatVersion, const std::string& encoding,
                 int encodingVersion, uint32_t seed)
    : dm_(std::make_unique<Datamodel>()), seed_(seed) {
    dm_->format = format;
    dm_->format_version = formatVersion;
    dm_->encoding = encoding;
    dm_->encoding_version = encodingVersion;
}

// Same bytes as the text form "%08x-%04x-%04x-%04x-%012x" (seed, seed>>16,
// seed, seed>>8, counter), the decompiler's long-standing id layout.
Guid Builder::NewId() {
    Guid g{};
    auto put = [&](int at, uint64_t v, int bytes) {
        for (int i = bytes - 1; i >= 0; --i, v >>= 8) g[at + i] = static_cast<uint8_t>(v);
    };
    put(0, seed_, 4);
    put(4, (seed_ >> 16) & 0xFFFF, 2);
    put(6, seed_ & 0xFFFF, 2);
    put(8, (seed_ >> 8) & 0xFFFF, 2);
    put(10, next_++, 6);
    return g;
}

Element& Builder::Begin(const char* className, const Guid& id, const std::string& name) {
    current_ = dm_->CreateElement(id, className);
    current_->name = name;
    if (!dm_->root) dm_->root = current_;
    return *current_;
}

Attribute& Builder::Push(const char* key, AttrType type, AttrValue value) {
    current_->attributes.push_back({key, type, std::move(value)});
    return current_->attributes.back();
}

void Builder::Ref(Element& on, const char* k, const Guid& id) {
    on.attributes.push_back({k, AttrType::Element, ElementPtr{}});
    pending_.push_back({&on, on.attributes.size() - 1, {id}, false});
}

void Builder::RefArray(const char* k, const std::vector<Guid>& ids) {
    Push(k, AttrType::ElementArray, std::vector<ElementPtr>(ids.size()));
    pending_.push_back({current_, current_->attributes.size() - 1, ids, true});
}

void Builder::TimeArray(const char* k, const std::vector<float>& seconds) {
    std::vector<dmx::Time> v;
    v.reserve(seconds.size());
    for (float s : seconds) v.push_back({s});
    Push(k, AttrType::TimeArray, std::move(v));
}

Datamodel& Builder::Finish() {
    for (const PendingRef& p : pending_) {
        AttrValue& value = p.owner->attributes[p.attr].value;
        if (!p.array) {
            value = IsNull(p.ids[0]) ? nullptr : dm_->FindById(p.ids[0]);
            continue;
        }
        auto& refs = std::get<std::vector<ElementPtr>>(value);
        for (size_t i = 0; i < p.ids.size(); ++i)
            refs[i] = IsNull(p.ids[i]) ? nullptr : dm_->FindById(p.ids[i]);
    }
    pending_.clear();
    return *dm_;
}

} // namespace pulse::dmx

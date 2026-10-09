// dmx_binary.cpp - binary DMX decoder and encoder.
//
// Covers every version Valve emitted: 1 (no dictionary at all), 2/3 (inline
// string values, 16-bit dictionary), 4 (dictionary string values), 5 and 9
// (32-bit dictionary). Version-specific behaviour is isolated in StringDict +
// the type table. 6-8 never existed and are rejected.
//
// NOTE: validated primarily against keyvalues2 fixtures so far; binary needs a
// real sample to lock the byte layout per the project testing protocol. On an
// unsupported version it fails cleanly rather than guessing.

#include "dmx/dmx.h"

#include <cmath>
#include <cstdio>
#include <cstring>
#include <map>
#include <string>
#include <type_traits>
#include <unordered_map>

namespace pulse::dmx {
namespace {

// Bounds-checked little-endian reader.
class Reader {
public:
    Reader(const uint8_t* p, size_t n) : p_(p), end_(p + n) {}

    bool ok() const { return !bad_; }
    bool eof() const { return p_ >= end_; }

    uint8_t U8() { if (Need(1)) return *p_++; return 0; }
    int32_t I32() {
        if (!Need(4)) return 0;
        int32_t v;
        std::memcpy(&v, p_, 4);
        p_ += 4;
        return v;
    }
    uint16_t U16() {
        if (!Need(2)) return 0;
        uint16_t v;
        std::memcpy(&v, p_, 2);
        p_ += 2;
        return v;
    }
    float F32() {
        if (!Need(4)) return 0;
        float v;
        std::memcpy(&v, p_, 4);
        p_ += 4;
        return v;
    }
    // Null-terminated string.
    std::string CStr() {
        std::string s;
        while (p_ < end_ && *p_ != 0) s += static_cast<char>(*p_++);
        if (p_ < end_) ++p_; // consume null
        else bad_ = true;
        return s;
    }
    void Bytes(void* dst, size_t n) {
        if (Need(n)) { std::memcpy(dst, p_, n); p_ += n; }
    }

private:
    bool Need(size_t n) {
        if (static_cast<size_t>(end_ - p_) < n) { bad_ = true; return false; }
        return true;
    }
    const uint8_t* p_;
    const uint8_t* end_;
    bool bad_ = false;
};

// Maps the on-disk type byte to AttrType. Two schemes exist:
//   ev 1..5 : scalars 1..14, arrays contiguous 15..28 (Element..MatrixArray).
//   ev >= 9 : scalars 1..14 (Time still 7; 15/16 are uint64/uint8, unsupported),
//             arrays encoded as (scalar | 0x20). modeldoc/Source-2 style.
AttrType TypeFromId(uint8_t id, int ev) {
    // Datamodel.NET Binary.cs SupportedAttributes: slot 7 is ObjectID (a 16-byte
    // guid) below ev 3, Time (int32) from ev 3. The reference has no ObjectID
    // reader either - fail instead of desyncing on the width difference.
    if (ev < 3 && (id == 7 || id == 14 + 7)) return AttrType::Unknown;
    if (ev >= 9) {
        bool is_array = (id & 0x20) != 0;
        uint8_t s = id & 0x1F; // scalar part
        if (s < 1 || s > 14) return AttrType::Unknown;
        if (!is_array) return static_cast<AttrType>(s);
        return static_cast<AttrType>(static_cast<int>(AttrType::ElementArray) + (s - 1));
    }
    if (id >= 1 && id <= 28) return static_cast<AttrType>(id);
    return AttrType::Unknown;
}

struct StringDict {
    bool dummy = false;        // no dictionary at all (v1)
    bool encoded_values = false; // scalar string values are dictionary indices
    int len_size = 2;          // bytes for the table count
    int idx_size = 2;          // bytes for an index
    std::vector<std::string> strings;

    // Version rules (Datamodel.NET Binary.cs: Dummy/LengthSize/IndiceSize):
    //   dummy (no table)        : ev == 1
    //   pooled string values    : ev >= 4
    //   table count width       : 4 bytes if ev >= 4, else 2
    //   index width             : 4 bytes if ev >= 5, else 2
    // So v1 = everything inline; v2/v3 = count16/idx16 inline values;
    // v4 = count32/idx16 pooled; v5/v9 = count32/idx32 pooled.
    static bool Configure(int ev, StringDict& d) {
        // 1-5 and 9 are the only versions Valve ever emitted (Binary.cs
        // CodecFormat attributes); 6-8 do not exist, so reject rather than
        // silently misparse one as v5.
        if (ev < 1 || (ev > 5 && ev != 9)) return false;
        d.dummy = (ev == 1);
        d.encoded_values = ev >= 4;
        d.len_size = ev >= 4 ? 4 : 2;
        d.idx_size = ev >= 5 ? 4 : 2;
        d.strings.clear();
        return true;
    }

    void ReadTable(Reader& r) {
        if (dummy) return;
        int32_t count = (len_size == 4) ? r.I32() : r.U16();
        strings.reserve(count > 0 ? count : 0);
        for (int32_t i = 0; i < count; ++i) strings.push_back(r.CStr());
    }

    // A dictionary-indexed string (element type, attribute name).
    std::string Indexed(Reader& r) const {
        if (dummy) return r.CStr();
        int32_t idx = (idx_size == 4) ? r.I32() : r.U16();
        if (idx < 0 || static_cast<size_t>(idx) >= strings.size()) return {};
        return strings[idx];
    }

    // A string *value*: dictionary-indexed only when encoded_values.
    std::string Value(Reader& r) const {
        if (encoded_values) return Indexed(r);
        return r.CStr();
    }
};

// Reads the scalar payload for `st` (a non-array, non-element type).
AttrValue ReadScalar(Reader& r, AttrType st, const StringDict& dict) {
    switch (st) {
        case AttrType::Int: return r.I32();
        case AttrType::Float: return r.F32();
        case AttrType::Bool: return r.U8() != 0;
        case AttrType::String: return dict.Value(r);
        case AttrType::Binary: {
            int32_t n = r.I32();
            Binary b(n > 0 ? n : 0);
            if (n > 0) r.Bytes(b.data(), b.size());
            return b;
        }
        case AttrType::Time: { int32_t ticks = r.I32(); return Time{ticks / 10000.0f}; }
        case AttrType::Color: { Color c; c.r = r.U8(); c.g = r.U8(); c.b = r.U8(); c.a = r.U8(); return c; }
        case AttrType::Vector2: { Vector2 v; v.x = r.F32(); v.y = r.F32(); return v; }
        case AttrType::Vector3: { Vector3 v; v.x = r.F32(); v.y = r.F32(); v.z = r.F32(); return v; }
        case AttrType::Vector4: { Vector4 v; v.x = r.F32(); v.y = r.F32(); v.z = r.F32(); v.w = r.F32(); return v; }
        case AttrType::QAngle: { QAngle a; a.x = r.F32(); a.y = r.F32(); a.z = r.F32(); return a; }
        case AttrType::Quaternion: { Quaternion q; q.x = r.F32(); q.y = r.F32(); q.z = r.F32(); q.w = r.F32(); return q; }
        case AttrType::Matrix: { Matrix m; for (float& f : m.m) f = r.F32(); return m; }
        default: return std::monostate{};
    }
}

// Resolve a binary element reference index to a pointer (null/extern handled).
Element* ResolveRef(Reader& r, AttrType /*scalar=Element*/, const StringDict& dict,
                    const std::vector<Element*>& stubs, Datamodel& dm) {
    int32_t idx = r.I32();
    if (idx == -1) return nullptr;            // null
    if (idx == -2) {                          // external reference by guid string
        Guid g{};
        if (GuidFromString(dict.Value(r), g)) return dm.FindById(g);
        return nullptr;
    }
    if (idx >= 0 && static_cast<size_t>(idx) < stubs.size()) return stubs[idx];
    return nullptr;
}

template <typename T, typename ReadOne>
std::vector<T> ReadArray(Reader& r, int32_t n, ReadOne one) {
    std::vector<T> v;
    v.reserve(n > 0 ? n : 0);
    for (int32_t i = 0; i < n; ++i) v.push_back(one());
    return v;
}

} // namespace

bool ParseBinary(Datamodel& dm, const uint8_t* body, size_t len, std::string* err) {
    auto fail = [&](const std::string& m) {
        if (err && err->empty()) *err = "binary dmx: " + m;
        return false;
    };

    // The header line is written as a null-terminated string, so a stray null
    // may lead the body. Skip it.
    if (len > 0 && body[0] == 0) { ++body; --len; }

    StringDict dict;
    if (!StringDict::Configure(dm.encoding_version, dict))
        return fail("unsupported encoding version " +
                    std::to_string(dm.encoding_version));

    Reader r(body, len);

    // Encoding version 9 (model 22 / modeldoc) prepends an int32 count of
    // "prefix elements" before the string table. It is 0 in practice; non-zero
    // prefix blocks are not yet supported (need a sample to lock the layout).
    if (dm.encoding_version >= 9) {
        int32_t prefix = r.I32();
        if (prefix != 0)
            return fail("binary 9 prefix elements not supported (count=" +
                        std::to_string(prefix) + ")");
    }

    dict.ReadTable(r);
    if (!r.ok()) return fail("truncated string table");

    int32_t element_count = r.I32();
    if (element_count < 0) return fail("bad element count");

    // Pass 1: create stub elements (type, name, id) so refs can resolve.
    std::vector<Element*> stubs;
    stubs.reserve(element_count);
    for (int32_t i = 0; i < element_count; ++i) {
        std::string type = dict.Indexed(r);
        std::string name = dict.Value(r); // dict-encoded only when encoded_values
        Guid id{};
        r.Bytes(id.data(), 16);
        if (!r.ok()) return fail("truncated element header");
        Element* e = dm.CreateElement(id, type);
        e->name = name;
        stubs.push_back(e);
    }
    if (!stubs.empty()) dm.root = stubs.front();

    // Pass 2: attributes.
    for (Element* e : stubs) {
        int32_t attr_count = r.I32();
        if (attr_count < 0) return fail("bad attribute count");
        for (int32_t a = 0; a < attr_count; ++a) {
            std::string aname = dict.Indexed(r);
            uint8_t tid = r.U8();
            AttrType type = TypeFromId(tid, dm.encoding_version);
            if (type == AttrType::Unknown) return fail("unknown attribute type id");

            if (type == AttrType::Element) {
                Element* ref = ResolveRef(r, type, dict, stubs, dm);
                e->attributes.push_back({aname, type, ref});
            } else if (!IsArrayType(type)) {
                if (aname == "name" && type == AttrType::String) {
                    // already captured as element name; still store it
                }
                e->attributes.push_back({aname, type, ReadScalar(r, type, dict)});
            } else {
                AttrType st = ScalarOfArray(type);
                int32_t n = r.I32();
                if (n < 0) return fail("bad array length");
                Attribute attr{aname, type, {}};
                if (st == AttrType::Element) {
                    attr.value = ReadArray<ElementPtr>(r, n, [&] {
                        return ResolveRef(r, st, dict, stubs, dm);
                    });
                } else if (st == AttrType::String) {
                    // String array values are inline (not dictionary-indexed).
                    attr.value = ReadArray<std::string>(r, n, [&] { return r.CStr(); });
                } else {
                    // Generic: read each scalar and re-box into the right vector.
                    switch (st) {
                        case AttrType::Int: attr.value = ReadArray<int32_t>(r, n, [&] { return r.I32(); }); break;
                        case AttrType::Float: attr.value = ReadArray<float>(r, n, [&] { return r.F32(); }); break;
                        case AttrType::Bool: attr.value = ReadArray<bool>(r, n, [&] { return r.U8() != 0; }); break;
                        case AttrType::Binary: attr.value = ReadArray<Binary>(r, n, [&] { int32_t k = r.I32(); Binary b(k > 0 ? k : 0); if (k > 0) r.Bytes(b.data(), b.size()); return b; }); break;
                        case AttrType::Time: attr.value = ReadArray<Time>(r, n, [&] { return Time{r.I32() / 10000.0f}; }); break;
                        case AttrType::Color: attr.value = ReadArray<Color>(r, n, [&] { Color c; c.r = r.U8(); c.g = r.U8(); c.b = r.U8(); c.a = r.U8(); return c; }); break;
                        case AttrType::Vector2: attr.value = ReadArray<Vector2>(r, n, [&] { Vector2 v; v.x = r.F32(); v.y = r.F32(); return v; }); break;
                        case AttrType::Vector3: attr.value = ReadArray<Vector3>(r, n, [&] { Vector3 v; v.x = r.F32(); v.y = r.F32(); v.z = r.F32(); return v; }); break;
                        case AttrType::Vector4: attr.value = ReadArray<Vector4>(r, n, [&] { Vector4 v; v.x = r.F32(); v.y = r.F32(); v.z = r.F32(); v.w = r.F32(); return v; }); break;
                        case AttrType::QAngle: attr.value = ReadArray<QAngle>(r, n, [&] { QAngle q; q.x = r.F32(); q.y = r.F32(); q.z = r.F32(); return q; }); break;
                        case AttrType::Quaternion: attr.value = ReadArray<Quaternion>(r, n, [&] { Quaternion q; q.x = r.F32(); q.y = r.F32(); q.z = r.F32(); q.w = r.F32(); return q; }); break;
                        case AttrType::Matrix: attr.value = ReadArray<Matrix>(r, n, [&] { Matrix m; for (float& f : m.m) f = r.F32(); return m; }); break;
                        default: return fail("unhandled array scalar type");
                    }
                }
                e->attributes.push_back(std::move(attr));
            }
            if (!r.ok()) return fail("truncated attribute data");
        }
    }
    return r.ok() ? true : fail("truncated stream");
}

namespace {

// Little-endian appender for the encoder.
class Writer {
public:
    explicit Writer(std::string& out) : out_(out) {}
    void U8(uint8_t v) { out_.push_back(static_cast<char>(v)); }
    void U16(uint16_t v) { Raw(&v, 2); }
    void I32(int32_t v) { Raw(&v, 4); }
    void F32(float v) { Raw(&v, 4); }
    void CStr(const std::string& s) { out_.append(s.c_str(), s.size() + 1); }
    void Raw(const void* p, size_t n) { out_.append(static_cast<const char*>(p), n); }

private:
    std::string& out_;
};

// Inverse of TypeFromId.
uint8_t TypeToId(AttrType t, int ev) {
    const int id = static_cast<int>(t);
    if (ev < 9 || !IsArrayType(t)) return static_cast<uint8_t>(id);
    return static_cast<uint8_t>((id - static_cast<int>(AttrType::ElementArray) + 1) | 0x20);
}

int32_t Ticks(float seconds) { return static_cast<int32_t>(std::floor(10000.0f * seconds + 0.5f)); }

// Every std::vector alternative except Binary, which is a scalar byte blob.
template <class T> struct IsArrayValue : std::false_type {};
template <class T> struct IsArrayValue<std::vector<T>> : std::true_type {};
template <> struct IsArrayValue<Binary> : std::false_type {};

bool IsNameAttribute(const Attribute& a) { return a.name == "name" && a.type == AttrType::String; }

} // namespace

bool WriteBinary(const Datamodel& dm, std::string& out, std::string* err) {
    const int ev = dm.encoding_version;
    if (ev < 2 || (ev > 5 && ev != 9)) {
        if (err) *err = "binary dmx: cannot write encoding version " + std::to_string(ev);
        return false;
    }
    // The table pools class and attribute names, plus element names and scalar
    // string values from encoding 4 on (inline before). Array strings are inline.
    const bool pooledValues = ev >= 4;
    const bool wideIndex = ev >= 5;
    std::map<std::string, int> pool;
    std::vector<const std::string*> strings;
    auto intern = [&](const std::string& s) {
        const auto ins = pool.emplace(s, static_cast<int>(strings.size()));
        if (ins.second) strings.push_back(&ins.first->first);
    };
    std::unordered_map<const Element*, int32_t> index;
    for (size_t k = 0; k < dm.elements.size(); ++k) {
        const Element& e = *dm.elements[k];
        index[&e] = static_cast<int32_t>(k);
        intern(e.className);
        if (pooledValues) intern(e.name);
        for (const Attribute& a : e.attributes) {
            if (IsNameAttribute(a)) continue;
            intern(a.name);
            if (a.type == AttrType::String && pooledValues) intern(std::get<std::string>(a.value));
        }
    }

    Writer w(out);
    auto sidx = [&](const std::string& s) {
        const int i = pool.at(s);
        if (wideIndex) w.I32(i);
        else w.U16(static_cast<uint16_t>(i));
    };
    auto ref = [&](const Element* e) { w.I32(e ? index.at(e) : -1); };

    char header[160];
    std::snprintf(header, sizeof header, "<!-- dmx encoding binary %d format %s %d -->\n", ev,
                  dm.format.c_str(), dm.format_version);
    out += header;
    w.U8(0); // the reader expects a null-terminated header line
    if (ev >= 9) w.I32(0); // prefix element count

    if (pooledValues) w.I32(static_cast<int32_t>(strings.size()));
    else w.U16(static_cast<uint16_t>(strings.size()));
    for (const std::string* s : strings) w.CStr(*s);

    w.I32(static_cast<int32_t>(dm.elements.size()));
    for (const auto& e : dm.elements) {
        sidx(e->className);
        if (pooledValues) sidx(e->name);
        else w.CStr(e->name);
        w.Raw(e->id.data(), e->id.size());
    }

    // A string inside an array is always inline; a scalar one follows pooledValues.
    auto value = [&](const auto& v, bool inArray) {
        using T = std::decay_t<decltype(v)>;
        if constexpr (std::is_same_v<T, ElementPtr>) ref(v);
        else if constexpr (std::is_same_v<T, int32_t>) w.I32(v);
        else if constexpr (std::is_same_v<T, float>) w.F32(v);
        else if constexpr (std::is_same_v<T, bool>) w.U8(v ? 1 : 0);
        else if constexpr (std::is_same_v<T, std::string>) {
            if (pooledValues && !inArray) sidx(v);
            else w.CStr(v);
        } else if constexpr (std::is_same_v<T, Binary>) {
            w.I32(static_cast<int32_t>(v.size()));
            w.Raw(v.data(), v.size());
        } else if constexpr (std::is_same_v<T, Time>) w.I32(Ticks(v.seconds));
        else if constexpr (std::is_same_v<T, Color>) { w.U8(v.r); w.U8(v.g); w.U8(v.b); w.U8(v.a); }
        else if constexpr (std::is_same_v<T, Vector2>) { w.F32(v.x); w.F32(v.y); }
        else if constexpr (std::is_same_v<T, Vector3> || std::is_same_v<T, QAngle>) {
            w.F32(v.x); w.F32(v.y); w.F32(v.z);
        } else if constexpr (std::is_same_v<T, Vector4> || std::is_same_v<T, Quaternion>) {
            w.F32(v.x); w.F32(v.y); w.F32(v.z); w.F32(v.w);
        } else if constexpr (std::is_same_v<T, Matrix>) {
            for (float f : v.m) w.F32(f);
        }
    };
    for (const auto& e : dm.elements) {
        int32_t count = 0;
        for (const Attribute& a : e->attributes) count += IsNameAttribute(a) ? 0 : 1;
        w.I32(count);
        for (const Attribute& a : e->attributes) {
            if (IsNameAttribute(a)) continue;
            sidx(a.name);
            w.U8(TypeToId(a.type, ev));
            std::visit([&](const auto& v) {
                using T = std::decay_t<decltype(v)>;
                if constexpr (IsArrayValue<T>::value) {
                    w.I32(static_cast<int32_t>(v.size()));
                    for (const auto& item : v) value(static_cast<typename T::value_type>(item), true);
                } else if constexpr (!std::is_same_v<T, std::monostate>) {
                    value(v, false);
                }
            }, a.value);
        }
    }
    return true;
}

} // namespace pulse::dmx

// dmx.h - PulseMDL
//
// Generic DMX (Datamodel Exchange) reader and writer. A clean-room C++ port of
// the format logic from Artfunkel's Datamodel.NET (MIT); no .NET dependency.
//
// One generic element/attribute graph (NOT model-specific) for both directions.
// Encodings: keyvalues2 (read: also _flat and nested; write: flat) and binary
// (read 1-5 and 9, write 2-5 and 9). Version meaning is the caller's concern.

#ifndef PULSEMDL_DMX_H
#define PULSEMDL_DMX_H

#include <array>
#include <cstdint>
#include <map>
#include <memory>
#include <string>
#include <variant>
#include <vector>

namespace pulse::dmx {

// --- Scalar value types ----------------------------------------------------
using Guid = std::array<uint8_t, 16>;
using Binary = std::vector<uint8_t>;

struct Vector2 { float x = 0, y = 0; };
struct Vector3 { float x = 0, y = 0, z = 0; };
struct Vector4 { float x = 0, y = 0, z = 0, w = 0; };
struct Quaternion { float x = 0, y = 0, z = 0, w = 1; };
struct QAngle { float x = 0, y = 0, z = 0; };
struct Color { uint8_t r = 0, g = 0, b = 0, a = 255; };
struct Matrix { float m[16] = {}; };
struct Time { float seconds = 0; }; // distinct from float so the variant can tell them apart

std::string GuidToString(const Guid& g);
bool GuidFromString(const std::string& s, Guid& out); // accepts the 8-4-4-4-12 form

class Element;
using ElementPtr = Element*; // non-owning; lifetime owned by Datamodel

enum class AttrType : uint8_t {
    Unknown = 0,
    // scalars
    Element, Int, Float, Bool, String, Binary, Time, Color,
    Vector2, Vector3, Vector4, QAngle, Quaternion, Matrix,
    // arrays (kept in the same order as the scalars above)
    ElementArray, IntArray, FloatArray, BoolArray, StringArray, BinaryArray,
    TimeArray, ColorArray, Vector2Array, Vector3Array, Vector4Array,
    QAngleArray, QuaternionArray, MatrixArray,
};

bool IsArrayType(AttrType t);
AttrType ScalarOfArray(AttrType t); // ElementArray -> Element, etc.

using AttrValue = std::variant<
    std::monostate,
    ElementPtr, int32_t, float, bool, std::string, Binary, Time, Color,
    Vector2, Vector3, Vector4, QAngle, Quaternion, Matrix,
    std::vector<ElementPtr>, std::vector<int32_t>, std::vector<float>,
    std::vector<bool>, std::vector<std::string>, std::vector<Binary>,
    std::vector<Time>, std::vector<Color>, std::vector<Vector2>,
    std::vector<Vector3>, std::vector<Vector4>, std::vector<QAngle>,
    std::vector<Quaternion>, std::vector<Matrix>>;

struct Attribute {
    std::string name;
    AttrType type = AttrType::Unknown;
    AttrValue value;
    // .pulsemdl extension: custom semantic type token from keyvalues2 (e.g.
    // "rendermesh", "animation"). The value is stored as String; the loader
    // resolves what it refers to. Empty for standard DMX types.
    std::string semantic;
};

// A single DMX element: a typed bag of named attributes plus an id and name.
class Element {
public:
    Guid id{};
    std::string className; // e.g. "DmeJoint", "DmeMesh"
    std::string name;      // cached copy of the "name" string attribute

    std::vector<Attribute> attributes;

    const Attribute* Get(const std::string& n) const;

    // Typed convenience accessors. Return nullptr / fallback when absent or of a
    // different type. Used by the MDL loader.
    ElementPtr GetElement(const std::string& n) const;
    const std::string* GetString(const std::string& n) const;
    int32_t GetInt(const std::string& n, int32_t fallback = 0) const;
    float GetFloat(const std::string& n, float fallback = 0.f) const;
    bool GetBool(const std::string& n, bool fallback = false) const;
    Vector3 GetVector3(const std::string& n, Vector3 fallback = {}) const;

    const std::vector<ElementPtr>* GetElementArray(const std::string& n) const;
    const std::vector<int32_t>* GetIntArray(const std::string& n) const;
    const std::vector<float>* GetFloatArray(const std::string& n) const;
    const std::vector<std::string>* GetStringArray(const std::string& n) const;
    const std::vector<Vector3>* GetVector3Array(const std::string& n) const;
    const std::vector<Vector2>* GetVector2Array(const std::string& n) const;
    const std::vector<Quaternion>* GetQuaternionArray(const std::string& n) const;
};

// The whole loaded document.
class Datamodel {
public:
    std::string encoding;       // "keyvalues2" | "binary"
    int encoding_version = 0;
    std::string format;         // "model"
    int format_version = 0;

    ElementPtr root = nullptr;  // first top-level element
    std::vector<std::unique_ptr<Element>> elements; // ownership

    Element* CreateElement(const Guid& id, const std::string& className);
    Element* FindById(const Guid& id) const;

    // Set an element's id and add it to the id index. Used by the text codec,
    // which discovers an element's id only after creating it.
    void IndexElement(Element* e, const Guid& id);

    // Load from a file or memory buffer. On failure returns nullptr and (if
    // provided) fills *err with a human-readable message.
    static std::unique_ptr<Datamodel> Load(const std::string& path,
                                           std::string* err = nullptr);
    static std::unique_ptr<Datamodel> LoadFromMemory(const uint8_t* data,
                                                     size_t size,
                                                     std::string* err = nullptr);

private:
    std::map<Guid, Element*> by_id_;
};

Guid NewGuid(); // random v4
inline bool IsNull(const Guid& g) { return g == Guid{}; }

// Writes `dm` in its own encoding, encoding_version, format and format_version.
// Elements go out in `elements` order (root first); a "name" string attribute
// is carried by the element header instead.
bool Save(const Datamodel& dm, std::vector<uint8_t>& out, std::string* err = nullptr);
bool Save(const Datamodel& dm, const std::string& path, std::string* err = nullptr);

// Builds a fresh document element by element. Elements are stored in Begin order
// and a reference may name an element Begun later; Finish() resolves them.
class Builder {
public:
    Builder(const std::string& format, int formatVersion, const std::string& encoding,
            int encodingVersion, uint32_t seed = 0);

    // Deterministic: the seed fills the first groups, a counter the last.
    Guid NewId();

    Element& Begin(const char* className, const Guid& id, const std::string& name);
    void End() {} // keeps emit sites reading as blocks
    Element& Current() { return *current_; }
    Element* Root() { return dm_->root; }

    Attribute& Push(const char* key, AttrType type, AttrValue value);
    void Str(const char* k, const std::string& v) { Push(k, AttrType::String, v); }
    void Int(const char* k, int32_t v) { Push(k, AttrType::Int, v); }
    void Float(const char* k, float v) { Push(k, AttrType::Float, v); }
    void Bool(const char* k, bool v) { Push(k, AttrType::Bool, v); }
    void Time(const char* k, float seconds) { Push(k, AttrType::Time, dmx::Time{seconds}); }
    template <class V> void Vec3(const char* k, const V& v) {
        Push(k, AttrType::Vector3, Vector3{v.x, v.y, v.z});
    }
    template <class Q> void Quat(const char* k, const Q& q) {
        Push(k, AttrType::Quaternion, Quaternion{q.x, q.y, q.z, q.w});
    }
    void Ref(const char* k, const Guid& id) { Ref(Current(), k, id); }
    void Ref(Element& on, const char* k, const Guid& id);
    void RefArray(const char* k, const std::vector<Guid>& ids);
    void IntArray(const char* k, const std::vector<int32_t>& v) { Push(k, AttrType::IntArray, v); }
    void FloatArray(const char* k, const std::vector<float>& v) { Push(k, AttrType::FloatArray, v); }
    void TimeArray(const char* k, const std::vector<float>& seconds);
    void StrArray(const char* k, const std::vector<std::string>& v) { Push(k, AttrType::StringArray, v); }
    template <class Q> void QuatArray(const char* k, const std::vector<Q>& v) {
        std::vector<Quaternion> out;
        out.reserve(v.size());
        for (const Q& q : v) out.push_back({q.x, q.y, q.z, q.w});
        Push(k, AttrType::QuaternionArray, std::move(out));
    }
    template <class V> void V3Array(const char* k, const std::vector<V>& v) {
        std::vector<Vector3> out;
        out.reserve(v.size());
        for (const V& p : v) out.push_back({p.x, p.y, p.z});
        Push(k, AttrType::Vector3Array, std::move(out));
    }
    template <class V> void V2Array(const char* k, const std::vector<V>& v) {
        std::vector<Vector2> out;
        out.reserve(v.size());
        for (const V& p : v) out.push_back({p.x, p.y});
        Push(k, AttrType::Vector2Array, std::move(out));
    }

    // Resolves references (an id never Begun becomes null) and hands the document over.
    Datamodel& Finish();
    bool Save(const std::string& path, std::string* err = nullptr) { return dmx::Save(Finish(), path, err); }

private:
    struct PendingRef { Element* owner; size_t attr; std::vector<Guid> ids; bool array; };
    std::unique_ptr<Datamodel> dm_;
    Element* current_ = nullptr;
    std::vector<PendingRef> pending_;
    uint32_t seed_;
    uint32_t next_ = 1;
};

// Codec entry points (buffer is the bytes *after* the header line). Internal,
// but declared here so the codecs can be in separate translation units.
bool ParseKeyValues2(Datamodel& dm, const char* body, size_t len, std::string* err);
bool ParseBinary(Datamodel& dm, const uint8_t* body, size_t len, std::string* err);
bool WriteKeyValues2(const Datamodel& dm, std::string& out, std::string* err);
bool WriteBinary(const Datamodel& dm, std::string& out, std::string* err);

} // namespace pulse::dmx

#endif // PULSEMDL_DMX_H

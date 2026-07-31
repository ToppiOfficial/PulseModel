// dmx_keyvalues2.cpp - text (keyvalues2 / keyvalues2_flat) DMX decoder.
//
// Grammar (per element):
//   "ClassName" { "attr" "type" value  ... }
// where value is a quoted scalar, a [ ... ] array, or - when "type" is itself an
// element class name - a nested { ... } element. Element references use
// type "element" with a guid string ("" = null). Resolution of references is
// deferred to a second pass so forward references work.
//
// .pulsemdl extensions (hand-authored format, not standard DMX):
//   - "attr" "customtype" "value"  - an unknown type token followed by a quoted
//     string is a semantic reference (e.g. "rendermesh", "animation"). Stored
//     as a String attribute with Attribute::semantic = the type token.
//   - element_array entries may be "entryname" "ClassName" { ... } - the
//     leading string names the inline element.

#include "dmx/dmx.h"

#include <cstdlib>
#include <cstring>
#include <optional>

namespace pulse::dmx {
namespace {

// ---- Tokenizer ----
enum class Tok { String, LBrace, RBrace, LBracket, RBracket, Comma, End };

class Lexer {
public:
    Lexer(const char* p, size_t n) : p_(p), end_(p + n) {}

    Tok Next(std::string& str) {
        SkipTrivia();
        if (p_ >= end_) return Tok::End;
        char c = *p_;
        switch (c) {
            case '{': ++p_; return Tok::LBrace;
            case '}': ++p_; return Tok::RBrace;
            case '[': ++p_; return Tok::LBracket;
            case ']': ++p_; return Tok::RBracket;
            case ',': ++p_; return Tok::Comma;
            case '"': return ReadString(str);
            default:
                // Bare token (rare in KV2, but be lenient).
                return ReadBare(str);
        }
    }

    // Look at the next token without consuming.
    Tok Peek(std::string& str) {
        const char* save = p_;
        Tok t = Next(str);
        p_ = save;
        return t;
    }

    bool error = false;

private:
    void SkipTrivia() {
        for (;;) {
            while (p_ < end_ && (*p_ == ' ' || *p_ == '\t' || *p_ == '\r' ||
                                 *p_ == '\n'))
                ++p_;
            // "<!-- ... -->" comments.
            if (end_ - p_ >= 4 && std::strncmp(p_, "<!--", 4) == 0) {
                const char* e = p_;
                while (e + 3 <= end_ && std::strncmp(e, "-->", 3) != 0) ++e;
                p_ = (e + 3 <= end_) ? e + 3 : end_;
                continue;
            }
            break;
        }
    }

    Tok ReadString(std::string& str) {
        str.clear();
        ++p_; // opening quote
        while (p_ < end_) {
            char c = *p_++;
            if (c == '\\' && p_ < end_) {
                char n = *p_++;
                switch (n) {
                    case 'n': str += '\n'; break;
                    case 't': str += '\t'; break;
                    case '"': str += '"'; break;
                    case '\\': str += '\\'; break;
                    default: str += n; break;
                }
            } else if (c == '"') {
                return Tok::String;
            } else {
                str += c;
            }
        }
        error = true;
        return Tok::End;
    }

    Tok ReadBare(std::string& str) {
        str.clear();
        while (p_ < end_) {
            char c = *p_;
            if (c == ' ' || c == '\t' || c == '\r' || c == '\n' || c == '{' ||
                c == '}' || c == '[' || c == ']' || c == ',')
                break;
            str += c;
            ++p_;
        }
        return Tok::String;
    }

    const char* p_;
    const char* end_;
};

// ---- scalar string -> value helpers ----
int ParseFloats(const std::string& s, float* out, int n) {
    const char* p = s.c_str();
    int i = 0;
    while (i < n) {
        char* e = nullptr;
        float v = std::strtof(p, &e);
        if (e == p) break;
        out[i++] = v;
        p = e;
    }
    return i;
}

// Valve's text unserializer normalizes every parsed quaternion
// (tier1 utlbufferutil Unserialize(CUtlBuffer&, Quaternion&) calls
// QuaternionNormalize). Binary DMX does NOT do this - keyvalues2 only.
// Same arithmetic as the reference: float sum, double sqrt, w-z-y-x order.
Quaternion NormalizeParsedQuat(Quaternion q) {
    float radius = q.x * q.x + q.y * q.y + q.z * q.z + q.w * q.w;
    if (radius) {
        radius = static_cast<float>(sqrt(static_cast<double>(radius)));
        float iradius = 1.0f / radius;
        q.w *= iradius;
        q.z *= iradius;
        q.y *= iradius;
        q.x *= iradius;
    }
    return q;
}

int32_t ParseInt(const std::string& s) {
    return static_cast<int32_t>(std::strtol(s.c_str(), nullptr, 10));
}

bool ParseBool(const std::string& s) {
    return s == "1" || s == "true" || s == "True";
}

Color ParseColor(const std::string& s) {
    float f[4] = {0, 0, 0, 255};
    int n = ParseFloats(s, f, 4);
    Color c;
    c.r = static_cast<uint8_t>(f[0]);
    c.g = static_cast<uint8_t>(f[1]);
    c.b = static_cast<uint8_t>(f[2]);
    c.a = static_cast<uint8_t>(n >= 4 ? f[3] : 255);
    return c;
}

Binary ParseBinaryHex(const std::string& s) {
    Binary b;
    int hi = -1;
    auto hv = [](char c) -> int {
        if (c >= '0' && c <= '9') return c - '0';
        if (c >= 'a' && c <= 'f') return c - 'a' + 10;
        if (c >= 'A' && c <= 'F') return c - 'A' + 10;
        return -1;
    };
    for (char c : s) {
        int v = hv(c);
        if (v < 0) continue;
        if (hi < 0) hi = v;
        else { b.push_back(static_cast<uint8_t>((hi << 4) | v)); hi = -1; }
    }
    return b;
}

// Map a keyvalues2 type-name token to an AttrType. Returns nullopt when the
// token is not a known type (meaning it's an inline element class name).
std::optional<AttrType> TypeFromName(const std::string& t) {
    struct E { const char* n; AttrType t; };
    static const E table[] = {
        {"element", AttrType::Element},     {"int", AttrType::Int},
        {"float", AttrType::Float},         {"bool", AttrType::Bool},
        {"string", AttrType::String},       {"binary", AttrType::Binary},
        {"time", AttrType::Time},           {"color", AttrType::Color},
        {"vector2", AttrType::Vector2},     {"vector3", AttrType::Vector3},
        {"vector4", AttrType::Vector4},     {"qangle", AttrType::QAngle},
        {"quaternion", AttrType::Quaternion}, {"matrix", AttrType::Matrix},
        {"element_array", AttrType::ElementArray}, {"int_array", AttrType::IntArray},
        {"float_array", AttrType::FloatArray}, {"bool_array", AttrType::BoolArray},
        {"string_array", AttrType::StringArray}, {"binary_array", AttrType::BinaryArray},
        {"time_array", AttrType::TimeArray}, {"color_array", AttrType::ColorArray},
        {"vector2_array", AttrType::Vector2Array}, {"vector3_array", AttrType::Vector3Array},
        {"vector4_array", AttrType::Vector4Array}, {"qangle_array", AttrType::QAngleArray},
        {"quaternion_array", AttrType::QuaternionArray}, {"matrix_array", AttrType::MatrixArray},
    };
    for (const auto& e : table)
        if (t == e.n) return e.t;
    return std::nullopt;
}

// Convert a scalar string to a non-element AttrValue of the given type.
AttrValue ScalarValue(AttrType t, const std::string& s) {
    float f[16] = {};
    switch (t) {
        case AttrType::Int: return ParseInt(s);
        case AttrType::Float: ParseFloats(s, f, 1); return f[0];
        case AttrType::Bool: return ParseBool(s);
        case AttrType::String: return s;
        case AttrType::Binary: return ParseBinaryHex(s);
        case AttrType::Time: { ParseFloats(s, f, 1); return Time{f[0]}; }
        case AttrType::Color: return ParseColor(s);
        case AttrType::Vector2: ParseFloats(s, f, 2); return Vector2{f[0], f[1]};
        case AttrType::Vector3: ParseFloats(s, f, 3); return Vector3{f[0], f[1], f[2]};
        case AttrType::Vector4: ParseFloats(s, f, 4); return Vector4{f[0], f[1], f[2], f[3]};
        case AttrType::QAngle: ParseFloats(s, f, 3); return QAngle{f[0], f[1], f[2]};
        case AttrType::Quaternion: ParseFloats(s, f, 4); return NormalizeParsedQuat(Quaternion{f[0], f[1], f[2], f[3]});
        case AttrType::Matrix: { Matrix m; ParseFloats(s, m.m, 16); return m; }
        default: return std::monostate{};
    }
}

// ---- Parser ----
struct PendingRef {
    Element* owner;
    size_t attr_index;
    int array_index; // -1 for scalar element attribute
    Guid target;
};

class Parser {
public:
    Parser(Datamodel& dm, const char* p, size_t n) : dm_(dm), lex_(p, n) {}

    bool Run(std::string* err) {
        std::string s;
        for (;;) {
            Tok t = lex_.Next(s);
            if (t == Tok::End) break;
            if (t != Tok::String) { return Fail(err, "expected element type name"); }
            Element* e = ParseElement(s, err);
            if (!e) return false;
            if (!dm_.root) dm_.root = e;
        }
        if (lex_.error) return Fail(err, "unterminated string");
        // Resolve deferred element references.
        for (const auto& r : pending_) {
            Element* target = dm_.FindById(r.target);
            Attribute& a = r.owner->attributes[r.attr_index];
            if (r.array_index < 0) {
                a.value = target; // ElementPtr (may be null)
            } else {
                std::get<std::vector<ElementPtr>>(a.value)[r.array_index] = target;
            }
        }
        return true;
    }

private:
    bool Fail(std::string* err, const char* msg) {
        if (err && err->empty()) *err = std::string("keyvalues2: ") + msg;
        return false;
    }

    // Consumes a "{ ... }" body for an element of the given class name.
    Element* ParseElement(const std::string& className, std::string* err) {
        std::string s;
        if (lex_.Next(s) != Tok::LBrace) { Fail(err, "expected '{'"); return nullptr; }

        Element* e = dm_.CreateElement(Guid{}, className);
        for (;;) {
            Tok t = lex_.Next(s);
            if (t == Tok::RBrace) break;
            if (t != Tok::String) { Fail(err, "expected attribute name or '}'"); return nullptr; }
            std::string attrName = s;

            if (lex_.Next(s) != Tok::String) { Fail(err, "expected attribute type"); return nullptr; }
            std::string typeName = s;

            // The element id is special: set it and register the element.
            if (typeName == "elementid") {
                if (lex_.Next(s) != Tok::String) { Fail(err, "expected guid"); return nullptr; }
                Guid g{};
                if (attrName == "id" && GuidFromString(s, g)) Register(e, g);
                continue;
            }

            auto known = TypeFromName(typeName);
            if (!known) {
                std::string peeked;
                if (lex_.Peek(peeked) == Tok::String) {
                    // .pulsemdl semantic reference: "attr" "customtype" "value".
                    // Stored as a string; the type token is kept in `semantic`.
                    lex_.Next(s);
                    e->attributes.push_back({attrName, AttrType::String, s});
                    e->attributes.back().semantic = typeName;
                    continue;
                }
                // Inline element: typeName is a class name, value is a { } block.
                Element* child = ParseElement(typeName, err);
                if (!child) return nullptr;
                e->attributes.push_back({attrName, AttrType::Element, child});
                continue;
            }

            AttrType type = *known;
            if (type == AttrType::Element) {
                if (lex_.Next(s) != Tok::String) { Fail(err, "expected element guid"); return nullptr; }
                size_t idx = e->attributes.size();
                e->attributes.push_back({attrName, AttrType::Element, (ElementPtr) nullptr});
                Guid g{};
                if (GuidFromString(s, g)) pending_.push_back({e, idx, -1, g});
                continue;
            }

            if (IsArrayType(type)) {
                if (!ParseArray(e, attrName, type, err)) return nullptr;
                continue;
            }

            // plain scalar
            if (lex_.Next(s) != Tok::String) { Fail(err, "expected scalar value"); return nullptr; }
            if (attrName == "name" && type == AttrType::String) e->name = s;
            e->attributes.push_back({attrName, type, ScalarValue(type, s)});
        }
        return e;
    }

    bool ParseArray(Element* e, const std::string& attrName, AttrType type,
                    std::string* err) {
        std::string s;
        if (lex_.Next(s) != Tok::LBracket) return Fail(err, "expected '['");

        size_t attrIdx = e->attributes.size();
        e->attributes.push_back({attrName, type, {}});
        Attribute& attr = e->attributes[attrIdx];

        if (type == AttrType::ElementArray) {
            std::vector<ElementPtr> arr;
            for (;;) {
                Tok t = lex_.Next(s);
                if (t == Tok::RBracket) break;
                if (t == Tok::Comma) continue;
                if (t != Tok::String) return Fail(err, "bad element_array entry");
                if (s == "element") {
                    if (lex_.Next(s) != Tok::String) return Fail(err, "expected guid");
                    int ai = static_cast<int>(arr.size());
                    arr.push_back(nullptr);
                    Guid g{};
                    if (GuidFromString(s, g)) pending_.push_back({e, attrIdx, ai, g});
                } else {
                    // .pulsemdl extension: "entryname" "ClassName" { ... } -
                    // a leading string names the inline element.
                    std::string entryName;
                    std::string peeked;
                    if (lex_.Peek(peeked) == Tok::String) {
                        entryName = s;
                        lex_.Next(s); // s = class name
                    }
                    Element* child = ParseElement(s, err); // inline element
                    if (!child) return false;
                    if (!entryName.empty() && child->name.empty())
                        child->name = entryName;
                    arr.push_back(child);
                }
            }
            attr.value = std::move(arr);
            return true;
        }

        // Scalar arrays: collect quoted entries until ']'.
        AttrType st = ScalarOfArray(type);
        std::vector<std::string> raw;
        for (;;) {
            Tok t = lex_.Next(s);
            if (t == Tok::RBracket) break;
            if (t == Tok::Comma) continue;
            if (t != Tok::String) return Fail(err, "bad array entry");
            raw.push_back(s);
        }
        BuildScalarArray(attr, st, raw);
        return true;
    }

    template <typename T, typename F>
    static std::vector<T> Map(const std::vector<std::string>& raw, F f) {
        std::vector<T> v;
        v.reserve(raw.size());
        for (const auto& s : raw) v.push_back(f(s));
        return v;
    }

    void BuildScalarArray(Attribute& attr, AttrType st,
                          const std::vector<std::string>& raw) {
        switch (st) {
            case AttrType::Int: attr.value = Map<int32_t>(raw, [](const std::string& s) { return ParseInt(s); }); break;
            case AttrType::Float: attr.value = Map<float>(raw, [](const std::string& s) { float f; ParseFloats(s, &f, 1); return f; }); break;
            case AttrType::Bool: attr.value = Map<bool>(raw, [](const std::string& s) { return ParseBool(s); }); break;
            case AttrType::String: attr.value = raw; break;
            case AttrType::Binary: attr.value = Map<Binary>(raw, [](const std::string& s) { return ParseBinaryHex(s); }); break;
            case AttrType::Time: attr.value = Map<Time>(raw, [](const std::string& s) { float f; ParseFloats(s, &f, 1); return Time{f}; }); break;
            case AttrType::Color: attr.value = Map<Color>(raw, [](const std::string& s) { return ParseColor(s); }); break;
            case AttrType::Vector2: attr.value = Map<Vector2>(raw, [](const std::string& s) { float f[2] = {}; ParseFloats(s, f, 2); return Vector2{f[0], f[1]}; }); break;
            case AttrType::Vector3: attr.value = Map<Vector3>(raw, [](const std::string& s) { float f[3] = {}; ParseFloats(s, f, 3); return Vector3{f[0], f[1], f[2]}; }); break;
            case AttrType::Vector4: attr.value = Map<Vector4>(raw, [](const std::string& s) { float f[4] = {}; ParseFloats(s, f, 4); return Vector4{f[0], f[1], f[2], f[3]}; }); break;
            case AttrType::QAngle: attr.value = Map<QAngle>(raw, [](const std::string& s) { float f[3] = {}; ParseFloats(s, f, 3); return QAngle{f[0], f[1], f[2]}; }); break;
            case AttrType::Quaternion: attr.value = Map<Quaternion>(raw, [](const std::string& s) { float f[4] = {}; ParseFloats(s, f, 4); return NormalizeParsedQuat(Quaternion{f[0], f[1], f[2], f[3]}); }); break;
            case AttrType::Matrix: attr.value = Map<Matrix>(raw, [](const std::string& s) { Matrix m; ParseFloats(s, m.m, 16); return m; }); break;
            default: break;
        }
    }

    void Register(Element* e, const Guid& g) { dm_.IndexElement(e, g); }

    Datamodel& dm_;
    Lexer lex_;
    std::vector<PendingRef> pending_;
};

} // namespace

bool ParseKeyValues2(Datamodel& dm, const char* body, size_t len, std::string* err) {
    Parser p(dm, body, len);
    return p.Run(err);
}

} // namespace pulse::dmx

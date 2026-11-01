// dmxvalidate.cpp - model DMX checks run before a file is loaded, and the
// standard vertex-stream lookup the loader shares with them.
//
// A stream counts only when the reference CDmeVertexData would register it: listed
// in vertexFormat, values present, and an int Indices array (skinning excepted).

#include "dmxloader.h"
#include "diagnostic.h"
#include "strcompat.h"

#include <algorithm>
#include <cctype>
#include <cstddef>
#include <cstdint>
#include <cstring>
#include <limits>
#include <set>
#include <string>
#include <type_traits>
#include <variant>
#include <vector>

namespace pulse::source {
namespace {

namespace dmx = pulse::dmx;

// Model 1-18 use the legacy names only; model 22 tooling knows both and prefers $0.
struct Stream {
    const char* legacy;
    const char* indexed;
    dmx::AttrType type;
    const char* typeName;
    bool skin; // shares the position indices, so it has no Indices array
};

constexpr Stream kStreams[] = {
    {"positions", "position$0", dmx::AttrType::Vector3Array, "vector3_array", false},
    {"normals", "normal$0", dmx::AttrType::Vector3Array, "vector3_array", false},
    {"textureCoordinates", "texcoord$0", dmx::AttrType::Vector2Array, "vector2_array", false},
    {"jointWeights", "blendweights$0", dmx::AttrType::FloatArray, "float_array", true},
    {"jointIndices", "blendindices$0", dmx::AttrType::IntArray, "int_array", true},
    {"balance", "balance$0", dmx::AttrType::FloatArray, "float_array", false},
    {"speed", "speed$0", dmx::AttrType::FloatArray, "float_array", false},
    {"wrinkle", "wrinkle$0", dmx::AttrType::FloatArray, "float_array", false},
};

const Stream* FindStream(const char* legacy) {
    for (const Stream& stream : kStreams)
        if (std::strcmp(stream.legacy, legacy) == 0) return &stream;
    return nullptr;
}

std::string Folded(std::string text) {
    std::transform(text.begin(), text.end(), text.begin(),
                   [](unsigned char ch) { return static_cast<char>(std::tolower(ch)); });
    return text;
}

const dmx::Attribute* FindAttribute(const dmx::Element& e, const std::string& name) {
    for (const dmx::Attribute& a : e.attributes)
        if (_stricmp(a.name.c_str(), name.c_str()) == 0) return &a;
    return nullptr;
}

bool Declared(const dmx::Element& e, const std::string& name) {
    const auto* format = e.GetStringArray("vertexFormat");
    if (!format) return false;
    for (const std::string& field : *format)
        if (_stricmp(field.c_str(), name.c_str()) == 0) return true;
    return false;
}

size_t ArraySize(const dmx::Attribute& a) {
    return std::visit([](const auto& value) -> size_t {
        using T = std::decay_t<decltype(value)>;
        if constexpr (std::is_same_v<T, std::vector<dmx::Vector3>> ||
                      std::is_same_v<T, std::vector<dmx::Vector2>> ||
                      std::is_same_v<T, std::vector<float>> ||
                      std::is_same_v<T, std::vector<int32_t>>)
            return value.size();
        return 0;
    }, a.value);
}

std::vector<const char*> AcceptedNames(const Stream& stream, int modelVersion) {
    if (modelVersion == 22) return {stream.indexed, stream.legacy};
    return {stream.legacy};
}

struct Registered {
    const dmx::Attribute* values = nullptr;
    const dmx::Attribute* indices = nullptr;
};

// Why `name` is not registered on `e`, or "" when it is (or is not there at all).
std::string SkipReason(const dmx::Element& e, const std::string& name, const Stream& stream,
                       Registered* out) {
    const dmx::Attribute* values = FindAttribute(e, name);
    const bool declared = Declared(e, name);
    if (!values && !declared) return {};
    if (!declared) return "is not listed in vertexFormat";
    if (!values || values->type != stream.type) return std::string("has no ") + stream.typeName + " values";

    const dmx::Attribute* indices = stream.skin ? nullptr : FindAttribute(e, name + "Indices");
    if (!stream.skin && (!indices || indices->type != dmx::AttrType::IntArray))
        return "has no int_array " + name + "Indices";

    if (out) *out = {values, indices};
    return {};
}

Registered Resolve(const dmx::Element& e, int modelVersion, const Stream& stream) {
    for (const char* name : AcceptedNames(stream, modelVersion)) {
        Registered found;
        if (SkipReason(e, name, stream, &found).empty() && found.values) return found;
    }
    return {};
}

struct Check {
    const dmx::Datamodel& dm;
    const std::string& file;
    std::string* err;

    bool Fail(const dmx::Element& e, const std::string& field, const std::string& problem) const {
        if (err)
            *err = file + ": DMX model " + std::to_string(dm.format_version) + " " + e.className +
                   " '" + e.name + "', field '" + field + "' " + problem;
        return false;
    }

    void Warn(const dmx::Element& e, const std::string& field, const std::string& problem) const {
        pulse::diag::Warnf("%s: %s '%s', stream '%s' %s - skipped", file.c_str(), e.className.c_str(),
                           e.name.c_str(), field.c_str(), problem.c_str());
    }
};

// Index arrays must stay inside their values; base streams also cover every corner.
bool CheckIndices(const Check& check, const dmx::Element& e, const Registered& r, bool delta,
                  size_t corners) {
    const auto* indices = std::get_if<std::vector<int32_t>>(&r.indices->value);
    const size_t values = ArraySize(*r.values);
    const size_t expected = delta ? values : corners;
    if (!indices || indices->size() != expected)
        return check.Fail(e, r.indices->name, "needs " + std::to_string(expected) + " indices");

    for (int32_t index : *indices)
        if (index < 0 || (!delta && static_cast<size_t>(index) >= values))
            return check.Fail(e, r.indices->name,
                              delta ? "has a negative index"
                                    : "indexes past its " + std::to_string(values) + " values");
    return true;
}

bool ValidateVertexData(const Check& check, const dmx::Element& e) {
    const int version = check.dm.format_version;
    const bool delta = e.className == "DmeVertexDeltaData";

    std::set<std::string> seen;
    if (const auto* format = e.GetStringArray("vertexFormat"))
        for (const std::string& field : *format)
            if (!seen.insert(Folded(field)).second)
                pulse::diag::Warnf("%s: %s '%s' lists '%s' more than once in vertexFormat",
                                   check.file.c_str(), e.className.c_str(), e.name.c_str(), field.c_str());

    // the other scheme's names: a mis-tagged base mesh fails, a delta is only skipped
    for (const Stream& stream : kStreams) {
        if (version == 22) break;
        if (!FindAttribute(e, stream.indexed) && !Declared(e, stream.indexed)) continue;
        if (!delta)
            return check.Fail(e, stream.indexed, "is a model 22 name; this file declares model " +
                                                 std::to_string(version));
        check.Warn(e, stream.indexed, "is a model 22 name in a model " + std::to_string(version) + " file");
    }

    for (const Stream& stream : kStreams)
        for (const char* name : AcceptedNames(stream, version)) {
            const std::string reason = SkipReason(e, name, stream, nullptr);
            if (!reason.empty()) check.Warn(e, name, reason);
        }

    const Registered positions = Resolve(e, version, *FindStream("positions"));
    if (!delta && !positions.values)
        return check.Fail(e, version == 22 ? "position$0" : "positions",
                          "is missing - the mesh has no vertices");

    const auto* cornerIndices = positions.indices
        ? std::get_if<std::vector<int32_t>>(&positions.indices->value) : nullptr;
    const size_t corners = cornerIndices ? cornerIndices->size() : 0;
    for (const Stream& stream : kStreams) {
        if (stream.skin) continue;
        const Registered r = Resolve(e, version, stream);
        if (r.values && !CheckIndices(check, e, r, delta, corners)) return false;
    }
    if (delta) return true;

    const dmx::Attribute* jointCount = FindAttribute(e, "jointCount");
    const auto* influenceCount = jointCount ? std::get_if<int32_t>(&jointCount->value) : nullptr;
    if (jointCount && (jointCount->type != dmx::AttrType::Int || !influenceCount || *influenceCount < 0))
        return check.Fail(e, jointCount->name, "must be a nonnegative int");

    const int influences = influenceCount ? *influenceCount : 0;
    if (influences <= 0) return true;

    // resolved the way the loader asks, so a skinned mesh can never load unweighted
    const size_t vertices = ArraySize(*positions.values);
    if (vertices > std::numeric_limits<size_t>::max() / static_cast<size_t>(influences))
        return check.Fail(e, jointCount->name, "is too large for the vertex count");
    const size_t expected = vertices * static_cast<size_t>(influences);
    for (const char* legacy : {"jointWeights", "jointIndices"}) {
        const std::string name = DmxStreamName(e, version, legacy);
        const dmx::Attribute* values = name.empty() ? nullptr : FindAttribute(e, name);
        if (!values) {
            const Stream& stream = *FindStream(legacy);
            return check.Fail(e, version == 22 ? stream.indexed : stream.legacy,
                              "is missing - jointCount is " + std::to_string(influences) +
                              " but the mesh has no readable skinning");
        }
        if (ArraySize(*values) != expected)
            return check.Fail(e, values->name, "needs " + std::to_string(expected) +
                                               " values (jointCount x vertices)");
    }
    return true;
}

// Delta indices address their base stream; wrinkle deltas address the UV values.
bool ValidateDeltaTargets(const Check& check, const dmx::Element& mesh, const dmx::Element& base) {
    const auto* deltas = mesh.GetElementArray("deltaStates");
    if (!deltas) return true;

    const int version = check.dm.format_version;
    for (const dmx::Element* delta : *deltas) {
        if (!delta || delta->className != "DmeVertexDeltaData")
            return check.Fail(mesh, "deltaStates", "must hold DmeVertexDeltaData elements");

        for (const Stream& stream : kStreams) {
            if (stream.skin) continue;
            const Registered r = Resolve(*delta, version, stream);
            if (!r.values) continue;

            const Stream& target = std::strcmp(stream.legacy, "wrinkle") == 0
                ? *FindStream("textureCoordinates") : stream;
            const Registered baseStream = Resolve(base, version, target);
            const size_t count = baseStream.values ? ArraySize(*baseStream.values) : 0;
            for (int32_t index : std::get<std::vector<int32_t>>(r.indices->value))
                if (static_cast<size_t>(index) >= count)
                    return check.Fail(*delta, r.indices->name, "indexes past base stream '" +
                                                               std::string(target.legacy) + "'");
        }
    }
    return true;
}

} // namespace

std::string DmxStreamName(const dmx::Element& e, int modelVersion, const char* legacyName) {
    // "jointIndices" is a stream of its own, so an exact name wins over the Indices suffix
    constexpr size_t suffix = sizeof("Indices") - 1;
    std::string name = legacyName;
    const Stream* stream = FindStream(legacyName);
    const bool indices = !stream && name.size() > suffix &&
                         name.compare(name.size() - suffix, suffix, "Indices") == 0;
    if (indices) {
        name.resize(name.size() - suffix);
        stream = FindStream(name.c_str());
    }
    if (!stream) return {};
    const Registered r = Resolve(e, modelVersion, *stream);
    const dmx::Attribute* a = indices ? r.indices : r.values;
    return a ? a->name : std::string();
}

const dmx::Element* DmxMeshBindState(const dmx::Element* mesh) {
    if (const dmx::Element* bind = mesh->GetElement("bindState")) return bind;
    if (const auto* states = mesh->GetElementArray("baseStates")) {
        for (const dmx::Element* state : *states)
            if (state && state->name == "bind") return state;
        if (!states->empty()) return states->front();
    }
    return mesh->GetElement("currentState");
}

bool ValidateDmxModel(const dmx::Datamodel& dm, const std::string& file, std::string* err) {
    const bool supported = dm.format == "model" &&
        ((dm.format_version >= 1 && dm.format_version <= 18) || dm.format_version == 22);
    if (!supported) {
        if (err)
            *err = file + ": unsupported DMX format '" + dm.format + " " +
                   std::to_string(dm.format_version) + "', expected model 1-18 or model 22";
        return false;
    }
    if (!dm.root) {
        if (err) *err = file + ": DMX has no root element";
        return false;
    }

    const Check check{dm, file, err};
    for (const auto& e : dm.elements)
        if ((e->className == "DmeVertexData" || e->className == "DmeVertexDeltaData") &&
            !ValidateVertexData(check, *e))
            return false;

    for (const auto& mesh : dm.elements) {
        if (mesh->className != "DmeMesh" && mesh->className != "DmePhysicsShape") continue;
        const dmx::Element* base = DmxMeshBindState(mesh.get());
        if (!base || base->className != "DmeVertexData")
            return check.Fail(*mesh, "bindState", "must be a DmeVertexData");
        if (!ValidateDeltaTargets(check, *mesh, *base)) return false;
    }
    return true;
}

} // namespace pulse::source

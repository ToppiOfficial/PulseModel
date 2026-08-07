// writephy.cpp - PulseMDL .phy writer
//
// Mirrors the reference CollisionModel_Write. The
// binary hulls were already serialized into PhysicsSolid::blob by
// BuildCollisionModel; this file only frames them and appends the plaintext
// keyvalues tail.

#include "writer.h"

#include <cstdio>
#include <cstring>

#include "format/phy.h"

namespace pulse::writer {

namespace fmt = pulse::format;
namespace cm = pulse::compile;

namespace {

void Append(std::vector<uint8_t>& buf, const void* data, size_t size) {
    const uint8_t* p = static_cast<const uint8_t*>(data);
    buf.insert(buf.end(), p, p + size);
}

void AppendText(std::vector<uint8_t>& buf, const char* text) {
    Append(buf, text, std::strlen(text));
}

// The reference writes every key with printf and these exact format strings;
// the engine's parser is whitespace-tolerant but tools that diff .phy text are
// not, so the formats are kept identical ("%f" -> 6 decimal places).
void KeyInt(std::vector<uint8_t>& buf, const char* key, int v) {
    char tmp[256];
    std::snprintf(tmp, sizeof(tmp), "\"%s\" \"%d\"\n", key, v);
    AppendText(buf, tmp);
}

void KeyString(std::vector<uint8_t>& buf, const char* key, const char* v) {
    char tmp[512];
    std::snprintf(tmp, sizeof(tmp), "\"%s\" \"%s\"\n", key, v);
    AppendText(buf, tmp);
}

void KeyFloat(std::vector<uint8_t>& buf, const char* key, float v) {
    char tmp[256];
    std::snprintf(tmp, sizeof(tmp), "\"%s\" \"%f\"\n", key, v);
    AppendText(buf, tmp);
}

void KeyIntPair(std::vector<uint8_t>& buf, const char* key, int v0, int v1) {
    char tmp[256];
    std::snprintf(tmp, sizeof(tmp), "\"%s\" \"%d,%d\"\n", key, v0, v1);
    AppendText(buf, tmp);
}

} // namespace

std::vector<uint8_t> BuildPhy(cm::CompiledModel& m, int32_t checksum) {
    std::vector<uint8_t> buf;
    if (m.physSolids.empty()) return buf;

    // ---- header ----
    fmt::phyheader_t header{};
    header.size = sizeof(fmt::phyheader_t);
    header.id = 0;
    header.solidCount = static_cast<int32_t>(m.physSolids.size());
    header.checkSum = checksum;
    Append(buf, &header, sizeof(header));

    // ---- binary collision blobs: int32 size, then the blob ----
    for (const cm::PhysicsSolid& s : m.physSolids) {
        int32_t size = static_cast<int32_t>(s.blob.size());
        Append(buf, &size, sizeof(size));
        Append(buf, s.blob.data(), s.blob.size());
    }

    // ---- solid keyvalue sections ----
    // Mass is distributed by volume share, so it can only be resolved once
    // every solid is built. A zero total would divide by zero on a degenerate
    // model, so it falls back to 1 exactly like the reference.
    float totalVolume = 0.0f;
    for (const cm::PhysicsSolid& s : m.physSolids)
        totalVolume += s.volume * s.massBias;
    if (totalVolume <= 0.0f) totalVolume = 1.0f;

    int solidIndex = 0;
    for (cm::PhysicsSolid& s : m.physSolids) {
        s.mass = ((s.volume * s.massBias) / totalVolume) * m.physTotalMass;
        if (s.mass < 1.0f) s.mass = 1.0f;

        AppendText(buf, "solid {\n");
        KeyInt(buf, "index", solidIndex);
        KeyString(buf, "name", s.name.c_str());
        if (!s.parent.empty())
            KeyString(buf, "parent", s.parent.c_str());
        KeyFloat(buf, "mass", s.mass);
        KeyString(buf, "surfaceprop", s.surfaceprop.c_str());
        KeyFloat(buf, "damping", s.damping);
        KeyFloat(buf, "rotdamping", s.rotdamping);
        if (s.drag != -1.0f)
            KeyFloat(buf, "drag", s.drag);
        KeyFloat(buf, "inertia", s.inertia);
        KeyFloat(buf, "volume", s.volume);
        if (s.massBias != 1.0f)
            KeyFloat(buf, "massbias", s.massBias);
        AppendText(buf, "}\n");
        solidIndex++;
    }

    // ---- ragdoll constraints ----
    // One per body that has a parent. A single body has neither a parent nor a
    // constraint, so this whole block is naturally empty for a prop.
    for (size_t childIndex = 0; childIndex < m.physSolids.size(); childIndex++) {
        const cm::PhysicsSolid& s = m.physSolids[childIndex];
        if (s.parentIndex < 0 || s.parentIndex == static_cast<int>(childIndex))
            continue;

        AppendText(buf, "ragdollconstraint {\n");
        KeyInt(buf, "parent", s.parentIndex);
        KeyInt(buf, "child", static_cast<int>(childIndex));
        static const char* kAxisKeys[3][3] = {
            {"xmin", "xmax", "xfriction"},
            {"ymin", "ymax", "yfriction"},
            {"zmin", "zmax", "zfriction"},
        };
        for (int a = 0; a < 3; a++) {
            KeyFloat(buf, kAxisKeys[a][0], s.axisMin[a]);
            KeyFloat(buf, kAxisKeys[a][1], s.axisMax[a]);
            KeyFloat(buf, kAxisKeys[a][2], s.axisFriction[a]);
        }
        AppendText(buf, "}\n");
    }

    // ---- collision rules ----
    // no_self_collisions wins outright: with self-collision off there is
    // nothing for a pair list to allow.
    if (m.physNoSelfCollisions) {
        AppendText(buf, "collisionrules {\n");
        KeyInt(buf, "selfcollisions", 0);
        AppendText(buf, "}\n");
    } else if (!m.physCollisionPairs.empty()) {
        AppendText(buf, "collisionrules {\n");
        for (const std::pair<int, int>& pair : m.physCollisionPairs)
            KeyIntPair(buf, "collisionpair", pair.first, pair.second);
        AppendText(buf, "}\n");
    }

    // ---- animated friction ----
    if (m.physHasAnimatedFriction) {
        AppendText(buf, "animatedfriction {\n");
        KeyFloat(buf, "animfrictionmin", static_cast<float>(m.physAnimFrictionMin));
        KeyFloat(buf, "animfrictionmax", static_cast<float>(m.physAnimFrictionMax));
        KeyFloat(buf, "animfrictiontimein", m.physAnimFrictionTimeIn);
        KeyFloat(buf, "animfrictiontimeout", m.physAnimFrictionTimeOut);
        KeyFloat(buf, "animfrictiontimehold", m.physAnimFrictionTimeHold);
        AppendText(buf, "}\n");
    }

    // ---- editparams ----
    AppendText(buf, "editparams {\n");
    KeyString(buf, "rootname", m.physRootName.c_str());
    KeyFloat(buf, "totalmass", m.physTotalMass);
    if (m.physConcave)
        KeyInt(buf, "concave", 1);
    for (const std::pair<std::string, std::string>& merge : m.physJointMerges) {
        char tmp[512];
        std::snprintf(tmp, sizeof(tmp), "%s,%s", merge.first.c_str(), merge.second.c_str());
        KeyString(buf, "jointmerge", tmp);
    }
    AppendText(buf, "}\n");

    // The engine parses the text tail until this NUL.
    buf.push_back(0);
    return buf;
}

} // namespace pulse::writer

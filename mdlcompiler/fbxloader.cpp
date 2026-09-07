// fbxloader.cpp - PulseMDL
//
// See fbxloader.h. ufbx does the parsing; this file only maps its scene onto
// Source. The corner unify/sort/mesh build is shared with the SMD path
// (BuildUnifiedMeshes in source.h).

#include "strcompat.h"

#include "fbxloader.h"

#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <string>
#include <unordered_map>
#include <vector>

#include "math/math.h"
#include "pulselimits.h"
#include "ufbx.h"

namespace pm = pulse::math;
namespace lim = pulse::limits;

namespace pulse::source {

namespace {

// ufbx_matrix is column-major (m<row><col> named fields, 4 columns of 3);
// matrix3x4 is row-major float[3][4]. Same contents, transposed storage.
pm::matrix3x4 ToMatrix3x4(const ufbx_matrix& m) {
    pm::matrix3x4 o;
    for (int c = 0; c < 4; ++c)
        for (int r = 0; r < 3; ++r)
            o.m[r][c] = static_cast<float>(m.cols[c].v[r]);
    return o;
}

pm::Vector3 ToVec3(const ufbx_vec3& v) {
    return pm::Vector3{static_cast<float>(v.x), static_cast<float>(v.y), static_cast<float>(v.z)};
}

// DFS from the root so localBone[] is parents-before-children, which the
// compile stage's bind-matrix fold depends on. The FBX root and the empty
// container nodes above the skeleton (an exporter's "root" / "Armature" Null)
// are dropped - they are scene structure, not joints, and the DMX loader skips
// the same container dag. `inSkeleton` keeps that to the chain above the first
// real node: a Null *inside* a rig is a helper joint and stays.
void AddNodes(const ufbx_node* node, int parentIndex, bool inSkeleton, Source& out,
              std::unordered_map<uint32_t, int>& nodeToBone) {
    const bool isContainer = !node->attrib || node->attrib_type == UFBX_ELEMENT_EMPTY;
    int index = parentIndex;
    if (inSkeleton || !isContainer) {
        index = static_cast<int>(out.localBone.size());
        LocalBone lb;
        lb.name = std::string(node->name.data, node->name.length);
        lb.parent = parentIndex;
        // a mesh node is exporter structure, not a joint - same exemption the
        // DMX loader gives mesh dags
        lb.isNonSkeletal = node->mesh != nullptr;
        out.localBone.push_back(lb);
        nodeToBone[node->typed_id] = index;
        inSkeleton = true;
    }
    for (size_t i = 0; i < node->children.count; ++i)
        AddNodes(node->children.data[i], index, inSkeleton, out, nodeToBone);
}

// Rest transforms become the bind pose: an FBX exported in a pose
// other than its bind pose will bind wrong - the fix is ufbx_scene.poses
// (is_bind_pose) per skin cluster, added when a sample actually needs it.
void BuildSkeleton(const ufbx_scene* scene, float scale, Source& out,
                   std::unordered_map<uint32_t, int>& nodeToBone) {
    AddNodes(scene->root_node, -1, false, out, nodeToBone);
    out.numbones = static_cast<int>(out.localBone.size());

    SourceAnim anim;
    anim.name = "BindPose";
    anim.numframes = 1;
    anim.frames.assign(1, std::vector<SrcBonePose>(out.numbones));
    out.boneToPose.resize(out.numbones);

    for (size_t i = 0; i < scene->nodes.count; ++i) {
        const ufbx_node* node = scene->nodes.data[i];
        auto it = nodeToBone.find(node->typed_id);
        if (it == nodeToBone.end())
            continue;
        const int b = it->second;
        const int parent = out.localBone[b].parent;

        // a top bone's FBX parent is a dropped container, so take its world
        // transform - that folds the container's transform in instead of
        // losing it.
        pm::matrix3x4 local =
            ToMatrix3x4(parent == -1 ? node->node_to_world : node->node_to_parent);
        for (int r = 0; r < 3; ++r)
            local.m[r][3] *= scale;
        pm::MatrixAngles(local, anim.frames[0][b].rot, anim.frames[0][b].pos);

        out.boneToPose[b] =
            parent == -1 ? local : pm::ConcatTransforms(out.boneToPose[parent], local);
    }

    out.anims.push_back(std::move(anim));
}

// Per-corner skin weights. An unskinned mesh rides its own node bone.
SrcBoneWeight CornerWeights(const ufbx_mesh* mesh, const ufbx_skin_deformer* skin,
                            const std::vector<int>& clusterToBone, uint32_t index,
                            int meshBone) {
    SrcBoneWeight bw;
    if (skin) {
        const uint32_t vi = mesh->vertex_indices.data[index];
        const ufbx_skin_vertex& sv = skin->vertices.data[vi];
        int bones[lim::kMaxSrcBoneWeights];
        float weights[lim::kMaxSrcBoneWeights];
        int count = 0;
        for (uint32_t w = 0; w < sv.num_weights && count < lim::kMaxSrcBoneWeights; ++w) {
            const ufbx_skin_weight& sw = skin->weights.data[sv.weight_begin + w];
            const int bone = clusterToBone[sw.cluster_index];
            if (bone < 0 || sw.weight <= 0)
                continue;
            bones[count] = bone;
            weights[count] = static_cast<float>(sw.weight);
            ++count;
        }
        if (count > 0) {
            bw.numbones = SortAndBalanceBones(count, lim::kMaxSrcBoneWeights, bones, weights);
            for (int k = 0; k < bw.numbones; ++k) {
                bw.bone[k] = bones[k];
                bw.weight[k] = weights[k];
            }
            return bw;
        }
    }
    bw.numbones = 1;
    bw.bone[0] = meshBone;
    bw.weight[0] = 1.0f;
    return bw;
}

// ---------------------------------------------------------------------------
// Blend shapes -> morphs. One blend channel is one delta state; its offsets are
// per logical vertex, so a delta fans out to every unified vertex that vertex
// became (a UV or material seam splits one source vertex into several).
//
// FBX carries no stereo balance channel, so every vertex takes the DMX loader's
// 1.0 default.
// ---------------------------------------------------------------------------
void LoadMorphs(const std::vector<const ufbx_node*>& meshes, const std::vector<int>& bases,
                const std::vector<std::vector<int>>& srcToUnified, float scale, Source& out) {
    // one morph per channel NAME - the same shape key can live on several
    // meshes, and the compile stage wants a single delta state driving them all
    for (size_t m = 0; m < meshes.size(); ++m) {
        const ufbx_node* node = meshes[m];
        const ufbx_mesh* mesh = node->mesh;
        for (size_t d = 0; d < mesh->blend_deformers.count; ++d) {
            const ufbx_blend_deformer* def = mesh->blend_deformers.data[d];
            for (size_t c = 0; c < def->channels.count; ++c) {
                const ufbx_blend_channel* chan = def->channels.data[c];
                const ufbx_blend_shape* shape = chan->target_shape;
                if (!shape || shape->num_offsets == 0)
                    continue;
                const std::string name = std::string(chan->name.data, chan->name.length);

                SrcMorphAnim* morph = nullptr;
                for (SrcMorphAnim& existing : out.morphs)
                    if (existing.name == name) { morph = &existing; break; }
                if (!morph) {
                    out.morphs.emplace_back();
                    morph = &out.morphs.back();
                    morph->name = name;
                }

                for (size_t k = 0; k < shape->num_offsets; ++k) {
                    const uint32_t v = shape->offset_vertices.data[k];
                    if (v >= mesh->num_vertices)
                        continue;
                    const int srcVertex = bases[m] + static_cast<int>(v);
                    if (srcVertex >= static_cast<int>(srcToUnified.size()))
                        continue;

                    // offsets are geometry-space directions, placed the same
                    // way the positions they displace were
                    pm::Vector3 pos = ToVec3(ufbx_transform_direction(
                        &node->geometry_to_world, shape->position_offsets.data[k]));
                    pos.x *= scale; pos.y *= scale; pos.z *= scale;
                    // Blender sizes normal_offsets to match but fills it with
                    // zeros, so in practice these arrive empty; an exporter
                    // that does write them gets them through to ndelta.
                    pm::Vector3 nrm;
                    if (k < shape->normal_offsets.count)
                        nrm = ToVec3(ufbx_transform_direction(&node->geometry_to_world,
                                                              shape->normal_offsets.data[k]));
                    // Blender writes an offset entry for every vertex in the
                    // shape's group, noise included, where a DMX delta state is
                    // sparse. Same thresholds the DMX exporter applies: a
                    // position delta counts above 1e-5, a normal delta above a
                    // 1-dot of 1e-3, which for unit normals is |d|^2 >= 2e-3.
                    const float posLen2 = pos.x * pos.x + pos.y * pos.y + pos.z * pos.z;
                    const float nrmLen2 = nrm.x * nrm.x + nrm.y * nrm.y + nrm.z * nrm.z;
                    if (posLen2 <= 1e-10f && nrmLen2 < 2e-3f)
                        continue;

                    for (int unified : srcToUnified[srcVertex]) {
                        SrcVertAnim va;
                        va.vertex = unified;
                        va.speed = 1.0f; // FBX carries no per-vertex morph speed/balance
                        va.side = 1.0f;
                        va.pos = pos;
                        va.normal = nrm;
                        morph->vanims.push_back(va);
                    }
                }
            }
        }
    }

    // AddFlexKeys: one flexkey per delta state.
    for (const SrcMorphAnim& morph : out.morphs) {
        bool seen = false;
        for (const SrcFlexKey& key : out.flexkeys)
            if (_stricmp(key.name.c_str(), morph.name.c_str()) == 0) { seen = true; break; }
        if (!seen) {
            SrcFlexKey key;
            key.name = morph.name;
            out.flexkeys.push_back(std::move(key));
        }
    }
}

} // namespace

bool LoadFbxSource(const std::string& path, Source& out, MaterialTable& mats, float scale,
                   std::string* err, bool morphSource, MeshFilter* filter, bool animOnly) {
    ufbx_load_opts opts = {};
    // Source is right-handed Z-up; ADJUST_TRANSFORMS bakes the conversion into
    // each node so it survives dropping the root node.
    opts.target_axes = ufbx_axes_right_handed_z_up;
    opts.space_conversion = UFBX_SPACE_CONVERSION_ADJUST_TRANSFORMS;
    // FBX units are the author's problem ($scale / $upaxis), same as DMX.
    opts.target_unit_meters = 0.0f;
    // PRESERVE, not MODIFY_GEOMETRY: baking the geometry transform into the
    // mesh does NOT bake it into the blend shape offsets, so deltas would come
    // out short of the transform the base vertices already took. Everything
    // here goes through geometry_to_world, which carries it either way - and
    // PRESERVE also never injects the helper nodes AddNodes would see as bones.
    opts.geometry_transform_handling = UFBX_GEOMETRY_TRANSFORM_HANDLING_PRESERVE;
    opts.generate_missing_normals = true;
    opts.ignore_animation = true;
    opts.ignore_embedded = true;

    ufbx_error error;
    ufbx_scene* scene = ufbx_load_file_len(path.c_str(), path.size(), &opts, &error);
    if (!scene) {
        if (err) {
            char buf[1024];
            ufbx_format_error(buf, sizeof(buf), &error);
            *err = buf;
        }
        return false;
    }

    std::unordered_map<uint32_t, int> nodeToBone;
    BuildSkeleton(scene, scale, out, nodeToBone);
    if (out.numbones <= 0) {
        if (err) *err = "the scene has no nodes";
        ufbx_free_scene(scene);
        return false;
    }

    if (animOnly) {
        ufbx_free_scene(scene);
        return true;
    }

    std::vector<TriInput> tris;
    std::vector<uint32_t> triIndices;
    // every mesh's logical vertices laid end to end, so one id space covers the
    // whole source. Morph deltas are looked up in it.
    std::vector<const ufbx_node*> morphMeshes;
    std::vector<int> morphBase;
    int srcVertexCount = 0;

    for (size_t i = 0; i < scene->nodes.count; ++i) {
        const ufbx_node* node = scene->nodes.data[i];
        const ufbx_mesh* mesh = node->mesh;
        if (!mesh)
            continue;
        // a rejected mesh skips its geometry only - the node is already a bone
        if (filter && !filter->Keep(std::string(mesh->name.data, mesh->name.length),
                                    std::string(node->name.data, node->name.length)))
            continue;
        const int meshBone = nodeToBone[node->typed_id];
        const int vertexBase = srcVertexCount;
        srcVertexCount += static_cast<int>(mesh->num_vertices);
        morphMeshes.push_back(node);
        morphBase.push_back(vertexBase);

        // one deformer only - stacked skins are a rig authoring mistake here
        const ufbx_skin_deformer* skin =
            mesh->skin_deformers.count ? mesh->skin_deformers.data[0] : nullptr;
        std::vector<int> clusterToBone;
        if (skin) {
            clusterToBone.reserve(skin->clusters.count);
            for (size_t c = 0; c < skin->clusters.count; ++c) {
                const ufbx_node* bn = skin->clusters.data[c]->bone_node;
                auto it = bn ? nodeToBone.find(bn->typed_id) : nodeToBone.end();
                clusterToBone.push_back(it == nodeToBone.end() ? -1 : it->second);
            }
        }

        // per-instance materials win over the mesh's own list (ufbx note)
        const ufbx_material_list& matList =
            node->materials.count ? node->materials : mesh->materials;
        std::vector<int> materialIndex(matList.count, 0);
        std::vector<char> materialRemoved(matList.count, 0);
        for (size_t m = 0; m < matList.count; ++m) {
            const ufbx_string& name = matList.data[m]->name;
            std::string nm(name.data, name.length);
            // $removemeshword: leave the slot out of the table; faces referencing
            // it are dropped below, so its verts never unify.
            if (filter && filter->MaterialRemoved(nm)) {
                materialRemoved[m] = 1;
                continue;
            }
            // Relative-path flagged like DMX: no derived "models/<outname>/",
            // the empty cdmaterials entry resolves the name as authored.
            materialIndex[m] = mats.UseTextureAsMaterial(mats.LookupTexture(nm.c_str()));
        }

        triIndices.resize(mesh->max_face_triangles * 3);
        for (size_t f = 0; f < mesh->faces.count; ++f) {
            const ufbx_face face = mesh->faces.data[f];
            int material = 0;
            bool removed = false;
            if (mesh->face_material.count && !materialIndex.empty()) {
                const uint32_t mi = mesh->face_material.data[f];
                if (mi < materialIndex.size()) {
                    material = materialIndex[mi];
                    removed = materialRemoved[mi];
                }
            } else if (!materialIndex.empty()) {
                material = materialIndex[0];
                removed = materialRemoved[0];
            }
            if (removed)
                continue;

            const uint32_t numTris =
                ufbx_triangulate_face(triIndices.data(), triIndices.size(), mesh, face);
            for (uint32_t t = 0; t < numTris; ++t) {
                TriInput tri;
                tri.material = material;
                for (int j = 0; j < 3; ++j) {
                    const uint32_t index = triIndices[t * 3 + j];
                    TriCorner& corner = tri.v[j];
                    // Source vertices are model space; bones carry the same
                    // hierarchy, so both come from the node's world transform.
                    corner.position = ToVec3(ufbx_transform_position(
                        &node->geometry_to_world,
                        ufbx_get_vertex_vec3(&mesh->vertex_position, index)));
                    corner.position.x *= scale;
                    corner.position.y *= scale;
                    corner.position.z *= scale;
                    if (mesh->vertex_normal.exists) {
                        corner.normal = ToVec3(ufbx_transform_direction(
                            &node->geometry_to_world,
                            ufbx_get_vertex_vec3(&mesh->vertex_normal, index)));
                        pm::VectorNormalize(corner.normal);
                    }
                    if (mesh->vertex_uv.exists) {
                        const ufbx_vec2 uv = ufbx_get_vertex_vec2(&mesh->vertex_uv, index);
                        corner.texcoord = pm::Vector2{static_cast<float>(uv.x),
                                                      1.0f - static_cast<float>(uv.y)};
                    }
                    corner.boneweight =
                        CornerWeights(mesh, skin, clusterToBone, index, meshBone);
                    if (morphSource)
                        corner.srcVertex =
                            vertexBase + static_cast<int>(mesh->vertex_indices.data[index]);
                }
                tris.push_back(tri);
            }
        }
    }

    std::vector<std::vector<int>> srcToUnified;
    BuildUnifiedMeshes(tris, out, morphSource ? &srcToUnified : nullptr);
    if (morphSource)
        LoadMorphs(morphMeshes, morphBase, srcToUnified, scale, out);

    ufbx_free_scene(scene);
    return true;
}

} // namespace pulse::source

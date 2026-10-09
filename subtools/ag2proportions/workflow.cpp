// workflow.cpp - one job end to end: locate the addon, read the stock assets,
// build the poses and the wrapper, write the outputs, then compile.

#include <algorithm>
#include <cmath>
#include <cstdio>
#include <functional>
#include <iostream>
#include <variant>
#include <stdexcept>

#include "ag2.h"
#include "strcompat.h"
#include "vpk.h"

namespace ag2 {
namespace {

namespace dmx = pulse::dmx;
using std::runtime_error;

// Every file a run writes into the output folder; a rerun clears these first.
const char* const kGeneratedFiles[] = {
    "proportions_worldmodel.vnmgraph", "proportions_uimodel.vnmgraph", "proportions.vnmclip", "proportions.dmx", "reference.dmx",
    "model_with_helpers.dmx", "debug/rig_merged.dmx", "debug/skeleton_descriptor.kv3",
    "debug/report.kv3", "debug/MODEL_SETUP.txt",
};


bool Disabled(const kv::Value& node) {
    if (!node.Has("disabled"))
        return false;
    const kv::Value& flag = node.At("disabled");
    return std::holds_alternative<bool>(flag.data) && flag.Boolean();
}

// Visits every enabled node; a node ModelDoc has disabled is skipped with its children.
void Walk(kv::Value& value, const std::function<void(kv::Value&)>& visit) {
    if (value.IsObject()) {
        if (Disabled(value))
            return;
        visit(value);
        for (auto& member : value.Members())
            Walk(member.second, visit);
    } else if (value.IsArray()) {
        for (auto& item : value.Items())
            Walk(item, visit);
    }
}

std::string ClassOf(const kv::Value& node) {
    return node.Has("_class") ? node.At("_class").String() : std::string();
}

std::string ReadText(const fs::path& path) {
    const auto bytes = kv::ReadFile(path.u8string());
    return {bytes.begin(), bytes.end()};
}

// The VMDL must sit in <cs2>/content/csgo_addons/<addon>/; the output folder is
// a sibling of it named after it.
Paths Locate(const fs::path& vmdl) {
    Paths p;
    for (fs::path dir = vmdl.parent_path(); dir.has_relative_path(); dir = dir.parent_path()) {
        if (dir.parent_path().filename() == "csgo_addons" &&
            dir.parent_path().parent_path().filename() == "content") {
            p.addon = dir;
            break;
        }
    }
    if (p.addon.empty())
        throw runtime_error("the VMDL is not inside content/csgo_addons/<addon>");
    p.cs2 = p.addon.parent_path().parent_path().parent_path();
    if (!fs::exists(p.cs2 / "game/csgo/pak01_dir.vpk"))
        throw runtime_error("no CS2 install above the addon (game/csgo/pak01_dir.vpk is missing)");
    p.output = vmdl.parent_path() / vmdl.stem();
    p.relative = p.output.lexically_relative(p.addon).generic_u8string();
    return p;
}

// A ModelDoc path is addon-relative; a model-relative one is accepted too.
fs::path ResolveImport(const std::string& name, const Paths& paths, const fs::path& vmdl) {
    const fs::path file = fs::u8path(name);
    for (const fs::path& candidate : {paths.addon / file, vmdl.parent_path() / file})
        if (fs::is_regular_file(candidate))
            return fs::weakly_canonical(candidate);
    throw runtime_error("cannot find VMDL import " + name);
}

struct SkeletonSource {
    fs::path path;
    bool overwrite = false;
};

// The enabled SkeletonFiles in VMDL order; ModelDoc merges them in that order.
std::vector<SkeletonSource> SkeletonImports(kv::Value& vmdl, const Paths& paths, const fs::path& vmdlPath) {
    std::vector<SkeletonSource> sources;
    Walk(vmdl, [&](kv::Value& node) {
        if (ClassOf(node) != "SkeletonFile" || !node.Has("filename"))
            return;
        SkeletonSource source;
        source.path = ResolveImport(node.At("filename").String(), paths, vmdlPath);
        source.overwrite = node.Has("merge_behavior") && node.At("merge_behavior").String() == "overwrite_existing";
        sources.push_back(source);
    });
    return sources;
}

// Without a SkeletonFile, ModelDoc takes the bones from the render meshes: use
// the one carrying the most CS2 core bones (the body). Empty if none has any.
fs::path RenderMeshRig(kv::Value& vmdl, const Paths& paths, const fs::path& vmdlPath) {
    std::vector<std::string> imports;
    Walk(vmdl, [&](kv::Value& node) {
        if (ClassOf(node) == "RenderMeshFile" && node.Has("filename") &&
            fs::u8path(node.At("filename").String()).extension() == ".dmx")
            imports.push_back(node.At("filename").String());
    });
    fs::path best;
    size_t bestCount = 0;
    for (const std::string& name : imports) {
        const fs::path file = ResolveImport(name, paths, vmdlPath);
        std::string err;
        auto dm = dmx::Datamodel::Load(file.u8string(), &err);
        const size_t count = dm ? CountBones(*dm) : 0;
        if (count > bestCount) {
            best = file;
            bestCount = count;
        }
    }
    return best;
}

pm::Vector3 Triple(const kv::Value& v, const std::string& bone) {
    const auto& items = v.Items();
    if (items.size() != 3)
        throw runtime_error("Bone node " + bone + " needs three origin/angles values");
    return {static_cast<float>(items[0].Number()), static_cast<float>(items[1].Number()),
            static_cast<float>(items[2].Number())};
}

// ModelDoc Bone nodes under Skeleton, parent-first. origin is parent-local and
// angles a QAngle in degrees (pitch, yaw, roll); nesting gives the parent.
std::vector<Bone> BoneNodes(const kv::Value& vmdl) {
    std::vector<Bone> bones;
    std::function<void(const kv::Value&, int)> visit = [&](const kv::Value& node, int parent) {
        if (!node.IsObject() || Disabled(node))
            return;
        if (ClassOf(node) == "Bone") {
            Bone b;
            b.name = node.At("name").String();
            b.parent = parent;
            b.position = node.Has("origin") ? Triple(node.At("origin"), b.name) : pm::Vector3{};
            const pm::Vector3 a = node.Has("angles") ? Triple(node.At("angles"), b.name) : pm::Vector3{};
            pm::AngleQuaternion(pm::RadianEuler{a.z * pm::kDeg2Rad, a.x * pm::kDeg2Rad, a.y * pm::kDeg2Rad}, b.rotation);
            parent = static_cast<int>(bones.size());
            bones.push_back(b);
        }
        if (node.Has("children") && node.At("children").IsArray())
            for (const kv::Value& child : node.At("children").Items())
                visit(child, parent);
    };
    std::function<void(const kv::Value&)> findSkeletons = [&](const kv::Value& node) {
        if (node.IsObject()) {
            if (Disabled(node))
                return;
            if (ClassOf(node) == "Skeleton") {
                visit(node, -1);
                return;
            }
            for (const auto& member : node.Members())
                findSkeletons(member.second);
        } else if (node.IsArray()) {
            for (const kv::Value& item : node.Items())
                findSkeletons(item);
        }
    };
    findSkeletons(vmdl);
    return bones;
}

// A stock helper as a ModelDoc Bone node, for a Bone-node skeleton.
kv::Value HelperNode(const Bone& bone) {
    pm::RadianEuler e;
    pm::QuaternionAngles(bone.rotation, e);
    return kv::Value::Object{
        {"_class", "Bone"},
        {"name", bone.name},
        {"origin", kv::Value::Array{double(bone.position.x), double(bone.position.y), double(bone.position.z)}},
        {"angles", kv::Value::Array{double(e.y * pm::kRad2Deg), double(e.z * pm::kRad2Deg), double(e.x * pm::kRad2Deg)}},
        {"do_not_discard", true},
    };
}

std::unique_ptr<dmx::Datamodel> LoadDmx(const fs::path& path) {
    std::string err;
    auto dm = dmx::Datamodel::Load(path.u8string(), &err);
    if (!dm)
        throw runtime_error("cannot load " + path.u8string() + ": " + err);
    return dm;
}

// Appends `helper` under the Bone node named after its stock parent.
void InsertHelperNode(kv::Value& model, const Bone& helper, const std::string& parentName) {
    bool placed = false;
    if (parentName.empty()) {
        Walk(model, [&](kv::Value& node) {
            if (placed || ClassOf(node) != "Skeleton")
                return;
            if (!node.Has("children"))
                node["children"] = kv::Value::Array{};
            node["children"].Items().push_back(HelperNode(helper));
            placed = true;
        });
        if (!placed)
            model["rootNode"]["children"].Items().push_back(kv::Value::Object{
                {"_class", "Skeleton"}, {"children", kv::Value::Array{HelperNode(helper)}}});
        return;
    }
    Walk(model, [&](kv::Value& node) {
        if (placed || ClassOf(node) != "Bone" || !node.Has("name") ||
            _stricmp(node.At("name").String().c_str(), parentName.c_str()) != 0)
            return;
        if (!node.Has("children"))
            node["children"] = kv::Value::Array{};
        node["children"].Items().push_back(HelperNode(helper));
        placed = true;
    });
    if (!placed)
        throw runtime_error("no Bone node " + parentName + " to hold weapon helper " + helper.name);
}

// write_model: the VMDL is edited in place (worldmodel graph bindings, the rig
// import to the helper DMX, node helpers as Bone nodes). The first edit keeps
// the untouched file as <name>.vmdl.bak, which ModelDoc never compiles.
void EditModel(const std::string& originalText, kv::Value model, const Job& job, const Paths& paths,
               const fs::path& modelDmx, bool dmxHelpers,
               const std::vector<Bone>& stock, const std::vector<Bone>& retainedBones) {
    for (const Bone& binding : retainedBones) {
        bool found = false;
        Walk(model, [&](kv::Value& node) {
            if (ClassOf(node) == "Bone" && node.Has("name") &&
                _stricmp(node.At("name").String().c_str(), binding.name.c_str()) == 0) {
                node["do_not_discard"] = true;
                found = true;
            }
        });
        if (!found)
            InsertHelperNode(model, binding, binding.parent >= 0 ? stock[binding.parent].name : std::string());
    }
    const std::string graph = paths.relative + "/proportions_worldmodel.vnmgraph";
    const std::string uiGraph = paths.relative + "/proportions_uimodel.vnmgraph";
    kv::Value* list = nullptr;
    Walk(model, [&](kv::Value& node) {
        if (ClassOf(node) == "AnimGraph2List")
            list = &node;
    });
    if (!list) {
        auto& children = model["rootNode"]["children"].Items();
        children.push_back(kv::Value::Object{{"_class", "AnimGraph2List"}, {"children", kv::Value::Array{}}});
        list = &children.back();
    }
    // The game plays the entries named worldmodel and uimodel (the UI preview).
    bool haveWorld = false, haveUi = false;
    for (auto& node : (*list)["children"].Items()) {
        const std::string cls = ClassOf(node);
        if (cls == "AnimGraph2" && node.Has("name") && node.At("name").String() == "worldmodel") {
            node["filename"] = graph;
            haveWorld = true;
        } else if (cls == "AnimGraph2" && node.Has("name") && node.At("name").String() == "uimodel") {
            node["filename"] = uiGraph;
            haveUi = true;
        }
    }
    auto& graphs = (*list)["children"].Items();
    graphs.erase(std::remove_if(graphs.begin(), graphs.end(), [&](const kv::Value& node) {
        return ClassOf(node) == "DefaultAnimGraph2" && node.Has("filename") && node.At("filename").String() == graph;
    }), graphs.end());
    if (!haveWorld)
        (*list)["children"].Items().push_back(
            kv::Value::Object{{"_class", "AnimGraph2"}, {"name", "worldmodel"}, {"filename", graph}});
    if (!haveUi)
        (*list)["children"].Items().push_back(
            kv::Value::Object{{"_class", "AnimGraph2"}, {"name", "uimodel"}, {"filename", uiGraph}});


    Walk(model, [&](kv::Value& node) {
        if (!node.Has("filename"))
            return;
        const std::string name = node.At("filename").String();
        const fs::path file = fs::u8path(name);
        if (file.extension() == ".dmx" && dmxHelpers && fs::equivalent(ResolveImport(name, paths, job.vmdl), modelDmx))
            node["filename"] = paths.relative + "/model_with_helpers.dmx";
    });

    const size_t header = originalText.find("-->");
    if (header == std::string::npos)
        throw runtime_error("the VMDL has no KV3 header");
    fs::path backup = job.vmdl;
    backup += ".bak";
    if (!fs::exists(backup))
        kv::WriteFile(backup.u8string(), originalText);
    kv::WriteFile(job.vmdl.u8string(), originalText.substr(0, header + 3) + "\n" + kv::TextBody(model) + "\n");
}

std::string SetupNotes(const Paths& paths, const std::vector<Bone>& dmxHelpers, const std::vector<Bone>& nodeHelpers,
                       const std::vector<Bone>& stock, const std::vector<std::string>& warnings, bool wroteModel) {
    std::string note = "AG2 PROPORTIONS (experimental)\n\n"
                       "Set the worldmodel AnimGraph2 to:\n  " +
                       paths.relative + "/proportions_worldmodel.vnmgraph\n\n";
    if (!dmxHelpers.empty()) {
        note += "Missing weapon helpers were added to model_with_helpers.dmx. Point the matching\n"
                "SkeletonFile and RenderMeshFile imports at:\n  " + paths.relative + "/model_with_helpers.dmx\n"
                "or author these bones yourself to fine-tune them:\n";
        for (const Bone& h : dmxHelpers)
            note += "  " + h.name + " (parent " + stock[h.parent].name + ")\n";
        note += "\n";
    }
    if (!nodeHelpers.empty()) {
        note += "Missing weapon helpers whose parent is a VMDL Bone node; add them as Bone nodes\n"
                "under that parent (write_model does this):\n";
        auto inline3 = [](const kv::Value& v) {
            char b[96];
            const auto& i = v.Items();
            std::snprintf(b, sizeof b, "[ %.6g %.6g %.6g ]", i[0].Number(), i[1].Number(), i[2].Number());
            return std::string(b);
        };
        for (const Bone& h : nodeHelpers) {
            const kv::Value node = HelperNode(h);
            note += "  " + h.name + " under " + stock[h.parent].name + ": origin " + inline3(node.At("origin")) +
                    " angles " + inline3(node.At("angles")) + "\n";
        }
        note += "\n";
    }
    if (wroteModel)
        note += "The input VMDL was edited in place; its first backup is <model>.vmdl.bak.\n\n";
    note += "Keep root_motion -> wpnPivot -> wpn as Bone nodes with do_not_discard enabled.\n"
            "write_model retains this chain using the model's existing bind transforms.\n\n";
    note += "Attachment starting points (keep any you already tuned):\n"
            "  weapon         parent wpn     origin 0 0 0         angles 0 0 0\n"
            "  weapon_hand_r  parent hand_R  origin -2.6 -1.4 0   angles 0 180 0\n"
            "  weapon_hand_l  parent hand_L  origin 2.6 1.4 0     angles 0 0 180\n\n"
            "Keep viewmodel/UI graph entries. The clip changes translations only; it does not\n"
            "fix bind axes or re-rig the mesh. Compile the VMDL in ModelDoc, then test pistol,\n"
            "rifle, knife, reload, crouch, aim pitch, movement and first-person arms.\n\n";
    for (const std::string& warning : warnings)
        note += "WARNING: " + warning + "\n";
    return note;
}

kv::Value Report(const Job& job, const std::string& rig, const std::string& pose, const Paths& paths,
                 const std::vector<Bone>& stock, const std::vector<Bone>& target) {
    auto triple = [](const pm::Vector3& v) {
        return kv::Value::Array{double(v.x), double(v.y), double(v.z)};
    };
    kv::Value::Array bones;
    for (size_t i = 0; i < stock.size(); ++i)
        bones.push_back(kv::Value::Object{{"bone", stock[i].name},
                                          {"reference", triple(stock[i].position)},
                                          {"target", triple(target[i].position)}});
    return kv::Value::Object{
        {"model", job.vmdl.generic_u8string()},
        {"rig", rig},
        {"pose", pose},
        {"graph", paths.relative + "/proportions_worldmodel.vnmgraph"},
        {"boneTranslations", bones},
        {"compiled", false},
    };
}

bool IsBindRotationNote(const std::string& warning) { return warning.rfind("bind rotation differs", 0) == 0; }
bool IsInfoNote(const std::string& warning) { return warning.rfind("standalone pelvis", 0) == 0; }

// What the user has to do next, on the console; the full notes are --debug only.
void PrintSummary(const Paths& paths, const std::vector<Bone>& dmxHelpers, const std::vector<Bone>& nodeHelpers,
                  const std::vector<Bone>& stock, const std::vector<std::string>& warnings, const Job& job) {
    std::cout << "Set the worldmodel AnimGraph2 to " << paths.relative << "/proportions_worldmodel.vnmgraph\n"
              << "Set the uimodel AnimGraph2 to " << paths.relative << "/proportions_uimodel.vnmgraph\n";
    if (job.writeModel)
        std::cout << "Edited " << job.vmdl.filename().u8string() << " (original kept as "
                  << job.vmdl.filename().u8string() << ".bak)\n";
    if (!dmxHelpers.empty() && !job.writeModel)
        std::cout << "Weapon helpers added to model_with_helpers.dmx; point the SkeletonFile import at it\n";
    if (!nodeHelpers.empty() && !job.writeModel) {
        std::cout << "Add these Bone nodes:\n";
        for (const Bone& h : nodeHelpers) {
            const kv::Value node = HelperNode(h);
            const auto& o = node.At("origin").Items();
            const auto& a = node.At("angles").Items();
            std::printf("  %s under %s: origin [ %g %g %g ] angles [ %g %g %g ]\n", h.name.c_str(),
                        stock[h.parent].name.c_str(), o[0].Number(), o[1].Number(), o[2].Number(), a[0].Number(),
                        a[1].Number(), a[2].Number());
        }
    }
    const auto rotations = std::count_if(warnings.begin(), warnings.end(), IsBindRotationNote);
    for (const std::string& warning : warnings)
        if (!IsBindRotationNote(warning))
            std::cout << (IsInfoNote(warning) ? "NOTE: " : "WARNING: ") << warning << '\n';
    if (rotations)
        std::cout << "NOTE: " << rotations << " bones have a bind rotation unlike stock; the clip moves "
                  << "translations only (--debug lists them)\n";
}

} // namespace

void Generate(const Job& job, const Options& options) {
    std::cout << "Processing " << job.vmdl.u8string() << '\n';
    const Paths paths = Locate(job.vmdl);
    const std::string vmdlText = ReadText(job.vmdl);
    kv::Value vmdl = kv::ParseText(vmdlText);
    // rig files: model_dmx (in place of every SkeletonFile), else the enabled
    // SkeletonFiles, else the body render mesh
    std::vector<SkeletonSource> sources;
    if (!job.modelDmx.empty())
        sources.push_back({job.modelDmx, false});
    else
        sources = SkeletonImports(vmdl, paths, job.vmdl);
    bool meshRig = false;
    if (sources.empty()) {
        const fs::path mesh = RenderMeshRig(vmdl, paths, job.vmdl);
        if (!mesh.empty())
            sources.push_back({mesh, false});
        meshRig = !mesh.empty();
    }
    const std::vector<Bone> nodes = BoneNodes(vmdl);
    if (sources.empty() && nodes.empty())
        throw runtime_error("the VMDL has no skeleton (no SkeletonFile, render mesh bones or Bone nodes); set model_dmx in the job");

    const Vpk package(paths.cs2 / "game/csgo/pak01_dir.vpk");
    std::vector<uint8_t> skeletonBytes = package.Read(std::string(kStockSkeleton) + "_c");
    const StockSkeleton installed = ReadStockSkeleton(skeletonBytes);
    if (!job.vnmskel.empty()) {
        skeletonBytes = kv::ReadFile(job.vnmskel.u8string());
        RequireSameSkeleton(installed, ReadStockSkeleton(skeletonBytes));
    }
    const std::string clip = paths.relative + "/proportions.vnmclip";
    const std::vector<uint8_t> graphBytes = package.Read(std::string(kStockGraph) + "_c");
    const std::vector<uint8_t> uiGraphBytes = package.Read(std::string(kStockUiGraph) + "_c");
    const kv::Value graph = WrapperGraph(kv::ReadResource(graphBytes), kStockGraph, clip);
    const kv::Value uiGraph = WrapperGraph(kv::ReadResource(uiGraphBytes), kStockUiGraph, clip);

    // The rig as ModelDoc builds it from the Bone nodes and the files.
    std::vector<std::string> warnings;
    std::vector<NodeOverride> overrides;
    std::vector<std::unique_ptr<dmx::Datamodel>> loaded;
    std::vector<RigFile> files;
    for (const SkeletonSource& source : sources) {
        loaded.push_back(LoadDmx(source.path));
        files.push_back({loaded.back().get(), source.overwrite});
    }
    const auto rig = BuildRig(nodes, files, &overrides);
    std::vector<Bone> target = TargetPose(*rig, installed.bones, warnings, true);
    std::vector<uint8_t> mergedRig; // for debug/, before any helper is added
    std::string err;
    if (!dmx::Save(*rig, mergedRig, &err))
        throw runtime_error(err);
    if (!job.proportions.empty())
        target = TargetPose(*LoadDmx(job.proportions), installed.bones, warnings, false);
    // Weapon attachments are the user's to author and tune; only their bones are checked.
    struct WeaponAttachment { const char *name, *parent; };
    const WeaponAttachment kAttachments[] = {{"weapon", "wpn"}, {"weapon_hand_r", "hand_R"}, {"weapon_hand_l", "hand_L"}};
    for (const WeaponAttachment& want : kAttachments) {
        std::string parent;
        bool found = false;
        Walk(vmdl, [&](kv::Value& node) {
            if (ClassOf(node) == "Attachment" && node.Has("name") && node.At("name").String() == want.name) {
                found = true;
                parent = node.Has("parent_bone") ? node.At("parent_bone").String() : std::string();
            }
        });
        if (!found)
            warnings.push_back(std::string("no ") + want.name + " attachment; add one on " + want.parent +
                               " or the weapon will not follow the hands");
        else if (_stricmp(parent.c_str(), want.parent) != 0)
            warnings.push_back(std::string("attachment ") + want.name + " is on " + (parent.empty() ? "no bone" : parent) +
                               "; Valve's agents put it on " + want.parent);
    }

    // ModelDoc refuses a primary root bone that the merge left under a parent
    Walk(vmdl, [&](kv::Value& node) {
        if (ClassOf(node) != "BoneMarkupList" || !node.Has("primary_root_bone"))
            return;
        const std::string root = node.At("primary_root_bone").String();
        const std::string parent = root.empty() ? std::string() : BoneParent(*rig, root);
        if (!parent.empty())
            warnings.push_back("BoneMarkupList primary_root_bone \"" + root + "\" sits under " + parent +
                               ", so ModelDoc will refuse to compile; remove it or name a root bone");
    });
    std::vector<std::string> overridden;
    for (const NodeOverride& o : overrides) {
        if (!IsCoreBone(o.name))
            continue;
        overridden.push_back(o.name);
        if (o.moved > 1e-3f) {
            warnings.push_back("VMDL Bone node " + o.name + " differs from the SkeletonFile's by " +
                               std::to_string(o.moved) + " units and wins; its proportions come from the node");
        } else {
            warnings.push_back("VMDL Bone node nests " + o.name + " under a different parent than the " +
                               "SkeletonFile; its file transform is converted into the retained parent's space");
        }
    }

    // Missing helpers go where root_motion lives: into an untouched copy of the
    // first SkeletonFile that has it, or as Bone nodes. ModelDoc culls a render
    // mesh's unweighted bones, so with a render-mesh rig only Bone nodes count.
    std::vector<std::string> nodeNames;
    for (const Bone& node : nodes)
        nodeNames.push_back(node.name);
    const std::vector<std::string> present = BoneNames(*rig);
    fs::path modelDmx; // the file the helper copy is made from
    std::unique_ptr<dmx::Datamodel> helperCopy;
    std::vector<Bone> dmxHelpers, nodeHelpers;
    if (meshRig &&
        std::none_of(nodes.begin(), nodes.end(), [](const Bone& b) { return b.name == "root_motion"; }))
        throw runtime_error("the skeleton comes from a render mesh, whose unweighted bones ModelDoc culls; "
                            "add root_motion as a VMDL Bone node");
    for (size_t i = 0; i < sources.size() && !meshRig && modelDmx.empty(); ++i) {
        const std::vector<std::string> names = BoneNames(*loaded[i]);
        if (std::find(names.begin(), names.end(), "root_motion") != names.end())
            modelDmx = sources[i].path;
    }
    if (!modelDmx.empty()) {
        helperCopy = LoadDmx(modelDmx);
        HelperPlan plan = AddWeaponHelpers(*helperCopy, installed.bones, present, nodeNames);
        dmxHelpers = std::move(plan.inRig);
        nodeHelpers = std::move(plan.asNodes);
    } else {
        nodeHelpers = AddWeaponHelpers(*BuildRig(nodes, {}), installed.bones, present, nodeNames).inRig;
    }
    std::sort(warnings.begin(), warnings.end());
    warnings.erase(std::unique(warnings.begin(), warnings.end()), warnings.end());

    // A target equal to stock compiles to a clip that changes nothing.
    float change = 0;
    for (size_t i = 0; i < target.size(); ++i)
        change = std::max(change, std::abs(target[i].position.x - installed.bones[i].position.x) +
                                      std::abs(target[i].position.y - installed.bones[i].position.y) +
                                      std::abs(target[i].position.z - installed.bones[i].position.z));
    if (change < 1e-3f) {
        std::string zero = "the target pose equals the stock skeleton, so the clip changes nothing";
        if (!overridden.empty())
            zero += "; VMDL Bone nodes replace your SkeletonFile's core bones (keep nodes for bones the file lacks)";
        warnings.insert(warnings.begin(), zero);
    }

    fs::create_directories(paths.output);
    // a job may read a previous run's helper copy; never delete an input
    auto clear = [&](const fs::path& file) {
        std::vector<fs::path> inputs{job.proportions, job.vnmskel};
        for (const SkeletonSource& source : sources)
            inputs.push_back(source.path);
        for (const fs::path& input : inputs)
            if (!input.empty() && fs::exists(file) && fs::equivalent(file, input))
                return;
        fs::remove(file);
    };
    for (const char* name : kGeneratedFiles)
        clear(paths.output / name);
    std::error_code ec;
    fs::remove(paths.output / "debug", ec); // only when empty
    std::cout << "Output: " << paths.output.u8string() << '\n';

    WriteHeldPose(paths.output / "reference.dmx", installed.bones, installed.lowLodCount);
    WriteHeldPose(paths.output / "proportions.dmx", target, installed.lowLodCount);
    const kv::Value descriptor = SkeletonDescriptor(ReadStockSkeleton(skeletonBytes), paths.relative + "/reference.dmx");
    if (!dmxHelpers.empty()) {
        if (!dmx::Save(*helperCopy, (paths.output / "model_with_helpers.dmx").u8string(), &err))
            throw runtime_error(err);
    }
    kv::WriteFile((paths.output / "proportions.vnmclip").u8string(), kv::WriteText(ProportionClip(paths.relative)));
    kv::WriteFile((paths.output / "proportions_worldmodel.vnmgraph").u8string(), kv::WriteText(graph));
    kv::WriteFile((paths.output / "proportions_uimodel.vnmgraph").u8string(), kv::WriteText(uiGraph));
    if (job.writeModel) {
        AddWeaponHelpers(*rig, installed.bones, {}, {});
        EditModel(vmdlText, vmdl, job, paths, modelDmx, !dmxHelpers.empty(), installed.bones,
                  ModelBoneBindings(*rig, installed.bones));
    }
    // what the clip was built from, for issue reports
    if (options.debug) {
        std::string rigName;
        for (const SkeletonSource& source : sources)
            rigName += (rigName.empty() ? "" : " + ") + source.path.generic_u8string() +
                       (source.overwrite ? " (overwrite_existing)" : "");
        if (!nodes.empty())
            rigName += (rigName.empty() ? "" : " + ") + std::to_string(nodes.size()) + " VMDL Bone nodes";
        const std::string poseName = job.proportions.empty() ? rigName : job.proportions.generic_u8string();
        kv::WriteFile((paths.output / "debug/rig_merged.dmx").u8string(), mergedRig);
        kv::WriteFile((paths.output / "debug/skeleton_descriptor.kv3").u8string(), kv::WriteText(descriptor));
        kv::WriteFile((paths.output / "debug/report.kv3").u8string(),
                      kv::WriteText(Report(job, rigName, poseName, paths, installed.bones, target)));
        kv::WriteFile((paths.output / "debug/MODEL_SETUP.txt").u8string(),
                      SetupNotes(paths, dmxHelpers, nodeHelpers, installed.bones, warnings, job.writeModel));
    }
    PrintSummary(paths, dmxHelpers, nodeHelpers, installed.bones, warnings, job);

    if (!options.generateOnly)
        Compile(options, paths, skeletonBytes, graphBytes, uiGraphBytes, descriptor);
    std::cout << "DONE: " << paths.output.u8string()
              << (options.generateOnly ? " (source assets only)\n" : " (graph and clip compiled)\n");
}

} // namespace ag2

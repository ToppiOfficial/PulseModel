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
    kWorldGraphFile, kUiGraphFile, kClipFile, kProportionsFile, kReferenceFile, kSkeletonFile,
    "debug/skeleton_descriptor.kv3", "debug/report.kv3", "debug/MODEL_SETUP.txt",
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

struct RigSource {
    fs::path path;
    float scale = 1.0f; // the import's import_scale, baked into the skeleton
};

// model_dmx, else the render mesh carrying the most CS2 core bones (the body),
// else the SkeletonFile that does. The generated skeleton is never a source.
RigSource FindRig(kv::Value& vmdl, const Job& job, const Paths& paths) {
    const fs::path generated = paths.output / kSkeletonFile;
    std::vector<RigSource> meshes, files;
    Walk(vmdl, [&](kv::Value& node) {
        const std::string cls = ClassOf(node);
        if ((cls != "RenderMeshFile" && cls != "SkeletonFile") || !node.Has("filename"))
            return;
        const std::string name = node.At("filename").String();
        if (_stricmp(fs::u8path(name).extension().u8string().c_str(), ".dmx") != 0)
            return;
        RigSource source{ResolveImport(name, paths, job.vmdl),
                         node.Has("import_scale") ? static_cast<float>(node.At("import_scale").Number()) : 1.0f};
        if (fs::exists(generated) && fs::equivalent(source.path, generated))
            return;
        (cls == "RenderMeshFile" ? meshes : files).push_back(source);
    });
    if (!job.modelDmx.empty()) {
        for (const auto* list : {&meshes, &files})
            for (const RigSource& source : *list)
                if (fs::equivalent(source.path, job.modelDmx))
                    return {job.modelDmx, source.scale};
        return {job.modelDmx, 1.0f};
    }
    for (const auto* list : {&meshes, &files}) {
        RigSource best;
        size_t bestCount = 0;
        for (const RigSource& source : *list) {
            std::string err;
            auto dm = dmx::Datamodel::Load(source.path.u8string(), &err);
            const size_t count = dm ? CountBones(*dm) : 0;
            if (count > bestCount) {
                best = source;
                bestCount = count;
            }
        }
        if (bestCount)
            return best;
    }
    throw runtime_error("no DMX import carries the CS2 body bones; set model_dmx in the job");
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

std::unique_ptr<dmx::Datamodel> LoadDmx(const fs::path& path) {
    std::string err;
    auto dm = dmx::Datamodel::Load(path.u8string(), &err);
    if (!dm)
        throw runtime_error("cannot load " + path.u8string() + ": " + err);
    return dm;
}

// write_model: the VMDL is edited in place (the generated skeleton as its only
// SkeletonFile, no Bone nodes, graph bindings). The first edit keeps the
// untouched file as <name>.vmdl.bak, which ModelDoc never compiles.
void EditModel(const std::string& originalText, kv::Value model, const Job& job, const Paths& paths) {
    std::function<void(kv::Value&)> strip = [&](kv::Value& node) {
        if (!node.IsObject() || !node.Has("children") || !node.At("children").IsArray())
            return;
        auto& children = node["children"].Items();
        children.erase(std::remove_if(children.begin(), children.end(), [](const kv::Value& child) {
            const std::string cls = ClassOf(child);
            return (cls == "SkeletonFile" || cls == "Bone") && !Disabled(child);
        }), children.end());
        for (kv::Value& child : children)
            strip(child);
    };
    kv::Value* skeleton = nullptr;
    Walk(model, [&](kv::Value& node) {
        if (ClassOf(node) != "Skeleton")
            return;
        strip(node);
        if (!skeleton)
            skeleton = &node;
    });
    if (!skeleton) {
        auto& children = model["rootNode"]["children"].Items();
        children.push_back(kv::Value::Object{{"_class", "Skeleton"}, {"children", kv::Value::Array{}}});
        skeleton = &children.back();
    }
    if (!skeleton->Has("children"))
        (*skeleton)["children"] = kv::Value::Array{};
    auto& skeletonChildren = (*skeleton)["children"].Items();
    skeletonChildren.insert(skeletonChildren.begin(), kv::Value::Object{
        {"_class", "SkeletonFile"},
        {"name", "ag2_skeleton"},
        {"filename", paths.relative + "/" + kSkeletonFile},
        {"import_scale", 1.0},
        {"merge_behavior", "do_not_modify_existing"},
    });

    const std::string graph = paths.relative + "/" + kWorldGraphFile;
    const std::string uiGraph = paths.relative + "/" + kUiGraphFile;
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

    const size_t header = originalText.find("-->");
    if (header == std::string::npos)
        throw runtime_error("the VMDL has no KV3 header");
    fs::path backup = job.vmdl;
    backup += ".bak";
    if (!fs::exists(backup))
        kv::WriteFile(backup.u8string(), originalText);
    kv::WriteFile(job.vmdl.u8string(), originalText.substr(0, header + 3) + "\n" + kv::TextBody(model) + "\n");
}

std::string SetupNotes(const Paths& paths, const SkeletonBuild& skeleton, const std::vector<std::string>& warnings,
                       bool wroteModel) {
    std::string note = "AG2 PROPORTIONS (experimental)\n\n"
                       "Use this as the VMDL's only SkeletonFile, with no Bone nodes (write_model does this):\n  " +
                       paths.relative + "/" + kSkeletonFile + "\n\n";
    auto list = [&](const char* title, const std::vector<std::string>& names) {
        if (names.empty())
            return;
        note += title;
        for (const std::string& name : names)
            note += "  " + name + "\n";
        note += "\n";
    };
    list("Stock bones added at their stock offset:\n", skeleton.added);
    list("Bones moved under their stock parent (model-space pose kept):\n", skeleton.reparented);
    list("Bones kept from VMDL Bone nodes:\n", skeleton.fromNodes);
    note += "Set the worldmodel AnimGraph2 to:\n  " + paths.relative + "/" + kWorldGraphFile + "\n\n";
    if (wroteModel)
        note += "The input VMDL was edited in place; its first backup is <model>.vmdl.bak.\n\n";
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
        {"graph", paths.relative + "/" + kWorldGraphFile},
        {"boneTranslations", bones},
        {"compiled", false},
    };
}

bool IsAxisNote(const std::string& warning) { return warning.rfind("bone axes differ", 0) == 0; }

// What the user has to do next, on the console; the full notes are --debug only.
void PrintSummary(const Paths& paths, const SkeletonBuild& skeleton, const std::vector<std::string>& warnings,
                  const Job& job) {
    std::cout << "Skeleton " << paths.relative << "/" << kSkeletonFile << ": " << skeleton.added.size()
              << " stock bones added, " << skeleton.reparented.size() << " moved under their stock parent\n";
    if (job.writeModel)
        std::cout << "Edited " << job.vmdl.filename().u8string() << " (original kept as "
                  << job.vmdl.filename().u8string() << ".bak)\n";
    else
        std::cout << "Make it the VMDL's only SkeletonFile and remove the VMDL's Bone nodes\n"
                  << "Set the worldmodel AnimGraph2 to " << paths.relative << "/" << kWorldGraphFile << "\n"
                  << "Set the uimodel AnimGraph2 to " << paths.relative << "/" << kUiGraphFile << "\n";
    const auto axes = std::count_if(warnings.begin(), warnings.end(), IsAxisNote);
    for (const std::string& warning : warnings)
        if (!IsAxisNote(warning))
            std::cout << "WARNING: " << warning << '\n';
    if (axes)
        std::cout << "WARNING: " << axes << " bones have axes unlike stock and may be placed off "
                  << "(--debug lists them)\n";
}

} // namespace

void Generate(const Job& job, const Options& options) {
    std::cout << "Processing " << job.vmdl.u8string() << '\n';
    const Paths paths = Locate(job.vmdl);
    const std::string vmdlText = ReadText(job.vmdl);
    kv::Value vmdl = kv::ParseText(vmdlText);
    const RigSource rigSource = FindRig(vmdl, job, paths);
    const std::vector<Bone> nodes = BoneNodes(vmdl);

    const Vpk package(paths.cs2 / "game/csgo/pak01_dir.vpk");
    std::vector<uint8_t> skeletonBytes = package.Read(std::string(kStockSkeleton) + "_c");
    const StockSkeleton installed = ReadStockSkeleton(skeletonBytes);
    if (!job.vnmskel.empty()) {
        skeletonBytes = kv::ReadFile(job.vnmskel.u8string());
        RequireSameSkeleton(installed, ReadStockSkeleton(skeletonBytes));
    }
    const std::string clip = paths.relative + "/" + kClipFile;
    const std::vector<uint8_t> graphBytes = package.Read(std::string(kStockGraph) + "_c");
    const std::vector<uint8_t> uiGraphBytes = package.Read(std::string(kStockUiGraph) + "_c");
    const kv::Value graph = WrapperGraph(kv::ReadResource(graphBytes), kStockGraph, clip);
    const kv::Value uiGraph = WrapperGraph(kv::ReadResource(uiGraphBytes), kStockUiGraph, clip);

    std::vector<std::string> warnings;
    const SkeletonBuild skeleton =
        BuildSkeleton(ReadRig(*LoadDmx(rigSource.path), rigSource.scale, false), installed.bones, nodes);
    const std::vector<Bone> target =
        job.proportions.empty()
            ? TargetPose(skeleton.bones, installed.bones, warnings)
            : TargetPose(BuildSkeleton(ReadRig(*LoadDmx(job.proportions), rigSource.scale, true), installed.bones, {})
                             .bones,
                         installed.bones, warnings);
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

    // ModelDoc refuses a primary root bone that has a parent
    Walk(vmdl, [&](kv::Value& node) {
        if (ClassOf(node) != "BoneMarkupList" || !node.Has("primary_root_bone"))
            return;
        const std::string root = node.At("primary_root_bone").String();
        for (const Bone& bone : skeleton.bones)
            if (!root.empty() && _stricmp(bone.name.c_str(), root.c_str()) == 0 && bone.parent >= 0)
                warnings.push_back("BoneMarkupList primary_root_bone \"" + root + "\" sits under " +
                                   skeleton.bones[bone.parent].name +
                                   ", so ModelDoc will refuse to compile; remove it or name a root bone");
    });
    std::sort(warnings.begin(), warnings.end());
    warnings.erase(std::unique(warnings.begin(), warnings.end()), warnings.end());

    // A target equal to stock compiles to a clip that changes nothing.
    float change = 0;
    for (size_t i = 0; i < target.size(); ++i)
        change = std::max(change, std::abs(target[i].position.x - installed.bones[i].position.x) +
                                      std::abs(target[i].position.y - installed.bones[i].position.y) +
                                      std::abs(target[i].position.z - installed.bones[i].position.z));
    if (change < 1e-3f)
        warnings.insert(warnings.begin(), "the target pose equals the stock skeleton, so the clip changes nothing");

    fs::create_directories(paths.output);
    // a job may read a previous run's output; never delete an input
    auto clear = [&](const fs::path& file) {
        for (const fs::path& input : {job.proportions, job.vnmskel, rigSource.path})
            if (!input.empty() && fs::exists(file) && fs::equivalent(file, input))
                return;
        fs::remove(file);
    };
    for (const char* name : kGeneratedFiles)
        clear(paths.output / name);
    std::error_code ec;
    for (const char* folder : {"debug", "graphs", "anims", "dmx"})
        fs::remove(paths.output / folder, ec); // only when empty
    std::cout << "Output: " << paths.output.u8string() << '\n';

    for (const char* name : {kSkeletonFile, kReferenceFile})
        fs::create_directories((paths.output / name).parent_path());
    WriteSkeleton(paths.output / kSkeletonFile, skeleton.bones);
    WriteHeldPose(paths.output / kReferenceFile, installed.bones, installed.lowLodCount);
    WriteHeldPose(paths.output / kProportionsFile, target, installed.lowLodCount);
    const kv::Value descriptor = SkeletonDescriptor(ReadStockSkeleton(skeletonBytes), paths.relative + "/" + kReferenceFile);
    kv::WriteFile((paths.output / kClipFile).u8string(), kv::WriteText(ProportionClip(paths.relative)));
    kv::WriteFile((paths.output / kWorldGraphFile).u8string(), kv::WriteText(graph));
    kv::WriteFile((paths.output / kUiGraphFile).u8string(), kv::WriteText(uiGraph));
    if (job.writeModel)
        EditModel(vmdlText, vmdl, job, paths);
    // what the clip was built from, for issue reports
    if (options.debug) {
        const std::string rigName = rigSource.path.generic_u8string();
        const std::string poseName = job.proportions.empty() ? rigName : job.proportions.generic_u8string();
        kv::WriteFile((paths.output / "debug/skeleton_descriptor.kv3").u8string(), kv::WriteText(descriptor));
        kv::WriteFile((paths.output / "debug/report.kv3").u8string(),
                      kv::WriteText(Report(job, rigName, poseName, paths, installed.bones, target)));
        kv::WriteFile((paths.output / "debug/MODEL_SETUP.txt").u8string(),
                      SetupNotes(paths, skeleton, warnings, job.writeModel));
    }
    PrintSummary(paths, skeleton, warnings, job);

    if (!options.generateOnly)
        Compile(options, paths, skeletonBytes, graphBytes, uiGraphBytes, descriptor);
    std::cout << "DONE: " << paths.output.u8string()
              << (options.generateOnly ? " (source assets only)\n" : " (graph and clip compiled)\n");
}

} // namespace ag2

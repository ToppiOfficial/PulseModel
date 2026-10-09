// compiler.cpp - runs Valve's resourcecompiler headless in a throwaway
// workspace, validates the clip and graph it produces, then publishes them.

#include <chrono>
#include <cmath>
#include <cstdlib>
#include <iostream>
#include <random>
#include <stdexcept>
#include <thread>
#include <utility>

#include "ag2.h"
#include "templates.h"

#ifdef _WIN32
#define NOMINMAX
#include <windows.h>
#else
#include <cerrno>
#include <csignal>
#include <fcntl.h>
#include <sys/wait.h>
#include <unistd.h>
#endif

namespace ag2 {
namespace {

using std::runtime_error;
using Env = std::vector<std::pair<std::string, std::string>>;

constexpr int kTimeoutSeconds = 120;

std::string ReadText(const fs::path& path) {
    const auto bytes = kv::ReadFile(path.u8string());
    return {bytes.begin(), bytes.end()};
}

#ifdef _WIN32
std::wstring Wide(const std::string& text) {
    const int n = MultiByteToWideChar(CP_UTF8, MB_ERR_INVALID_CHARS, text.data(), static_cast<int>(text.size()), nullptr, 0);
    if (n == 0 && !text.empty())
        throw runtime_error("invalid UTF-8 in a compiler argument");
    std::wstring out(n, L'\0');
    MultiByteToWideChar(CP_UTF8, 0, text.data(), static_cast<int>(text.size()), out.data(), n);
    return out;
}

// CommandLineToArgvW quoting: backslashes double only before a quote.
std::wstring QuoteArgument(const std::wstring& text) {
    std::wstring out = L"\"";
    size_t slashes = 0;
    for (wchar_t c : text) {
        if (c == L'\\') {
            ++slashes;
            continue;
        }
        out.append(c == L'"' ? slashes * 2 + 1 : slashes, L'\\');
        slashes = 0;
        out += c;
    }
    out.append(slashes * 2, L'\\');
    return out + L'"';
}

// The inherited environment with `env` replacing same-named variables.
std::wstring EnvironmentBlock(const Env& env) {
    std::wstring block;
    wchar_t* inherited = GetEnvironmentStringsW();
    if (!inherited)
        throw runtime_error("cannot read the process environment");
    for (const wchar_t* entry = inherited; *entry; entry += wcslen(entry) + 1) {
        bool replaced = false;
        for (const auto& [name, value] : env) {
            const std::wstring prefix = Wide(name) + L"=";
            replaced |= _wcsnicmp(entry, prefix.c_str(), prefix.size()) == 0;
        }
        if (!replaced)
            block.append(entry).push_back(L'\0');
    }
    FreeEnvironmentStringsW(inherited);
    for (const auto& [name, value] : env)
        block.append(Wide(name + "=" + value)).push_back(L'\0');
    block.push_back(L'\0');
    return block;
}

std::string GetEnv(const char* name) {
    const DWORD n = GetEnvironmentVariableW(Wide(name).c_str(), nullptr, 0);
    if (n == 0)
        return {};
    std::wstring value(n, L'\0');
    GetEnvironmentVariableW(Wide(name).c_str(), value.data(), n);
    value.resize(n - 1);
    return fs::path(value).u8string();
}

// Runs `exe args` in `cwd` with stdout and stderr in `log`. Only the log and a
// NUL stdin are inherited. Returns the exit code.
int RunProcess(const fs::path& exe, const std::vector<std::string>& args, const fs::path& cwd,
               const fs::path& log, const Env& env) {
    SECURITY_ATTRIBUTES inherit{sizeof(inherit), nullptr, TRUE};
    HANDLE out = CreateFileW(log.c_str(), GENERIC_WRITE, FILE_SHARE_READ, &inherit, CREATE_ALWAYS,
                             FILE_ATTRIBUTE_NORMAL, nullptr);
    if (out == INVALID_HANDLE_VALUE)
        throw runtime_error("cannot create " + log.u8string());
    HANDLE in = CreateFileW(L"NUL", GENERIC_READ, FILE_SHARE_READ | FILE_SHARE_WRITE, &inherit, OPEN_EXISTING, 0, nullptr);

    std::wstring command = QuoteArgument(exe.wstring());
    for (const std::string& arg : args)
        command += L" " + QuoteArgument(Wide(arg));
    std::wstring environment = EnvironmentBlock(env);

    HANDLE handles[2] = {out, in};
    SIZE_T size = 0;
    InitializeProcThreadAttributeList(nullptr, 1, 0, &size);
    std::vector<char> attributeBuffer(size);
    auto* attributes = reinterpret_cast<LPPROC_THREAD_ATTRIBUTE_LIST>(attributeBuffer.data());
    const bool listed = InitializeProcThreadAttributeList(attributes, 1, 0, &size) &&
                        UpdateProcThreadAttribute(attributes, 0, PROC_THREAD_ATTRIBUTE_HANDLE_LIST, handles,
                                                  sizeof(HANDLE) * (in == INVALID_HANDLE_VALUE ? 1 : 2), nullptr, nullptr);

    STARTUPINFOEXW startup{};
    startup.StartupInfo.cb = sizeof(startup);
    startup.StartupInfo.dwFlags = STARTF_USESTDHANDLES;
    startup.StartupInfo.hStdOutput = out;
    startup.StartupInfo.hStdError = out;
    startup.StartupInfo.hStdInput = in == INVALID_HANDLE_VALUE ? nullptr : in;
    startup.lpAttributeList = listed ? attributes : nullptr;
    PROCESS_INFORMATION process{};
    const bool launched = listed && CreateProcessW(exe.c_str(), command.data(), nullptr, nullptr, TRUE,
                                                   CREATE_NO_WINDOW | CREATE_UNICODE_ENVIRONMENT | EXTENDED_STARTUPINFO_PRESENT,
                                                   environment.data(), cwd.c_str(), &startup.StartupInfo, &process);
    const DWORD launchError = GetLastError();
    if (listed)
        DeleteProcThreadAttributeList(attributes);
    CloseHandle(out);
    if (in != INVALID_HANDLE_VALUE)
        CloseHandle(in);
    if (!launched)
        throw runtime_error("cannot start " + exe.u8string() + " (Windows error " + std::to_string(launchError) + ")");
    CloseHandle(process.hThread);

    if (WaitForSingleObject(process.hProcess, kTimeoutSeconds * 1000) != WAIT_OBJECT_0) {
        TerminateProcess(process.hProcess, 1);
        WaitForSingleObject(process.hProcess, 5000);
        CloseHandle(process.hProcess);
        throw runtime_error("the resource compiler timed out, see " + log.u8string());
    }
    DWORD code = 1;
    GetExitCodeProcess(process.hProcess, &code);
    CloseHandle(process.hProcess);
    return static_cast<int>(code);
}
#else
std::string GetEnv(const char* name) {
    const char* value = std::getenv(name);
    return value ? value : "";
}

// Wine maps the host root to Z: by default.
std::string WinePath(const fs::path& path) { return "Z:" + fs::absolute(path).generic_u8string(); }

int RunProcess(const fs::path& exe, const std::vector<std::string>& args, const fs::path& cwd,
               const fs::path& log, const Env& env) {
    std::vector<std::string> storage{exe.u8string()};
    storage.insert(storage.end(), args.begin(), args.end());
    std::vector<char*> argv;
    for (std::string& s : storage)
        argv.push_back(s.data());
    argv.push_back(nullptr);

    const pid_t child = fork();
    if (child < 0)
        throw runtime_error("cannot fork the resource compiler");
    if (child == 0) {
        setpgid(0, 0);
        const int out = open(log.c_str(), O_WRONLY | O_CREAT | O_TRUNC | O_CLOEXEC, 0644);
        const int in = open("/dev/null", O_RDONLY);
        if (out < 0 || in < 0 || chdir(cwd.c_str()) != 0 || dup2(out, STDOUT_FILENO) < 0 ||
            dup2(out, STDERR_FILENO) < 0 || dup2(in, STDIN_FILENO) < 0)
            _exit(126);
        for (const auto& [name, value] : env)
            setenv(name.c_str(), value.c_str(), 1);
        execvp(argv[0], argv.data());
        _exit(127);
    }

    int status = 0;
    const auto deadline = std::chrono::steady_clock::now() + std::chrono::seconds(kTimeoutSeconds);
    for (;;) {
        const pid_t done = waitpid(child, &status, WNOHANG);
        if (done == child)
            break;
        if (done < 0 && errno != EINTR)
            throw runtime_error("cannot wait for the resource compiler");
        if (std::chrono::steady_clock::now() >= deadline) {
            kill(-child, SIGKILL);
            waitpid(child, &status, 0);
            throw runtime_error("the resource compiler timed out, see " + log.u8string());
        }
        std::this_thread::sleep_for(std::chrono::milliseconds(50));
    }
    return WIFEXITED(status) ? WEXITSTATUS(status) : 128;
}
#endif

std::string Prepend(const std::string& front, const char* variable, char separator) {
    const std::string rest = GetEnv(variable);
    return rest.empty() ? front : front + separator + rest;
}

// The installed compiler binaries for the platform that will run them.
struct Toolchain {
    std::string platform;
    fs::path bin, modBin;
    std::vector<std::string> files; // [0] is the executable
    std::string modtools;
};

Toolchain FindToolchain(const fs::path& cs2, bool windowsBinaries) {
    Toolchain t;
    t.platform = windowsBinaries ? "win64" : "linuxsteamrt64";
    t.bin = cs2 / "game/bin" / t.platform;
    t.modBin = cs2 / "game/csgo/bin" / t.platform;
    t.files = windowsBinaries
                  ? std::vector<std::string>{"resourcecompiler.exe", "resourcecompiler.dll", "tier0.dll", "assetsystem.dll"}
                  : std::vector<std::string>{"resourcecompiler", "libresourcecompiler.so", "libtier0.so", "libassetsystem.so"};
    t.modtools = windowsBinaries ? "modtools.dll" : "libmodtools.so";
    for (const std::string& name : t.files)
        if (!fs::exists(t.bin / name))
            throw runtime_error("missing " + (t.bin / name).u8string() +
                                "; install CS2 Workshop Tools or use --generate-only");
    if (!fs::exists(t.modBin / t.modtools))
        throw runtime_error("missing " + (t.modBin / t.modtools).u8string());
    return t;
}

// A private game/content pair: copies of the compiler binaries, the installed
// asset registry plus the AG2 types, and the stock skeleton and graph. Nothing
// installed is modified.
void BuildWorkspace(const fs::path& ws, const Toolchain& t, const fs::path& cs2, const Paths& paths,
                    const std::vector<uint8_t>& skeleton, const std::vector<uint8_t>& graph,
                    const std::vector<uint8_t>& uiGraph,
                    const kv::Value& descriptor) {
    const fs::path game = ws / "game/ag2", content = ws / "content/ag2";
    fs::create_directories(ws / "game/bin" / t.platform);
    fs::create_directories(game / "bin" / t.platform);
    for (const std::string& name : t.files)
        fs::copy_file(t.bin / name, ws / "game/bin" / t.platform / name);
    fs::copy_file(t.modBin / t.modtools, game / "bin" / t.platform / t.modtools);

    kv::Value registry = kv::ParseText(ReadText(cs2 / "game/bin/assettypes_common.txt"));
    const kv::Value extra = kv::ParseText(kAssetTypesTemplate);
    for (const auto& [key, type] : extra.At("assettypes").Members()) {
        bool known = false;
        for (const auto& current : registry.At("assettypes").Members())
            known |= current.second.Has("m_Ext") && current.second.At("m_Ext").String() == type.At("m_Ext").String();
        if (!known)
            registry["assettypes"][key] = type;
    }
    kv::WriteFile((ws / "game/bin/assettypes_common.txt").u8string(), kv::WriteText(registry));
    if (fs::exists(cs2 / "game/bin/assettypes_internal.txt"))
        fs::copy_file(cs2 / "game/bin/assettypes_internal.txt", ws / "game/bin/assettypes_internal.txt");

    kv::WriteFile((game / "gameinfo.gi").u8string(),
                  "\"GameInfo\" { game \"AG2 Proportions\" FileSystem { SteamAppId 730 SearchPaths { Game ag2 Game " +
                      kv::Quote((cs2 / "game/csgo").generic_u8string()) + " Game " +
                      kv::Quote((cs2 / "game/core").generic_u8string()) + " Mod ag2 } } }\n");
    kv::WriteFile((game / (std::string(kStockSkeleton) + "_c")).u8string(), skeleton);
    kv::WriteFile((game / (std::string(kStockGraph) + "_c")).u8string(), graph);
    kv::WriteFile((game / (std::string(kStockUiGraph) + "_c")).u8string(), uiGraph);

    // the clip compile needs the skeleton's source form at its canonical path
    kv::WriteFile((content / kStockSkeleton).u8string(), kv::WriteText(descriptor));
    const fs::path sources = content / fs::u8path(paths.relative);
    fs::create_directories(sources / "graphs");
    fs::create_directories(sources / "anims");
    for (const char* name : {"reference.dmx", "proportions.dmx", kClipFile, kWorldGraphFile, kUiGraphFile})
        fs::copy_file(paths.output / name, sources / name);
}

// The clip must be additive on the stock skeleton with every track a static
// translation and an identity delta rotation.
void ValidateClip(const kv::Value& clip, size_t boneCount) {
    if (!clip.At("m_bIsAdditive").Boolean() || clip.At("m_skeleton").String() != kStockSkeleton ||
        clip.At("m_nNumFrames").Integer() < 1)
        throw runtime_error("the compiled clip is not an additive clip on the stock skeleton");
    const auto& tracks = clip.At("m_trackCompressionSettings").Items();
    if (tracks.size() != boneCount)
        throw runtime_error("the compiled clip's bone count differs from stock");
    for (const auto& track : tracks) {
        const auto& r = track.At("m_constantRotation").Items();
        const bool identity = r.size() == 4 &&
                              std::abs(r[0].Number()) + std::abs(r[1].Number()) + std::abs(r[2].Number()) <= 0.001 &&
                              std::abs(std::abs(r[3].Number()) - 1.0) <= 0.001;
        if (!track.At("m_bIsRotationStatic").Boolean() || !track.At("m_bIsTranslationStatic").Boolean() || !identity)
            throw runtime_error("the compiled clip is not a held translation-only pose");
    }
    std::cout << "Validated clip: " << tracks.size() << " static tracks with identity rotations\n";
}

void ValidateGraph(const kv::Value& graph) {
    bool snap = false, layer = false, reference = false;
    for (const auto& node : graph.At("m_nodes").Items()) {
        const std::string cls = node.At("_class").String();
        snap |= cls == "CNmSnapWeaponNode::CDefinition";
        layer |= cls == "CNmLayerBlendNode::CDefinition";
        reference |= cls == "CNmReferencedGraphNode::CDefinition";
    }
    if (!snap || !layer || !reference)
        throw runtime_error("the compiled graph lacks its referenced graph, additive layer or SnapWeapon node");
}

} // namespace

void Compile(const Options& options, const Paths& paths, const std::vector<uint8_t>& skeleton,
             const std::vector<uint8_t>& graph, const std::vector<uint8_t>& uiGraph, const kv::Value& descriptor) {
#ifdef _WIN32
    (void)options;
    const bool windowsBinaries = true;
#else
    const bool windowsBinaries = options.compilerRunner == "wine";
#endif
    const Toolchain tools = FindToolchain(paths.cs2, windowsBinaries);
    std::random_device random;
    const fs::path ws = fs::temp_directory_path() /
                        ("pulse-ag2-" + std::to_string(std::chrono::system_clock::now().time_since_epoch().count()) +
                         "-" + std::to_string(random()));
    try {
        BuildWorkspace(ws, tools, paths.cs2, paths, skeleton, graph, uiGraph, descriptor);
        const fs::path game = ws / "game/ag2", content = ws / "content/ag2";
        const kv::Value expected = kv::ReadResource(skeleton);

        for (const char* name : {kClipFile, kWorldGraphFile, kUiGraphFile}) {
            const std::string asset = paths.relative + "/" + name;
            fs::path exe = ws / "game/bin" / tools.platform / tools.files[0];
            std::vector<std::string> args{"-game", game.u8string(), "-i", (content / fs::u8path(asset)).u8string(),
                                          "-nop4", "-empty", "-f"};
#ifdef _WIN32
            const Env env{{"PATH", Prepend(tools.bin.u8string() + ";" + tools.modBin.u8string(), "PATH", ';')}};
#else
            Env env{{"LD_LIBRARY_PATH", Prepend(tools.bin.u8string() + ":" + tools.modBin.u8string(), "LD_LIBRARY_PATH", ':')}};
            if (windowsBinaries) {
                args[1] = WinePath(game);
                args[3] = WinePath(content / fs::u8path(asset));
                args.insert(args.begin(), WinePath(exe));
                exe = fs::u8path(options.compilerRunner);
                env.push_back({"WINEPATH", WinePath(tools.bin) + ";" + WinePath(tools.modBin)});
            }
#endif
            const fs::path log = ws / (fs::u8path(name).filename().u8string() + ".compile.log");
            const int code = RunProcess(exe, args, ws, log, env);
            const fs::path compiled = game / fs::u8path(asset + "_c");
            if (code != 0 || !fs::exists(compiled) || ReadText(log).find("0 failed") == std::string::npos)
                throw runtime_error(std::string("compiling ") + name + " failed, see " + log.u8string());

            const kv::Value data = kv::ReadResource(kv::ReadFile(compiled.u8string()));
            if (fs::path(name).extension() == ".vnmclip") {
                // the clip compile also rebuilt the skeleton from its descriptor
                const kv::Value rebuilt =
                    kv::ReadResource(kv::ReadFile((game / (std::string(kStockSkeleton) + "_c")).u8string()));
                if (kv::TextBody(expected.At("m_boneIDs")) != kv::TextBody(rebuilt.At("m_boneIDs")) ||
                    kv::TextBody(expected.At("m_parentIndices")) != kv::TextBody(rebuilt.At("m_parentIndices")))
                    throw runtime_error("the workspace skeleton's bone order differs from stock");
                ValidateClip(data, expected.At("m_boneIDs").Items().size());
            } else {
                ValidateGraph(data);
            }
        }

        const fs::path published = paths.cs2 / "game/csgo_addons" / paths.addon.filename() / fs::u8path(paths.relative);
        for (const char* source : {kClipFile, kWorldGraphFile, kUiGraphFile}) {
            const fs::path name = fs::u8path(std::string(source) + "_c");
            fs::create_directories((published / name).parent_path());
            fs::copy_file(game / fs::u8path(paths.relative) / name, published / name,
                          fs::copy_options::overwrite_existing);
            std::cout << "Compiled: " << (published / name).u8string() << '\n';
        }

        const fs::path reportPath = paths.output / "debug/report.kv3";
        if (fs::exists(reportPath)) {
            kv::Value report = kv::ParseText(ReadText(reportPath));
            report["compiled"] = true;
            report["compiledOutput"] = published.generic_u8string();
            kv::WriteFile(reportPath.u8string(), kv::WriteText(report));
        }
    } catch (const std::exception& e) {
        throw runtime_error(std::string(e.what()) + " (workspace kept at " + ws.u8string() + ")");
    }
    std::error_code ec;
    fs::remove_all(ws, ec);
}

} // namespace ag2

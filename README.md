<p align="center">
<img alt="PulseModel" src="app.svg" width="128">
</p>

<h1 align="center">PulseModel</h1>

<p align="center">
<img alt="Windows x64" src="https://img.shields.io/badge/platform-Windows%20x64-blue?style=for-the-badge&logo=windows&logoColor=white">
<img alt="Linux x64" src="https://img.shields.io/badge/platform-Linux%20x64-blue?style=for-the-badge&logo=linux&logoColor=white">
<img alt="model version" src="https://img.shields.io/badge/model%20version-49-informational?style=for-the-badge">
<img alt="license" src="https://img.shields.io/badge/license-MIT-success?style=for-the-badge">
</p>

A Source engine model toolkit targeting **MDL v49** (HL2, TF2, L4D2, GMod,
CS:S, SFM), with `mdlcompiler` and `mdldecompiler` for Windows and Linux.

> **Beta.** New commands may be experimental and can change or be removed between
> versions as concepts are tested. Expect rough edges and script compatibility changes.

## Download

[Stable releases](https://github.com/ToppiOfficial/PulseModel/releases) or latest dev builds:

- **[Windows x64 (dev)](https://nightly.link/ToppiOfficial/PulseModel/workflows/dev/main/pulsemodel-win64.zip)**
- **[Linux x64 (dev)](https://nightly.link/ToppiOfficial/PulseModel/workflows/dev/main/pulsemodel-linux64.zip)**

Both tools are in the archive's `PulseModel/` folder.

## mdlcompiler

Compiles DMX, SMD and FBX. Supports DMX up to model format 22 / binary encoding 9,
plus LODs, bodygroups, flexes, procedural bones, IK, animation layering, physics,
mesh editing and QC preprocessing.

Compiler limits are defined in `libs/pulselimits.h` and may exceed engine limits.
A successful compile does not guarantee the model will load in the target game.

### Usage

```
mdlcompiler <file.qc|file.qczip> [-game <dir>] [-modelname <path>] [-defvar <name> <value>]
            [-includesearchdir <dir>] [-filesearchdir <dir>] [-tempcontent <dir>]
            [-vtxformat <0|1>] [-nodx80]
            [-striplods] [-minlod <lod>] [-definebones] [-verify] [-dumpmaterials]
            [-legacysequencebounds] [-perfmetrics] [-pause]
```

`.qczip` packages contain `model.qc` at the root and optional bundled source files.

| Option | Meaning |
| --- | --- |
| `-game <dir>` | Mod directory to install into; output lands in `<dir>/models/<modelname>.mdl`. `-outdir` is a synonym. |
| `-modelname <path>` | Overrides the script's `$modelname`. |
| `-defvar <name> <value>` | Define a script variable (`$name$`) before the script runs. Repeatable; the script cannot override it. |
| `-includesearchdir <dir>` | Extra fallback directory for `$include`, searched after any `$addincludesearchdir`. Repeatable. |
| `-filesearchdir <dir>` | Extra fallback directory for source files, searched after any `$addsearchdir`. Repeatable. |
| `-tempcontent <dir>` | Synonym for `-filesearchdir`. |
| `-vtxformat <0\|1>` | `.vtx` layout, overriding the script's `$vtxformat`. 0 = legacy (TF2/L4D2/GMod/HL2), 1 = full (SFM/CS:GO/ASW). |
| `-nodx80` | Skip the DirectX 8 `.dx80.vtx` output (also `$nodx80`). Only written at vtx format 0. |
| `-striplods` | Ignore all `$lod` and `$shadowlod` commands and compile only the original root LOD. |
| `-minlod <lod>` | Discard higher-detail LODs and promote the zero-based LOD index to root, overriding `$minlod` in the script. |
| `-definebones` | Print the compiled skeleton as `$definebone` lines and stop - nothing is written. |
| `-verify` | Compile the model without writing output files. |
| `-legacysequencebounds` | Use per-frame vertex skinning for sequence bounds. The default precomputes per-bone vertex boxes and transforms those boxes per frame, which is faster but can produce wider bounds. |
| `-dumpmaterials` | Print the names of materials used by the compiled model. |
| `-perfmetrics` | Print wall time in ms for each stage of the compile. |
| `-pause` | Wait for a keypress before exiting (drag-and-drop runs). |

### For external tools

The versioned API reads JSON requests from stdin, writes JSON records to stdout,
and sends compiler logs to stderr.

| Option | Meaning |
| --- | --- |
| `-api` | Use the versioned integration interface for command catalog, QC analysis and compilation with dependency reporting. |

## mdldecompiler

Reads **MDL v44-49** with sibling `.vvd`/`.vtx`/`.phy`/`.ani` files and writes
QC, DMX meshes and animations. Unreadable meshes or animations are reported and skipped.

### Usage

```
mdldecompiler <file.mdl|folder> ... [-outdir <dir>] [-forceversion <n>]
              [-dmxencoding <enc>] [-dmxmodel <n>] [-smdanimation] [-newcommands]
              [-nomesh] [-noanimation] [-declaresequence] [-datamodeljoints]
              [-pause] [-perfmetrics]
```

Accepts multiple inputs and recursive folder input. Each model gets its own folder
under `decompiled <version>/` beside the input. QC uses stock studiomdl commands by default.

| Option | Meaning |
| --- | --- |
| `-outdir <dir>` | Put the `decompiled <version>` wrapper under `<dir>` instead of beside the input; absolute, or relative to the current directory. |
| `-forceversion <n>` | Read the file as version `<n>`, ignoring the header. Some models carry a wrong version to break decompilers; point this at the real one. |
| `-dmxencoding <enc>` | How the `.dmx` meshes are encoded: `binary` (default) or `keyvalues2` text. |
| `-dmxmodel <n>` | The `format model` version they declare: 15 (default), 1, 18, or 22 (Source 2 ModelDoc). |
| `-smdanimation` | Write animation clips as `.smd` instead of `.dmx`. |
| `-newcommands` | Use newer `mdlcompiler` commands in the `.qc` instead of stock-studiomdl commands. |
| `-nomesh` | Skip the mesh `.dmx` files, the `.phy` hull, the `.vrd` and the joints `.dmx`; the script still lists their commands. |
| `-noanimation` | Skip the animation clip files; the script still lists the sequences and animations. |
| `-declaresequence` | Also write a `<name>.qci` listing every sequence, in order, as `$declaresequence` - paste into a model that `$includemodel`'s this one. |
| `-datamodeljoints` | Write hitboxes, procedural bones and jigglebones to `meshes/<name>_joints.dmx` and import it with one `$datamodeljoints` line instead of the raw `$hboxset`/`$driverbone`/`.vrd`/`$jigglebone` commands. Follows `-dmxencoding`/`-dmxmodel`; works with either command style. The script then needs `mdlcompiler` - stock studiomdl has no `$datamodeljoints`. Hitbox names are dropped (a DMX hitbox has none). |
| `-pause` | Wait for a keypress before exiting (drag-and-drop runs). |
| `-perfmetrics` | Print wall time in ms per process once the run ends; a batch sums each process across all models. |

## Building

64-bit Windows and Linux. Dependencies are vendored - no vcpkg, no conan.

Windows (CMake + Ninja + MSVC):

```
cmake --preset x64-release
cmake --build --preset x64-release
```

Both binaries land in `out/build/x64-release/PulseModel/`.

Linux (CMake + GCC + Make):

```
cmake -S . -B ~/pulsemodel-build -DCMAKE_BUILD_TYPE=Release
cmake --build ~/pulsemodel-build -j2
```

Both binaries land in `~/pulsemodel-build/PulseModel/`.

## Layout

```
libs/           C++ libraries: DMX reader, math, binary format headers,
                collision, and vendored third-party code.
                pulselimits.h holds every hard limit.
mdlcompiler/    The app: script loader, compile stage, .mdl/.vtx/.phy writers.
mdldecompiler/  The reverse tool: .mdl -> .qc + .dmx meshes + .smd anims.
```

## Third-party code and references

Vendored under `libs/`:

- [meshoptimizer](https://github.com/zeux/meshoptimizer) - vertex cache and
  overdraw optimization, LOD simplification. MIT.
- [V-HACD](https://github.com/kmammou/v-hacd) - convex decomposition, used to
  generate collision from a render mesh. BSD-3-Clause.
- [ufbx](https://github.com/ufbx/ufbx) - FBX reader. MIT.
- [nlohmann/json](https://github.com/nlohmann/json) - parses `-api` requests. MIT.

Referenced, not vendored:

- [Datamodel.NET](https://github.com/Artfunkel/Datamodel.NET) (Artfunkel, MIT) -
  the C++ DMX reader was rewritten from it, so there is no .NET dependency.
- [VPhysics-Jolt](https://github.com/misyltoad/VPhysics-Jolt) - read alongside
  V-HACD to work out the `.phy` / IVP collision format. `libs/minicollision` is
  our own code written from that understanding.
- [Crowbar](https://github.com/ZeqMacaw/Crowbar) (ZeqMacaw, CC BY-SA 3.0) -
  cross-checked our own expansion of combination flex ops (2-way, n-way, eyelid)
  into stock `.qc` min/max expressions. The math is the engine's (SDK `studio.cpp`
  flex eval); no Crowbar code was used.
- Valve's Source SDK 2013 - read as documentation of the file format.

## License

MIT. See [LICENSE](LICENSE).

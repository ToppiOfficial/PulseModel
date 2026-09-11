<h1 align="center">PulseModel</h1>

<p align="center">
<img alt="Windows x64" src="https://img.shields.io/badge/platform-Windows%20x64-blue?style=for-the-badge&logo=windows&logoColor=white">
<img alt="Linux x64" src="https://img.shields.io/badge/platform-Linux%20x64-blue?style=for-the-badge&logo=linux&logoColor=white">
<img alt="model version" src="https://img.shields.io/badge/model%20version-49-informational?style=for-the-badge">
<img alt="license" src="https://img.shields.io/badge/license-MIT-success?style=for-the-badge">
</p>

A Source-engine model toolkit for **studiomdl version 49** (HL2, TF2, L4D2,
GMod, CS:S, SFM). One build produces two executables:

- **`mdlcompiler`** - compile script + DMX/SMD/FBX sources -> `.mdl`, `.vvd`,
  `.vtx`, `.phy`.
- **`mdldecompiler`** - an existing `.mdl` -> a compile script with `.dmx`
  meshes and animations beside it.

> **Beta, Windows and Linux.** It compiles models that load and render in-engine, but
> expect rough edges and script-command changes between versions.

## mdlcompiler

Supports meshes, LODs, bodygroups, materials and skins; skeletons, bone markup,
collapse rules and bind-pose edits; procedural bones (jiggle, driver, aim-at,
VRD); animations and sequences, including demand-loaded `.ani`, layering, pose
parameters, IK chains and weight lists; flex/morph, face markup, eyeballs and
eyelids; physics hulls, ragdoll joints, hitboxes and convex decomposition; and a
script preprocessor (`$include`, variables, macros, `$if`/`$ifdef`/`$switch`).

DMX is the main source format; SMD and FBX also work. DMX files are read up to
model format 22 / binary encoding 9 (Source 2 ModelDoc), and lower versions still
work. Both `.vtx` layouts ship: legacy (`-vtxformat 0`, TF2/L4D2/GMod/HL2) and
full (`-vtxformat 1`, SFM/CS:GO/ASW).

**Limits are not Valve's.** The compiler rejects only counts the file format
cannot express; several limits are uncapped (bones, meshes, attachments,
materials, animations). Its own ceilings live in `libs/pulselimits.h`. So a model
that compiles is not guaranteed to load - the engine you ship to decides that.

### The compile script

The script is `.pulseqc` (a `.qc` extension is read as the same format). It looks
like QC and shares much of its vocabulary, but **it is not stock QC** - commands
have been removed, renamed and reshaped, and an unrecognized `$command` is an
error. A stock `.qc` input is read directly - the loader accepts the stock
spellings alongside the `.pulseqc` ones, so no conversion step is needed.

### Usage

```
mdlcompiler <file.pulseqc> [-game <dir>] [-modelname <path>] [-defvar <name> <value>]
            [-includesearchdir <dir>] [-filesearchdir <dir>] [-tempcontent <dir>]
            [-vtxformat <0|1>]
            [-striplods] [-minlod <lod>] [-definebones] [-verify] [-dumpmaterials]
            [-perfmetrics]
            [-dumpcommands] [-pause]
```

| Option | Meaning |
| --- | --- |
| `-game <dir>` | Mod directory to install into; output lands in `<dir>/models/<modelname>.mdl`. `-outdir` is a synonym. |
| `-modelname <path>` | Overrides the script's `$modelname`. |
| `-defvar <name> <value>` | Define a script variable (`$name$`) before the script runs. Repeatable; the script cannot override it. |
| `-includesearchdir <dir>` | Extra fallback directory for `$include`, searched after any `$addincludesearchdir`. Repeatable. |
| `-filesearchdir <dir>` | Extra fallback directory for source files, searched after any `$addsearchdir`. Repeatable. |
| `-tempcontent <dir>` | Synonym for `-filesearchdir`. |
| `-vtxformat <0\|1>` | `.vtx` layout, overriding the script's `$vtxformat`. 0 = legacy (TF2/L4D2/GMod/HL2), 1 = full (SFM/CS:GO/ASW). |
| `-striplods` | Ignore all `$lod` and `$shadowlod` commands and compile only the original root LOD. |
| `-minlod <lod>` | Discard higher-detail LODs and promote the zero-based LOD index to root, overriding `$minlod` in the script. |
| `-definebones` | Print the compiled skeleton as `$definebone` lines and stop - nothing is written. |
| `-verify` | Compile the model without writing output files. |
| `-dumpmaterials` | Print the names of materials used by the compiled model. |
| `-perfmetrics` | Print wall time in ms for each stage of the compile. |
| `-dumpcommands` | Print every accepted `$command`, one per line, and exit. |
| `-pause` | Wait for a keypress before exiting (drag-and-drop runs). |

The QC command `$minlod <lod>` sets the same zero-based minimum LOD. A
command-line `-minlod` value takes precedence.

## mdldecompiler

An `.mdl` plus its sibling `.vvd`/`.vtx`/`.phy`/`.ani` becomes a compile script
with `meshes/*.dmx` and `anims/*` beside it, ready to feed back into
`mdlcompiler`. Model versions **44 through 49** are read - static props up to
full characters with skeletons, animations, flexes, eyeballs, LODs, bodygroups,
hitboxes and ragdoll constraints. One unreadable mesh or animation does not abort
the job; it writes what it can parse and tells you what it skipped.

### Usage

```
mdldecompiler <file.mdl|folder> ... [-outdir <dir>] [-forceversion <n>]
              [-dmxencoding <enc>] [-dmxmodel <n>] [-smdanimation] [-pulseqc] [-pause]
              [-perfmetrics]
```

Several inputs may be given at once (drag-and-drop works); a folder decompiles
every `.mdl` under it, recursively. Pass `-pause` to hold the window open before
exiting so the summary stays on screen.

Each model lands in its own folder under a `decompiled <version>` wrapper beside
the input. By default it writes a stock-studiomdl `.qc`; pass `-pulseqc` for the
`.pulseqc` form. Either compiles back through `mdlcompiler`.

| Option | Meaning |
| --- | --- |
| `-outdir <dir>` | Put the `decompiled <version>` wrapper under `<dir>` instead of beside the input; absolute, or relative to the current directory. |
| `-forceversion <n>` | Read the file as version `<n>`, ignoring the header. Some models carry a wrong version to break decompilers; point this at the real one. |
| `-dmxencoding <enc>` | How the `.dmx` meshes are encoded: `binary` (default) or `keyvalues2` text. |
| `-dmxmodel <n>` | The `format model` version they declare: 15 (default), 1, 18, or 22 (Source 2 ModelDoc). |
| `-smdanimation` | Write animation clips as `.smd` instead of `.dmx`. |
| `-pulseqc` | Write a `.pulseqc` instead of the default stock-studiomdl `.qc`. |
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
mdldecompiler/  The reverse tool: .mdl -> .pulseqc + .dmx meshes + .smd anims.
```

## Third-party code and references

Vendored under `libs/`:

- [meshoptimizer](https://github.com/zeux/meshoptimizer) - vertex cache and
  overdraw optimization, LOD simplification. MIT.
- [V-HACD](https://github.com/kmammou/v-hacd) - convex decomposition, used to
  generate collision from a render mesh. BSD-3-Clause.
- [ufbx](https://github.com/ufbx/ufbx) - FBX reader. MIT.

Referenced, not vendored:

- [Datamodel.NET](https://github.com/Artfunkel/Datamodel.NET) (Artfunkel, MIT) -
  the C++ DMX reader was rewritten from it, so there is no .NET dependency.
- [VPhysics-Jolt](https://github.com/misyltoad/VPhysics-Jolt) - read alongside
  V-HACD to work out the `.phy` / IVP collision format. `libs/minicollision` is
  our own code written from that understanding.
- Valve's Source SDK 2013 - read as documentation of the file format.

## License

MIT. See [LICENSE](LICENSE).

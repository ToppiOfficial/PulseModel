<h1 align="center">PulseModel</h1>

<p align="center">
<img alt="status" src="https://img.shields.io/badge/status-beta-orange?style=for-the-badge">
<img alt="platform" src="https://img.shields.io/badge/platform-Windows%20x64-blue?style=for-the-badge&logo=windows&logoColor=white">
<img alt="Linux" src="https://img.shields.io/badge/linux-untested-lightgrey?style=for-the-badge&logo=linux&logoColor=white">
<img alt="model version" src="https://img.shields.io/badge/model%20version-49-informational?style=for-the-badge">
<img alt="license" src="https://img.shields.io/badge/license-MIT-success?style=for-the-badge">
</p>

> **Beta, Windows only.** It compiles real models that load and render in-engine,
> but expect rough edges and script-command changes between versions. The code
> avoids Windows-only APIs so a Linux build should be achievable, but it has not
> been attempted and is not currently planned.

A Source-engine model toolkit, targeting **studiomdl version 49** (HL2, TF2, L4D2, GMod, CS:S, SFM). One build produces two executables:

- **`mdlcompiler`** - a compile script plus DMX/SMD source assets -> Valve's
  binary model format (`.mdl`, `.vvd`, `.vtx`, `.phy`).
- **`mdldecompiler`** - the same pipeline backwards: an existing `.mdl` ->
  a compile script with DMX meshes and SMD animations beside it.

## mdlcompiler

- **DMX is the first-class source format.** SMD is supported as legacy input.
  DMX files are read up to **model format 22 / binary encoding 9** - the Source 2
  (ModelDoc) - and any lower version still works, so older exporters need
  no change.
- Meshes, LODs, bodygroups, materials, `$texturegroup` skins.
- Skeletons, bone markup, collapse rules, bind-pose edits.
- Procedural bones: jiggle bones, driver bones, aim-at bones, VRD.
- Animation and sequences, including demand-loaded `.ani` blocks, layering,
  pose parameters, IK chains and weight lists.
- Flex/morph, flex controllers and rules, face markup, eyeballs, eyelids.
- Physics: convex hulls, ragdoll joints, hitboxes, automatic convex
  decomposition of a render mesh.
- A script preprocessor: `$include`, variables, macros, `$if`/`$ifdef`/`$switch`
  conditionals.
- Both `.vtx` layouts - legacy (`-vtxformat 0`, TF2/L4D2/GMod/HL2) and full
  (`-vtxformat 1`, SFM/CS:GO/ASW).

### A note on limits

**PulseModel does not enforce Valve's studiomdl limits.** Its own ceilings live in
`libs/pulselimits.h`, and they are deliberately not a copy of any engine's
runtime caps - studiomdl's numbers differ per branch and every fork raises them.
The rule here is that the compiler rejects only what the **file format** cannot
express: a count that would overflow a field on disk. Anything the format can
represent is allowed through, and several limits are effectively uncapped
(bones, meshes, attachments, materials, animations).

The consequence is worth knowing before you push a model hard: a file that
compiles cleanly is not automatically a file the target engine will accept.
Exceeding what a given branch's `studio.h` expects can mean anything from
silently ignored data to visual corruption to a crash on load. The engine you
ship to is the thing that says no - not this compiler.

### The compile script

The compile script is `.pulseqc` (a `.qc` extension is accepted as the same
format). It looks like QC and shares much of its vocabulary, but **it is not
stock QC** - commands have been removed, renamed and reshaped, and an
unrecognized `$command` is a hard error rather than a warning.

### Usage

```
mdlcompiler <file.pulseqc> [-game <dir>] [-defvar <name> <value>]
            [-vtxformat <0|1>] [-definebones]
```

| Option | Meaning |
| --- | --- |
| `-game <dir>` | Mod directory to install into; output lands in `<dir>/models/<modelname>.mdl`. `-outdir` is a synonym. |
| `-defvar <name> <value>` | Define a script variable (`$name$`) before the script runs. Repeatable; the script cannot override it. |
| `-vtxformat <0\|1>` | `.vtx` layout, overriding the script's `$vtxformat`. 0 = legacy (TF2/L4D2/GMod/HL2), 1 = full (SFM/CS:GO/ASW). |
| `-definebones` | Print the compiled skeleton as `$definebone` lines and stop - nothing is written. |

## mdldecompiler

An existing `.mdl` (plus its sibling `.vvd`/`.vtx`/`.phy`/`.ani`) becomes a
compile script with `meshes/*.dmx` and `anims/*` beside it, ready to feed straight
back into `mdlcompiler`.

- **Model versions 44 through 49** are read.
- Anything from a static prop up: full character models with skeletons,
  animations and sequences, flexes and eyeballs, LODs, bodygroups, hitboxes,
  physics and ragdoll constraints.
- Writes either format. **`.pulseqc`** by default; **`-studiomdl`** writes a
  `.qc` that stock studiomdl accepts instead, using its command spellings and
  file layout.
- It never aborts the whole job over one unreadable mesh or animation - it
  writes what it can parse and tells you what it skipped.

### Usage

```
mdldecompiler <file.mdl|folder> ... [-o <file>] [-forceversion <n>]
              [-dmxencoding <enc>] [-dmxmodel <n>] [-smdanimation] [-studiomdl]
```

Several inputs may be given at once (drag-and-drop works); a folder decompiles
every `.mdl` under it, recursively.

| Option | Meaning |
| --- | --- |
| `-o <file>` | Script to write. Defaults to a folder named after the `.mdl`, next to it, holding the script and its meshes. Ignored with more than one model. |
| `-forceversion <n>` | Read the file as version `<n>` and ignore the header's version field. |
| `-dmxencoding <enc>` | How the `.dmx` meshes are encoded: `binary` (default) or `keyvalues2` text. |
| `-dmxmodel <n>` | The `format model` version they declare: 15 (default), 1, 18, or 22 (Source 2 ModelDoc). |
| `-smdanimation` | Write animation clips as `.smd` instead of `.dmx`. |
| `-studiomdl` | Write a stock-studiomdl `.qc` instead of a `.pulseqc`. |

`-forceversion` exists because some models carry a header version that does not
match their actual layout - a trick used to make them unreadable to decompilers.
Pointing it at the real version reads the file normally.

## Building

CMake + Ninja + MSVC, 64-bit only. Dependencies are vendored - no vcpkg, no
conan.

```
cmake --preset x64-release
cmake --build --preset x64-release
```

Both binaries land in `out/build/x64-release/PulseModel/`. Windows is the only supported and
tested target; the CMake files carry a Linux branch and the code avoids
Windows-only APIs, but that path is unverified.

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
- [V-HACD](https://github.com/kmammou/v-hacd) - approximate convex
  decomposition, used to generate collision from a render mesh. BSD-3-Clause.
- [ufbx](https://github.com/ufbx/ufbx) - FBX file reader, used to bring in FBX
  mesh sources alongside DMX/SMD. MIT.

Referenced, not vendored:

- [Datamodel.NET](https://github.com/Artfunkel/Datamodel.NET) (Artfunkel, MIT) -
  the reference implementation the C++ DMX reader was rewritten from, so the
  compiler stays a single native executable with no .NET dependency.
- [VPhysics-Jolt](https://github.com/misyltoad/VPhysics-Jolt) - read alongside
  V-HACD to work out the `.phy` / IVP collision format. `libs/minicollision`,
  the hull builder and `.phy` writer, is our own code written from that
  understanding.
- Valve's Source SDK 2013 - read as *documentation* of the file format.

## License

MIT. See [LICENSE](LICENSE).

# ag2proportions (experimental)

Builds a CS2 third-person AnimGraph2 wrapper that puts a custom model's bone
proportions on top of Valve's worldmodel graph.

The wrapper references the installed `animation/graphs/worldmodel/worldmodel.vnmgraph`,
applies a full-weight held additive proportion clip, then runs SnapWeapon. The
UI (`uimodel`) and first-person (`hudmodel`) entries keep their stock graphs. It
does not copy Valve's state machine, so changes inside that graph reach your
model automatically. If Valve changes the graph's control parameters or
skeleton, the tool refuses to build until the wrapper template is reviewed.

First-person arms are not touched. Author them on stock viewmodel proportions
and keep your viewmodel/UI graph entries as they are.

## Usage

Drag job files or `.vmdl` files onto `ag2proportions.exe`, or run:

```text
ag2proportions <job.kv3 | model.vmdl> [...] [--generate-only] [--compiler-runner wine]
```

A `.vmdl` on its own runs with every default. For anything else, write a job
file: KV3, with paths relative to the job file.

```text
<!-- kv3 encoding:text:version{e21c7f3c-8a33-41c5-9977-a76d3a32aa0d} format:generic:version{7412167c-06e9-4698-aff2-e63eb59037e7} -->
{
	vmdl = "agents/models/lainie/lainie.vmdl"
	model_dmx = "characters/lainie_shared/dmx/lainie.dmx"
	proportions = "characters/lainie_shared/dmx/proportions.dmx"
	vnmskel = "worldmodel.vnmskel_c"
	write_model = true
}
```

| Key | Use |
|---|---|
| `vmdl` | The model. Required. It must be inside `<CS2>/content/csgo_addons/<addon>/`. |
| `model_dmx` | The rig DMX. Default: the VMDL's `SkeletonFile` import, else the `RenderMeshFile` carrying the body bones, else none (a `Bone`-node skeleton). Needed when the VMDL has several `SkeletonFile`s. |
| `proportions` | A held pose with the bone positions you want. Default: the rig's bind pose. |
| `vnmskel` | A compiled `worldmodel.vnmskel_c`. Default: read from the installed `pak01` VPK. It must match the installed skeleton. |
| `write_model` | `true` to edit the VMDL in place: graph bindings, helper import or `Bone` nodes. The first edit keeps the original as `<name>.vmdl.bak`. |

| Option | Use |
|---|---|
| `--generate-only` | Write source assets without compiling them. |
| `--compiler-runner wine` | Linux: run the Windows resourcecompiler through Wine. |
| `--debug` | Also write a `debug/` folder for issue reports. |

The CS2 install is the one the addon lives in. Compiling needs CS2 Workshop
Tools; Valve's `resourcecompiler` runs headless, and the Workshop Tools UI is
not opened.

## Inputs

The rig is built the way ModelDoc builds it: the `SkeletonFile` DMX (or
`model_dmx`, or with neither, the body `RenderMeshFile` as Valve's agents do),
with the VMDL's `Bone` nodes merged on top. A `Bone` node sets a same-named file
bone's parent (its nesting) and its transform; with the `SkeletonFile`'s
`merge_behavior = "overwrite_existing"` the file keeps the transform instead.
The file's children stay attached, new names are added, and node order does not
matter. ModelDoc drops unweighted bones from a render mesh but keeps every
`SkeletonFile` bone.

| Setup | Result |
|---|---|
| `SkeletonFile` with every bone, CS2 names | Proportions from the file. |
| `SkeletonFile` with the body (pelvis to fingers, head), `root_motion`, IK bones and weapon helpers as `Bone` nodes | Proportions from the file; the nodes fill in the rest. The usual setup. |
| Render meshes carrying the body bones, `root_motion` and helpers as `Bone` nodes | Proportions from the body mesh. Valve's agents are built this way; `root_motion` must be a `Bone` node here. |
| `Bone` nodes only | Proportions from the nodes. |
| `Bone` nodes that also redefine body bones | Those bones take the node's values, and the tool names each one that moved. Pasting the stock agent's whole node tree gives stock proportions and a clip that changes nothing; the tool warns. |
| The stock agent's node tree pasted over a `SkeletonFile` with `merge_behavior = "overwrite_existing"` | Proportions from the file; the nodes only set the hierarchy and add missing bones. |
| No `root_motion` in the file or the nodes | Refused: add a `root_motion` Bone node. |

- **Bone names** must be the CS2 agent names (case is ignored) with stock
  parents. The 22 core bones are required; other stock bones are optional and
  keep their stock position. A pelvis with no parent keeps the stock
  horizontal origin and takes your height.
- **Weapon helpers:** the model needs `wpnPivot` (under `root_motion`) and `wpn`
  (under `wpnPivot`), from the file, `Bone` nodes or both; every shipped agent
  carries exactly these two. The graph's other eight helpers stay out of the
  model. A missing one goes into `model_with_helpers.dmx` when its parent is a
  `SkeletonFile` bone, otherwise it becomes a `Bone` node (listed in
  on the console, added by `write_model`).
- A separate **proportions DMX** only needs the 22 core bones minus
  `root_motion`.
- **The proportions pose** must be one clip with one log layer per position
  channel. Export a single held frame; multi-clip DMXs are refused.
- The clip only changes bone **translations**; rotations come from the stock
  skeleton. It adapts proportions, not skinning or bone axes.

## Output

Everything goes into a folder beside the VMDL, named after it. For
`models/lainie/lainie.vmdl` that is `models/lainie/lainie/`. A rerun overwrites
it.

| File | Contents |
|---|---|
| `proportions.vnmgraph`, `proportions.vnmclip` | The wrapper graph and the additive clip. |
| `proportions.dmx`, `reference.dmx` | The two held frames (30 FPS) the clip is built from. |
| `model_with_helpers.dmx` | Only when `wpnPivot`/`wpn` are missing under a `SkeletonFile` bone: a copy of that file with them added. |

The next steps (graph path, helper `Bone` nodes to add) and any warnings are
printed on the console. `--debug` adds a `debug/` folder to attach to issue
reports:

| File | Contents |
|---|---|
| `rig_merged.dmx` | The rig as the tool read it: the file with the `Bone` nodes merged in. |
| `skeleton_descriptor.kv3` | The stock skeleton source the clip compiles against. |
| `report.kv3` | Rig source and each bone's reference and target translation. |
| `MODEL_SETUP.txt` | Setup steps, attachment starting points and every rig note. |

Compiled `.vnmgraph_c` and `.vnmclip_c` go to the matching folder under
`game/csgo_addons/<addon>/`. Your original VMDL and DMX files are never
modified.

## ModelDoc setup

1. Point `DefaultAnimGraph2` and the worldmodel AnimGraph2 entry at the
   generated `proportions.vnmgraph`. Leave viewmodel/UI entries alone.
2. If `model_with_helpers.dmx` was written, point the `SkeletonFile` import at
   it; if the console lists helper `Bone` nodes, add them (`write_model`
   does both). Existing helpers and weights are kept.
3. Add any missing weapon attachments, using your rig's bone-name case. The
   tool warns when `weapon`, `weapon_hand_r` or `weapon_hand_l` is missing or on
   another bone; offsets are yours to tune.

   | Attachment | Parent | Origin | Angles |
   |---|---|---|---|
   | weapon | wpn | 0 0 0 | 0 0 0 |
   | weapon_hand_r | hand_R | -2.6 -1.4 0 | 0 180 0 |
   | weapon_hand_l | hand_L | 2.6 1.4 0 | 0 0 180 |

4. Compile the VMDL in ModelDoc and test in-game: pistol, rifle, knife,
   reload, crouch, aim pitch, movement and first-person arms.

> [!NOTE]
> A successful compile, or a clean look in Source 2 Viewer, does not prove the
> grip or IK is right in-game. `write_model` does not add attachments; check
> old AG1 attachment mappings by hand.

## Platforms

Windows and Linux run the same code. Linux can always `--generate-only`.
Compiling on Linux needs Workshop Tools binaries with the AG2 compilers;
neither native Linux nor Wine (which assumes Wine's default `Z:` drive mapping)
has been verified yet.

## Build and license

Built with the rest of PulseModel; set `PULSE_BUILD_AG2PROPORTIONS=OFF` to skip
it. The executable lands in `PulseModel/subtools/`.

PulseModel is MIT licensed. The bundled Zstandard decompressor is BSD-3-Clause;
its notice ships in `subtools/licenses/zstd.txt`.

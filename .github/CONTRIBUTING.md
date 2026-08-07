# Contributing to PulseModel

Thanks for looking. PulseModel is a small project with one maintainer, so the
most useful contributions are usually **good bug reports** - a full compile log
and the script that produced it. See the
[issue form](ISSUE_TEMPLATE/bug_report.yml) for what to include.

If you want to send code, read the rest of this page first. One rule below is
non-negotiable and will get a pull request closed unread.

## The one hard rule: clean-room provenance

**Do not contribute code copied or adapted from Valve's SDK, from studiomdl, or
from any studiomdl fork.** This project exists specifically so a Source model
compiler can ship under MIT, and a single pasted function would undo that.

That means:

- No pasted headers, structs, or function bodies from the Source SDK. Types are
  re-declared here by hand, in this project's own naming and style.
- No code lifted from studiomdl or a derivative of it, in any language.
- Reading a format specification and implementing it yourself is fine. Reading
  someone else's implementation and retyping it is not.

If you are unsure whether something you wrote crosses that line, say so in the
pull request. It is much easier to sort out before a merge than after.

By opening a pull request you confirm the code is yours to give and agree to it
being released under this project's MIT license.

## Building

See the Building section in the [README](../README.md). CMake + Ninja + MSVC,
64-bit, dependencies vendored under `libs/`.

## The other hard rule: don't regress the output

`.mdl`/`.vvd`/`.vtx`/`.phy` are strict about padding, alignment and string-table
ordering, so a change meant to be harmless usually is not.

There is **no test suite, no golden files and no `expected/` directory** in this
repository. Verification is:

1. It builds.
2. A real model compiles.
3. The result loads and looks right in-game.

For a refactor or a change to shared code, the strong check is a byte diff: build
the previous commit, compile the same model with both binaries, compare the
output. **A change billed as inert must move zero bytes.** If your change is
supposed to alter the output, say what changed and why in the pull request.

Say in the PR what you tested on - which model, which game, and which
`-vtxformat`. "Builds clean" is not a test.

## Scope and conventions

- **Every hard limit lives in `libs/pulselimits.h`** - no other file defines a
  limit constant. Limits reject only what the *file format* cannot express, not
  what a particular engine branch dislikes.
- **The layout is deliberately flat** - `libs/` and `mdlcompiler/` with few
  subfolders. A new folder or a new layer of indirection needs a reason beyond
  tidiness.
- **The script command surface is not stable yet.** Commands still get renamed
  and dropped between versions; nobody is running migration scripts, so changes
  land as clean edits rather than deprecation shims.
- C++17. Avoid Windows-only APIs in new code so a Linux build stays possible.

## Pull requests

- One feature or fix per pull request. A 400-line PR touching five unrelated
  things will sit unreviewed.
- Open an issue first for anything large - it may already be a deliberate
  omission rather than a gap.

## Code of conduct

Participation is covered by the [Code of Conduct](CODE_OF_CONDUCT.md).

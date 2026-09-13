#!/usr/bin/env python3
"""Merge the section blocks out of every commit since the last release.

Commit bodies use "<section>:" headers followed by "- " bullets; only the
mdlcompiler and mdldecompiler sections are kept. Bullets under any other header
(Misc, stray body lines) are dropped. "Bump version" bullets collapse to the
newest one.
"""

# Toppi: I am lazy writer.

import re
import sys

BULLET = re.compile(r"^\s*[-*]\s+(.*\S)\s*$")
BUMP = re.compile(r"bump\s+version", re.I)

# lowercase header -> display name; bullets under any other header are dropped.
SECTIONS = {"mdlcompiler": "mdlcompiler", "mdldecompiler": "mdldecompiler"}

sections = {}   # lowercase key -> [display name, [bullets]]
bumps = {}      # lowercase section key -> newest bump-version bullet


def add(key, text):
    entry = sections.setdefault(key.lower(), [key, []])
    if text not in entry[1]:
        entry[1].append(text)


# commits arrive oldest-first, each preceded by a NUL
for commit in sys.stdin.read().split("\0") if len(sys.argv) < 2 else \
        open(sys.argv[1], encoding="utf-8").read().split("\0"):
    lines = commit.strip("\n").split("\n")
    if not any(l.strip() for l in lines):
        continue
    section = None  # bullets before a known header are dropped
    for line in lines[1:]:  # line 0 is the subject
        if not line.strip():
            continue
        m = BULLET.match(line)
        if m:
            if section is None:
                continue
            if BUMP.search(m.group(1)):
                bumps[section.lower()] = (section, m.group(1))
            else:
                add(section, m.group(1))
        else:
            header = line.strip().rstrip(":").lower()
            section = SECTIONS.get(header)

for section, text in bumps.values():  # each tool versions independently
    add(section, text)

out = []
for name, items in sections.values():
    if not items:
        continue
    out.append(f"**{name}:**")
    out += [f"- {i}" for i in items]
    out.append("")
print("\n".join(out).strip() or "No changes.")

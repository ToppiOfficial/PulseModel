// importqc - stock studiomdl .qc -> .pulseqc rewriter.
//
// A text pass, not a compile: the commands whose shape changed are rewritten
// and every other line passes through untouched. The input file is never
// written to.
//
// Covered so far: $bodygroup / $body / $model -> $rendermesh + $modelgroup,
// plus the $model body options, which are top-level and global in .pulseqc
// ($flexcontroller, $flexrule, $flexlocalvar, $eyeball, $mouth, $eyelid, and
// the VTA flex list, which becomes a $rendermesh $vta block). $hboxset gathers
// the flat $hbox lines that follow it into its block. $include converts
// the file it names too (.qci -> .pulseqci) and re-points the path at the copy.

#ifndef PULSEMODEL_IMPORTQC_H
#define PULSEMODEL_IMPORTQC_H

#include <string>

namespace pulse::importqc {

// Convert the .qc at `in` and write the .pulseqc to `out`. Prints one line per
// rewritten command. False = nothing written, *err says why.
bool Convert(const std::string& in, const std::string& out, std::string* err);

// Where Convert writes by default: <dir>/<stem>.pulseqc (.pulseqci for a .qci),
// or <stem>.converted.<ext> when that would be the input file itself.
std::string DefaultOutput(const std::string& in);

} // namespace pulse::importqc

#endif // PULSEMODEL_IMPORTQC_H

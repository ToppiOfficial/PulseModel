// apioutput.h - -api records for the files a decompile writes.

#ifndef MDLDECOMPILER_APIOUTPUT_H
#define MDLDECOMPILER_APIOUTPUT_H

#include <string>

namespace mdldecompiler {

// Reports a written file as an @output record with its absolute path; a no-op
// outside -api. Called once the file is complete.
void RecordOutput(const std::string& path);

} // namespace mdldecompiler

#endif // MDLDECOMPILER_APIOUTPUT_H

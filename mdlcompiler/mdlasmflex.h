// mdlasmflex.h - the Flex Graph of a model assembly, turned into flex controllers, local vars and rules.

#ifndef PULSEMODEL_MDLASMFLEX_H
#define PULSEMODEL_MDLASMFLEX_H

#include <string>

#include "compile.h"
#include "mdlasmdoc.h"

namespace pulse::mdlasm {

// Registers the graph on `in`. Run once the model's morph descs exist, so a Morph Value or Flex Output
// can name them. A node that cannot compile is reported as a warning and its rule is left out.
bool RegisterFlexGraph(const Document& document, compile::CompileInput& in, std::string* err);

} // namespace pulse::mdlasm

#endif // PULSEMODEL_MDLASMFLEX_H

// skeletonedit.h - PulseMDL
//
// The `$skeletonedit { ... }` block of a `$rendermesh`: bone selection plus
// realign, collapse, create and scale, applied to the render mesh's private
// Source after `$meshedit`. See docs/skeletonedit.md.

#ifndef PULSEMDL_SKELETONEDIT_H
#define PULSEMDL_SKELETONEDIT_H

#include <string>
#include <vector>

#include "meshedit.h"
#include "source.h"

namespace pulse::source {

enum class SkeletonOperation {
    SelectBone, SelectBoneTree, SelectBoneSubstring, InvertSelection, SelectUnskinned,
    RealignBone, CollapseBone, CreateBone, ScaleBone
};

struct EditSkeletonOperation {
    SkeletonOperation kind = SkeletonOperation::InvertSelection;
    SelectionCombine combine = SelectionCombine::Add;
    std::string name;   // bone to select / create / scale, or the substring
    std::string parent; // $createbone; empty = no parent
    Vector3 origin{}, angles{};
    Vector3 scale{1.0f, 1.0f, 1.0f};
    bool hasOrigin = false, hasAngles = false, worldAxes = false, absolute = false;
    bool affectChildren = false, affectMeshes = false;
    bool keepParents = false;    // $selectunskinnedbones
    bool selectNew = false;      // $createbone
    bool ignoreSelectWarn = false;
    int line = 0;
};

// Each block starts with nothing selected. Returns false with *err / *errorLine set on failure.
bool ApplyEditSkeleton(Source& src, const std::vector<EditSkeletonOperation>& operations,
                       std::string* err, int* errorLine);

} // namespace pulse::source

#endif // PULSEMDL_SKELETONEDIT_H

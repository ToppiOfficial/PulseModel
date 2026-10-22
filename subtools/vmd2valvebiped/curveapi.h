#pragma once
#include <apiprotocol.h>
#include <array>
#include <cassert>
#include <iomanip>
#include <limits>
#include <stdexcept>

struct ApiOptions
{
    std::string method, face;
    std::vector<char> modelData; // the target .mdl sent in the request, instead of a path
    double fps = 30;
    matrix3x4 transform, orientation, distanceTransform;
    // Camera eyes stay inside [boundaryMin, boundaryMax] once mapped by boundaryTransform.
    bool boundary = false;
    Vector3 boundaryMin, boundaryMax;
    matrix3x4 boundaryTransform;
};

std::string Number(double value)
{
    if (!std::isfinite(value)) throw std::runtime_error("Non-finite converted animation value");

    std::ostringstream out;
    out << std::setprecision(17) << value;
    return out.str();
}

void ApiCapabilities()
{
    pulse::api::Record("capability", "{\"tool\":\"vmd2valvebiped\",\"curves\":1,\"motion\":1,\"camera\":1,\"boundary\":1}");
}

std::vector<float> ApiTimes(uint32_t last, double fps)
{
    const double duration = double(last) / 30;
    const double count = std::ceil(duration * fps);
    if (count >= 200000) throw std::runtime_error("Motion exceeds the conversion frame budget");

    std::vector<float> times;
    for (size_t i = 0; i <= size_t(count); ++i)
        times.push_back(float(std::min(double(i) / fps, duration) * 30));

    return times;
}

void Unwrap(std::vector<double>& values)
{
    for (size_t i = 1; i < values.size(); ++i)
        values[i] += 360 * std::round((values[i - 1] - values[i]) / 360);
}

std::vector<Vector3> ContinuousAngles(const std::vector<matrix3x4>& frames)
{
    std::vector<Vector3> result;
    for (const auto& frame : frames)
    {
        Vector3 angles, position;
        pm::MatrixAngles(frame, angles, position);
        angles = Mul(angles, pm::kRad2Deg);

        if (!result.empty())
        {
            Vector3 alternate{angles.x + 180, 180 - angles.y, angles.z + 180};
            for (int axis = 0; axis < 3; ++axis)
            {
                (&angles.x)[axis] += 360 * std::round(((&result.back().x)[axis] - (&angles.x)[axis]) / 360);
                (&alternate.x)[axis] += 360 * std::round(((&result.back().x)[axis] - (&alternate.x)[axis]) / 360);
            }
            if (Len(Sub(alternate, result.back())) < Len(Sub(angles, result.back()))) angles = alternate;
        }

        result.push_back(angles);
    }

    return result;
}

void EmitCurve(const char* property, const std::string& control, int component,
               const std::vector<float>& sourceTimes, std::vector<double> values, double tolerance,
               bool discrete = false, const std::set<size_t>& cuts = {})
{
    if (values.empty() || values.size() != sourceTimes.size()) throw std::runtime_error("Invalid converted curve");

    const size_t count = values.size();
    std::vector<double> times(count), slopes(count);
    for (size_t i = 0; i < count; ++i)
    {
        times[i] = double(sourceTimes[i]) / 30;
        if (!std::isfinite(values[i]) || (i && times[i] <= times[i - 1]))
            throw std::runtime_error("Invalid converted sample");
    }

    for (size_t i = 1; i + 1 < count; ++i)
    {
        const double a = (values[i] - values[i - 1]) / (times[i] - times[i - 1]);
        const double b = (values[i + 1] - values[i]) / (times[i + 1] - times[i]);
        slopes[i] = a * b > 0 ? std::copysign(std::min(std::abs(a), std::abs(b)), a) : 0;
    }
    if (count > 1)
    {
        slopes.front() = (values[1] - values[0]) / (times[1] - times[0]);
        slopes.back() = (values[count - 1] - values[count - 2]) / (times[count - 1] - times[count - 2]);
    }

    const auto sample = [&](size_t a, size_t b, size_t at)
    {
        if (discrete || cuts.count(b)) return values[a];
        const double duration = times[b] - times[a], t = (times[at] - times[a]) / duration, s = 1 - t;
        return s*s*s*values[a] + 3*s*s*t*(values[a] + slopes[a]*duration/3)
             + 3*s*t*t*(values[b] - slopes[b]*duration/3) + t*t*t*values[b];
    };

    std::set<size_t> keep{0, count - 1};
    for (auto cut : cuts)
    {
        keep.insert(cut);
        if (cut) keep.insert(cut - 1);
    }
    if (discrete) for (size_t i = 1; i < count; ++i)
        if (values[i] != values[i - 1]) { keep.insert(i); keep.insert(i - 1); }

    std::vector<std::pair<size_t, size_t>> pending;
    for (auto i = keep.begin(), j = i; ++j != keep.end(); i = j) pending.emplace_back(*i, *j);

    while (!pending.empty())
    {
        const auto [a, b] = pending.back();
        pending.pop_back();
        double error = tolerance;
        size_t split = a;
        for (size_t i = a + 1; i < b; ++i)
            if (const auto distance = std::abs(sample(a, b, i) - values[i]); distance > error) { error = distance; split = i; }

        if (split != a)
        {
            keep.insert(split);
            pending.emplace_back(a, split);
            pending.emplace_back(split, b);
        }
    }

    std::vector<size_t> keys(keep.begin(), keep.end());
    std::string json = "{\"property\":" + pulse::api::String(property) + ",\"control\":" + pulse::api::String(control)
        + ",\"component\":" + std::to_string(component) + ",\"base\":" + Number(values[0])
        + ",\"discrete\":" + (discrete ? "true" : "false") + ",\"keys\":[";
    for (size_t k = 0; k < keys.size(); ++k)
    {
        const size_t i = keys[k], previous = keys[k ? k - 1 : k], next = keys[std::min(k + 1, keys.size() - 1)];
        const double left = (times[i] - times[previous]) / 3, right = (times[next] - times[i]) / 3;
        if (k) json += ',';
        json += "{\"time\":" + Number(times[i]) + ",\"value\":" + Number(values[i])
            + ",\"interpolation\":" + ((discrete || cuts.count(next)) ? "0" : "2")
            + ",\"left\":{\"time\":" + Number(times[i] - left) + ",\"value\":" + Number(values[i] - slopes[i]*left) + ",\"type\":4}"
            + ",\"right\":{\"time\":" + Number(times[i] + right) + ",\"value\":" + Number(values[i] + slopes[i]*right) + ",\"type\":4}}";
        if (k + 1 < keys.size())
            for (size_t at = i + 1; at < next; ++at) assert(std::abs(sample(i, next, at) - values[at]) <= tolerance + 1e-8);
    }

    pulse::api::Record("channel", json + "]}");
}

// Moves each camera cut by one offset so its eye stays in the boundary, keeping the cut's motion
// and aim as authored. An axis the cut outgrows is centred and eased in near the walls instead.
void FitBoundary(std::vector<matrix3x4>& frames, const std::set<size_t>& cuts, const ApiOptions& api)
{
    const auto& m = api.boundaryTransform.m;
    const Vector3 rows[3] = {{m[0][0], m[0][1], m[0][2]}, {m[1][0], m[1][1], m[1][2]}, {m[2][0], m[2][1], m[2][2]}};
    const float determinant = Dot(rows[0], Cross(rows[1], rows[2]));
    if (!std::isfinite(determinant) || std::fabs(determinant) < 1e-12f)
        throw std::runtime_error("The camera boundary transform has a zero scale");

    // Columns of the inverse linear part, taking a boundary-space offset back to frame space
    const Vector3 columns[3] = {Mul(Cross(rows[1], rows[2]), 1 / determinant), Mul(Cross(rows[2], rows[0]), 1 / determinant),
                                Mul(Cross(rows[0], rows[1]), 1 / determinant)};

    std::vector<size_t> bounds{0};
    for (auto cut : cuts) if (cut > bounds.back() && cut < frames.size()) bounds.push_back(cut);
    bounds.push_back(frames.size());

    for (size_t s = 0; s + 1 < bounds.size(); ++s)
    {
        std::vector<Vector3> points;
        Vector3 low{std::numeric_limits<float>::max(), std::numeric_limits<float>::max(), std::numeric_limits<float>::max()};
        Vector3 high = Mul(low, -1);
        for (size_t i = bounds[s]; i < bounds[s + 1]; ++i)
        {
            points.push_back(pm::VectorTransform(Origin(frames[i]), api.boundaryTransform));
            for (int axis = 0; axis < 3; ++axis)
            {
                (&low.x)[axis] = std::min((&low.x)[axis], (&points.back().x)[axis]);
                (&high.x)[axis] = std::max((&high.x)[axis], (&points.back().x)[axis]);
            }
        }

        Vector3 shift;
        std::array<bool, 3> eased{};
        for (int axis = 0; axis < 3; ++axis)
        {
            const float minimum = (&api.boundaryMin.x)[axis], maximum = (&api.boundaryMax.x)[axis];
            const float lo = (&low.x)[axis], hi = (&high.x)[axis];
            eased[axis] = hi - lo > maximum - minimum;
            (&shift.x)[axis] = eased[axis] ? (minimum + maximum - lo - hi) / 2 : std::clamp(0.0f, minimum - lo, maximum - hi);
        }

        for (size_t k = 0; k < points.size(); ++k)
        {
            Vector3 fitted = Add(points[k], shift);
            for (int axis = 0; axis < 3; ++axis)
            {
                if (!eased[axis]) continue;
                // Unchanged in the middle half of the box, then a tanh knee that never reaches the wall
                const float minimum = (&api.boundaryMin.x)[axis], maximum = (&api.boundaryMax.x)[axis];
                const float knee = (maximum - minimum) / 4;
                float& value = (&fitted.x)[axis];
                if (value > maximum - knee) value = maximum - knee + knee * std::tanh((value - maximum + knee) / knee);
                else if (value < minimum + knee) value = minimum + knee - knee * std::tanh((minimum + knee - value) / knee);
            }

            const Vector3 offset = Sub(fitted, points[k]);
            const Vector3 moved = Add(Add(Mul(columns[0], offset.x), Mul(columns[1], offset.y)), Mul(columns[2], offset.z));
            SetOrigin(frames[bounds[s] + k], Add(Origin(frames[bounds[s] + k]), moved));
        }
    }
}

int EmitCameraApi(const ApiOptions& api, const std::string& path, const matrix3x4& basis, Vector3 anchor, float scale)
{
    std::map<std::string, std::vector<Key>> bones;
    Morphs morphs;
    std::vector<CamKey> cameras;
    uint32_t last = 0;
    std::string error;
    if (!LoadVmd(path, bones, morphs, cameras, last, error)) return Fail(error);
    if (cameras.empty()) return Fail("The VMD has no camera keys");

    auto times = ApiTimes(cameras.back().frame, api.fps);
    for (size_t i = 1; i < cameras.size(); ++i)
        if (cameras[i].frame - cameras[i-1].frame <= 1 || cameras[i].orthographic != cameras[i-1].orthographic)
        {
            times.push_back(float(cameras[i-1].frame));
            times.push_back(float(cameras[i].frame));
        }
    std::sort(times.begin(), times.end());
    times.erase(std::unique(times.begin(), times.end()), times.end());
    if (times.size() > 200000) return Fail("Camera motion exceeds the conversion sample budget");

    std::set<size_t> cuts;
    std::vector<matrix3x4> frames;
    std::vector<double> fovs, projection, distance;
    for (float time : times)
    {
        Vector3 eye, forward, up;
        float fov, radius;
        SampleCamera(cameras, time, eye, forward, up, fov, &radius);
        const auto focus = Add(eye, Mul(forward, radius));
        const auto current = std::upper_bound(cameras.begin(), cameras.end(), time,
            [](float value, const CamKey& key) { return value < key.frame; });
        const auto& key = current == cameras.begin() ? cameras.front() : *std::prev(current);
        const auto sourceIndex = size_t(&key - cameras.data());
        if (sourceIndex && key.frame == time && key.frame - cameras[sourceIndex-1].frame <= 1) cuts.insert(frames.size());

        forward = pm::VectorRotate(pm::VectorRotate(forward, basis), api.orientation);
        up = pm::VectorRotate(pm::VectorRotate(up, basis), api.orientation);
        pm::VectorNormalize(forward);
        Vector3 right = Cross(forward, up);
        pm::VectorNormalize(right);
        up = Cross(right, forward);

        matrix3x4 frame;
        for (int axis = 0; axis < 3; ++axis)
        {
            frame.m[axis][0] = (&right.x)[axis];
            frame.m[axis][1] = (&up.x)[axis];
            frame.m[axis][2] = -(&forward.x)[axis];
        }
        SetOrigin(frame, pm::VectorTransform(Add(anchor, Mul(pm::VectorRotate(eye, basis), scale)), api.transform));
        frames.push_back(frame);
        fovs.push_back(std::clamp(double(fov), 1.0, 179.0));
        projection.push_back(key.orthographic ? 1 : 0);

        const auto at = pm::VectorTransform(Add(anchor, Mul(pm::VectorRotate(focus, basis), scale)), api.distanceTransform);
        const auto worldEye = pm::VectorTransform(Add(anchor, Mul(pm::VectorRotate(eye, basis), scale)), api.distanceTransform);
        distance.push_back(std::max(0.001, double(Len(Sub(worldEye, at)))));
    }

    if (api.boundary) FitBoundary(frames, cuts, api);
    pulse::api::Record("clip", "{\"duration\":" + Number(double(cameras.back().frame)/30) + ",\"sourceFps\":30}");

    const auto angles = ContinuousAngles(frames);
    for (int axis = 0; axis < 3; ++axis)
    {
        std::vector<double> position, rotation;
        for (size_t i = 0; i < frames.size(); ++i)
        {
            position.push_back(frames[i].m[axis][3]);
            rotation.push_back((&angles[i].x)[axis]);
        }
        EmitCurve("Position", {}, axis, times, position, 0.05, false, cuts);
        EmitCurve("Rotation", {}, axis, times, rotation, 0.1, false, cuts);
    }
    EmitCurve("FOV", {}, 0, times, fovs, 0.1, false, cuts);
    EmitCurve("Orthographic", {}, 0, times, projection, 0, true);
    EmitCurve("Ortho distance", {}, 0, times, distance, 0.05, false, cuts);
    return 0;
}

void EmitMotionApi(const Mdl& model, const Target& target, const std::vector<float>& times,
                   const std::vector<std::vector<AnimPose>>& frames,
                   const std::vector<mdldecompiler::FlexTrack>& flex, bool body)
{
    pulse::api::Record("clip", "{\"duration\":" + Number(double(times.back())/30) + ",\"sourceFps\":30}");
    const auto* bones = model.At<fm::mstudiobone_t>(model.buf.data(), model.hdr->boneindex, model.hdr->numbones);

    if (body) for (int bone = 0; bone < target.n; ++bone)
    {
        if (bones[bone].proctype) continue;
        std::vector<matrix3x4> local;
        for (const auto& frame : frames)
        {
            matrix3x4 matrix;
            pm::AngleMatrix(frame[bone].rot, frame[bone].pos, matrix);
            local.push_back(matrix);
        }

        const auto angles = ContinuousAngles(local);
        for (int axis = 0; axis < 3; ++axis)
        {
            std::vector<double> position, rotation;
            for (size_t i = 0; i < frames.size(); ++i)
            {
                position.push_back((&frames[i][bone].pos.x)[axis]);
                rotation.push_back((&angles[i].x)[axis]);
            }
            EmitCurve("Bone position", target.names[bone], axis, times, position, 0.05);
            EmitCurve("Bone rotation", target.names[bone], axis, times, rotation, 0.1);
        }
    }

    if (body)
    {
        const auto* chains = model.At<fm::mstudioikchain_t>(model.buf.data(), model.hdr->ikchainindex, model.hdr->numikchains);
        for (int c = 0; chains && c < model.hdr->numikchains; ++c)
        {
            if (chains[c].numlinks != 3) continue;
            const auto* links = model.At<fm::mstudioiklink_t>(&chains[c], chains[c].linkindex, 3);
            if (!links) continue;

            bool valid = true;
            for (int i = 0; i < 3; ++i)
                valid &= links[i].bone >= 0 && links[i].bone < target.n && !bones[std::clamp(links[i].bone, 0, target.n-1)].proctype;
            if (!valid) continue;

            std::array<std::vector<double>, 3> endpoints, poles;
            for (const auto& frame : frames)
            {
                std::vector<matrix3x4> world(target.n);
                for (int b = 0; b < target.n; ++b)
                {
                    matrix3x4 local;
                    pm::AngleMatrix(frame[b].rot, frame[b].pos, local);
                    world[b] = target.parent[b] >= 0 ? pm::ConcatTransforms(world[target.parent[b]], local) : local;
                }

                const auto root = Origin(world[links[0].bone]), mid = Origin(world[links[1].bone]), end = Origin(world[links[2].bone]);
                auto axis = Sub(end, root);
                pm::VectorNormalize(axis);
                auto bend = Sub(Sub(mid, root), Mul(axis, Dot(Sub(mid, root), axis)));
                if (pm::VectorNormalize(bend) < 1e-5f)
                {
                    bend = pm::VectorRotate(links[0].kneeDir, world[links[0].bone]);
                    if (pm::VectorNormalize(bend) < 1e-5f) bend = Cross(axis, std::fabs(axis.z) < 0.9f ? Vector3{0,0,1} : Vector3{0,1,0});
                }

                const auto pole = Add(mid, Mul(bend, std::max(1.0f, Len(Sub(mid, root)))));
                for (int a = 0; a < 3; ++a)
                {
                    endpoints[a].push_back((&end.x)[a]);
                    poles[a].push_back((&pole.x)[a]);
                }
            }

            const auto name = model.Str(&chains[c], chains[c].sznameindex);
            for (int a = 0; a < 3; ++a)
            {
                EmitCurve("IK target", name, a, times, endpoints[a], 0.05);
                EmitCurve("IK pole", name, a, times, poles[a], 0.05);
            }
        }
    }

    const bool gaze = std::any_of(flex.begin(), flex.end(), [](const auto& channel) {
        return !_stricmp(channel.name.c_str(), "eyes_updown") || !_stricmp(channel.name.c_str(), "eyes_rightleft");
    });
    if (gaze) EmitCurve("Eyes follow camera", {}, 0, times, std::vector<double>(times.size(), 0), 0, true);

    for (const auto& channel : flex)
    {
        std::vector<double> values;
        for (auto value : channel.values)
            values.push_back(channel.max > channel.min ? std::clamp(double(value-channel.min)/(channel.max-channel.min), 0.0, 1.0) : 0);
        EmitCurve("Flex", channel.name, 0, times, values, 0.001);
    }
}

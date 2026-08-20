// perf.h - -perfmetrics timing. Scoped timers record their wall time; the
// whole table prints once at the end of the compile (pulse::perf::Report).

#ifndef PULSEMDL_PERF_H
#define PULSEMDL_PERF_H

#include <chrono>

namespace pulse::perf {

extern bool g_enabled; // -perfmetrics (defined in main.cpp)

// Repeat entries under the same tag+name are summed and counted.
void Record(const char* tag, const char* name, double ms);
void Report();

struct Timer {
    const char* tag;
    const char* name;
    std::chrono::steady_clock::time_point t0;
    Timer(const char* tg, const char* n)
        : tag(tg), name(n), t0(std::chrono::steady_clock::now()) {}
    ~Timer() {
        if (g_enabled)
            Record(tag, name,
                   std::chrono::duration<double, std::milli>(
                       std::chrono::steady_clock::now() - t0).count());
    }
};

} // namespace pulse::perf

#define PULSE_PERF_CAT2(a, b) a##b
#define PULSE_PERF_CAT(a, b) PULSE_PERF_CAT2(a, b)
#define PULSE_PERF(tag, name) \
    pulse::perf::Timer PULSE_PERF_CAT(pulse_perf_, __LINE__)(tag, name)

#endif // PULSEMDL_PERF_H

// perf.h - -perfmetrics timing. Scoped timers record their wall time; the whole
// table prints once at the end of a run (pulse::perf::Report).

#ifndef PULSEMDL_PERF_H
#define PULSEMDL_PERF_H

#include <chrono>
#include <cstdio>
#include <cstring>
#include <vector>

namespace pulse::perf {

inline bool g_enabled = false; // -perfmetrics

struct Entry {
    const char* tag;
    const char* name;
    double ms = 0;
    int calls = 0;
};
inline std::vector<Entry> g_entries;

// Repeat entries under the same tag+name are summed and counted.
inline void Record(const char* tag, const char* name, double ms) {
    for (Entry& e : g_entries) {
        if (std::strcmp(e.tag, tag) == 0 && std::strcmp(e.name, name) == 0) {
            e.ms += ms;
            e.calls++;
            return;
        }
    }
    g_entries.push_back({tag, name, ms, 1});
}

// One table at the end, grouped by tag in the order the tags first appeared.
// A stage under 1 ms is dropped - it is noise next to the ones that matter.
inline void Report() {
    if (!g_enabled)
        return;
    std::printf("---------------------\n-perfmetrics\n");
    std::vector<const char*> tags;
    for (const Entry& e : g_entries) {
        bool seen = false;
        for (const char* t : tags)
            seen = seen || std::strcmp(t, e.tag) == 0;
        if (!seen)
            tags.push_back(e.tag);
    }
    for (const char* tag : tags) {
        std::printf("  [%s]\n", tag);
        for (const Entry& e : g_entries) {
            if (std::strcmp(e.tag, tag) != 0 || e.ms < 1.0)
                continue;
            if (e.calls > 1)
                std::printf("    %-30s %9.1f ms  (%d calls)\n", e.name, e.ms, e.calls);
            else
                std::printf("    %-30s %9.1f ms\n", e.name, e.ms);
        }
    }
}

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

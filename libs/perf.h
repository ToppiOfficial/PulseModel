// perf.h - -perfmetrics timing. Scoped timers record their wall time; the whole
// table prints once at the end of a run (pulse::perf::Report).

#ifndef PULSEMDL_PERF_H
#define PULSEMDL_PERF_H

#include <chrono>
#include <cstdio>
#include <algorithm>
#include <cstring>
#include <mutex>
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
inline std::vector<const char*> g_sortedTags;
inline std::mutex g_mutex;

// Repeat entries under the same tag+name are summed and counted. Thread-safe.
inline void Record(const char* tag, const char* name, double ms) {
    std::lock_guard<std::mutex> lock(g_mutex);
    for (Entry& e : g_entries) {
        if (std::strcmp(e.tag, tag) == 0 && std::strcmp(e.name, name) == 0) {
            e.ms += ms;
            e.calls++;
            return;
        }
    }
    g_entries.push_back({tag, name, ms, 1});
}

// The tag prints slowest first instead of in first-recorded order.
inline void SortTag(const char* tag) {
    std::lock_guard<std::mutex> lock(g_mutex);
    for (const char* t : g_sortedTags)
        if (std::strcmp(t, tag) == 0)
            return;
    g_sortedTags.push_back(tag);
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
        std::vector<const Entry*> rows;
        for (const Entry& e : g_entries)
            if (std::strcmp(e.tag, tag) == 0 && e.ms >= 1.0)
                rows.push_back(&e);
        const bool sorted = std::any_of(g_sortedTags.begin(), g_sortedTags.end(),
                                        [tag](const char* t) { return std::strcmp(t, tag) == 0; });
        if (sorted)
            std::stable_sort(rows.begin(), rows.end(),
                             [](const Entry* a, const Entry* b) { return a->ms > b->ms; });
        for (const Entry* e : rows) {
            if (e->calls > 1)
                std::printf("    %-30s %9.1f ms  (%d calls)\n", e->name, e->ms, e->calls);
            else
                std::printf("    %-30s %9.1f ms\n", e->name, e->ms);
        }
    }
}

struct Timer {
    const char* tag;
    const char* name;
    std::chrono::steady_clock::time_point t0;
    Timer(const char* tg, const char* n)
        : tag(tg), name(n), t0(std::chrono::steady_clock::now()) {}
    double Elapsed() const {
        return std::chrono::duration<double, std::milli>(std::chrono::steady_clock::now() - t0)
            .count();
    }
    // Records now instead of at scope exit, for a span that ends mid-scope.
    void Stop() {
        if (g_enabled && name)
            Record(tag, name, Elapsed());
        name = nullptr;
    }
    ~Timer() { Stop(); }
};

} // namespace pulse::perf

#define PULSE_PERF_CAT2(a, b) a##b
#define PULSE_PERF_CAT(a, b) PULSE_PERF_CAT2(a, b)
#define PULSE_PERF(tag, name) \
    pulse::perf::Timer PULSE_PERF_CAT(pulse_perf_, __LINE__)(tag, name)

#endif // PULSEMDL_PERF_H

// l4d_animswap - generate a $declaresequence swap script for L4D2 survivor mods.
//
// A survivor mod must keep vanilla's sequence indices or client and server
// disagree about which animation is playing. This walks vanilla's own
// $includemodel chain to recover that index order, then renames each slot to
// the matching animation from a donor survivor so the donor's include claims it.

#include <algorithm>
#include <cctype>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <map>
#include <set>
#include <string>
#include <utility>
#include <vector>

#include "format/mdl.h"

namespace fm = pulse::format;

namespace {

// Survivor name -> the codename Valve embeds in file and sequence names. Group
// picks the default donor chain: swapping inside a build-alike set keeps the
// retarget from stretching limbs.
struct Survivor {
    const char* survivor;
    const char* code;
    int group; // 0 female, 1 male
};
const Survivor kSurvivors[] = {
    {"zoey", "teenangst", 0}, {"rochelle", "producer", 0},
    {"bill", "namvet", 1},    {"francis", "biker", 1},
    {"louis", "manager", 1},  {"coach", "coach", 1},
    {"nick", "gambler", 1},   {"ellis", "mechanic", 1},
};

std::string Lower(std::string s) {
    for (char& c : s) c = static_cast<char>(std::tolower(static_cast<unsigned char>(c)));
    return s;
}

const Survivor* FindSurvivor(const std::string& name) {
    const std::string want = Lower(name);
    for (const Survivor& h : kSurvivors)
        if (want == h.survivor || want == h.code) return &h;
    return nullptr;
}

std::set<std::string> CodeTokens() {
    std::set<std::string> s;
    for (const Survivor& h : kSurvivors) s.insert(h.code);
    return s;
}

std::string Leaf(const std::string& path) {
    const size_t slash = path.find_last_of("/\\");
    return slash == std::string::npos ? path : path.substr(slash + 1);
}

// ---------------------------------------------------------------- mdl reading

struct Mdl {
    std::vector<char> bytes;
    std::vector<std::string> seqs;
    std::vector<std::string> acts; // "ACT_X <weight>", empty when the slot has none
    std::vector<std::string> includes;
};

bool LoadMdl(const std::string& path, Mdl& out) {
    FILE* f = std::fopen(path.c_str(), "rb");
    if (!f) return false;
    std::fseek(f, 0, SEEK_END);
    const long len = std::ftell(f);
    std::fseek(f, 0, SEEK_SET);
    out.bytes.resize(len > 0 ? static_cast<size_t>(len) : 0);
    const size_t got = out.bytes.empty() ? 0 : std::fread(out.bytes.data(), 1, out.bytes.size(), f);
    std::fclose(f);
    if (got != out.bytes.size() || out.bytes.size() < sizeof(fm::studiohdr_t)) return false;

    const char* base = out.bytes.data();
    const auto* hdr = reinterpret_cast<const fm::studiohdr_t*>(base);
    if (hdr->id != fm::kIdStudioHeader) return false;

    const auto str = [&](size_t off) {
        return off < out.bytes.size() ? std::string(base + off) : std::string();
    };
    for (int i = 0; i < hdr->numlocalseq; ++i) {
        const size_t at = static_cast<size_t>(hdr->localseqindex) + i * sizeof(fm::mstudioseqdesc_t);
        if (at + sizeof(fm::mstudioseqdesc_t) > out.bytes.size()) break;
        const auto* sd = reinterpret_cast<const fm::mstudioseqdesc_t*>(base + at);
        out.seqs.push_back(str(at + static_cast<size_t>(sd->szlabelindex)));
        std::string act = sd->szactivitynameindex
                              ? str(at + static_cast<size_t>(sd->szactivitynameindex))
                              : std::string();
        if (!act.empty()) act += " " + std::to_string(sd->actweight);
        out.acts.push_back(act);
    }
    for (int i = 0; i < hdr->numincludemodels; ++i) {
        const size_t at =
            static_cast<size_t>(hdr->includemodelindex) + i * sizeof(fm::mstudiomodelgroup_t);
        if (at + sizeof(fm::mstudiomodelgroup_t) > out.bytes.size()) break;
        const auto* mg = reinterpret_cast<const fm::mstudiomodelgroup_t*>(base + at);
        if (mg->sznameindex) out.includes.push_back(str(at + static_cast<size_t>(mg->sznameindex)));
    }
    return true;
}

// One survivor's resolved slot list: the name at each index and, for the
// model's own leading slots, the activity a mod must re-declare there.
struct Slots {
    std::vector<std::string> names;
    std::vector<std::string> acts;
};

// The engine indexes sequences as: the model's own locals, then each
// $includemodel in order, skipping names already taken. Replaying that walk
// recovers the slot order a mod has to line up with.
struct Resolver {
    std::string dir;
    std::map<std::string, Mdl> cache;
    bool missing = false;

    const Mdl* Get(const std::string& modelPath) {
        const std::string leaf = Leaf(modelPath);
        const std::string key = Lower(leaf);
        auto it = cache.find(key);
        if (it != cache.end()) return it->second.bytes.empty() ? nullptr : &it->second;
        Mdl m;
        if (!LoadMdl(dir + "/" + leaf, m)) {
            std::fprintf(stderr, "  cannot read %s\n", leaf.c_str());
            missing = true;
            cache[key];
            return nullptr;
        }
        return &(cache[key] = std::move(m));
    }

    void Walk(const Mdl& m, Slots& order, std::set<std::string>& seen) {
        for (size_t i = 0; i < m.seqs.size(); ++i) {
            if (!seen.insert(Lower(m.seqs[i])).second) continue;
            order.names.push_back(m.seqs[i]);
            order.acts.push_back(i < m.acts.size() ? m.acts[i] : std::string());
        }
        for (const std::string& inc : m.includes)
            if (const Mdl* sub = Get(inc)) Walk(*sub, order, seen);
    }

    Slots Order(const Survivor& h) {
        Slots order;
        std::set<std::string> seen;
        if (const Mdl* m = Get("survivor_" + std::string(h.code) + ".mdl")) Walk(*m, order, seen);
        return order;
    }
};

// -------------------------------------------------------------- name matching

// Two keys per name: the codename token swapped for a wildcard, and the same
// with it dropped. The second catches sets that spell one animation both with
// and without the character prefix (coach_Run_Pistol vs Run_Pistol).
void Keys(const std::string& name, const std::set<std::string>& codes, std::string& swapped,
          std::string& stripped) {
    swapped.clear();
    stripped.clear();
    const std::string low = Lower(name);
    for (size_t i = 0;;) {
        const size_t end = low.find('_', i);
        const std::string tok = low.substr(i, (end == std::string::npos ? low.size() : end) - i);
        const bool isCode = codes.count(tok) != 0;
        if (!swapped.empty()) swapped += '_';
        swapped += isCode ? "@" : tok;
        if (!isCode) {
            if (!stripped.empty()) stripped += '_';
            stripped += tok;
        }
        if (end == std::string::npos) break;
        i = end + 1;
    }
}

struct Index {
    std::map<std::string, std::string> swapped, stripped;

    void Build(const std::vector<std::string>& names, const std::string& own,
               const std::set<std::string>& codes) {
        std::set<std::string> ownSw, ownSt;
        std::string sw, st;
        for (const std::string& n : names) {
            Keys(n, codes, sw, st);
            // A set carries some animations under another survivor's name
            // (anim_biker.mdl has NamVet_* entries). Own spelling wins the key.
            const bool mine = Lower(n).find(own) != std::string::npos;
            if (!swapped.count(sw) || (mine && !ownSw.count(sw))) swapped[sw] = n;
            if (!stripped.count(st) || (mine && !ownSt.count(st))) stripped[st] = n;
            if (mine) {
                ownSw.insert(sw);
                ownSt.insert(st);
            }
        }
    }
};

// ------------------------------------------------------------------ resolution

struct Result {
    std::vector<std::string> names;
    std::vector<int> from; // chain index, or -1 for "kept the vanilla name"
};

// A donor name may fill only one slot: a repeated $declaresequence compiles to
// an empty sequence, so a second claimant drops to the next donor instead.
Result Build(const std::vector<std::string>& order, const std::vector<Index>& idx,
             const std::set<std::string>& codes) {
    Result r;
    std::set<std::string> taken;
    std::string sw, st;
    for (const std::string& v : order) {
        Keys(v, codes, sw, st);
        std::string pick;
        int from = -1;
        for (size_t d = 0; d < idx.size() && pick.empty(); ++d) {
            const std::map<std::string, std::string>* tabs[2] = {&idx[d].swapped, &idx[d].stripped};
            const std::string* keys[2] = {&sw, &st};
            for (int k = 0; k < 2; ++k) {
                const auto it = tabs[k]->find(*keys[k]);
                if (it != tabs[k]->end() && !taken.count(Lower(it->second))) {
                    pick = it->second;
                    from = static_cast<int>(d);
                    break;
                }
            }
        }
        if (pick.empty()) pick = v;
        taken.insert(Lower(pick));
        r.names.push_back(pick);
        r.from.push_back(from);
    }
    return r;
}

// --------------------------------------------------------------------- output

bool Write(const std::string& path, const Survivor& rig, const Survivor& tgt,
           const std::vector<const Survivor*>& chain, const Result& res, int skip, const Slots& order,
           Resolver& mdls, const std::string& ref) {
    std::vector<int> won(chain.size(), 0);
    int kept = 0;
    for (size_t i = static_cast<size_t>(skip); i < res.from.size(); ++i) {
        if (res.from[i] < 0) ++kept;
        else ++won[static_cast<size_t>(res.from[i])];
    }

    // Every name the emitted includes can actually supply. A slot outside this
    // set is one vanilla kept local, so a declare there compiles empty.
    // Which files a survivor pulls in is his own model's business - Louis has no
    // gestures_manager.mdl, he rides Francis's. Take the list from vanilla.
    const auto includesOf = [&](const Survivor& h) {
        const Mdl* m = mdls.Get("survivor_" + std::string(h.code) + ".mdl");
        return m ? m->includes : std::vector<std::string>();
    };
    std::set<std::string> reachable;
    const auto reach = [&](const Survivor& h) {
        for (const std::string& inc : includesOf(h))
            if (const Mdl* m = mdls.Get(inc))
                for (const std::string& s : m->seqs) reachable.insert(Lower(s));
    };
    for (size_t d = 0; d < chain.size(); ++d)
        if (won[d]) reach(*chain[d]);
    reach(tgt);

    int local = 0;
    for (size_t i = static_cast<size_t>(skip); i < res.names.size(); ++i)
        if (!reachable.count(Lower(res.names[i]))) ++local;

    FILE* f = std::fopen(path.c_str(), "wb");
    if (!f) {
        std::fprintf(stderr, "cannot write %s\n", path.c_str());
        return false;
    }
    std::fprintf(f, "// l4d_animswap: a %s-rigged model filling %s's sequence slots.\n", rig.survivor,
                 tgt.survivor);
    // An included sequence carries its own activity, so only the slots the
    // model defines itself need one spelled out.
    std::fprintf(f, "// %d slots below. The first %d are the model's own - vanilla put these\n"
                    "// there, and your script must define %s in the same order:\n",
                 static_cast<int>(res.names.size()) - skip, skip, skip == 1 ? "it" : "them");
    for (int i = 0; i < skip && i < static_cast<int>(order.names.size()); ++i) {
        const std::string& act = order.acts[static_cast<size_t>(i)];
        std::fprintf(f, "//   [%d] %-24s REPLACE ME - your own $sequence", i,
                     order.names[static_cast<size_t>(i)].c_str());
        if (!act.empty()) std::fprintf(f, ", keeping activity %s", act.c_str());
        std::fputc('\n', f);
    }
    for (size_t d = 0; d < chain.size(); ++d)
        std::fprintf(f, "// %-10s %5d slots\n", chain[d]->survivor, won[d]);
    // With no donors the whole list is the target's own, so naming each is noise.
    std::fprintf(f, chain.empty() ? "// %-10s %5d slots - vanilla's own list, nothing swapped\n"
                                  : "// %-10s %5d slots - no donor equivalent, these keep %s\n",
                 tgt.survivor, kept, tgt.survivor);
    if (!chain.empty())
        for (size_t i = static_cast<size_t>(skip); i < res.from.size(); ++i)
            if (res.from[i] < 0) std::fprintf(f, "//   %s\n", res.names[i].c_str());
    if (local)
        std::fprintf(f, ref.empty()
                            ? "// %d further slot(s) below are marked REPLACE ME - vanilla keeps\n"
                              "// those local, so no $includemodel can fill them.\n"
                            : "// %d further slot(s) below are emitted as local $sequence - no\n"
                              "// $includemodel can fill them.\n",
                     local);
    std::fputc('\n', f);

    // The skipped slots come first in the file too, so mark where they belong.
    for (int i = 0; i < skip && i < static_cast<int>(order.names.size()); ++i) {
        const std::string& act = order.acts[static_cast<size_t>(i)];
        std::fprintf(f, "// [%d] %s here - your own $sequence%s%s\n", i,
                     order.names[static_cast<size_t>(i)].c_str(), act.empty() ? "" : ", activity ",
                     act.c_str());
    }
    if (skip) std::fputc('\n', f);

    bool prevLocal = false;
    for (size_t i = static_cast<size_t>(skip); i < res.names.size(); ++i) {
        if (reachable.count(Lower(res.names[i]))) {
            if (prevLocal) std::fputc('\n', f); // keep the local runs easy to spot
            std::fprintf(f, "$declaresequence \"%s\"\n", res.names[i].c_str());
            prevLocal = false;
            continue;
        }
        // Vanilla keeps this slot local, so a declare here compiles empty. With
        // a reference sequence to point at we can emit the local outright.
        const std::string& act = order.acts[i];
        if (!prevLocal) std::fputc('\n', f);
        prevLocal = true;
        if (ref.empty()) {
            std::fprintf(f, "// REPLACE ME - slot %d. No $includemodel provides \"%s\";\n"
                            "// vanilla defines it locally%s%s. Put your own $sequence here\n"
                            "// instead of this declare, or the slot compiles empty.\n"
                            "$declaresequence \"%s\"\n",
                         static_cast<int>(i), res.names[i].c_str(),
                         act.empty() ? "" : " with activity ", act.c_str(), res.names[i].c_str());
            continue;
        }
        std::fprintf(f, "$sequence %s \"%s\" FPS 30", res.names[i].c_str(), ref.c_str());
        if (!act.empty()) std::fprintf(f, " activity %s", act.c_str());
        std::fprintf(f, "   // slot %d - no $includemodel provides this\n", static_cast<int>(i));
    }

    std::fputc('\n', f);
    const auto include = [&](const Survivor& h) {
        for (const std::string& inc : includesOf(h))
            std::fprintf(f, "$includemodel \"survivors/%s\"\n", Leaf(inc).c_str());
        std::fputc('\n', f);
    };
    for (size_t d = 0; d < chain.size(); ++d)
        if (won[d]) include(*chain[d]); // a donor that filled nothing is dead weight
    include(tgt);                       // last resort for every slot no donor claimed
    std::fclose(f);

    std::printf("%s\n", path.c_str());
    for (size_t d = 0; d < chain.size(); ++d) std::printf("    %-10s %5d\n", chain[d]->survivor, won[d]);
    std::printf("    %-10s %5d (fallback)\n", tgt.survivor, kept);
    return true;
}

void PrintHeader() {
    std::printf("-------------------------------\n");
    std::printf("PulseModel [L4D2 Anim Swap]\n");
    std::printf("version:   %s\n", PULSEMODEL_VERSION);
    std::printf("developer: Toppi\n");
    std::printf("-------------------------------\n");
}

void Usage() {
    std::printf(
        "L4D2 survivor $declaresequence swap generator\n\n"
        "  l4d_animswap -vanilla <dir> -rig <name> -replace <name> [options]\n"
        "  l4d_animswap -vanilla <dir> -all [-outdir <dir>]\n\n"
        "  -vanilla <dir>  folder with vanilla survivor_*.mdl, anim_*.mdl, gestures_*.mdl\n"
        "  -rig <name>     survivor whose skeleton the model is rigged to (first donor)\n"
        "  -replace <name> survivor whose sequence indices must be matched\n"
        "  -via <a,b,...>  donors tried after -rig; default is the rig's build-alike group\n"
        "  -skip <n>       leading slots the model defines itself (default 2)\n"
        "  -ref <name>     source clip for slots no include can fill; emits a real\n"
        "                  $sequence there instead of a declare that compiles empty\n"
        "  -o <file>       output path (default Anims_RigTo<Rig>_Replace<Target>.qci)\n"
        "  -all            every rig x replace pair, written into -outdir\n"
        "  -selftest       run the name-matching checks and exit\n\n"
        "  survivors: bill francis louis zoey coach nick ellis rochelle\n");
}

std::vector<const Survivor*> ParseVia(const std::string& arg, bool& ok) {
    std::vector<const Survivor*> v;
    ok = true;
    for (size_t i = 0;;) {
        const size_t e = arg.find(',', i);
        const std::string one = arg.substr(i, (e == std::string::npos ? arg.size() : e) - i);
        if (!one.empty()) {
            if (const Survivor* h = FindSurvivor(one)) v.push_back(h);
            else {
                std::fprintf(stderr, "unknown survivor '%s'\n", one.c_str());
                ok = false;
            }
        }
        if (e == std::string::npos) break;
        i = e + 1;
    }
    return v;
}

int SelfTest() {
    const std::set<std::string> codes = CodeTokens();
    std::string sw, st;
    int bad = 0;
    const auto expect = [&](const char* in, const char* wantSw, const char* wantSt) {
        Keys(in, codes, sw, st);
        if (sw != wantSw || st != wantSt) {
            std::fprintf(stderr, "Keys(%s) = %s / %s, want %s / %s\n", in, sw.c_str(), st.c_str(),
                         wantSw, wantSt);
            ++bad;
        }
    };
    expect("NamVet_AimMatrix_Rifle_Standing", "@_aimmatrix_rifle_standing",
           "aimmatrix_rifle_standing");
    expect("Run_Pistol", "run_pistol", "run_pistol");
    expect("coach_gesture_head_yes", "@_gesture_head_yes", "gesture_head_yes");

    // Own spelling beats a borrowed one for the same key, whatever the order.
    Index idx;
    idx.Build({"NamVet_Idle_Rifle", "biker_Idle_Rifle"}, "biker", codes);
    if (idx.swapped["@_idle_rifle"] != "biker_Idle_Rifle") {
        std::fprintf(stderr, "own-codename tiebreak failed\n");
        ++bad;
    }

    // Two target slots wanting one donor name: the second must fall through
    // rather than declare a duplicate (a duplicate compiles to an empty slot).
    std::vector<Index> chain(2);
    chain[0].Build({"biker_Idle_Rifle"}, "biker", codes);
    chain[1].Build({"coach_Idle_Rifle", "Idle_Rifle"}, "coach", codes);
    const Result r = Build({"coach_Idle_Rifle", "Idle_Rifle"}, chain, codes);
    if (r.names[0] != "biker_Idle_Rifle" || r.names[1] == r.names[0]) {
        std::fprintf(stderr, "claim-once failed: %s / %s\n", r.names[0].c_str(),
                     r.names[1].c_str());
        ++bad;
    }
    std::printf(bad ? "selftest: %d FAILED\n" : "selftest: ok\n", bad);
    return bad ? 1 : 0;
}

} // namespace

int main(int argc, char** argv) {
    PrintHeader();
    std::string dir, out, outdir = ".", viaArg, ref;
    const Survivor* rig = nullptr;
    const Survivor* tgt = nullptr;
    int skip = 2;
    bool all = false, haveVia = false;

    for (int i = 1; i < argc; ++i) {
        const char* a = argv[i];
        const auto next = [&]() -> const char* { return (i + 1 < argc) ? argv[++i] : ""; };
        if (!std::strcmp(a, "-vanilla")) dir = next();
        else if (!std::strcmp(a, "-rig")) rig = FindSurvivor(next());
        else if (!std::strcmp(a, "-replace")) tgt = FindSurvivor(next());
        else if (!std::strcmp(a, "-via")) { viaArg = next(); haveVia = true; }
        else if (!std::strcmp(a, "-skip")) skip = std::atoi(next());
        else if (!std::strcmp(a, "-o")) out = next();
        else if (!std::strcmp(a, "-outdir")) outdir = next();
        else if (!std::strcmp(a, "-ref")) ref = next();
        else if (!std::strcmp(a, "-all")) all = true;
        else if (!std::strcmp(a, "-selftest")) return SelfTest();
        else { Usage(); return 1; }
    }
    if (dir.empty() || skip < 0 || (!all && (!rig || !tgt))) {
        Usage();
        return 1;
    }

    const std::set<std::string> codes = CodeTokens();
    Resolver res{dir, {}, false};
    std::map<std::string, Slots> orders;
    for (const Survivor& h : kSurvivors) {
        orders[h.survivor] = res.Order(h);
        if (orders[h.survivor].names.size() <= static_cast<size_t>(skip)) {
            std::fprintf(stderr, "no usable sequence list for %s - check survivor_%s.mdl in %s\n",
                         h.survivor, h.code, dir.c_str());
            return 1;
        }
    }
    if (res.missing)
        std::fprintf(stderr, "warning: some includes were unreadable, slot order may be short\n");

    std::vector<std::pair<const Survivor*, const Survivor*>> jobs;
    if (all) {
        for (const Survivor& r : kSurvivors)
            for (const Survivor& t : kSurvivors)
                if (&r != &t) jobs.emplace_back(&r, &t);
    } else {
        jobs.emplace_back(rig, tgt);
    }

    for (const auto& job : jobs) {
        std::vector<const Survivor*> wanted{job.first};
        if (haveVia) {
            bool ok = false;
            for (const Survivor* h : ParseVia(viaArg, ok)) wanted.push_back(h);
            if (!ok) return 1;
        } else if (job.first != job.second) {
            // Rig and target being the same survivor is the no-swap case: vanilla's
            // own list, so pulling in the build-alike group would only steal slots.
            for (const Survivor& h : kSurvivors)
                if (&h != job.first && h.group == job.first->group) wanted.push_back(&h);
        }
        // The target is not a donor - it is the fallback that its own
        // $includemodel provides, so drop it here along with any repeat.
        std::vector<const Survivor*> chain;
        for (const Survivor* h : wanted)
            if (h != job.second && std::find(chain.begin(), chain.end(), h) == chain.end())
                chain.push_back(h);

        std::vector<Index> idx(chain.size());
        for (size_t d = 0; d < chain.size(); ++d)
            idx[d].Build(orders[chain[d]->survivor].names, chain[d]->code, codes);

        const Slots& order = orders[job.second->survivor];
        const Result built = Build(order.names, idx, codes);

        std::string path = out;
        if (path.empty() || all) {
            std::string r = job.first->survivor, t = job.second->survivor;
            r[0] = static_cast<char>(std::toupper(static_cast<unsigned char>(r[0])));
            t[0] = static_cast<char>(std::toupper(static_cast<unsigned char>(t[0])));
            path = (all ? outdir + "/" : std::string()) + "Anims_RigTo" + r + "_Replace" + t +
                   ".qci";
        }
        if (!Write(path, *job.first, *job.second, chain, built, skip, order, res, ref)) return 1;
    }
    return 0;
}

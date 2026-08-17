// flexrig.cpp - the .mdl's flex rules back into a DMX combination operator.
//
// A rule is a fetch per raw control, then COMBO, then a fetch group per
// dominator. The fetch opcode says which slot of which input control it reads
// (FETCH1 = passthru, 2WAY_0/1 = the two halves of a 2-way, NWAY = one slot of
// an n-way), and the driven delta's NAME is the underscore join of its raw
// control names - so the names come back off the rules that use them.

#include "flexrig.h"

#include <algorithm>
#include <cmath>
#include <cstring>
#include <map>

namespace mdldecompiler {
namespace {

// One control slot a rule fetched, before the raw name is known.
struct Fetch {
    int ctrl = 0;      // flex controller index the op named
    int slot = 0;      // which raw control of that input control
    int slots = 1;     // how many the input control has
    int type = 0;      // FLEXCONTROLLER_REMAP_*
    int multi = -1;    // n-way / eyelid multi_ controller
    int eyesUpDown = -1; // eyelid: the controller its window tracks
};

struct Decoded {
    std::vector<Fetch> combo;
    std::vector<std::vector<Fetch>> dominators;
    bool ok = false;
};

// The n-way envelope AddBodyFlexFetchRule builds: 4 consts around the slot's
// peak plus the multi controller. Recovering slot and count from it is the
// inverse - a 2-slot n-way has a zero step and only the end patterns.
bool NWaySlot(const float* c, int& slot, int& slots) {
    auto count = [](float step) {
        return step > 0.0f ? static_cast<int>(std::lround(2.0f / step)) + 1 : 2;
    };
    if (c[0] == -11.0f && c[1] == -10.0f) {
        slot = 0;
        slots = count(c[3] - c[2]);
        return true;
    }
    if (c[2] == 10.0f && c[3] == 11.0f) {
        slots = count(c[1] - c[0]);
        slot = slots - 1;
        return true;
    }
    const float step = c[3] - c[2];
    if (step <= 0.0f)
        return false;
    slots = count(step);
    slot = static_cast<int>(std::lround((c[1] + 1.0f) / step));
    return slot > 0 && slot < slots - 1;
}

// The op stream, split into the combination and its dominator groups. Fetches
// accumulate; COMBO/DOMINATE consume the trailing n, and whatever is left over
// at the end is the combination (a single-control delta emits no COMBO).
Decoded DecodeRule(const fm::mstudioflexop_t* ops, int n) {
    Decoded d;
    std::vector<Fetch> pending;
    std::vector<float> consts;
    auto take = [&](int count, std::vector<Fetch>& out) {
        if (count <= 0 || static_cast<size_t>(count) > pending.size())
            return false;
        out.assign(pending.end() - count, pending.end());
        pending.erase(pending.end() - count, pending.end());
        return true;
    };

    for (int i = 0; i < n; ++i) {
        const int32_t idx = ops[i].d.index;
        switch (ops[i].op) {
            case fm::STUDIO_CONST: consts.push_back(ops[i].d.value); continue;
            case fm::STUDIO_FETCH1:
                pending.push_back({idx, 0, 1, fm::FLEXCONTROLLER_REMAP_PASSTHRU, -1});
                break;
            case fm::STUDIO_2WAY_0:
            case fm::STUDIO_2WAY_1:
                pending.push_back({idx, ops[i].op == fm::STUDIO_2WAY_0 ? 0 : 1, 2,
                                   fm::FLEXCONTROLLER_REMAP_2WAY, -1});
                break;
            case fm::STUDIO_NWAY: {
                if (consts.size() < 5)
                    return d;
                const float* c = &consts[consts.size() - 5];
                Fetch f{idx, 0, 0, fm::FLEXCONTROLLER_REMAP_NWAY,
                        static_cast<int>(c[4])};
                if (!NWaySlot(c, f.slot, f.slots))
                    return d;
                pending.push_back(f);
                break;
            }
            // eyelid: the three consts are the eyes-updown, blink and CloseLid
            // controllers, and the op names the multi_ one
            case fm::STUDIO_DME_LOWER_EYELID:
            case fm::STUDIO_DME_UPPER_EYELID: {
                if (consts.size() < 3)
                    return d;
                const float* c = &consts[consts.size() - 3];
                pending.push_back({static_cast<int>(c[2]),
                                   ops[i].op == fm::STUDIO_DME_LOWER_EYELID ? 0 : 1, 2,
                                   fm::FLEXCONTROLLER_REMAP_EYELID, idx,
                                   static_cast<int>(c[0])});
                break;
            }
            case fm::STUDIO_COMBO:
                if (!d.combo.empty() || !take(idx, d.combo))
                    return d;
                break;
            case fm::STUDIO_DOMINATE: {
                std::vector<Fetch> group;
                if (!take(idx, group))
                    return d;
                d.dominators.push_back(std::move(group));
                break;
            }
            // an expression rule is not a combination at all
            default: return d;
        }
        consts.clear();
    }

    if (d.combo.empty()) {
        if (pending.size() != 1)
            return d;
        d.combo = pending;
    } else if (!pending.empty()) {
        return d;
    }
    d.ok = true;
    return d;
}

// Delta-state name per flexdesc, from the geometry: a stereo pair collapses back
// to the one delta it was split from.
std::map<int, std::string> DeltaNames(const Mdl& m, const std::vector<std::string>& descs,
                                      const std::map<int, LidRole>& lids) {
    std::map<int, std::string> out;
    ForEachFlex(m, [&](const fm::mstudioflex_t& fx) {
        if (fx.flexdesc < 0 || static_cast<size_t>(fx.flexdesc) >= descs.size())
            return;
        const std::string name = DeltaName(descs, lids, fx);
        out[fx.flexdesc] = name;
        if (fx.flexpair > 0 && static_cast<size_t>(fx.flexpair) < descs.size())
            out[fx.flexpair] = name;
    });
    return out;
}

std::vector<std::string> Split(const std::string& s) {
    std::vector<std::string> out;
    for (size_t p = 0;;) {
        const size_t e = s.find('_', p);
        out.push_back(s.substr(p, e == std::string::npos ? e : e - p));
        if (e == std::string::npos)
            return out;
        p = e + 1;
    }
}

// The flexcontrollerui entry each controller belongs to. Byte offsets are
// relative to the ui struct, so the controller index is arithmetic on them.
struct Ui {
    std::string name;
    bool stereo = false;
    int remaptype = 0;
    int slot0 = -1, slot1 = -1, multi = -1;
};

std::vector<Ui> ReadUi(const Mdl& m, std::map<int, int>& ctrlToUi) {
    const fm::studiohdr_t& h = *m.hdr;
    std::vector<Ui> out;
    const fm::mstudioflexcontrollerui_t* ui = m.At<fm::mstudioflexcontrollerui_t>(
        m.buf.data(), h.flexcontrolleruiindex, h.numflexcontrollerui);
    for (int i = 0; ui && i < h.numflexcontrollerui; ++i) {
        const int32_t base = static_cast<int32_t>(reinterpret_cast<const char*>(&ui[i]) -
                                                  m.buf.data());
        auto ctrl = [&](int32_t off) {
            if (off == 0)
                return -1;
            const int32_t rel = base + off - h.flexcontrollerindex;
            const int idx = rel / static_cast<int32_t>(sizeof(fm::mstudioflexcontroller_t));
            return (rel >= 0 && idx < h.numflexcontrollers) ? idx : -1;
        };
        Ui u;
        u.name = m.Str(&ui[i], ui[i].sznameindex);
        u.stereo = ui[i].stereo != 0;
        u.remaptype = ui[i].remaptype;
        u.slot0 = ctrl(ui[i].szindex0);
        u.slot1 = ctrl(ui[i].szindex1);
        u.multi = ctrl(ui[i].szindex2);
        for (int c : {u.slot0, u.slot1, u.multi})
            if (c >= 0)
                ctrlToUi[c] = static_cast<int>(out.size());
        out.push_back(std::move(u));
    }
    return out;
}

} // namespace

FlexRig BuildFlexRig(const Mdl& m) {
    const fm::studiohdr_t& h = *m.hdr;
    FlexRig rig;
    const fm::mstudioflexrule_t* rules =
        m.At<fm::mstudioflexrule_t>(m.buf.data(), h.flexruleindex, h.numflexrules);
    if (!rules || h.numflexrules <= 0)
        return rig;

    const std::vector<std::string> descs = FlexDescNames(m);
    const std::map<int, LidRole> lidRoles = LidRoles(m);
    const std::map<int, std::string> deltaOf = DeltaNames(m, descs, lidRoles);
    std::map<int, int> ctrlToUi;
    const std::vector<Ui> uis = ReadUi(m, ctrlToUi);

    // A rule's target name, delta or not: a shape with no vertex data still
    // names its controls, and the slot that only ever means "off" is usually one
    // of those.
    auto targetName = [&](int desc) {
        const auto d = deltaOf.find(desc);
        if (d != deltaOf.end())
            return d->second;
        const std::string& n = descs[desc];
        if (n.empty())
            return n;
        const char other = n.back() == 'L' ? 'R' : (n.back() == 'R' ? 'L' : '\0');
        if (other && std::find(descs.begin(), descs.end(),
                               n.substr(0, n.size() - 1) + other) != descs.end())
            return n.substr(0, n.size() - 1);
        return n;
    };

    // pass 1: decode every rule, and name the slots it fetched off the delta it
    // drives - the delta name IS the joined raw control names
    struct Rule {
        int desc = 0;
        std::string delta;
        Decoded d;
    };
    std::vector<Rule> decoded;
    std::map<std::pair<int, int>, std::string> rawName; // (ui, slot) -> name
    std::map<int, int> slotCount;                       // ui -> n-way slots
    std::map<int, int> eyesUpDown;                      // ui -> eyelid's tracked controller

    for (int i = 0; i < h.numflexrules; ++i) {
        const fm::mstudioflexop_t* ops =
            m.At<fm::mstudioflexop_t>(&rules[i], rules[i].opindex, rules[i].numops);
        if (!ops || rules[i].flex < 0 || rules[i].flex >= h.numflexdesc ||
            lidRoles.count(rules[i].flex))
            continue;
        Rule r{rules[i].flex, targetName(rules[i].flex), DecodeRule(ops, rules[i].numops)};
        if (!r.d.ok) {
            if (deltaOf.count(r.desc))
                ++rig.dropped;
            continue;
        }
        const std::vector<std::string> parts = Split(r.delta);
        if (parts.size() == r.d.combo.size()) {
            for (size_t j = 0; j < parts.size(); ++j) {
                const Fetch& f = r.d.combo[j];
                const auto ui = ctrlToUi.find(f.ctrl);
                if (ui == ctrlToUi.end())
                    continue;
                rawName.emplace(std::make_pair(ui->second, f.slot), parts[j]);
                if (f.type == fm::FLEXCONTROLLER_REMAP_NWAY)
                    slotCount[ui->second] = f.slots;
                if (f.type == fm::FLEXCONTROLLER_REMAP_EYELID && f.eyesUpDown >= 0)
                    eyesUpDown[ui->second] = f.eyesUpDown;
            }
        }
        // only a shape with vertex data can carry a rule on the way back in
        if (deltaOf.count(r.desc))
            decoded.push_back(std::move(r));
    }

    // pass 2: the input controls, one per ui entry any rule fetched. A control
    // missing a slot name cannot be written, and every corrective through it
    // goes with it.
    const fm::mstudioflexcontroller_t* fc = m.At<fm::mstudioflexcontroller_t>(
        m.buf.data(), h.flexcontrollerindex, h.numflexcontrollers);
    std::map<int, int> uiToControl; // ui index -> rig.controls index
    for (const auto& kv : ctrlToUi) {
        const int uiIdx = kv.second;
        if (uiToControl.count(uiIdx))
            continue;
        const Ui& u = uis[uiIdx];
        int slots = 1;
        switch (u.remaptype) {
            case fm::FLEXCONTROLLER_REMAP_PASSTHRU: slots = 1; break;
            case fm::FLEXCONTROLLER_REMAP_2WAY: slots = 2; break;
            case fm::FLEXCONTROLLER_REMAP_NWAY: {
                const auto n = slotCount.find(uiIdx);
                if (n == slotCount.end())
                    continue;
                slots = n->second;
                break;
            }
            case fm::FLEXCONTROLLER_REMAP_EYELID: slots = 2; break;
            default: continue;
        }

        RigControl c;
        c.name = u.name;
        c.stereo = u.stereo;
        if (u.remaptype == fm::FLEXCONTROLLER_REMAP_EYELID) {
            c.eyelid = true;
            const auto e = eyesUpDown.find(uiIdx);
            if (fc && e != eyesUpDown.end() && e->second >= 0 &&
                e->second < h.numflexcontrollers)
                c.eyesUpDownFlex = m.Str(&fc[e->second], fc[e->second].sznameindex);
        }
        bool complete = true;
        for (int s = 0; s < slots; ++s) {
            const auto n = rawName.find({uiIdx, s});
            if (n == rawName.end()) {
                complete = false;
                break;
            }
            c.rawControls.push_back(n->second);
        }
        if (!complete)
            continue;

        // the compile-side default is -1..1 for a 2-way or an eyelid, 0..1 else
        if (fc && u.slot0 >= 0 && u.slot0 < h.numflexcontrollers) {
            const float defMin = (u.remaptype == fm::FLEXCONTROLLER_REMAP_2WAY ||
                                  u.remaptype == fm::FLEXCONTROLLER_REMAP_EYELID)
                                     ? -1.0f : 0.0f;
            if (fc[u.slot0].min != defMin || fc[u.slot0].max != 1.0f) {
                c.hasRange = true;
                c.min = fc[u.slot0].min;
                c.max = fc[u.slot0].max;
            }
        }
        const bool eyelid = c.eyelid;
        uiToControl[uiIdx] = static_cast<int>(rig.controls.size());
        rig.controls.push_back(std::move(c));
        for (int ctrl : {u.slot0, u.slot1, u.multi})
            if (ctrl >= 0)
                rig.controllers.insert(ctrl);
        // an eyelid remap also brings its own "blink" controller
        if (eyelid && fc) {
            for (int j = 0; j < h.numflexcontrollers; ++j)
                if (std::strcmp(m.Str(&fc[j], fc[j].sznameindex), "blink") == 0)
                    rig.controllers.insert(j);
        }
    }

    // pass 3: correctives, one per delta - the L and R rules of a stereo pair
    // describe the same one
    std::set<std::string> seen;
    for (const Rule& r : decoded) {
        auto names = [&](const std::vector<Fetch>& fetches, std::vector<std::string>& out) {
            for (const Fetch& f : fetches) {
                const auto ui = ctrlToUi.find(f.ctrl);
                if (ui == ctrlToUi.end() || !uiToControl.count(ui->second))
                    return false;
                const auto n = rawName.find({ui->second, f.slot});
                if (n == rawName.end())
                    return false;
                out.push_back(n->second);
            }
            return true;
        };

        RigCorrective cor;
        cor.delta = r.delta;
        if (!names(r.d.combo, cor.combo)) {
            ++rig.dropped;
            continue;
        }
        // the compiler splits the delta name on every '_' and matches each part
        // against the control names (case-insensitively), so a raw control whose
        // own name has one cannot round trip
        const std::vector<std::string> parts = Split(r.delta);
        bool spelled = parts.size() == cor.combo.size();
        for (size_t j = 0; spelled && j < parts.size(); ++j)
            spelled = _stricmp(parts[j].c_str(), cor.combo[j].c_str()) == 0;
        if (!spelled) {
            ++rig.dropped;
            continue;
        }
        bool ok = true;
        for (const std::vector<Fetch>& group : r.d.dominators) {
            std::vector<std::string> dom;
            if (!names(group, dom)) {
                ok = false;
                break;
            }
            cor.dominators.push_back(std::move(dom));
        }
        if (!ok) {
            ++rig.dropped;
            continue;
        }
        rig.descs.insert(r.desc);
        if (seen.insert(cor.delta).second)
            rig.correctives.push_back(std::move(cor));
    }

    // pass 4: per-delta dominators back into the global rules they came from. A
    // rule applies to every delta whose combination contains all of
    // `suppressed`, so the smallest carriers of a dominator group are the
    // combinations that rule was written against.
    std::map<std::vector<std::string>, std::vector<std::vector<std::string>>> carriers;
    for (const RigCorrective& cor : rig.correctives)
        for (const std::vector<std::string>& dom : cor.dominators)
            carriers[dom].push_back(cor.combo);

    for (auto& kv : carriers) {
        std::vector<std::vector<std::string>> minimal;
        for (const std::vector<std::string>& c : kv.second) {
            const std::set<std::string> cs(c.begin(), c.end());
            bool covered = false;
            for (const std::vector<std::string>& other : kv.second) {
                const std::set<std::string> os(other.begin(), other.end());
                if (os.size() < cs.size() && std::includes(cs.begin(), cs.end(), os.begin(),
                                                           os.end())) {
                    covered = true;
                    break;
                }
            }
            if (!covered)
                minimal.push_back(c);
        }
        std::sort(minimal.begin(), minimal.end());
        minimal.erase(std::unique(minimal.begin(), minimal.end()), minimal.end());
        for (const std::vector<std::string>& s : minimal)
            rig.dominations.push_back({kv.first, s});
    }

    // and check the inversion: replay the rules the way the compiler will and
    // count the deltas whose dominators come back different
    for (const RigCorrective& cor : rig.correctives) {
        const std::set<std::string> combo(cor.combo.begin(), cor.combo.end());
        std::set<std::vector<std::string>> rebuilt;
        for (const RigDomination& d : rig.dominations) {
            const std::set<std::string> sup(d.suppressed.begin(), d.suppressed.end());
            if (std::includes(combo.begin(), combo.end(), sup.begin(), sup.end()))
                rebuilt.insert(d.dominators);
        }
        const std::set<std::vector<std::string>> had(cor.dominators.begin(),
                                                     cor.dominators.end());
        if (rebuilt != had)
            ++rig.domMismatch;
    }
    return rig;
}

} // namespace mdldecompiler

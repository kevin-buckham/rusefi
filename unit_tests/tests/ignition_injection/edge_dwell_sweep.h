/**
 * @file edge_dwell_sweep.h
 *
 * Flag-aware helpers on top of edge_dwell_harness.h (kept apart because they touch
 * engineConfiguration->crankingEdgeDwell, which does not exist on the unmodified fork that Task 1 is recorded on).
 *
 * sweep() is THE stress test of the charge-start predictor: 12 noise seeds x 6 phase alignments of one scenario,
 * with the compression-ripple amplitude and the per-stroke combustion impulse as single parameters, so the same
 * sweep can be re-run with the ripple measured from a real composite-logger capture.
 */
#pragma once

#include "pch.h"

#include "edge_dwell_harness.h"

#include <map>
#include <utility>

namespace edgetest {

constexpr double kCrankDwellMs = 4.5;       // the tune's ignitionDwellForCrankingMs since 2026-10-17
constexpr double kBackupFactor = 3.0;       // edge-mode over-dwell backup = 3 x configured dwell
constexpr double kBackupMs = kBackupFactor * kCrankDwellMs;
constexpr double kBackupMarginMs = 2.0;     // DA#4: max dwell must stay at least this far below the backup
constexpr double kLockedErrDeg = 2.0;       // commanded 14 BTDC -> the 12 BTDC tooth is 2 deg later

/** the three independent config bits of the feature, each default off */
struct Bits {
	bool edge = false;   // crankingEdgeDwell: cranking edge spark + predictor
	bool latch = false;  // edgeDwellPostStartLatch: post-start instability latch
	bool e4 = false;     // dwellFixOnRpmUpdate: E4 dwell fix at the RPM update
};

inline Trace runTraceBits(const Profile& p, const Bits& bits, const std::function<void()>& tweak = nullptr) {
	EngineTestHelper eth(engine_type_e::TEST_ENGINE);
	configureCrank(eth);
	engineConfiguration->crankingEdgeDwell = bits.edge;
	engineConfiguration->edgeDwellPostStartLatch = bits.latch;
	engineConfiguration->dwellFixOnRpmUpdate = bits.e4;
	if (tweak) {
		tweak();
	}
	Trace t;
	drive(eth, t, p);
	return t;
}

inline Trace runTrace(const Profile& p, bool edgeDwell, const std::function<void()>& tweak = nullptr) {
	Bits b;
	b.edge = edgeDwell;
	return runTraceBits(p, b, tweak);
}

/** sparks after the given tooth (the first 720 deg only exist to get the first RPM and a full window) */
inline std::vector<Spark> sparksAfter(const Trace& t, size_t firstTooth) {
	std::vector<Spark> out;
	double t0 = NT2US((double)t.teeth[firstTooth].nt);
	for (const Spark& s : analyze(t)) {
		if (s.tUs > t0) {
			out.push_back(s);
		}
	}
	return out;
}

/** per spark-counter bookkeeping of the coil edges: every counter must see exactly one charge and exactly one spark */
struct Integrity {
	int sparks = 0;
	int charges = 0;
	int doubleSpark = 0;   // a counter with two spark edges
	int doubleCharge = 0;  // a counter with two charge edges
	int dangling = 0;      // charged, never sparked: the coil would stay charged (burn)
	int noCharge = 0;      // sparked without a charge: a lost spark (misfire, the safe direction)
};

inline Integrity integrity(const Trace& t) {
	std::map<uint32_t, std::pair<int, int>> byCounter; // counter -> (charges, sparks)
	for (const Coil& c : t.coils) {
		if (c.charge) {
			byCounter[c.counter].first++;
		} else {
			byCounter[c.counter].second++;
		}
	}
	Integrity r;
	const uint32_t last = byCounter.empty() ? 0 : byCounter.rbegin()->first; // the run may end between a charge and its spark
	for (const auto& kv : byCounter) {
		if (kv.first == last && kv.second.second == 0) {
			continue;
		}
		r.charges += kv.second.first;
		r.sparks += kv.second.second;
		r.doubleCharge += kv.second.first > 1;
		r.doubleSpark += kv.second.second > 1;
		r.dangling += kv.second.first > 0 && kv.second.second == 0;
		r.noCharge += kv.second.first == 0 && kv.second.second > 0;
	}
	return r;
}

struct SweepStats {
	int sparks = 0;
	int uncharged = 0;      // spark with no coil charge before it (a lost spark: misfire, the safe direction)
	int maxConsecutiveLost = 0; // longest run of back-to-back lost sparks (cylinder events in firing order) in any one run
	int early = 0;          // spark delivered more advanced than the tooth lock (kLockedErrDeg - 0.05)
	int pastMargin = 0;     // actual dwell above backup - margin
	double maxDwellMs = 0;
	double minErrDeg = 99;  // most advanced delivered spark, delivered minus commanded (negative = earlier than commanded)
	double maxErrDeg = -99; // latest delivered spark
	void add(const SweepStats& o) {
		sparks += o.sparks;
		uncharged += o.uncharged;
		maxConsecutiveLost = std::max(maxConsecutiveLost, o.maxConsecutiveLost);
		early += o.early;
		pastMargin += o.pastMargin;
		maxDwellMs = std::max(maxDwellMs, o.maxDwellMs);
		minErrDeg = std::min(minErrDeg, o.minErrDeg);
		maxErrDeg = std::max(maxErrDeg, o.maxErrDeg);
	}
	double lostPercent() const {
		return sparks ? 100.0 * uncharged / sparks : 0;
	}
};

/**
 * impulseMode: 0 none, 1 alternating +-impulse per stroke, 2 random per stroke.
 * edge = false runs TODAY's firmware path (all bits off) over the same traces: its delivered errors, lost sparks and dwell are
 * the comparison column; 'early' is then counted against the 14 BTDC command (errDeg < 0) instead of the 12 BTDC tooth.
 */
inline SweepStats sweep(const Profile& scenario, double ripple, int impulseMode, float impulse, bool edge = true) {
	SweepStats st;
	for (int seed = 1; seed <= 12; seed++) {
		for (float shift : {0.0f, 30.0f, 60.0f, 90.0f, 120.0f, 150.0f}) {
			Profile p = scenario;
			p.ripple = (float)ripple;
			p.impulseMode = impulseMode;
			p.impulse = impulse;
			p.noise = 0.03f;
			p.seed = seed;
			p.tdcShiftDeg = shift;
			Trace t = runTrace(p, edge);
			int run = 0;
			for (const Spark& s : sparksAfter(t, 60)) {
				st.sparks++;
				if (s.dwellMs < 0) {
					st.uncharged++;
					run++;
					st.maxConsecutiveLost = std::max(st.maxConsecutiveLost, run);
					continue;
				}
				run = 0;
				st.maxDwellMs = std::max(st.maxDwellMs, s.dwellMs);
				st.minErrDeg = std::min(st.minErrDeg, s.errDeg);
				st.maxErrDeg = std::max(st.maxErrDeg, s.errDeg);
				if (s.dwellMs > kBackupMs - kBackupMarginMs) {
					st.pastMargin++;
				}
				if (s.errDeg < (edge ? kLockedErrDeg - 0.05 : -0.05)) {
					st.early++;
				}
			}
		}
	}
	return st;
}

} // namespace edgetest

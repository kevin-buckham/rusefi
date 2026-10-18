/**
 * @file test_edge_dwell_baseline.cpp
 *
 * Characterization of TODAY's cranking spark/dwell scheduling on the 12-tooth crank (no crankingEdgeDwell flag
 * involved). Pins (1) the stale-RPM problem the edge-dwell work addresses and (2) a fingerprint of every coil
 * edge of three scenarios, so that "flag off == byte-for-byte today" can be asserted after the change.
 */

#include "pch.h"

#include "edge_dwell_harness.h"

using namespace edgetest;

static Trace run(const Profile& p) {
	EngineTestHelper eth(engine_type_e::TEST_ENGINE);
	configureCrank(eth);
	Trace t;
	drive(eth, t, p);
	return t;
}

TEST(edgeDwellBaseline, flashSparksAreLateBecauseStoredRpmIsStale) {
	Profile p = flash();
	p.tdcShiftDeg = 0;
	p.noise = 0.03f;
	p.seed = 777;
	Trace t = run(p);
	auto sparks = analyze(t);
	double tFlash = NT2US(t.teeth[60].nt);

	int lateByMoreThan30 = 0;
	for (const Spark& s : sparks) {
		if (s.tUs > tFlash && s.tUs < tFlash + 130000) {
			if (s.errDeg > 30) {
				lateByMoreThan30++;
			}
			// today the dwell is NOT the problem before the RPM update: both edges come off the same tooth
			EXPECT_NEAR(s.dwellMs, 4.5, 0.05);
		}
	}
	// observed on 2bbc712: the first three sparks after the flash land +45..+93 deg late
	EXPECT_EQ(3, lateByMoreThan30);
}

TEST(edgeDwellBaseline, dwellCollapsesAtTheRpmUpdate) {
	Profile p = flash(230);
	Trace t = run(p);
	auto sparks = analyze(t);
	double minRatio = 9;
	for (const Spark& s : sparks) {
		if (s.dwellMs > 0 && s.plannedMs > 0) {
			minRatio = std::min(minRatio, s.dwellMs / s.plannedMs);
		}
	}
	// observed: 0.53 (logged 'Ignition: Dwell deviation' was -0.58 on the car)
	EXPECT_LT(minRatio, 0.6);
}

// Fingerprints hash integer-rounded microsecond edge times (libm-independent in principle); values recorded on
// macOS (Apple clang, this machine) at 2bbc712 with the unmodified firmware.
TEST(edgeDwellBaseline, coilEdgeFingerprints) {
	Profile steadyP = steady(210, 100);
	steadyP.noise = 0.03f;
	steadyP.seed = 1;
	Trace a = run(steadyP);
	EXPECT_EQ(24u, a.coils.size());
	EXPECT_EQ(12710460977540607407ull, coilFingerprint(a));

	Profile f = flash();
	f.noise = 0.03f;
	f.seed = 777;
	Trace b = run(f);
	EXPECT_EQ(24u, b.coils.size());
	EXPECT_EQ(17651081836922608781ull, coilFingerprint(b));

	Profile d = dieBack();
	d.noise = 0.03f;
	d.seed = 4242;
	d.tdcShiftDeg = 60;
	Trace c = run(d);
	EXPECT_EQ(28u, c.coils.size());
	EXPECT_EQ(12064423630106333747ull, coilFingerprint(c));
}

/**
 * @file test_edge_dwell.cpp
 *
 * crankingEdgeDwell (tooth-locked cranking spark), edgeDwellPostStartLatch (post-start instability latch) and
 * dwellFixOnRpmUpdate (E4): three independent bits, each default off. Scenario harness and speed model are in
 * edge_dwell_harness.h.
 */

#include "pch.h"

#include "edge_dwell_sweep.h"

using namespace edgetest;

namespace {

Trace run(const Profile& p, bool edgeDwell) {
	return runTrace(p, edgeDwell);
}

} // namespace

TEST(edgeDwell, allThreeBitsDefaultToOff) {
	EngineTestHelper eth(engine_type_e::TEST_ENGINE);
	EXPECT_FALSE(engineConfiguration->crankingEdgeDwell);
	EXPECT_FALSE(engineConfiguration->edgeDwellPostStartLatch);
	EXPECT_FALSE(engineConfiguration->dwellFixOnRpmUpdate);
}

TEST(edgeDwell, toothPhaseAccessorIsBitIdenticalToTheDecodedPhase) {
	EngineTestHelper eth(engine_type_e::TEST_ENGINE);
	configureCrank(eth);
	Trace t;
	drive(eth, t, steady(210, 60));
	TriggerCentral* tc = getTriggerCentral();
	// the first teeth arrive before the decoder has sync and carry no phase
	for (size_t k = 3; k < t.teeth.size(); k++) {
		const Tooth& tooth = t.teeth[k];
		bool found = false;
		for (uint32_t i = 0; i < tc->engineCycleEventCount; i++) {
			found |= (tc->getToothEnginePhase(i) == tooth.phase);	// exact float equality on purpose
		}
		EXPECT_TRUE(found) << tooth.phase;
	}
}

TEST(edgeDwell, flagOnFeedsTheWindowFromTheFirstSyncedTooth) {
	Profile p = steady(210, 100);
	float slow = 0;
	float last = 0;
	p.afterTooth = [&](size_t) {
		slow = std::max(slow, engine->triggerCentral.toothWindow.slowUsPerDeg(360));
		last = engine->triggerCentral.toothWindow.lastUsPerDeg();
	};
	run(p, true);
	// 210 rpm = 1260 deg/s = 794 us/deg on average; the slowest compression interval is slower than that
	EXPECT_GT(slow, 794);
	EXPECT_LT(slow, 1300);
	EXPECT_GT(last, 400);
}

TEST(edgeDwell, flagOffNeverFeedsTheWindowAndDoesNotRaiseTheLiveBits) {
	Profile p = steady(210, 100);
	bool anyBit = false;
	float slow = 0;
	p.afterTooth = [&](size_t) {
		anyBit |= engine->ignitionState.edgeDwellActive;
		anyBit |= engine->ignitionState.edgeDwellWindowValid;
		slow = std::max(slow, engine->triggerCentral.toothWindow.slowUsPerDeg(360));
	};
	run(p, false);
	EXPECT_FALSE(anyBit);
	EXPECT_EQ(0, slow);
}

TEST(edgeDwell, dwellStaysNearTheConfiguredTimeWhenRpmIsUpdatedBetweenPlanAndSchedule) {
	// a crank that speeds up 1 percent per tooth (about 4 percent per stroke); E4 (dwellFixOnRpmUpdate) alone, no latch bit set: the
	// stored rpm, updated once per 720 deg, always lags, and the dwell angle latched at the previous fire is too short
	double worst[2];
	for (int on = 0; on < 2; on++) {
		EngineTestHelper eth(engine_type_e::TEST_ENGINE);
		configureCrank(eth);
		engineConfiguration->dwellFixOnRpmUpdate = on == 1;	// E4 alone: the other two bits stay off
		setArrayValues(config->sparkDwellValues, 4.0f); // running dwell 4 ms flat; cranking dwell stays 4.5 ms
		Profile p = steady(1000, 80);
		double r = 1000;
		for (int i = 0; i < 100; i++) {
			r *= 1.01;
			p.rpm.push_back(r);
		}
		for (int i = 0; i < 60; i++) {
			p.rpm.push_back(r);
		}
		p.ripple = 0.05f;
		int latched = 0;
		p.afterTooth = [&](size_t) {
			latched += engine->ignitionState.edgeDwellUnstable;
		};
		Trace t;
		drive(eth, t, p);
		ASSERT_EQ(RUNNING, t.finalState);
		EXPECT_EQ(0, latched) << "this test measures the E4 bit alone: the instability latch is a different bit and stays off";
		double w = 9;
		for (const Spark& s : analyze(t)) {
			if (s.tUs > NT2US((double)t.teeth[90].nt) && s.dwellMs > 0) {
				w = std::min(w, s.dwellMs / 4.0);
			}
		}
		worst[on] = w;
	}
	printf("E4 worst actual/4ms dwell while the crank speeds up 1%% per tooth: flag off %.2f, on %.2f\n", worst[0], worst[1]);
	EXPECT_LT(worst[0], 0.65);
	EXPECT_GT(worst[1], worst[0] + 0.15);
}

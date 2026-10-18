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

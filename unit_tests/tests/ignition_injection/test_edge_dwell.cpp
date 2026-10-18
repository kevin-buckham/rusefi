/**
 * @file test_edge_dwell.cpp
 *
 * crankingEdgeDwell (tooth-locked cranking spark), edgeDwellPostStartLatch (post-start instability latch) and
 * dwellFixOnRpmUpdate (E4): three independent bits, each default off. Scenario harness and speed model are in
 * edge_dwell_harness.h.
 */

#include "pch.h"

#include "edge_dwell_harness.h"

using namespace edgetest;

TEST(edgeDwell, allThreeBitsDefaultToOff) {
	EngineTestHelper eth(engine_type_e::TEST_ENGINE);
	EXPECT_FALSE(engineConfiguration->crankingEdgeDwell);
	EXPECT_FALSE(engineConfiguration->edgeDwellPostStartLatch);
	EXPECT_FALSE(engineConfiguration->dwellFixOnRpmUpdate);
}

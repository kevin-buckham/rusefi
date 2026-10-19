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

TEST(edgeDwell, steadyCrankIsToothLockedWithTheConfiguredDwell) {
	for (float shift : {0.0f, 30.0f, 60.0f, 90.0f, 120.0f, 150.0f}) {
		for (float noise : {0.0f, 0.03f}) {
			Profile p = steady(210, 140);
			p.noise = noise;
			p.seed = 5;
			p.tdcShiftDeg = shift;
			Trace t = run(p, true);
			auto sparks = sparksAfter(t, 60);
			ASSERT_GE(sparks.size(), 12u) << shift;
			for (const Spark& s : sparks) {
				// commanded 14 BTDC, the 12 BTDC tooth is the first one that is not more advanced: +2.0 deg, exactly
				EXPECT_NEAR(2.0, s.errDeg, 0.05) << "shift " << shift << " noise " << noise;
				EXPECT_GT(s.dwellMs, 0) << "spark without charge, shift " << shift;
				if (noise == 0) {
					EXPECT_NEAR(kCrankDwellMs, s.dwellMs, 0.1) << "shift " << shift;
				} else {
					// 3% jitter on a 25 ms tooth interval moves the dwell by about +-1.5 ms
					EXPECT_GE(s.dwellMs, 2.0) << "shift " << shift;
					EXPECT_LE(s.dwellMs, 2 * kCrankDwellMs) << "shift " << shift;
				}
			}
			EXPECT_EQ(0, t.sparkOutOfOrder) << shift;
		}
	}
}

TEST(edgeDwell, theVeryFirstSparksOfAStartAreAlreadyToothLockedAndCharged) {
	for (float shift : {0.0f, 90.0f}) {
		Profile p = steady(210, 60);
		p.tdcShiftDeg = shift;
		Trace t = run(p, true);
		auto sparks = analyze(t);	// every spark, from the first one the ECU ever schedules
		ASSERT_GE(sparks.size(), 4u) << shift;
		for (const Spark& s : sparks) {
			EXPECT_NEAR(2.0, s.errDeg, 0.05) << shift;
			EXPECT_GT(s.dwellMs, 0) << shift;
		}
	}
}

TEST(edgeDwell, flashOverspeedStaysOnTheToothAndKeepsDwell) {
	for (float shift : {0.0f, 60.0f, 120.0f}) {
		Profile p = flash();
		p.noise = 0.03f;
		p.seed = 777;
		p.tdcShiftDeg = shift;
		Trace t = run(p, true);
		double tFlash = NT2US((double)t.teeth[60].nt);
		int checked = 0;
		for (const Spark& s : sparksAfter(t, 60)) {
			if (s.tUs < tFlash + 130000) {
				checked++;
				EXPECT_NEAR(2.0, s.errDeg, 0.05) << "shift " << shift;
				EXPECT_GE(s.dwellMs, 1.0) << "shift " << shift;
				EXPECT_LE(s.dwellMs, kBackupMs - kBackupMarginMs) << "shift " << shift;
			}
		}
		EXPECT_GE(checked, 3) << shift;
		EXPECT_EQ(0, t.sparkOutOfOrder) << shift;
	}
}

TEST(edgeDwell, aCommandBelowTwelveDegreesKeepsTheLegacyPathAndIsBitIdenticalToFlagOff) {
	// what the cranking table commands at stored 0 / 43 / 70 / 120 / 150 / 171 rpm (crankingAdvance 0/14/16/16 at 0/200/400)
	for (double commanded : {-5.0, 0.0, 3.0, 5.0, 8.4, 10.5, 11.9}) {
		for (float shift : {0.0f, 60.0f, 120.0f}) {
			Profile p = steady(210, 100);
			p.noise = 0.03f;
			p.seed = 2;
			p.tdcShiftDeg = shift;
			bool anyEdge = false;
			p.afterTooth = [&](size_t) {
				anyEdge |= engine->ignitionState.edgeDwellSparkEdge || engine->ignitionState.edgeDwellActive;
			};
			Trace on = runTrace(p, true, [&]() { engineConfiguration->fixedTiming = commanded; });
			p.afterTooth = nullptr;
			Trace off = runTrace(p, false, [&]() { engineConfiguration->fixedTiming = commanded; });
			EXPECT_FALSE(anyEdge) << "commanded " << commanded << " shift " << shift;
			ASSERT_GE(on.coils.size(), 10u);
			EXPECT_EQ(coilFingerprint(off), coilFingerprint(on)) << "commanded " << commanded << " shift " << shift;
			EXPECT_EQ(off.coils.size(), on.coils.size());
		}
	}
}

TEST(edgeDwell, aCommandOfTwelveDegreesOrMoreIsEdgeLockedAndNeverMoreAdvancedThanCommandedNorAnAtdcSpark) {
	for (double commanded : {12.0, 12.5, 14.1, 16.0, 20.0, 30.0}) {
		for (float shift : {0.0f, 60.0f, 120.0f}) {
			Profile p = steady(210, 100);
			p.noise = 0.03f;
			p.seed = 2;
			p.tdcShiftDeg = shift;
			int edgeBitTeeth = 0;
			p.afterTooth = [&](size_t n) {
				if (n > 70) {
					edgeBitTeeth += engine->ignitionState.edgeDwellActive;
				}
			};
			Trace t = runTrace(p, true, [&]() { engineConfiguration->fixedTiming = commanded; });
			auto sparks = sparksAfter(t, 60);
			ASSERT_GE(sparks.size(), 6u);
			EXPECT_GT(edgeBitTeeth, 0) << commanded;
			for (const Spark& s : sparks) {
				double deliveredBtdc = commanded - s.errDeg;
				EXPECT_GE(s.errDeg, -0.06) << "commanded " << commanded << " shift " << shift;	// never more advanced
				EXPECT_GE(deliveredBtdc, 12.0 - 0.06) << "commanded " << commanded;				// never 18 ATDC
				EXPECT_GT(s.dwellMs, 0) << "commanded " << commanded;
			}
		}
	}
}

TEST(edgeDwell, theCommandCrossingTwelveDegreesMidCrankNeverDoublesOrStrandsACharge) {
	// the commanded advance moves across 12 BTDC (as it does while the stored rpm climbs through about 171 rpm):
	// events flip legacy <-> edge between cycles, in both directions
	for (int direction = 0; direction < 2; direction++) {
		for (int n = 60; n < 84; n++) {
			Profile p = steady(210, 170);
			p.ripple = 0.3f;
			p.noise = 0.03f;
			p.seed = 6;
			const double lo = 8.0, hi = 16.0;
			p.beforeTooth = [=](size_t i) {
				if ((int)i == n) {
					engineConfiguration->fixedTiming = direction == 0 ? hi : lo;
				}
			};
			Trace t = runTrace(p, true, [=]() { engineConfiguration->fixedTiming = direction == 0 ? lo : hi; });
			Integrity i = integrity(t);
			EXPECT_EQ(0, i.doubleCharge + i.doubleSpark + i.dangling) << "direction " << direction << " at tooth " << n;
			EXPECT_EQ(0, i.noCharge) << "direction " << direction << " at tooth " << n;
			EXPECT_EQ(0, t.sparkOutOfOrder) << "direction " << direction << " at tooth " << n;
			EXPECT_GE(i.sparks, 22) << "direction " << direction << " at tooth " << n;
		}
	}
}

TEST(edgeDwell, edgeModeStaysInactiveWhileSpinningUpEvenThoughIsCrankingIsTrueThere) {
	// isCranking() is true in SPINNING_UP with a non-zero cached rpm (faster spin-up is off in our tune, but pin it):
	// edge mode keys on the CRANKING state alone. Faster spin-up is switched ON here only to reach that state.
	Profile p = steady(210, 130);
	int forcedTeeth = 0;
	int activeWhileForced = 0;
	int activeWhileCranking = 0;
	p.afterTooth = [&](size_t n) {
		if (engine->rpmCalculator.getState() == SPINNING_UP && engine->rpmCalculator.isCranking()) {
			forcedTeeth++;
			activeWhileForced += engine->ignitionState.edgeDwellActive;
		} else if (n > 70 && engine->rpmCalculator.getState() == CRANKING) {
			activeWhileCranking += engine->ignitionState.edgeDwellActive;
		}
	};
	runTrace(p, true, []() { engineConfiguration->isFasterEngineSpinUpEnabled = true; });
	printf("spinning-up teeth with isCranking() true: %d\n", forcedTeeth);
	EXPECT_GE(forcedTeeth, 1) << "the scenario must reach SPINNING_UP with a non-zero rpm for this test to mean anything";
	EXPECT_EQ(0, activeWhileForced);
	EXPECT_GT(activeWhileCranking, 10);
}

TEST(edgeDwell, flagOffDeliversWhatTodayDeliversForEveryCommand) {
	// the table the plan quotes: today's delivered advance for the same commands spans a wide range; pinned loosely
	struct Row { double commanded; double todayMinBtdc; double todayMaxBtdc; };
	const Row rows[] = {
		{0.0, -4.9, 3.5},
		{3.0, -0.7, 5.6},
		{5.0, 2.2, 7.0},
		{8.4, 6.9, 16.0},
		{10.5, 9.9, 17.5},
		{11.9, 11.9, 18.5},
		{12.0, 12.0, 18.6},
		{14.1, 4.3, 22.0},
		{16.0, 6.6, 23.4},
	};
	for (const Row& r : rows) {
		double lo = 99, hi = -99;
		for (float shift : {0.0f, 30.0f, 60.0f, 90.0f, 120.0f, 150.0f}) {
			for (int seed = 1; seed <= 4; seed++) {
				Profile p = steady(210, 100);
				p.ripple = 0.3f;
				p.noise = 0.03f;
				p.seed = seed;
				p.tdcShiftDeg = shift;
				Trace t = runTrace(p, false, [&]() { engineConfiguration->fixedTiming = r.commanded; });
				for (const Spark& s : sparksAfter(t, 60)) {
					lo = std::min(lo, r.commanded - s.errDeg);
					hi = std::max(hi, r.commanded - s.errDeg);
				}
			}
		}
		printf("today table: commanded %.1f: today delivers %.1f..%.1f BTDC (edge mode: %s)\n", r.commanded, lo, hi, r.commanded >= 12.0 ? "12.0 tooth" : "legacy, same as today");
		EXPECT_NEAR(r.todayMinBtdc, lo, 0.2) << r.commanded;
		EXPECT_NEAR(r.todayMaxBtdc, hi, 0.2) << r.commanded;
	}
}

TEST(edgeDwell, stateMachineCachedRpmAndTimingAreIdenticalWithTheFlagOnAndOff) {
	struct Snap { int state; float rpm; float adv0; float adv1; float adv2; float adv3; };
	std::vector<Snap> a, b;
	for (int on = 0; on < 2; on++) {
		std::vector<Snap>& out = on ? b : a;
		Profile p = dieBack();
		p.noise = 0.03f;
		p.seed = 4242;
		p.afterTooth = [&](size_t) {
			out.push_back({(int)engine->rpmCalculator.getState(), engine->rpmCalculator.getCachedRpm(),
				engine->engineState.timingAdvance[0], engine->engineState.timingAdvance[1],
				engine->engineState.timingAdvance[2], engine->engineState.timingAdvance[3]});
		};
		run(p, on == 1);
	}
	ASSERT_EQ(a.size(), b.size());
	for (size_t i = 0; i < a.size(); i++) {
		EXPECT_EQ(a[i].state, b[i].state) << i;
		EXPECT_EQ(a[i].rpm, b[i].rpm) << i;
		EXPECT_EQ(a[i].adv0, b[i].adv0) << i;
		EXPECT_EQ(a[i].adv1, b[i].adv1) << i;
		EXPECT_EQ(a[i].adv2, b[i].adv2) << i;
		EXPECT_EQ(a[i].adv3, b[i].adv3) << i;
	}
}

// ---- controller ruling 4: gaps in the brief's coverage ----

TEST(edgeDwell, edgeModeIsInactiveOnRisingAndFallingWheelsAndWithOddFireWastedSpark) {
	for (int variant = 0; variant < 2; variant++) {
		Profile p = steady(210, 100);
		p.noise = 0.03f;
		p.seed = 2;
		bool anyEdge = false;
		bool conditionHeld = true;
		p.afterTooth = [&](size_t) {
			anyEdge |= engine->ignitionState.edgeDwellSparkEdge || engine->ignitionState.edgeDwellActive;
			if (variant == 0) {
				conditionHeld &= !getTriggerCentral()->triggerShape.useOnlyRisingEdges;
			} else {
				conditionHeld &= engine->engineState.useOddFireWastedSpark;
			}
		};
		auto tweak = [&]() {
			if (variant == 0) {
				getTriggerCentral()->triggerShape.useOnlyRisingEdges = false;
			} else {
				engine->engineState.useOddFireWastedSpark = true;
				engineConfiguration->ignitionMode = IM_WASTED_SPARK;
			}
		};
		Trace on = runTrace(p, true, tweak);
		p.afterTooth = nullptr;
		Trace off = runTrace(p, false, tweak);
		EXPECT_TRUE(conditionHeld) << "the scenario condition must hold for the whole run, variant " << variant;
		EXPECT_FALSE(anyEdge) << "variant " << variant;
		ASSERT_GE(on.coils.size(), 10u) << variant;
		EXPECT_EQ(coilFingerprint(off), coilFingerprint(on)) << "variant " << variant;
	}
}

TEST(edgeDwell, switchingTheBitOnMidCrankNeverDoublesStrandsOrAdvancesASpark) {
	// the window is not fed while the bit is off, so it is empty when the bit is turned on: no early spark, no skipped spark
	for (int n = 40; n < 64; n += 3) {
		Profile p = steady(210, 120);
		p.noise = 0.03f;
		p.seed = 9;
		int validBefore = 0;
		bool edgeBeforeSwitch = false;
		int invalidActive = 0;
		p.beforeTooth = [&](size_t i) {
			if ((int)i == n) {
				engineConfiguration->crankingEdgeDwell = true;
			}
		};
		p.afterTooth = [&](size_t i) {
			if ((int)i >= n) {
				invalidActive += engine->ignitionState.edgeDwellActive && !engine->ignitionState.edgeDwellWindowValid;
			}
			if ((int)i < n) {
				validBefore += engine->ignitionState.edgeDwellWindowValid;
				edgeBeforeSwitch |= engine->ignitionState.edgeDwellActive;
			}
		};
		Trace t = runTrace(p, false);
		Trace off = runTrace(steady(210, 120), false);
		printf("bit switched on at tooth %d: teeth with edge active but window not yet valid: %d\n", n, invalidActive);
		EXPECT_EQ(0, validBefore) << n;
		EXPECT_FALSE(edgeBeforeSwitch) << n;
		Integrity i = integrity(t);
		EXPECT_EQ(0, i.doubleCharge + i.doubleSpark + i.dangling + i.noCharge) << "switched at tooth " << n;
		EXPECT_EQ(0, t.sparkOutOfOrder) << n;
		Integrity io = integrity(off);
		EXPECT_GE(i.sparks, io.sparks - 1) << "a spark was skipped, switched at tooth " << n;
		for (const Spark& s : analyze(t)) {
			if (s.tUs > NT2US((double)t.teeth[n].nt)) {
				EXPECT_GE(s.errDeg, -0.06 - 8.0) << "switched at tooth " << n;	// 14 BTDC command: nothing beyond today's own legacy range
			}
		}
	}
}

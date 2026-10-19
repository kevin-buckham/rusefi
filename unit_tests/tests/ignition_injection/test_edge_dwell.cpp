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

// Worst (most advanced) delivered spark, in degrees BTDC, over every point at which the crank can slow by factor k right
// after a tooth, steady R rpm before. commandedBtdc is the advance the cranking table would command at R (fixed timing
// in the harness). Only sparks after the slowdown are looked at. capFires counts sparks whose dwell reached the backup.
double worstDeliveredBtdc(double rpm, double k, bool edge, double commandedBtdc, int* capFires = nullptr) {
	double worst = 99;
	int caps = 0;
	for (int drop = 61; drop < 61 + 24; drop++) {
		Profile p = steady(rpm, drop);
		p.ripple = 0;
		for (int i = 0; i < 16; i++) {
			p.rpm.push_back(rpm / k);
		}
		Trace t = runTrace(p, edge, [&]() { engineConfiguration->fixedTiming = commandedBtdc; });
		double tSlow = NT2US((double)t.teeth[drop - 1].nt);
		for (const Spark& s : analyze(t)) {
			if (s.tUs > tSlow - 1000 && s.tUs < tSlow + 2000000) {
				worst = std::min(worst, s.errDeg);
				if (s.dwellMs > kBackupMs - 0.1) {
					caps++;
				}
			}
		}
	}
	if (capFires) {
		*capFires = caps;
	}
	return commandedBtdc - worst;
}

struct SweepRow {
	const char* name;
	int impulseMode;
	float impulse;
};

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
	EXPECT_GE(activeWhileCranking, 8);	// edgeDwellActive = an event was scheduled on the edge path at this tooth (4 per 24 teeth)
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
		Profile ref = p;	// the SAME profile with the bit never on
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
		Trace off = runTrace(ref, false);
		printf("bit switched on at tooth %d: teeth with edge active but window not yet valid: %d\n", n, invalidActive);
		EXPECT_EQ(0, validBefore) << n;
		EXPECT_FALSE(edgeBeforeSwitch) << n;
		Integrity i = integrity(t);
		EXPECT_EQ(0, i.doubleCharge + i.doubleSpark + i.dangling + i.noCharge) << "switched at tooth " << n;
		EXPECT_EQ(0, t.sparkOutOfOrder) << n;
		auto a = analyze(t);
		auto b = analyze(off);
		ASSERT_EQ(b.size(), a.size()) << "a spark was skipped or added, switched at tooth " << n;
		for (size_t k = 0; k < a.size(); k++) {
			// an edge-path spark is never earlier than commanded; a legacy one is exactly today's
			EXPECT_TRUE(a[k].errDeg >= -0.06 || std::fabs(a[k].errDeg - b[k].errDeg) < 0.5)
				<< "switched at tooth " << n << " spark " << k << " err " << a[k].errDeg << " legacy " << b[k].errDeg;
		}
	}
}

TEST(edgeDwell, theModeDroppingOrRisingBetweenTheLegacyAndTheEdgeDwellToothNeverSkipsOrDoubleChargesASpark) {
	// the angles are latched once per cycle but the mode gate is re-evaluated per tooth: toggle the bit at every tooth
	// of a stretch, both directions, with large commands (the dwell tooth of the edge plan is then later than the legacy one)
	for (double commanded : {14.0, 20.0, 29.0, 38.0}) {
		for (int direction = 0; direction < 2; direction++) {
			for (int n = 60; n < 84; n++) {
				Profile p = steady(210, 170);
				p.noise = 0.03f;
				p.seed = 6;
				p.beforeTooth = [=](size_t i) {
					if ((int)i == n) {
						engineConfiguration->crankingEdgeDwell = direction == 0 ? false : true;
					}
				};
				Profile refP = p;
				refP.beforeTooth = nullptr;
				Trace t = runTrace(p, direction == 0, [=]() { engineConfiguration->fixedTiming = commanded; });
				Trace ref = runTrace(refP, false, [=]() { engineConfiguration->fixedTiming = commanded; });
				Integrity i = integrity(t);
				const std::string what = "commanded " + std::to_string(commanded) + " direction " + std::to_string(direction) + " tooth " + std::to_string(n);
				EXPECT_EQ(0, i.doubleCharge + i.doubleSpark + i.dangling) << what;
				EXPECT_EQ(0, i.noCharge) << what;
				EXPECT_EQ(0, t.sparkOutOfOrder) << what;
				auto a = analyze(t);
				auto b = analyze(ref);
				ASSERT_EQ(b.size(), a.size()) << "a spark was skipped or added, " << what;
				for (size_t k = 0; k < a.size(); k++) {
					// an edge-path spark is never earlier than commanded; a legacy-path one is what today delivers
					EXPECT_TRUE(a[k].errDeg >= -0.06 || std::fabs(a[k].errDeg - b[k].errDeg) < 0.5)
						<< what << " spark " << k << " err " << a[k].errDeg << " legacy " << b[k].errDeg;
				}
			}
		}
	}
}

TEST(edgeDwell, overDwellBackupIsNeverWorseThanTodaysDeliveredAngleAt120And150And210Rpm) {
	// rpm and what crankingAdvance 0/14/16/16 at 0/200/400/1000 rpm commands there. At 120 and 150 rpm the command is
	// below 12 BTDC, so the event keeps the legacy path: edge == today. At 210 rpm (14.1) the 12 BTDC tooth and the
	// 3 x dwell backup apply.
	struct Row { double rpm; double cmd; double edge[4]; double today[4]; };
	// measured worst delivered BTDC for k = 1.5 / 2 / 3 / 4 (asserted +-0.3 deg)
	const Row rows[] = {
		{120, 8.4, {9.6, 10.2, 10.8, 12.0}, {9.6, 10.2, 10.8, 12.0}},
		{150, 10.5, {19.7, 25.2, 30.8, 42.0}, {19.7, 25.2, 30.8, 42.0}},
		{210, 14.1, {14.4, 21.3, 28.2, 42.0}, {23.4, 28.0, 32.7, 42.0}},
	};
	for (const Row& r : rows) {
		int ki = 0;
		for (double k : {1.5, 2.0, 3.0, 4.0}) {
			int idx = ki++;
			int caps = 0;
			double edge = worstDeliveredBtdc(r.rpm, k, true, r.cmd, &caps);
			double today = worstDeliveredBtdc(r.rpm, k, false, r.cmd);
			bool legacy = r.cmd < 12;
			printf("overdwell R=%.0f cmd=%.1f k=%.1f: edge worst %.1f BTDC (%d cap fires), today %.1f BTDC%s\n", r.rpm, r.cmd, k, edge, caps, today, legacy ? " (legacy path)" : "");
			EXPECT_NEAR(r.edge[idx], edge, 0.3) << r.rpm << " " << k;
			EXPECT_NEAR(r.today[idx], today, 0.3) << r.rpm << " " << k;
			EXPECT_LE(edge, today + 0.05) << "edge mode worse than today's delivered angle at " << r.rpm << " rpm k=" << k;
			if (legacy) {
				EXPECT_NEAR(edge, today, 0.01) << r.rpm << " " << k << ": below 12 BTDC the path must be today's";
			}
		}
	}
}

TEST(edgeDwell, flashAndDieBackNeverAdvanceTheSparkAndLoseAtMostAFewSparks) {
	for (float shift : {0.0f, 60.0f, 120.0f}) {
		Profile p = dieBack();
		p.noise = 0.03f;
		p.seed = 4242;
		p.tdcShiftDeg = shift;
		Trace t = run(p, true);
		EXPECT_EQ(CRANKING, t.finalState) << "stored rpm must stay below cranking.rpm for this scenario to mean anything";
		auto sparks = sparksAfter(t, 60);
		ASSERT_GE(sparks.size(), 8u);
		int lost = 0;
		for (const Spark& s : sparks) {
			if (s.dwellMs < 0) {
				lost++;
				continue;
			}
			EXPECT_GE(s.errDeg, kLockedErrDeg - 0.05) << "shift " << shift << " spark more advanced than the tooth lock";
			EXPECT_LE(s.dwellMs, kBackupMs - kBackupMarginMs) << "shift " << shift;
		}
		printf("die-back seed 4242 shift %.0f: %zu sparks, lost %d, spark out of order %d\n", shift, sparks.size(), lost, (int)t.sparkOutOfOrder);
		// owner decision 2026-10-17: a small fraction of lost sparks (misfire = safe direction) is accepted
		EXPECT_LE(lost, 2) << "shift " << shift;
		EXPECT_LE((int)t.sparkOutOfOrder, 2) << "shift " << shift;
		Integrity i = integrity(t);
		EXPECT_EQ(0, i.doubleCharge + i.doubleSpark + i.dangling) << "shift " << shift;
	}
}

// THE sweep, parameterised by the compression ripple amplitude. Run at +-30 percent (the old guess) and +-50 percent.
// When the composite-logger capture of a flag-off start gives the real ripple, add a row with that value.
// Lost sparks: the charge is refused because it would start after the spark (misfire, the safe direction).
static void checkSweepsAtRipple(double ripple) {
	const SweepRow rows[] = {{"no impulse", 0, 0}, {"alternating +-10%", 1, 0.10f}, {"random +-10%", 2, 0.10f}};
	for (const SweepRow& row : rows) {
		SweepStats all;
		struct Sc { const char* name; Profile p; double lostLimitPercent; int consecLimit; };
		const Sc scenarios[] = {{"steady 210", steady(210, 140), 2.0, 2}, {"flash 210->800", flash(), 3.0, 1}, {"flash + die-back", dieBack(), 12.0, 3}};
		for (const Sc& sc : scenarios) {
			SweepStats st = sweep(sc.p, ripple, row.impulseMode, row.impulse);
			SweepStats today = sweep(sc.p, ripple, row.impulseMode, row.impulse, false);
			printf("sweep ripple +-%.0f%% %-18s %-17s: %4d sparks, lost %3d (%.1f%%, max %d in a row), early %d, max dwell %.1f ms (backup %.1f), past margin %d | today: lost %d, delivered %.1f..%.1f deg vs command, %d earlier than commanded\n",
				ripple * 100, row.name, sc.name, st.sparks, st.uncharged, st.lostPercent(), st.maxConsecutiveLost, st.early, st.maxDwellMs, kBackupMs, st.pastMargin,
				today.uncharged, today.minErrDeg, today.maxErrDeg, today.early);
			EXPECT_EQ(0, st.early) << sc.name << " " << row.name << " ripple " << ripple;
			EXPECT_EQ(0, st.pastMargin) << sc.name << " " << row.name << " ripple " << ripple;
			EXPECT_LE(st.maxDwellMs, kBackupMs - kBackupMarginMs) << sc.name << " " << row.name << " ripple " << ripple;
			EXPECT_LE(st.lostPercent(), sc.lostLimitPercent) << sc.name << " " << row.name << " ripple " << ripple;
			EXPECT_LE(st.maxConsecutiveLost, sc.consecLimit) << sc.name << " " << row.name << " ripple " << ripple;
			all.add(st);
		}
		printf("sweep ripple +-%.0f%% %-18s TOTAL: %d sparks, lost %d (%.2f%%), max %d in a row, max dwell %.1f ms\n",
			ripple * 100, row.name, all.sparks, all.uncharged, all.lostPercent(), all.maxConsecutiveLost, all.maxDwellMs);
		EXPECT_LE(all.lostPercent(), 5.0);
	}
}

TEST(edgeDwell, sweepAtThirtyPercentRipple) {
	checkSweepsAtRipple(0.30);
}

TEST(edgeDwell, sweepAtFiftyPercentRipple) {
	checkSweepsAtRipple(0.50);
}

TEST(edgeDwell, dieBackTodayVersusEdgeDeliveredErrorAndLostSparks) {
	SweepStats today = sweep(dieBack(), 0.30, 0, 0, false);
	SweepStats edge = sweep(dieBack(), 0.30, 0, 0, true);
	printf("die-back today: %d sparks, lost %d, delivered minus commanded %.1f..%.1f deg, %d earlier than commanded\n",
		today.sparks, today.uncharged, today.minErrDeg, today.maxErrDeg, today.early);
	printf("die-back edge : %d sparks, lost %d (%.1f%%, max %d in a row), delivered minus commanded %.1f..%.1f deg, %d earlier than the tooth lock\n",
		edge.sparks, edge.uncharged, edge.lostPercent(), edge.maxConsecutiveLost, edge.minErrDeg, edge.maxErrDeg, edge.early);
	EXPECT_EQ(0, today.uncharged);
	EXPECT_NEAR(kLockedErrDeg, edge.minErrDeg, 0.05);
	EXPECT_NEAR(kLockedErrDeg, edge.maxErrDeg, 0.05);
	EXPECT_EQ(0, edge.early);
	EXPECT_LT(today.minErrDeg, -10.0);
	EXPECT_GT(today.maxErrDeg, 80.0);
	EXPECT_GT(today.early, 300);
	EXPECT_LE(edge.maxConsecutiveLost, 2);
	EXPECT_LE(edge.lostPercent(), 12.0);
}

TEST(edgeDwell, alternatingFireNoFireImpulseSweepFindsTheBreakingThreshold) {
	// one stroke fires and the next does not, amplitude 10..50 percent of speed in 2.5 percent steps, three scenarios,
	// ripple +-30 and +-50 percent. 180-deg stroke-twin predictor only.
	for (double ripple : {0.30, 0.50}) {
		double firstEarly = 0, firstEarlyDeliveredBtdc = 0, firstAdvancedBtdc = 0, firstAdvancedAt = 0, firstPastMargin = 0;
		for (int i = 4; i <= 20; i++) {
			float impulse = 0.025f * i;
			SweepStats all;
			for (const Profile& sc : {steady(210, 140), flash(), dieBack()}) {
				all.add(sweep(sc, ripple, 1, impulse));
			}
			double deliveredBtdc = 14.0 - all.minErrDeg;
			printf("impulse alternating +-%.1f%% (ripple +-%.0f%%): %d sparks, lost %d (%.1f%%, max %d in a row), early %d, most advanced delivered %.1f BTDC (commanded 14), max dwell %.1f ms\n",
				impulse * 100, ripple * 100, all.sparks, all.uncharged, all.lostPercent(), all.maxConsecutiveLost, all.early, deliveredBtdc, all.maxDwellMs);
			if (all.early > 0 && firstEarly == 0) {
				firstEarly = impulse;
				firstEarlyDeliveredBtdc = deliveredBtdc;
			}
			if (all.maxDwellMs > kBackupMs - kBackupMarginMs && firstPastMargin == 0) {
				firstPastMargin = impulse;
			}
			if (deliveredBtdc > 14.0 + 0.05 && firstAdvancedBtdc == 0) {
				firstAdvancedBtdc = deliveredBtdc;
				firstAdvancedAt = impulse;
			}
			// measured: +-30 percent ripple holds the tooth lock up to 25 percent and the dwell margin up to 22.5 percent;
			// +-50 percent ripple breaks earlier (first early spark at 22.5 percent), so the asserted range is 20 percent there
			float lockLimit = ripple < 0.4 ? 0.25f : 0.20f;
			float marginLimit = ripple < 0.4 ? 0.225f : 0.20f;
			if (impulse <= lockLimit + 1e-6f) {
				EXPECT_EQ(0, all.early) << impulse << " ripple " << ripple;
				EXPECT_LE(all.maxConsecutiveLost, 3) << impulse << " ripple " << ripple;
				EXPECT_LE(all.lostPercent(), 12.0) << impulse << " ripple " << ripple;
			}
			if (impulse <= marginLimit + 1e-6f) {
				EXPECT_LE(all.maxDwellMs, kBackupMs - kBackupMarginMs) << impulse << " ripple " << ripple;
			}
		}
		printf("BREAKING THRESHOLD (alternating impulse, ripple +-%.0f%%, 180-deg twin only): first early spark at %.1f%% delivered %.1f BTDC (tooth lock 12, commanded 14); first spark more advanced than commanded at %.1f%% (%.1f BTDC); first dwell past the %.1f ms margin limit at %.1f%%\n",
			ripple * 100, firstEarly * 100, firstEarlyDeliveredBtdc, firstAdvancedAt * 100, firstAdvancedBtdc, kBackupMs - kBackupMarginMs, firstPastMargin * 100);
		EXPECT_NEAR(ripple < 0.4 ? 0.275 : 0.225, firstEarly, 1e-6) << ripple;
		EXPECT_NEAR(ripple < 0.4 ? 13.2 : 14.6, firstEarlyDeliveredBtdc, 0.3) << ripple;
		EXPECT_NEAR(ripple < 0.4 ? 0.30 : 0.225, firstAdvancedAt, 1e-6) << ripple;
		EXPECT_NEAR(ripple < 0.4 ? 0.25 : 0.225, firstPastMargin, 1e-6) << ripple;
	}
}

TEST(edgeDwell, randomPerStrokeImpulsesUpToThirtyPercentNeverFireEarly) {
	for (float impulse : {0.05f, 0.10f, 0.15f, 0.20f, 0.30f}) {
		SweepStats all;
		for (const Profile& sc : {steady(210, 140), flash(), dieBack()}) {
			all.add(sweep(sc, 0.30, 2, impulse));
		}
		printf("impulse random +-%.0f%%: %d sparks, lost %d (%.2f%%, max %d in a row), early %d, max dwell %.1f ms\n",
			impulse * 100, all.sparks, all.uncharged, all.lostPercent(), all.maxConsecutiveLost, all.early, all.maxDwellMs);
		EXPECT_EQ(0, all.early) << impulse;
		EXPECT_LE(all.maxDwellMs, kBackupMs - kBackupMarginMs) << impulse;
		EXPECT_LE(all.maxConsecutiveLost, 1) << impulse;
		EXPECT_LE(all.lostPercent(), 6.0) << impulse;
	}
}

// ---------------------------------------------------------------------------------------------------------------------
// Task 8: failure modes. Glitch tooth, window reset, post-start latch (own bit), per-bit identity, mode flips.
// ---------------------------------------------------------------------------------------------------------------------

TEST(edgeDwell, aGlitchToothCostsAtMostTwoConsecutiveSparksAndIsNeverWorseThanTodayOnAngle) {
	for (int kind = 0; kind < 2; kind++) {
		for (int pos = 66; pos < 80; pos++) {
			double minErr[2] = {99, 99};
			int noCharge[2] = {0, 0};
			int consec[2] = {0, 0};
			size_t sparkCount[2] = {0, 0};
			for (int on = 0; on < 2; on++) {
				Profile p = steady(210, 140);
				p.noise = 0.02f;
				p.seed = 9;
				if (kind == 0) {
					p.missedTooth = pos;
				} else {
					p.noiseToothBefore = pos;
				}
				Trace t = run(p, on == 1);
				auto sparks = sparksAfter(t, 60);
				sparkCount[on] = sparks.size();
				int streak = 0;
				for (const Spark& s : sparks) {
					minErr[on] = std::min(minErr[on], s.errDeg);
					if (s.dwellMs < 0) {
						noCharge[on]++;
						consec[on] = std::max(consec[on], ++streak);
					} else {
						streak = 0;
						EXPECT_LE(s.dwellMs, kBackupMs + 0.1) << (kind ? "noise " : "missed ") << pos;
					}
				}
			}
			const char* what = kind ? "noise tooth" : "missed tooth";
			// the 30 deg phase slip a glitch leaves behind is the decoder's, identical with the bit on or off; edge mode must
			// not make the angle worse than that by more than 2.5 deg: a missing tooth never arrives, so the 3 x dwell backup
			// (13.5 ms after the charge) fires instead
			EXPECT_GE(minErr[1], minErr[0] - 2.5) << what << " " << pos;
			// today every spark keeps its charge because both edges come off one tooth; edge mode may lose some
			EXPECT_EQ(0, noCharge[0]) << what << " " << pos;
			EXPECT_LE(noCharge[1], 2) << what << " " << pos;
			EXPECT_LE(consec[1], 2) << what << " " << pos;
			EXPECT_LE(std::abs((int)sparkCount[1] - (int)sparkCount[0]), 1) << what << " " << pos;
		}
	}
}

TEST(edgeDwell, withoutAFullSpeedWindowTheSparkIsStillToothLockedAndTheChargeFallsBackToStoredRpm) {
	std::vector<int> windowValid;
	std::vector<int> active;
	Profile p = steady(210, 150);
	p.noise = 0.02f;
	p.seed = 3;
	p.beforeTooth = [](size_t n) {
		if (n == 90) {
			// what a trigger error, an engine stop or a phase re-sync does
			engine->triggerCentral.toothWindow.reset();
		}
	};
	p.afterTooth = [&](size_t) {
		windowValid.push_back(engine->ignitionState.edgeDwellWindowValid);
		active.push_back(engine->ignitionState.edgeDwellActive);
	};
	Trace t = run(p, true);

	// invalid right after the reset, valid again once 12 intervals (360 deg) have been seen
	EXPECT_EQ(1, windowValid[89]);
	EXPECT_EQ(0, windowValid[91]);
	EXPECT_EQ(0, windowValid[100]);
	EXPECT_EQ(1, windowValid[103]);
	// edgeDwellActive is per scheduling tooth (an event is scheduled every 6th tooth), so look at the whole stretch
	int activeDuring = 0;
	for (size_t k = 91; k <= 103; k++) {
		activeDuring += active[k];
	}
	EXPECT_GE(activeDuring, 2) << "edge mode stays in use (stored-rpm fallback) while the window refills";

	double tReset = NT2US((double)t.teeth[90].nt);
	int during = 0;
	for (const Spark& s : sparksAfter(t, 60)) {
		EXPECT_NEAR(2.0, s.errDeg, 0.05);
		if (s.tUs > tReset && s.tUs < tReset + 360000) {
			during++;
			EXPECT_GT(s.dwellMs, 0) << "no charge while falling back to stored rpm";
		}
	}
	EXPECT_GE(during, 2);
}

namespace {

// a start that catches (stored rpm passes cranking.rpm: state RUNNING) and then really falls to 400 rpm while the stored
// rpm still says 600+: the logged 18:00 start (stored 547, real about 400). decayPerTooth = speed factor per 30 deg tooth.
Profile postSwitchStart(double decayPerTooth) {
	Profile p = steady(210, 60);
	for (double r : {400.0, 600.0, 800.0}) {
		p.rpm.push_back(r);
	}
	for (int i = 0; i < 30; i++) {
		p.rpm.push_back(800);
	}
	double r = 800;
	for (int i = 0; i < 40; i++) {
		r = std::max(400.0, r * decayPerTooth);
		p.rpm.push_back(r);
	}
	for (int i = 0; i < 100; i++) {
		p.rpm.push_back(400);
	}
	p.ripple = 0.3f;
	p.noise = 0.03f;
	p.seed = 3;
	return p;
}

struct PostSwitchRun {
	Trace trace;
	std::vector<int> state;
	std::vector<int> unstable;
	std::vector<int> sparkEdge;
	std::vector<float> cachedRpm;
	std::vector<double> realRpm;
};

PostSwitchRun runPostSwitch(double decay, bool latch, const std::function<void()>& tweak = nullptr) {
	PostSwitchRun r;
	Profile p = postSwitchStart(decay);
	r.realRpm = p.rpm;
	p.afterTooth = [&](size_t) {
		r.state.push_back((int)engine->rpmCalculator.getState());
		r.unstable.push_back(engine->ignitionState.edgeDwellUnstable);
		r.sparkEdge.push_back(engine->ignitionState.edgeDwellSparkEdge);
		r.cachedRpm.push_back(engine->rpmCalculator.getCachedRpm());
	};
	Bits bits;
	bits.latch = latch;
	r.trace = runTraceBits(p, bits, [&]() {
		engineConfiguration->fixedTiming = 18.5f;
		if (tweak) {
			tweak();
		}
	});
	return r;
}

size_t toothOfSpark(const Trace& t, const Spark& s) {
	size_t k = 0;
	while (k + 1 < t.teeth.size() && NT2US((double)t.teeth[k + 1].nt) <= s.tUs) {
		k++;
	}
	return k;
}

} // namespace

TEST(edgeDwell, afterTheSwitchToRunningAnAbruptSlowdownNeverGetsASparkMoreAdvancedThanCommanded) {
	// commanded 18.5 BTDC (the idle value logged at 18:00). Abrupt: speed falls to 0.88 x per tooth (about 53 percent
	// per 180 deg), the per-cylinder period jumps far more than 18.75 percent and the latch takes over for a cycle.
	for (double decay : {0.88, 0.80}) {
		PostSwitchRun off = runPostSwitch(decay, false);
		PostSwitchRun on = runPostSwitch(decay, true);
		double offWorst = 99, onWorst = 99;
		int staleSparksOn = 0;
		for (const Spark& s : analyze(off.trace)) {
			size_t k = toothOfSpark(off.trace, s);
			if (k < off.state.size() && off.state[k] == RUNNING && off.cachedRpm[k] > 1.15 * off.realRpm[k]) {
				offWorst = std::min(offWorst, s.errDeg);
			}
		}
		for (const Spark& s : analyze(on.trace)) {
			size_t k = toothOfSpark(on.trace, s);
			if (k < on.state.size() && on.state[k] == RUNNING && on.cachedRpm[k] > 1.15 * on.realRpm[k]) {
				staleSparksOn++;
				onWorst = std::min(onWorst, s.errDeg);
				EXPECT_GE(s.errDeg, -0.05) << "decay " << decay << " tooth " << k << ": stale-rpm spark more advanced than commanded";
				EXPECT_EQ(1, on.sparkEdge[k]) << "tooth " << k;
			}
		}
		printf("post-switch decay %.2f: stale-rpm sparks in RUNNING, worst delivered minus commanded: today %.1f deg, edge mode %.1f deg (%d sparks)\n",
			decay, offWorst, onWorst, staleSparksOn);
		EXPECT_LE(offWorst, -5.0) << "the scenario must reproduce the early stale-rpm sparks of today's firmware";
		EXPECT_GE(staleSparksOn, 3) << decay;
		Integrity i = integrity(on.trace);
		EXPECT_EQ(0, i.doubleCharge + i.doubleSpark + i.dangling) << decay;
		EXPECT_EQ(0, on.trace.sparkOutOfOrder) << decay;
	}
}

TEST(edgeDwell, aSlowDecayBelowTheDetectorThresholdIsNotCoveredAndThatIsPinned) {
	// 0.97 x per tooth (about 17 percent per 180 deg) stays under the 18.75 percent rule: today's early stale-rpm sparks remain.
	// This is a known limit of the Mitsubishi style detector, listed under Design questions; the test pins it so that
	// nobody believes the latch covers every stale-rpm case.
	PostSwitchRun on = runPostSwitch(0.97, true);
	double worst = 99;
	for (const Spark& s : analyze(on.trace)) {
		size_t k = toothOfSpark(on.trace, s);
		if (k < on.state.size() && on.state[k] == RUNNING && on.cachedRpm[k] > 1.15 * on.realRpm[k]) {
			worst = std::min(worst, s.errDeg);
		}
	}
	printf("post-switch slow decay 0.97: worst stale-rpm spark with the flag on: %.1f deg vs commanded\n", worst);
	EXPECT_LT(worst, -5.0);
}

TEST(edgeDwell, theLiveBitsSayWhichPathScheduledEachSpark) {
	PostSwitchRun on = runPostSwitch(0.88, true);
	int unstableTeeth = 0;
	bool sawEdge = false;
	bool sawLegacyAfterLatch = false;
	size_t firstUnstable = 0;
	for (size_t k = 0; k < on.unstable.size(); k++) {
		if (on.unstable[k] && !unstableTeeth++) {
			firstUnstable = k;
		}
		sawEdge |= on.sparkEdge[k];
	}
	for (size_t k = on.unstable.size() - 30; k < on.unstable.size(); k++) {
		sawLegacyAfterLatch |= (on.unstable[k] == 0 && on.sparkEdge[k] == 0 && on.state[k] == RUNNING);
	}
	EXPECT_GT(unstableTeeth, 24) << "latched for at least one full cycle (24 teeth)";
	EXPECT_GT(firstUnstable, 55u);
	EXPECT_TRUE(sawEdge);
	EXPECT_TRUE(sawLegacyAfterLatch) << "once the latch has released a RUNNING spark is a legacy spark: bit 0";
}

TEST(edgeDwell, steadyRunningNeverLatchesAndStaysOnTheLegacyPath) {
	for (double rpm : {800.0, 1000.0, 1200.0}) {
		for (double ripple : {0.3, 0.5}) {
			for (int impulseMode = 0; impulseMode < 3; impulseMode++) {
				for (float shift : {0.0f, 60.0f, 120.0f}) {
					Profile p = steady(rpm, 300);
					p.ripple = (float)ripple;
					p.noise = 0.03f;
					p.seed = 21;
					p.tdcShiftDeg = shift;
					p.impulseMode = impulseMode;
					p.impulse = 0.05f; // 5 percent per-stroke combustion variation: a rough but healthy idle
					int latched = 0;
					int edgeSparks = 0;
					p.afterTooth = [&](size_t n) {
						if (n > 30) { // the very first stored-rpm update is allowed to look like a start
							latched += engine->ignitionState.edgeDwellUnstable;
							edgeSparks += engine->ignitionState.edgeDwellSparkEdge;
						}
					};
					Bits both;
					both.edge = true;
					both.latch = true;
					Trace t = runTraceBits(p, both);
					EXPECT_EQ(RUNNING, t.finalState);
					EXPECT_EQ(0, latched) << rpm << " ripple " << ripple << " impulse mode " << impulseMode << " shift " << shift;
					EXPECT_EQ(0, edgeSparks) << rpm;
				}
			}
		}
	}
}

TEST(edgeDwell, aTipInOrClutchDumpAboveTheThresholdNeverLatches) {
	for (int variant = 0; variant < 3; variant++) {
		Profile p = steady(1500, 60);
		p.ripple = 0.3f;
		p.noise = 0.03f;
		p.seed = 4;
		if (variant == 0) {
			for (double r : {1900.0, 2500.0, 3200.0, 3600.0}) { // tip-in: 1500 -> 3600 in 4 teeth
				p.rpm.push_back(r);
			}
			for (int i = 0; i < 40; i++) {
				p.rpm.push_back(3600);
			}
		} else if (variant == 1) {
			for (int i = 0; i < 20; i++) {
				p.rpm.push_back(3600);
			}
			for (double r : {2800.0, 2100.0, 1800.0}) { // clutch dump: 3600 -> 1800 in 3 teeth
				p.rpm.push_back(r);
			}
			for (int i = 0; i < 40; i++) {
				p.rpm.push_back(1800);
			}
		} else {
			for (int i = 0; i < 24; i++) { // blip: up and straight back down, both above the limit
				p.rpm.push_back(2400);
			}
			for (int i = 0; i < 24; i++) {
				p.rpm.push_back(1500);
			}
		}
		int latched = 0;
		p.afterTooth = [&](size_t n) {
			if (n > 30) {
				latched += engine->ignitionState.edgeDwellUnstable;
			}
		};
		Bits both;
		both.edge = true;
		both.latch = true;
		Trace t = runTraceBits(p, both);
		EXPECT_EQ(0, latched) << "variant " << variant;
		Integrity i = integrity(t);
		EXPECT_EQ(0, i.doubleCharge + i.doubleSpark + i.dangling) << variant;
	}
}

TEST(edgeDwell, modeFlipsBetweenTheDwellToothAndTheSparkToothNeverDoubleOrStrandACharge) {
	// the flag is toggled before each of 24 consecutive teeth (every alignment against the four ignition events):
	// that flips edge <-> legacy between the dwell tooth and the spark tooth of some event, in both directions
	for (int direction = 0; direction < 2; direction++) {
		for (int n = 60; n < 84; n++) {
			Profile p = steady(210, 160);
			p.ripple = 0.3f;
			p.noise = 0.03f;
			p.seed = 5;
			const bool initial = direction == 0;
			p.beforeTooth = [n, initial](size_t i) {
				if ((int)i == n) {
					engineConfiguration->crankingEdgeDwell = !initial;
				}
			};
			Trace t = runTrace(p, initial);
			Integrity i = integrity(t);
			EXPECT_EQ(0, i.doubleCharge + i.doubleSpark + i.dangling) << "direction " << direction << " at tooth " << n;
			EXPECT_EQ(0, i.noCharge) << "direction " << direction << " at tooth " << n;
			EXPECT_EQ(0, t.sparkOutOfOrder) << "direction " << direction << " at tooth " << n;
			EXPECT_GE(i.sparks, 21) << "direction " << direction << " at tooth " << n;
		}
	}
}

TEST(edgeDwell, theSwitchToRunningInsideTheFlashKeepsEveryChargeAndSpark) {
	Profile p = flash();
	p.noise = 0.03f;
	p.seed = 777;
	Trace t = runTrace(p, true);
	EXPECT_EQ(RUNNING, t.finalState);
	Integrity i = integrity(t);
	EXPECT_EQ(0, i.doubleCharge + i.doubleSpark + i.dangling);
	EXPECT_EQ(0, t.sparkOutOfOrder);
}

TEST(edgeDwell, belowTheInstabilityLimitTheRpmUpdateStillNeverShortensDwellMoreThanToday) {
	// all three bits on, 800 -> 1600 rpm long after the start: the post-start window is closed, so the latch cannot act and
	// the E4 branch alone governs the dwell. It must not be worse than today's legacy dwell.
	double worst[2];
	for (int on = 0; on < 2; on++) {
		EngineTestHelper eth(engine_type_e::TEST_ENGINE);
		configureCrank(eth);
		engineConfiguration->dwellFixOnRpmUpdate = on == 1;
		engineConfiguration->crankingEdgeDwell = on == 1;
		engineConfiguration->edgeDwellPostStartLatch = on == 1;
		setArrayValues(config->sparkDwellValues, 4.0f);
		Profile p = flash(230);
		p.rpm.resize(63 + 120, 800);
		for (double r : {1000.0, 1300.0, 1600.0}) {
			p.rpm.push_back(r);
		}
		for (int i = 0; i < 60; i++) {
			p.rpm.push_back(1600);
		}
		p.ripple = 0.05f;
		Trace t;
		drive(eth, t, p);
		double tRamp = NT2US((double)t.teeth[63 + 120].nt);
		double w = 9;
		for (const Spark& s : analyze(t)) {
			if (s.tUs > tRamp && s.tUs < tRamp + 120000 && s.dwellMs > 0) {
				w = std::min(w, s.dwellMs / 4.0);
			}
		}
		worst[on] = w;
	}
	printf("E4b worst actual/4ms dwell across the 800->1600 rpm update: flag off %.2f, on %.2f\n", worst[0], worst[1]);
	EXPECT_GE(worst[1], worst[0] - 0.02);
}

namespace {

// start that flashes to 800 at tooth 60-63 and then idles. The stored rpm (updated once per 720 deg) passes cranking.rpm at
// the next cycle boundary, so the CRANKING->RUNNING switch happens a few teeth later: it is MEASURED, not assumed. The
// post-start window is 8 cylinder events = 48 teeth (kWindowTeeth) after that switch.
constexpr int kWindowTeeth = 48;

Profile startThenIdle(int teethAfterSwitch) {
	Profile p = flash();
	p.rpm.resize(63 + teethAfterSwitch, 800);
	p.ripple = 0.3f;
	p.noise = 0.03f;
	p.seed = 31;
	return p;
}

// tooth index at which the state first reads RUNNING for the start profile (independent of any bit). Measured in a
// standalone run BEFORE any test run (fixture SetUp), never from inside another run.
size_t measureSwitchTooth() {
	Profile p = startThenIdle(100);
	size_t first = 0;
	p.afterTooth = [&](size_t n) {
		if (!first && engine->rpmCalculator.getState() == RUNNING) {
			first = n;
		}
	};
	runTraceBits(p, Bits());
	return first;
}

struct LatchObs {
	int unstableOutside = 0;
	int edgeOutside = 0;
	int unstableInside = 0;
	int running = 0;
};

LatchObs observeLatch(Profile& p, const Bits& bits, size_t switchAt) {
	LatchObs o;
	p.afterTooth = [&](size_t n) {
		if (engine->rpmCalculator.getState() == RUNNING) {
			o.running++;
		}
		if (n >= switchAt + kWindowTeeth + 2) {
			o.unstableOutside += engine->ignitionState.edgeDwellUnstable;
			o.edgeOutside += engine->ignitionState.edgeDwellActive;
		} else {
			o.unstableInside += engine->ignitionState.edgeDwellUnstable;
		}
	};
	runTraceBits(p, bits);
	return o;
}

} // namespace

class edgeDwellPostStart : public ::testing::Test {
protected:
	void SetUp() override {
		sw = measureSwitchTooth();
	}
	size_t sw = 0;
};

TEST_F(edgeDwellPostStart, theMeasuredSwitchToothIsAfterTheFlashAndLongBeforeTheWindowEnds) {
	EXPECT_GT(sw, 63u);
	EXPECT_LT(sw, 63u + 60u);
}

TEST_F(edgeDwellPostStart, theLatchNeverActsOutsideThePostStartWindowOnASagAndCatchIdle) {
	// idle 800 -> 320 -> 800 rpm long after the start (the stored rpm lags the sag exactly like at the 18:00 start): a real
	// speed step, the latch must stay out because the post-start window is over
	for (double bottom : {320.0, 450.0}) {
		Profile p = startThenIdle(100);
		for (double r : {650.0, 500.0, bottom}) {
			p.rpm.push_back(r);
		}
		for (int i = 0; i < 18; i++) {
			p.rpm.push_back(bottom);
		}
		for (double r : {500.0, 650.0}) {
			p.rpm.push_back(r);
		}
		for (int i = 0; i < 60; i++) {
			p.rpm.push_back(800);
		}
		// the sag starts after the window: 100 teeth of idle after the flash is later than the measured switch + 48
		ASSERT_GT(63u + 100u, sw + kWindowTeeth);
		Bits bits;
		bits.edge = true;
		bits.latch = true;
		LatchObs o = observeLatch(p, bits, sw);
		EXPECT_EQ(0, o.unstableOutside) << "bottom " << bottom;
		EXPECT_EQ(0, o.edgeOutside) << "bottom " << bottom << ": RUNNING spark scheduled by the edge path outside the window";
	}
}

TEST_F(edgeDwellPostStart, theLatchNeverActsOutsideThePostStartWindowOnAnIdleMisfireOfTenToTwentyPercent) {
	for (int impulseMode : {1, 2}) {
		for (float impulse : {0.10f, 0.15f, 0.20f}) {
			for (float shift : {0.0f, 60.0f, 120.0f}) {
				Profile p = startThenIdle(220);
				p.impulseMode = impulseMode;
				p.impulse = impulse;
				p.tdcShiftDeg = shift;
				Bits bits;
				bits.edge = true;
				bits.latch = true;
				LatchObs o = observeLatch(p, bits, sw);
				EXPECT_EQ(0, o.unstableOutside) << "mode " << impulseMode << " impulse " << impulse << " shift " << shift;
				EXPECT_EQ(0, o.edgeOutside) << "mode " << impulseMode << " impulse " << impulse << " shift " << shift;
			}
		}
	}
}

TEST_F(edgeDwellPostStart, theLatchWindowClosesAfterEightCylinderEventsNotBefore) {
	// the same abrupt step injected at different distances after the switch: inside 48 teeth it latches, after it never does
	auto stepRun = [&](int stepAfter) {
		Profile p = startThenIdle(60);
		p.rpm.resize(sw + stepAfter, 800);
		for (int i = 0; i < 30; i++) {
			p.rpm.push_back(400);
		}
		for (int i = 0; i < 60; i++) {
			p.rpm.push_back(800);
		}
		Bits bits;
		bits.latch = true;
		bool any = false;
		p.afterTooth = [&](size_t) { any |= engine->ignitionState.edgeDwellUnstable; };
		runTraceBits(p, bits);
		return any;
	};
	for (int stepAfter : {5, 20, 40}) {
		EXPECT_TRUE(stepRun(stepAfter)) << "a step " << stepAfter << " teeth after the switch must latch";
	}
	for (int stepAfter : {60, 90}) {
		EXPECT_FALSE(stepRun(stepAfter)) << "step " << stepAfter << " teeth after the switch is outside the window";
	}
}

TEST(edgeDwell, inTheWindowARunningCommandBelowTwelveUsesTheLegacyPathNotThe18AtdcTooth) {
	// the logged 18:00 slowdown with idle timing 10 BTDC: the latch trips, but the commanded advance is below 12, so every
	// event keeps today's path: the coil edges are bit-identical to the same run with the latch bit off
	for (double decay : {0.88, 0.80}) {
		PostSwitchRun on = runPostSwitch(decay, true, []() { engineConfiguration->fixedTiming = 10.0f; });
		PostSwitchRun off = runPostSwitch(decay, false, []() { engineConfiguration->fixedTiming = 10.0f; });
		int unstable = 0;
		for (size_t k = 0; k < on.unstable.size(); k++) {
			unstable += on.unstable[k];
		}
		EXPECT_GT(unstable, 24) << "the detector must still trip (it is the event gate that keeps the legacy path)";
		EXPECT_EQ(coilFingerprint(off.trace), coilFingerprint(on.trace)) << decay;
		EXPECT_EQ(off.trace.coils.size(), on.trace.coils.size());
	}
}

TEST(edgeDwell, eachBitIsAnIdentityOnItsOwnTerritoryWhenOffAndNeverReachesIntoTheOthers) {
	// (a) crankingEdgeDwell off, the other two on: a pure cranking trace is today's coil schedule, edge never active
	{
		Profile p = steady(210, 100);
		p.noise = 0.03f;
		p.seed = 1;
		Bits others;
		others.latch = true;
		others.e4 = true;
		bool anyEdge = false;
		p.afterTooth = [&](size_t) { anyEdge |= engine->ignitionState.edgeDwellActive; };
		Trace a = runTraceBits(p, others);
		p.afterTooth = nullptr;
		Bits e4Only;
		e4Only.e4 = true;
		Trace ref = runTraceBits(p, e4Only); // the latch bit is inert before the switch to RUNNING
		Trace off = runTrace(p, false);
		EXPECT_FALSE(anyEdge);
		EXPECT_EQ(coilFingerprint(ref), coilFingerprint(a)) << "with the cranking bit off, the latch bit must not change a pure cranking run";
		// and with only the latch bit set, the pure cranking run IS today's schedule
		Bits latchOnly;
		latchOnly.latch = true;
		EXPECT_EQ(coilFingerprint(off), coilFingerprint(runTraceBits(p, latchOnly)));
	}
	// (b) latch off, the other two on: after the switch to RUNNING the latch never raises and edge is not active in RUNNING
	{
		for (double decay : {0.88, 0.80}) {
			Profile p = postSwitchStart(decay);
			Bits others;
			others.edge = true;
			others.e4 = true;
			int unstable = 0;
			int activeRunning = 0;
			p.afterTooth = [&](size_t) {
				unstable += engine->ignitionState.edgeDwellUnstable;
				activeRunning += engine->rpmCalculator.getState() == RUNNING && engine->ignitionState.edgeDwellActive;
			};
			runTraceBits(p, others, []() { engineConfiguration->fixedTiming = 18.5f; });
			EXPECT_EQ(0, unstable) << decay;
			EXPECT_EQ(0, activeRunning) << decay;
		}
	}
	// (c) E4 off, the other two on: a smooth speed-up reproduces today's dwell collapse (E4 is the only thing that fixes it)
	{
		double worst[2];
		for (int e4 = 0; e4 < 2; e4++) {
			EngineTestHelper eth(engine_type_e::TEST_ENGINE);
			configureCrank(eth);
			engineConfiguration->crankingEdgeDwell = true;
			engineConfiguration->edgeDwellPostStartLatch = true;
			engineConfiguration->dwellFixOnRpmUpdate = e4 == 1;
			setArrayValues(config->sparkDwellValues, 4.0f);
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
			Trace t;
			drive(eth, t, p);
			double w = 9;
			for (const Spark& s : analyze(t)) {
				if (s.tUs > NT2US((double)t.teeth[90].nt) && s.dwellMs > 0) {
					w = std::min(w, s.dwellMs / 4.0);
				}
			}
			worst[e4] = w;
		}
		printf("E4 identity: worst dwell ratio with E4 off %.2f, on %.2f (other two bits on)\n", worst[0], worst[1]);
		EXPECT_LT(worst[0], 0.65);
		EXPECT_GT(worst[1], worst[0] + 0.15);
	}
}

TEST(edgeDwell, aSyncLossBetweenTheLegacyDwellToothAndTheEdgeDwellToothClearsThePendingEdgeDecision) {
	// an event the edge path has only half scheduled (edgeDwellCycleState 2) must not survive a trigger error: with the bits
	// off afterwards the next cycle takes the legacy path for every event (no edge spark, states back to 0)
	Profile p = steady(210, 200);
	p.noise = 0.02f;
	p.seed = 11;
	bool injected = false;
	size_t injectedAt = 0;
	int pendingAfter = -1;
	int edgeSparksAfter = 0;
	p.beforeTooth = [&](size_t n) {
		if (injected || n < 70) {
			return;
		}
		bool pending = false;
		for (size_t i = 0; i < efi::size(getIgnitionEvents()->elements); i++) {
			pending |= getIgnitionEvents()->elements[i].edgeDwellCycleState == 2;
		}
		if (pending) {
			injected = true;
			injectedAt = n;
			engineConfiguration->crankingEdgeDwell = false;
			engine->triggerCentral.triggerState.onTriggerError();
			pendingAfter = 0;
			for (size_t i = 0; i < efi::size(getIgnitionEvents()->elements); i++) {
				pendingAfter += getIgnitionEvents()->elements[i].edgeDwellCycleState == 2;
			}
		}
	};
	p.afterTooth = [&](size_t n) {
		if (injected && n > injectedAt) {
			edgeSparksAfter += engine->ignitionState.edgeDwellSparkEdge;
			for (size_t i = 0; i < efi::size(getIgnitionEvents()->elements); i++) {
				EXPECT_NE(2, getIgnitionEvents()->elements[i].edgeDwellCycleState) << "tooth " << n;
			}
		}
	};
	// a large command (38 BTDC) makes the edge dwell tooth later than the legacy one, so an event sits at state 2 between teeth
	Trace t = runTrace(p, true, []() { engineConfiguration->fixedTiming = 38.0f; });
	ASSERT_TRUE(injected) << "the scenario never produced a pending edge event";
	EXPECT_EQ(0, pendingAfter);
	EXPECT_EQ(0, edgeSparksAfter);
	Integrity i = integrity(t);
	EXPECT_EQ(0, i.doubleCharge + i.doubleSpark);
}

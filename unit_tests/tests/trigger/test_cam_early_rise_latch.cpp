/**
 * @file test_cam_early_rise_latch.cpp
 *
 * camEarlyRiseLatch: a single tooth cam rise that arrives after the 12 tooth crank synchronized but before the first RPM value
 * exists is remembered (as a primary synchronization counter value) and applied right after the first RPM tooth.
 * Car geometry: 12crank/24cam, VVT_SINGLE_TOOTH, single coil, phase sync required, no faster spin-up.
 */

#include "pch.h"

#include <vector>

namespace {

constexpr int kToothUs = 27000; // about 185 rpm

struct CamRise {
	int afterTooth;  // delivered after this (1 based) primary tooth
	int offsetUs;    // time after that tooth
};

struct Snapshot {
	bool phase;
	bool rpm;
	int counter;
	float phaseAdjustment;
	float decodedPhase;
	bool latchValid;
	bool used;
};

struct DriveResult {
	std::vector<Snapshot> afterTooth; // index 0 = after tooth 1
	int sparksBeforePhase = 0;
	int sparksAfterPhase = 0;
	int firstPhaseTooth = -1;
	int firstRpmTooth = -1;
	size_t closeToSyncWarnings = 0; // C6675 CUSTOM_VVT_SYNC_POSITION only
	bool ignitionBlockedByPhaseWhileNoPhase = true;
};

void configure(EngineTestHelper& eth, bool bitOn) {
	engineConfiguration->isFasterEngineSpinUpEnabled = false;
	engineConfiguration->isPhaseSyncRequiredForIgnition = true;
	engineConfiguration->ignitionMode = IM_ONE_COIL;
	engineConfiguration->cylindersCount = 4;
	engineConfiguration->firingOrder = FO_1_3_4_2;
	engineConfiguration->isInjectionEnabled = false;
	engineConfiguration->isIgnitionEnabled = true;
	engineConfiguration->cranking.rpm = 500;
	engineConfiguration->globalTriggerAngleOffset = 162;
	engineConfiguration->timingMode = TM_FIXED;
	engineConfiguration->fixedTiming = 10;
	engineConfiguration->vvtMode[0] = VVT_SINGLE_TOOTH;
	engineConfiguration->camEarlyRiseLatch = bitOn;
	eth.setTriggerType(trigger_type_e::TT_12_TOOTH_CRANK);
}

Snapshot snap() {
	TriggerCentral* tc = getTriggerCentral();
	return {
		tc->triggerState.hasSynchronizedPhase(),
		Sensor::getOrZero(SensorType::Rpm) > 0,
		(int)tc->triggerState.getSynchronizationCounter(),
		(float)tc->triggerState.m_phaseAdjustment,
		tc->currentEngineDecodedPhase,
		tc->camEarlyRise.valid,
		tc->camEarlyRiseUsed,
	};
}

/** teeth are delivered 27 ms apart; cam rises are delivered between teeth (they do not stretch the tooth interval) */
DriveResult drive(EngineTestHelper& eth, int teeth, const std::vector<CamRise>& rises, std::function<void(int)> hook = {}) {
	DriveResult r;
	int sparks = 0;
	// a limited spark (limp manager said no) still runs through the event bookkeeping: count only the real ones
	engine->onIgnitionEvent = [&](IgnitionEvent* e, bool state) {
		if (!state && !e->wasSparkLimited) {
			sparks++;
		}
	};
	int consumed = 0; // part of the coming tooth interval already spent on a cam rise
	for (int t = 1; t <= teeth; t++) {
		eth.moveTimeForwardAndInvokeEventsUs(kToothUs - consumed);
		eth.firePrimaryTriggerRise();
		consumed = 0;

		Snapshot s = snap();
		r.afterTooth.push_back(s);
		if (s.rpm && r.firstRpmTooth < 0) {
			r.firstRpmTooth = t;
		}
		if (s.phase && r.firstPhaseTooth < 0) {
			r.firstPhaseTooth = t;
		}
		getLimpManager()->onFastCallback();
		if (!s.phase) {
			EXPECT_EQ(ClearReason::EnginePhase, getLimpManager()->allowIgnition().reason) << "tooth " << t;
			r.sparksBeforePhase = sparks;
		}
		if (hook) {
			hook(t);
		}
		for (auto& c : rises) {
			if (c.afterTooth == t) {
				eth.moveTimeForwardAndInvokeEventsUs(c.offsetUs - consumed);
				consumed = c.offsetUs;
				hwHandleVvtCamSignal(TriggerValue::RISE, getTimeNowNt(), 0);
			}
		}
	}
	r.sparksAfterPhase = sparks - r.sparksBeforePhase;
	auto* w = eth.recentWarnings();
	for (int i = 0; i < w->getCount(); i++) {
		if (w->get(i).Code == ObdCode::CUSTOM_VVT_SYNC_POSITION) {
			r.closeToSyncWarnings++;
		}
	}
	return r;
}

} // namespace

using std::vector;

TEST(camEarlyRiseLatch, bitDefaultsToOff) {
	EngineTestHelper eth(engine_type_e::TEST_ENGINE);
	EXPECT_FALSE(engineConfiguration->camEarlyRiseLatch);
	EXPECT_FALSE(getTriggerCentral()->camEarlyRiseUsed);
}

// (a) pins today's behaviour: a rise between primary sync and the first RPM value is thrown away, phase waits for the next one
TEST(camEarlyRiseLatch, bitOffDiscardsTheEarlyRiseAndPhaseWaitsForTheNextOne) {
	EngineTestHelper eth(engine_type_e::TEST_ENGINE);
	configure(eth, /*bitOn*/ false);
	DriveResult r = drive(eth, 70, {{10, 5000}, {34, 5000}});
	ASSERT_GT(r.firstRpmTooth, 10) << "the early rise must really be before RPM";
	// the second rise is delivered after tooth 34, the snapshot of tooth 35 is the first one that can show phase
	EXPECT_EQ(35, r.firstPhaseTooth);
	for (const auto& s : r.afterTooth) {
		EXPECT_FALSE(s.latchValid);
		EXPECT_FALSE(s.used);
	}
	EXPECT_EQ(0, r.sparksBeforePhase);
}

// the bit off must also ignore a latch-eligible situation completely (nothing is recorded)
TEST(camEarlyRiseLatch, bitOffNeverRecordsAnything) {
	EngineTestHelper eth(engine_type_e::TEST_ENGINE);
	configure(eth, false);
	DriveResult r = drive(eth, 27, {{10, 5000}});
	EXPECT_EQ(-1, r.firstRpmTooth) << "still no RPM at tooth 27: the rise is 'early'";
	EXPECT_FALSE(getTriggerCentral()->camEarlyRise.valid);
}

// (b) phase at the first RPM tooth, and the same engine phase as the same rise delivered one cam period later (after RPM)
static void checkEquivalence(int tooth, int offsetUs) {
	SCOPED_TRACE(testing::Message() << "rise after tooth " << tooth << " +" << offsetUs << " us");
	const int kTeeth = 100;

	DriveResult early;
	{
		EngineTestHelper eth(engine_type_e::TEST_ENGINE);
		configure(eth, true);
		early = drive(eth, kTeeth, {{tooth, offsetUs}});
	}
	DriveResult late;
	{
		EngineTestHelper eth(engine_type_e::TEST_ENGINE);
		configure(eth, true);
		late = drive(eth, kTeeth, {{tooth + 24, offsetUs}});
	}

	ASSERT_GT(early.firstRpmTooth, tooth) << "the early rise must be before RPM";
	ASSERT_LT(late.firstRpmTooth, tooth + 24) << "the twin rise must be after RPM";
	// phase right at the first RPM tooth (not one cam period later)
	EXPECT_EQ(early.firstRpmTooth, early.firstPhaseTooth);
	EXPECT_GT(late.firstPhaseTooth, early.firstPhaseTooth);
	EXPECT_TRUE(early.afterTooth.back().used);
	EXPECT_FALSE(late.afterTooth.back().used);

	// from the moment both know the phase, the counter, the decoded phase and the adjustment agree tooth by tooth
	for (int t = late.firstPhaseTooth; t <= kTeeth; t++) {
		const Snapshot& a = early.afterTooth[t - 1];
		const Snapshot& b = late.afterTooth[t - 1];
		ASSERT_TRUE(a.phase && b.phase) << "tooth " << t;
		EXPECT_EQ(a.counter, b.counter) << "tooth " << t;
		EXPECT_FLOAT_EQ(a.decodedPhase, b.decodedPhase) << "tooth " << t;
		EXPECT_FLOAT_EQ(a.phaseAdjustment, b.phaseAdjustment) << "tooth " << t;
	}
	// the +-7 degree warning is judged on the same angle as the normal path
	EXPECT_EQ(early.closeToSyncWarnings, late.closeToSyncWarnings);
}

// first crank revolution of the 720 (counter 5), second revolution (counter 16), one tooth before RPM (counter 23);
// each with the rise right after a tooth, mid interval and right before the next tooth (tooth edge)
TEST(camEarlyRiseLatch, latchedRiseGivesTheSamePhaseAsTheSameRiseDeliveredAfterRpm) {
	const std::pair<int, int> cases[] = {
		{9, 5000}, {9, 13500}, {9, 26000}, {9, 300},
		{20, 5000}, {20, 26000}, {20, 300},
		{27, 5000}, {27, 26500},
		{5, 5000},
	};
	for (const auto& c : cases) {
		checkEquivalence(c.first, c.second);
	}
}

// a rise right after the sync tooth is 24 counter steps old when RPM arrives (dropped); one tooth later it is 23 (applied)
TEST(camEarlyRiseLatch, aRiseOlderThanOneCamPeriodIsDropped) {
	EngineTestHelper eth(engine_type_e::TEST_ENGINE);
	configure(eth, true);
	// tooth 4 is the primary sync tooth (counter 0); RPM arrives 24 counter steps later
	DriveResult r = drive(eth, 40, {{4, 5000}});
	ASSERT_GT(r.firstRpmTooth, 0);
	EXPECT_EQ(-1, r.firstPhaseTooth);
	EXPECT_FALSE(getTriggerCentral()->camEarlyRise.valid);
	EXPECT_FALSE(getTriggerCentral()->camEarlyRiseUsed);
}

TEST(camEarlyRiseLatch, aRiseOneCounterStepYoungerIsStillResolved) {
	EngineTestHelper eth(engine_type_e::TEST_ENGINE);
	configure(eth, true);
	DriveResult r = drive(eth, 40, {{5, 5000}});
	EXPECT_EQ(r.firstRpmTooth, r.firstPhaseTooth) << "23 counter steps old: still resolved";
}

// (c) trigger error: a latched rise belongs to a sync that is gone
TEST(camEarlyRiseLatch, aTriggerErrorDropsTheLatch) {
	EngineTestHelper eth(engine_type_e::TEST_ENGINE);
	configure(eth, true);
	bool latchedBefore = false, syncLost = false;
	DriveResult r = drive(eth, 90, {{10, 5000}}, [&](int t) {
		if (t == 14) {
			latchedBefore = getTriggerCentral()->camEarlyRise.valid;
			// a very late tooth is a trigger error
			eth.moveTimeForwardAndInvokeEventsUs(4 * kToothUs);
		}
		if (t == 15) {
			syncLost = !getTriggerCentral()->triggerState.getShaftSynchronized();
		}
	});
	EXPECT_TRUE(latchedBefore);
	EXPECT_TRUE(syncLost) << "the late tooth must really have been a sync loss";
	for (int t = 15; t <= 90; t++) {
		EXPECT_FALSE(r.afterTooth[t - 1].phase) << "tooth " << t << ": no cam rise since the error, so no phase";
		EXPECT_FALSE(r.afterTooth[t - 1].used) << "tooth " << t;
	}
	EXPECT_FALSE(r.afterTooth[14].latchValid);
	EXPECT_EQ(0, r.sparksBeforePhase + r.sparksAfterPhase) << "no spark without phase";
}

TEST(camEarlyRiseLatch, engineStopDropsTheLatch) {
	EngineTestHelper eth(engine_type_e::TEST_ENGINE);
	configure(eth, true);
	drive(eth, 12, {{10, 5000}});
	ASSERT_TRUE(getTriggerCentral()->camEarlyRise.valid);
	engine->rpmCalculator.setStopSpinning();
	EXPECT_FALSE(getTriggerCentral()->camEarlyRise.valid);
}

TEST(camEarlyRiseLatch, engineStopClearsTheUsedBit) {
	EngineTestHelper eth(engine_type_e::TEST_ENGINE);
	configure(eth, true);
	drive(eth, 40, {{10, 5000}});
	ASSERT_TRUE(getTriggerCentral()->camEarlyRiseUsed);
	engine->rpmCalculator.setStopSpinning();
	EXPECT_FALSE(getTriggerCentral()->camEarlyRiseUsed);
}

// switching the bit off while a rise is latched drops it
TEST(camEarlyRiseLatch, bitSwitchedOffWhileLatchedDropsTheLatch) {
	EngineTestHelper eth(engine_type_e::TEST_ENGINE);
	configure(eth, true);
	DriveResult r = drive(eth, 60, {{10, 5000}}, [&](int t) {
		if (t == 20) {
			ASSERT_TRUE(getTriggerCentral()->camEarlyRise.valid);
			engineConfiguration->camEarlyRiseLatch = false;
		}
	});
	EXPECT_EQ(-1, r.firstPhaseTooth);
	EXPECT_FALSE(getTriggerCentral()->camEarlyRise.valid);
}

// (d) no cam rise at all: nothing is invented
TEST(camEarlyRiseLatch, noCamRiseMeansNoPhaseAndNoSpark) {
	EngineTestHelper eth(engine_type_e::TEST_ENGINE);
	configure(eth, true);
	DriveResult r = drive(eth, 100, {});
	ASSERT_GT(r.firstRpmTooth, 0);
	EXPECT_EQ(-1, r.firstPhaseTooth);
	for (const auto& s : r.afterTooth) {
		EXPECT_FALSE(s.latchValid);
		EXPECT_FALSE(s.used);
	}
	EXPECT_EQ(0, r.sparksBeforePhase + r.sparksAfterPhase);
}

// a falling cam edge is never latched, only a rise is
TEST(camEarlyRiseLatch, aCamFallIsNeverLatched) {
	EngineTestHelper eth(engine_type_e::TEST_ENGINE);
	configure(eth, true);
	DriveResult r = drive(eth, 40, {}, [&](int t) {
		if (t == 10) {
			hwHandleVvtCamSignal(TriggerValue::FALL, getTimeNowNt(), 0);
		}
	});
	EXPECT_EQ(-1, r.firstPhaseTooth);
	EXPECT_FALSE(getTriggerCentral()->camEarlyRise.valid);
}

// a rise after RPM is the normal path: the bit changes nothing at all
TEST(camEarlyRiseLatch, aRiseAfterRpmIsIdenticalWithTheBitOnAndOff) {
	DriveResult on, off;
	{
		EngineTestHelper eth(engine_type_e::TEST_ENGINE);
		configure(eth, true);
		on = drive(eth, 80, {{40, 5000}});
	}
	{
		EngineTestHelper eth(engine_type_e::TEST_ENGINE);
		configure(eth, false);
		off = drive(eth, 80, {{40, 5000}});
	}
	ASSERT_EQ(on.afterTooth.size(), off.afterTooth.size());
	for (size_t i = 0; i < on.afterTooth.size(); i++) {
		EXPECT_EQ(on.afterTooth[i].phase, off.afterTooth[i].phase) << i;
		EXPECT_EQ(on.afterTooth[i].rpm, off.afterTooth[i].rpm) << i;
		EXPECT_EQ(on.afterTooth[i].counter, off.afterTooth[i].counter) << i;
		EXPECT_FLOAT_EQ(on.afterTooth[i].decodedPhase, off.afterTooth[i].decodedPhase) << i;
		EXPECT_FALSE(on.afterTooth[i].latchValid);
		EXPECT_FALSE(on.afterTooth[i].used);
	}
	EXPECT_EQ(on.sparksBeforePhase + on.sparksAfterPhase, off.sparksBeforePhase + off.sparksAfterPhase);
}

// only the newest early rise counts
TEST(camEarlyRiseLatch, aNewerEarlyRiseReplacesTheOlderOne) {
	DriveResult two, one;
	{
		EngineTestHelper eth(engine_type_e::TEST_ENGINE);
		configure(eth, true);
		two = drive(eth, 80, {{6, 5000}, {20, 5000}});
	}
	{
		EngineTestHelper eth(engine_type_e::TEST_ENGINE);
		configure(eth, true);
		one = drive(eth, 80, {{20, 5000}});
	}
	ASSERT_TRUE(two.afterTooth.back().phase);
	for (size_t i = (size_t)two.firstPhaseTooth - 1; i < two.afterTooth.size(); i++) {
		EXPECT_EQ(two.afterTooth[i].counter, one.afterTooth[i].counter) << i;
	}
}

// (f) the phase only opens the gate: sparks start after it and never before
TEST(camEarlyRiseLatch, sparkWaitsForPhaseAndRunsAfterIt) {
	EngineTestHelper eth(engine_type_e::TEST_ENGINE);
	configure(eth, true);
	DriveResult r = drive(eth, 80, {{10, 5000}});
	EXPECT_EQ(0, r.sparksBeforePhase);
	EXPECT_EQ(r.firstRpmTooth, r.firstPhaseTooth);
	EXPECT_GT(r.sparksAfterPhase, 0);
}

// the phase shift calls onEnginePhaseResync(), which drops the NEXT index 0 RPM sample. Applied after the first RPM (end of that
// tooth) the first RPM is therefore never lost; the dropped sample is the same one the normal post-RPM cam path drops.
TEST(camEarlyRiseLatch, firstRpmSurvivesTheLatchedApplicationAndStateStaysCranking) {
	std::vector<int> stateEarly, stateLate;
	std::vector<bool> rpmEarly, rpmLate;
	DriveResult early, late;
	auto rec = [](std::vector<int>& st, std::vector<bool>& rp) {
		return [&st, &rp](int) {
			st.push_back((int)engine->rpmCalculator.getState());
			rp.push_back(Sensor::getOrZero(SensorType::Rpm) > 0);
		};
	};
	{
		EngineTestHelper eth(engine_type_e::TEST_ENGINE);
		configure(eth, true);
		early = drive(eth, 100, {{20, 5000}}, rec(stateEarly, rpmEarly));
	}
	{
		EngineTestHelper eth(engine_type_e::TEST_ENGINE);
		configure(eth, true);
		late = drive(eth, 100, {{44, 5000}}, rec(stateLate, rpmLate));
	}
	ASSERT_GT(early.firstRpmTooth, 0);
	ASSERT_EQ(early.firstRpmTooth, late.firstRpmTooth) << "the latch must not delay the first RPM";
	ASSERT_EQ(early.firstRpmTooth, early.firstPhaseTooth);
	// once RPM exists it stays non-zero and CRANKING (not worse than the normal path), through the following cycle and beyond
	for (int t = early.firstRpmTooth; t <= 100; t++) {
		EXPECT_TRUE(rpmEarly[t - 1]) << "latched path, tooth " << t;
		EXPECT_EQ((int)spinning_state_e::CRANKING, stateEarly[t - 1]) << "latched path, tooth " << t;
		EXPECT_TRUE(rpmLate[t - 1]) << "normal path, tooth " << t;
		EXPECT_EQ((int)spinning_state_e::CRANKING, stateLate[t - 1]) << "normal path, tooth " << t;
	}
}

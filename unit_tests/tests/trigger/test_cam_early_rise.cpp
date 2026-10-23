#include "pch.h"

#include <vector>

// 12 tooth crank, single tooth cam, single coil, phase required for ignition. A tooth every 27 ms (185 rpm); the first
// RPM arrives after about 27 teeth, so a cam rise before that is "early".
namespace {

constexpr int toothUs = 27000;

struct CamRise {
	int afterTooth;
	int offsetUs;
};

struct Snapshot {
	bool phase;
	bool rpm;
	int counter;
	float decodedPhase;
	float phaseAdjustment;
	bool latched;
	bool used;
	int state;
};

struct Drive {
	std::vector<Snapshot> teeth; // [0] = after tooth 1
	int firstRpm = -1;
	int firstPhase = -1;
	int sparks = 0;
	int sparksBeforePhase = 0;
};

void configure(EngineTestHelper& eth, bool bitOn) {
	engineConfiguration->isFasterEngineSpinUpEnabled = false;
	engineConfiguration->isPhaseSyncRequiredForIgnition = true;
	engineConfiguration->ignitionMode = IM_ONE_COIL;
	engineConfiguration->cylindersCount = 4;
	engineConfiguration->firingOrder = FO_1_3_4_2;
	engineConfiguration->isInjectionEnabled = false;
	engineConfiguration->isIgnitionEnabled = true;
	engineConfiguration->timingMode = TM_FIXED;
	engineConfiguration->fixedTiming = 10;
	engineConfiguration->vvtMode[0] = VVT_SINGLE_TOOTH;
	engineConfiguration->camEarlyRiseLatch = bitOn;
	eth.setTriggerType(trigger_type_e::TT_12_TOOTH_CRANK);
}

// cam rises are delivered between teeth; hook(t) runs after tooth t
Drive drive(EngineTestHelper& eth, int teeth, const std::vector<CamRise>& rises, std::function<void(int)> hook = {}) {
	Drive r;
	engine->onIgnitionEvent = [&](IgnitionEvent* e, bool state) {
		// a limited spark still runs the bookkeeping
		if (!state && !e->wasSparkLimited) {
			r.sparks++;
		}
	};
	TriggerCentral* tc = getTriggerCentral();
	int spent = 0;
	for (int t = 1; t <= teeth; t++) {
		eth.moveTimeForwardAndInvokeEventsUs(toothUs - spent);
		eth.firePrimaryTriggerRise();
		spent = 0;

		Snapshot s = {
			tc->triggerState.hasSynchronizedPhase(),
			Sensor::getOrZero(SensorType::Rpm) > 0,
			tc->triggerState.getSynchronizationCounter(),
			tc->currentEngineDecodedPhase,
			(float)tc->triggerState.m_phaseAdjustment,
			tc->isCamEarlyRiseLatched(),
			tc->camEarlyRiseUsed,
			(int)engine->rpmCalculator.getState(),
		};
		r.teeth.push_back(s);
		if (s.rpm && r.firstRpm < 0) {
			r.firstRpm = t;
		}
		if (s.phase && r.firstPhase < 0) {
			r.firstPhase = t;
		}
		if (!s.phase) {
			r.sparksBeforePhase = r.sparks;
		}
		if (hook) {
			hook(t);
		}
		for (const auto& c : rises) {
			if (c.afterTooth == t) {
				eth.moveTimeForwardAndInvokeEventsUs(c.offsetUs);
				spent = c.offsetUs;
				hwHandleVvtCamSignal(TriggerValue::RISE, getTimeNowNt(), 0);
			}
		}
	}
	return r;
}

} // namespace

TEST(camEarlyRise, bitOffDiscards) {
	EngineTestHelper eth(engine_type_e::TEST_ENGINE);
	configure(eth, false);
	Drive r = drive(eth, 70, {{10, 5000}, {34, 5000}});
	ASSERT_GT(r.firstRpm, 10);
	// the second rise comes after tooth 34, so tooth 35 is the first to show phase
	EXPECT_EQ(35, r.firstPhase);
	for (const auto& s : r.teeth) {
		EXPECT_FALSE(s.latched);
		EXPECT_FALSE(s.used);
	}
}

TEST(camEarlyRise, phaseAtFirstRpm) {
	// early rises in both crank revolutions of the cycle, right after a tooth, mid interval and right before the next
	const CamRise cases[] = {
		{9, 300}, {9, 13500}, {9, 26000}, {20, 300}, {20, 13500}, {20, 26000}, {27, 5000}, {5, 5000},
	};
	for (const auto& c : cases) {
		SCOPED_TRACE(testing::Message() << "rise after tooth " << c.afterTooth << " +" << c.offsetUs << " us");

		Drive early;
		{
			EngineTestHelper eth(engine_type_e::TEST_ENGINE);
			configure(eth, true);
			early = drive(eth, 100, {c});
		}
		// the same rise one cam period later, after RPM, takes the normal path
		Drive live;
		{
			EngineTestHelper eth(engine_type_e::TEST_ENGINE);
			configure(eth, true);
			live = drive(eth, 100, {{c.afterTooth + 24, c.offsetUs}});
		}
		ASSERT_GT(early.firstRpm, c.afterTooth);
		ASSERT_LT(live.firstRpm, c.afterTooth + 24);
		EXPECT_EQ(early.firstRpm, early.firstPhase);
		EXPECT_GT(live.firstPhase, early.firstPhase);
		EXPECT_TRUE(early.teeth.back().used);
		EXPECT_FALSE(live.teeth.back().used);

		for (int t = live.firstPhase; t <= 100; t++) {
			const Snapshot& a = early.teeth[t - 1];
			const Snapshot& b = live.teeth[t - 1];
			EXPECT_EQ(a.counter, b.counter) << "tooth " << t;
			EXPECT_FLOAT_EQ(a.decodedPhase, b.decodedPhase) << "tooth " << t;
			EXPECT_FLOAT_EQ(a.phaseAdjustment, b.phaseAdjustment) << "tooth " << t;
		}
	}
}

TEST(camEarlyRise, riseAfterRpmSameAsOff) {
	Drive on, off;
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
	for (size_t i = 0; i < on.teeth.size(); i++) {
		EXPECT_EQ(on.teeth[i].phase, off.teeth[i].phase) << i;
		EXPECT_EQ(on.teeth[i].counter, off.teeth[i].counter) << i;
		EXPECT_FLOAT_EQ(on.teeth[i].decodedPhase, off.teeth[i].decodedPhase) << i;
		EXPECT_FALSE(on.teeth[i].used) << i;
	}
	EXPECT_EQ(on.sparks, off.sparks);
}

// tooth 4 is the sync tooth: that rise is a full cam period of teeth old when RPM arrives
TEST(camEarlyRise, tooOldDropped) {
	EngineTestHelper eth(engine_type_e::TEST_ENGINE);
	configure(eth, true);
	Drive r = drive(eth, 40, {{4, 5000}});
	ASSERT_GT(r.firstRpm, 0);
	EXPECT_EQ(-1, r.firstPhase);
	EXPECT_FALSE(getTriggerCentral()->isCamEarlyRiseLatched());
}

TEST(camEarlyRise, youngestAccepted) {
	EngineTestHelper eth(engine_type_e::TEST_ENGINE);
	configure(eth, true);
	Drive r = drive(eth, 40, {{5, 5000}});
	EXPECT_EQ(r.firstRpm, r.firstPhase);
}

// a rise latched before a trigger error belongs to a sync that is gone
TEST(camEarlyRise, triggerErrorDrops) {
	EngineTestHelper eth(engine_type_e::TEST_ENGINE);
	configure(eth, true);
	bool latched = false;
	bool syncLost = false;
	Drive r = drive(eth, 90, {{5, 5000}}, [&](int t) {
		if (t == 8) {
			latched = getTriggerCentral()->isCamEarlyRiseLatched();
			// a very late tooth is a trigger error
			eth.moveTimeForwardAndInvokeEventsUs(4 * toothUs);
		}
		if (t == 9) {
			syncLost = !getTriggerCentral()->triggerState.getShaftSynchronized();
		}
	});
	EXPECT_TRUE(latched);
	ASSERT_TRUE(syncLost);
	for (int t = 9; t <= 90; t++) {
		EXPECT_FALSE(r.teeth[t - 1].phase) << "tooth " << t;
		EXPECT_FALSE(r.teeth[t - 1].used) << "tooth " << t;
	}
	EXPECT_EQ(0, r.sparks);
}

TEST(camEarlyRise, engineStopDrops) {
	EngineTestHelper eth(engine_type_e::TEST_ENGINE);
	configure(eth, true);
	drive(eth, 12, {{10, 5000}});
	ASSERT_TRUE(getTriggerCentral()->isCamEarlyRiseLatched());
	engine->rpmCalculator.setStopSpinning();
	EXPECT_FALSE(getTriggerCentral()->isCamEarlyRiseLatched());

	drive(eth, 40, {{10, 5000}});
	ASSERT_TRUE(getTriggerCentral()->camEarlyRiseUsed);
	engine->rpmCalculator.setStopSpinning();
	EXPECT_FALSE(getTriggerCentral()->camEarlyRiseUsed);
}

TEST(camEarlyRise, noSparkBeforePhase) {
	EngineTestHelper eth(engine_type_e::TEST_ENGINE);
	configure(eth, true);
	Drive r = drive(eth, 80, {{10, 5000}});
	EXPECT_EQ(r.firstRpm, r.firstPhase);
	EXPECT_EQ(0, r.sparksBeforePhase);
	EXPECT_GT(r.sparks, 0);
}

TEST(camEarlyRise, noRiseNoPhase) {
	EngineTestHelper eth(engine_type_e::TEST_ENGINE);
	configure(eth, true);
	Drive r = drive(eth, 100, {});
	ASSERT_GT(r.firstRpm, 0);
	EXPECT_EQ(-1, r.firstPhase);
	EXPECT_EQ(0, r.sparks);
}

TEST(camEarlyRise, firstRpmKept) {
	Drive early, live;
	{
		EngineTestHelper eth(engine_type_e::TEST_ENGINE);
		configure(eth, true);
		early = drive(eth, 100, {{20, 5000}});
	}
	{
		EngineTestHelper eth(engine_type_e::TEST_ENGINE);
		configure(eth, true);
		live = drive(eth, 100, {{44, 5000}});
	}
	// the latch does not delay the first RPM, which stays set and CRANKING from then on
	ASSERT_EQ(early.firstRpm, live.firstRpm);
	ASSERT_EQ(early.firstRpm, early.firstPhase);
	for (int t = early.firstRpm; t <= 100; t++) {
		EXPECT_TRUE(early.teeth[t - 1].rpm) << "tooth " << t;
		EXPECT_EQ((int)spinning_state_e::CRANKING, early.teeth[t - 1].state) << "tooth " << t;
		EXPECT_TRUE(live.teeth[t - 1].rpm) << "tooth " << t;
		EXPECT_EQ((int)spinning_state_e::CRANKING, live.teeth[t - 1].state) << "tooth " << t;
	}
}

// two rises inside one cam period cannot both be real: no phase from the latch, the next live rise gives the right one
TEST(camEarlyRise, doubleRiseBeforeRpmFallsBack) {
	Drive twice, live;
	{
		EngineTestHelper eth(engine_type_e::TEST_ENGINE);
		configure(eth, true);
		twice = drive(eth, 100, {{9, 5000}, {14, 5000}, {33, 5000}});
	}
	{
		EngineTestHelper eth(engine_type_e::TEST_ENGINE);
		configure(eth, true);
		live = drive(eth, 100, {{33, 5000}});
	}
	ASSERT_GT(twice.firstRpm, 14);
	ASSERT_LT(twice.firstRpm, 33);
	EXPECT_EQ(34, twice.firstPhase);
	EXPECT_EQ(live.firstPhase, twice.firstPhase);
	for (const auto& s : twice.teeth) {
		EXPECT_FALSE(s.used);
	}
	for (int t = live.firstPhase; t <= 100; t++) {
		EXPECT_EQ(live.teeth[t - 1].counter, twice.teeth[t - 1].counter) << "tooth " << t;
		EXPECT_FLOAT_EQ(live.teeth[t - 1].decodedPhase, twice.teeth[t - 1].decodedPhase) << "tooth " << t;
	}
	EXPECT_EQ(0, twice.sparksBeforePhase);
}

// real rise, noise, real again: the last rise before RPM must not set a wrong phase window
TEST(camEarlyRise, noisyRiseThenLiveCorrect) {
	EngineTestHelper eth(engine_type_e::TEST_ENGINE);
	configure(eth, true);
	Drive r = drive(eth, 60, {{9, 5000}, {14, 5000}, {33, 5000}});
	ASSERT_GT(r.firstRpm, 14);
	for (int t = 1; t < 34; t++) {
		EXPECT_FALSE(r.teeth[t - 1].phase) << "tooth " << t;
	}
	EXPECT_EQ(34, r.firstPhase);
	EXPECT_EQ(0, r.sparksBeforePhase);
}

// the upstream default: the latch must stay correct and safe with fast spin up
TEST(camEarlyRise, phaseLockWithFastSpinUp) {
	for (int rise : {5, 10, 20}) {
		SCOPED_TRACE(rise);
		Drive early, live;
		{
			EngineTestHelper eth(engine_type_e::TEST_ENGINE);
			configure(eth, true);
			engineConfiguration->isFasterEngineSpinUpEnabled = true;
			early = drive(eth, 100, {{rise, 5000}});
		}
		{
			EngineTestHelper eth(engine_type_e::TEST_ENGINE);
			configure(eth, true);
			engineConfiguration->isFasterEngineSpinUpEnabled = true;
			live = drive(eth, 100, {{rise + 24, 5000}});
		}
		ASSERT_GT(early.firstRpm, 0);
		ASSERT_GT(live.firstPhase, 0);
		EXPECT_EQ(0, early.sparksBeforePhase);
		EXPECT_GE(early.firstPhase, early.firstRpm);
		EXPECT_LE(early.firstPhase, live.firstPhase);
		for (int t = live.firstPhase; t <= 100; t++) {
			EXPECT_EQ(early.teeth[t - 1].counter, live.teeth[t - 1].counter) << "tooth " << t;
			EXPECT_FLOAT_EQ(early.teeth[t - 1].decodedPhase, live.teeth[t - 1].decodedPhase) << "tooth " << t;
		}
	}
}

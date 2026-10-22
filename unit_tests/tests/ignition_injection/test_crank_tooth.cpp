#include "pch.h"

#include <functional>
#include <map>
#include <set>

namespace {

struct Edge {
	int cylinder;
	bool charge;
	efitick_t nt;
	uint32_t counter;
	float commanded;
	bool running;
};

struct Tooth {
	efitick_t nt;
	float phase;
};

// a charge as it was scheduled: the tooth it was decided on and the path it took
struct Charge {
	size_t tooth;
	bool toothLocked;
	float sparkPhase;
};

struct Capture {
	std::vector<Tooth> teeth;
	std::vector<Edge> edges;
	std::vector<Charge> charges;
	// teeth played when a tooth-locked charge was first scheduled while RUNNING
	int runningLockedAt = -1;
	int outOfOrder = 0;
};

// 12 tooth crank, 4 cylinders, one coil, 4.5 ms dwell, 14 BTDC: the 12 BTDC tooth is 2 degrees later
void setUpCrank(EngineTestHelper& eth, bool toothLocked, float triggerOffset = 162) {
	engineConfiguration->isFasterEngineSpinUpEnabled = false;
	engineConfiguration->ignitionMode = IM_ONE_COIL;
	engineConfiguration->cylindersCount = 4;
	engineConfiguration->firingOrder = FO_1_3_4_2;
	engineConfiguration->isInjectionEnabled = false;
	engineConfiguration->isIgnitionEnabled = true;
	engineConfiguration->cranking.rpm = 500;
	engineConfiguration->ignitionDwellForCrankingMs = 4.5f;
	engineConfiguration->globalTriggerAngleOffset = triggerOffset;
	engineConfiguration->timingMode = TM_FIXED;
	engineConfiguration->fixedTiming = 14;
	engineConfiguration->crankingToothLockedSpark = toothLocked;
	eth.setTriggerType(trigger_type_e::TT_12_TOOTH_CRANK);
}

// 17 BTDC is the farthest command a tooth still locks (5 degrees before its tooth), and the long dwell starts the
// charge early
void setUpWideCommand() {
	engineConfiguration->fixedTiming = 17;
	engineConfiguration->ignitionDwellForCrankingMs = 9;
}

// one tooth (30 degrees) per entry, at that crank rpm; before(n) runs ahead of tooth n
void play(EngineTestHelper& eth, Capture& run, const std::vector<double>& rpm,
		const std::function<void(size_t)>& before = nullptr,
		const std::function<void(IgnitionEvent*, bool)>& onEdge = nullptr) {
	engine->onIgnitionEvent = [&run, onEdge](IgnitionEvent* e, bool state) {
		run.edges.push_back({e->cylinderIndex, state, getTimeNowNt(), e->sparkCounter, e->sparkAngle,
			engine->rpmCalculator.getState() == RUNNING});
		if (onEdge) {
			onEdge(e, state);
		}
	};
	engine->onScheduleTurnSparkPinHighStartCharging = [&run](const IgnitionEvent& e, efitick_t, angle_t, efitick_t) {
		bool running = engine->rpmCalculator.getState() == RUNNING;
		if (run.runningLockedAt < 0 && engine->ignitionState.crankSparkToothLocked && running) {
			run.runningLockedAt = run.teeth.size();
		}
		run.charges.push_back({run.teeth.size(), e.crankToothPlan.valid, e.crankToothPlan.sparkPhase});
	};
	for (size_t n = 0; n < rpm.size(); n++) {
		if (before) {
			before(n);
		}
		eth.moveTimeForwardAndInvokeEventsUs((int)std::lround(5.0 / rpm[n] * 1e6));
		eth.firePrimaryTriggerRise();
		run.teeth.push_back({getTimeNowNt(), engine->triggerCentral.currentEngineDecodedPhase});
	}
	run.outOfOrder = engine->engineState.sparkOutOfOrderCounter;
}

std::vector<double> steady(double rpm, size_t teeth) {
	return std::vector<double>(teeth, rpm);
}

// steady cranking, then a flash to 800 rpm that holds
std::vector<double> flash() {
	std::vector<double> rpm = steady(210, 60);
	for (double r : {400.0, 600.0}) {
		rpm.push_back(r);
	}
	rpm.resize(rpm.size() + 41, 800);
	return rpm;
}

// steady cranking, then a slow climb through cranking_rpm (500) into RUNNING
std::vector<double> slowFlash() {
	std::vector<double> rpm = steady(210, 60);
	for (double r : {280.0, 330.0, 380.0, 430.0, 470.0, 490.0, 510.0, 540.0, 600.0, 700.0, 800.0}) {
		rpm.push_back(r);
	}
	rpm.resize(rpm.size() + 40, 850);
	return rpm;
}

struct Spark {
	double errDeg; // delivered minus commanded, positive is later
	double dwellMs;
};

// delivered angle and dwell of every spark after the given tooth, interpolating the crank position between teeth
std::vector<Spark> sparksAfter(const Capture& run, size_t firstTooth) {
	std::vector<Spark> out;
	for (const Edge& spark : run.edges) {
		if (spark.charge || spark.nt <= run.teeth[firstTooth].nt) {
			continue;
		}
		size_t k = firstTooth;
		while (k + 2 < run.teeth.size() && run.teeth[k + 1].nt <= spark.nt) {
			k++;
		}
		double span = std::fmod(run.teeth[k + 1].phase - run.teeth[k].phase + 1440.0, 720.0);
		double frac = (double)(spark.nt - run.teeth[k].nt) / (double)(run.teeth[k + 1].nt - run.teeth[k].nt);
		double err = std::fmod(run.teeth[k].phase + span * frac - spark.commanded + 1440.0, 720.0);
		double dwellMs = -1;
		for (const Edge& e : run.edges) {
			if (e.charge && e.counter == spark.counter) {
				dwellMs = NT2US((double)(spark.nt - e.nt)) / 1000.0;
			}
		}
		out.push_back({err > 360 ? err - 720 : err, dwellMs});
	}
	return out;
}

// every spark counter must see one charge and one spark
bool chargesAndSparksPair(const Capture& run) {
	std::map<uint32_t, std::pair<int, int>> byCounter;
	for (const Edge& e : run.edges) {
		(e.charge ? byCounter[e.counter].first : byCounter[e.counter].second)++;
	}
	uint32_t last = byCounter.empty() ? 0 : byCounter.rbegin()->first;
	for (const auto& kv : byCounter) {
		bool inFlight = kv.first == last && kv.second.second == 0;
		if (!inFlight && (kv.second.first != 1 || kv.second.second != 1)) {
			return false;
		}
	}
	return true;
}

bool sameEdges(const Capture& a, const Capture& b) {
	if (a.edges.size() != b.edges.size()) {
		return false;
	}
	for (size_t i = 0; i < a.edges.size(); i++) {
		const Edge &x = a.edges[i], &y = b.edges[i];
		if (x.cylinder != y.cylinder || x.charge != y.charge || x.nt != y.nt) {
			return false;
		}
	}
	return true;
}

bool anyPending() {
	for (const IgnitionEvent& e : getIgnitionEvents()->elements) {
		if (e.crankToothDecided && e.crankToothPlan.valid) {
			return true;
		}
	}
	return false;
}

// steady 230 rpm, one slow tooth at n, then back: the next spark tooth comes before the charge the slow twin put off
std::vector<double> slowThenRecover(size_t n) {
	std::vector<double> rpm = steady(230, n);
	rpm.push_back(115);
	rpm.resize(n + 41, 230);
	return rpm;
}

int sparksOf(const Capture& run, uint32_t counter) {
	int sparks = 0;
	for (const Edge& e : run.edges) {
		sparks += !e.charge && e.counter == counter;
	}
	return sparks;
}

int chargesOf(const Capture& run, uint32_t counter) {
	int charges = 0;
	for (const Edge& e : run.edges) {
		charges += e.charge && e.counter == counter;
	}
	return charges;
}

// the tooth at which a late (short dwell) charge was made, or -1
int lateTooth(const Capture& run, uint32_t& counter) {
	for (const Edge& charge : run.edges) {
		if (!charge.charge) {
			continue;
		}
		for (const Edge& spark : run.edges) {
			if (!spark.charge && spark.counter == charge.counter && NT2US((double)(spark.nt - charge.nt)) < 3000) {
				for (size_t k = 0; k < run.teeth.size(); k++) {
					if (run.teeth[k].nt == charge.nt) {
						counter = charge.counter;
						return (int)k;
					}
				}
			}
		}
	}
	return -1;
}

// the trigger stops: everything in flight runs out, and no coil may be left charging
void expectSettled(EngineTestHelper& eth, const char* what) {
	eth.moveTimeForwardAndInvokeEventsUs(1000000);
	EXPECT_FALSE(enginePins.coils[0].getLogicValue()) << what;
	for (const IgnitionEvent& e : getIgnitionEvents()->elements) {
		EXPECT_FALSE(e.chargedOutputs[0]) << what;
	}
}

// first slow-tooth position whose recovery makes a late charge: its slow-tooth index, the tooth the late charge is
// made at (time moves on to the next tooth with its events, so it is made after before(k + 1)) and its spark counter
bool findLate(size_t& n, int& k, uint32_t& counter) {
	for (n = 60; n < 72; n++) {
		Capture probe;
		EngineTestHelper eth(engine_type_e::TEST_ENGINE);
		setUpCrank(eth, true);
		play(eth, probe, slowThenRecover(n));
		k = lateTooth(probe, counter);
		if (k >= 0) {
			return true;
		}
	}
	return false;
}

} // namespace

TEST(crankTooth, bitOffUnchanged) {
	EngineTestHelper eth(engine_type_e::TEST_ENGINE);
	setUpCrank(eth, false);
	bool anyLiveBit = false;
	Capture run;
	play(eth, run, steady(210, 100), [&](size_t) {
		anyLiveBit |= engine->ignitionState.crankSparkToothLocked || engine->ignitionState.crankSparkNotLockable;
	});
	EXPECT_FALSE(anyLiveBit);
	EXPECT_EQ(0, engine->module<CrankToothSpark>()->window().slowUsPerDeg());
	EXPECT_GT(sparksAfter(run, 60).size(), 6u);
}

TEST(crankTooth, steadyCrankLocksToTooth) {
	EngineTestHelper eth(engine_type_e::TEST_ENGINE);
	setUpCrank(eth, true);
	Capture run;
	play(eth, run, steady(210, 100));
	auto sparks = sparksAfter(run, 60);
	ASSERT_GE(sparks.size(), 6u);
	for (const Spark& s : sparks) {
		EXPECT_NEAR(2.0, s.errDeg, 0.05);
		EXPECT_NEAR(4.5, s.dwellMs, 0.1);
	}
	EXPECT_EQ(0, run.outOfOrder);

	// the spark tooth of a plan is compared with the decoded phase by equality: it must be a phase that arrived
	size_t locked = 0;
	for (const Charge& c : run.charges) {
		if (c.toothLocked) {
			locked++;
			EXPECT_TRUE(std::any_of(run.teeth.begin(), run.teeth.end(), [&](const Tooth& t) {
				return t.phase == c.sparkPhase;
			}));
		}
	}
	EXPECT_GT(locked, 6u);
}

TEST(crankTooth, flashKeepsDwell) {
	EngineTestHelper eth(engine_type_e::TEST_ENGINE);
	setUpCrank(eth, true);
	Capture run;
	play(eth, run, flash());
	EXPECT_EQ(RUNNING, engine->rpmCalculator.getState());
	for (const Spark& s : sparksAfter(run, 60)) {
		EXPECT_NEAR(2.0, s.errDeg, 0.05);
		EXPECT_GT(s.dwellMs, 1.0);
		EXPECT_LT(s.dwellMs, 38.0);
	}
	EXPECT_TRUE(chargesAndSparksPair(run));
}

// faster spin-up (the firmware default): events prepared in SPINNING_UP stay normal, none fires early
TEST(crankTooth, spinningUpNotLocked) {
	EngineTestHelper eth(engine_type_e::TEST_ENGINE);
	setUpCrank(eth, true);
	engineConfiguration->isFasterEngineSpinUpEnabled = true;
	int spinningUpTeeth = 0;
	bool lockedOnceCranking = false;
	Capture run;
	play(eth, run, steady(210, 130), [&](size_t) {
		if (engine->rpmCalculator.getState() == SPINNING_UP && engine->rpmCalculator.isCranking()) {
			spinningUpTeeth++;
			EXPECT_FALSE(engine->ignitionState.crankSparkToothLocked);
		}
		lockedOnceCranking |= engine->ignitionState.crankSparkToothLocked;
	});
	EXPECT_GT(spinningUpTeeth, 0);
	EXPECT_TRUE(lockedOnceCranking);
	for (const Spark& s : sparksAfter(run, 10)) {
		EXPECT_GT(s.errDeg, -0.5);
	}
	EXPECT_EQ(0, run.outOfOrder);
}

TEST(crankTooth, noToothInReachUsesNormalPath) {
	for (double command : {5.0, 10.5, 20.0}) {
		Capture on, off;
		bool notLockable = false;
		{
			EngineTestHelper eth(engine_type_e::TEST_ENGINE);
			setUpCrank(eth, true);
			engineConfiguration->fixedTiming = command;
			play(eth, on, steady(210, 100), [&](size_t) {
				notLockable |= engine->ignitionState.crankSparkNotLockable;
			});
		}
		{
			EngineTestHelper eth(engine_type_e::TEST_ENGINE);
			setUpCrank(eth, false);
			engineConfiguration->fixedTiming = command;
			play(eth, off, steady(210, 100));
		}
		EXPECT_TRUE(sameEdges(off, on)) << command;
		EXPECT_TRUE(notLockable) << command;
	}
}

// the speed recovers after one slow tooth, so the spark tooth comes before the charge the slow twin put off: the
// spark is late, not skipped, never early, and its charge is the short late dwell
TEST(crankTooth, lateNotLost) {
	size_t lateSparks = 0;
	for (size_t n = 60; n < 72; n++) {
		Capture on, off;
		{
			EngineTestHelper eth(engine_type_e::TEST_ENGINE);
			setUpCrank(eth, true);
			play(eth, on, slowThenRecover(n));
		}
		{
			EngineTestHelper eth(engine_type_e::TEST_ENGINE);
			setUpCrank(eth, false);
			play(eth, off, slowThenRecover(n));
		}
		EXPECT_EQ(0, on.outOfOrder) << n;
		EXPECT_TRUE(chargesAndSparksPair(on)) << n;
		auto sparks = sparksAfter(on, 50);
		EXPECT_GE(sparks.size(), sparksAfter(off, 50).size() - 1) << n;
		for (const Spark& s : sparks) {
			EXPECT_GT(s.errDeg, -0.1) << n;
			EXPECT_LT(s.errDeg, 15.0) << n;
			EXPECT_LT(s.dwellMs, 40.0) << n;
			lateSparks += s.dwellMs < 3.0;
		}
	}
	EXPECT_GT(lateSparks, 0u);
}

// The late spark is refused or off: the spark whose tooth came before its charge is skipped, never fired early, the
// planned charge does not start after it, and the next cycle goes on as usual.
void expectSkipsCleanly(uint8_t lateLimit, float dwellMs) {
	size_t skippedSparks = 0;
	for (size_t n = 60; n < 72; n++) {
		EngineTestHelper eth(engine_type_e::TEST_ENGINE);
		setUpCrank(eth, true);
		engineConfiguration->crankingToothLateSparkMaxDeg = lateLimit;
		engineConfiguration->ignitionDwellForCrankingMs = dwellMs;
		Capture run;
		play(eth, run, slowThenRecover(n));
		std::set<uint32_t> skipped;
		for (const Edge& e : run.edges) {
			if (!e.charge && chargesOf(run, e.counter) == 0) {
				skipped.insert(e.counter);
			}
		}
		skippedSparks += skipped.size();
		for (uint32_t counter : skipped) {
			EXPECT_EQ(1, sparksOf(run, counter)) << n;
		}
		EXPECT_FALSE(engine->ignitionState.crankSparkLate) << n;
		EXPECT_GE(sparksAfter(run, n + 2).size(), 2u) << n;
		for (const Spark& s : sparksAfter(run, 50)) {
			EXPECT_GT(s.errDeg, -0.1) << n;
		}
		// a stale charge would show up as a charge of a skipped spark, or bail out of order, once time runs on
		eth.moveTimeForwardAndInvokeEventsUs(1000000);
		for (uint32_t counter : skipped) {
			EXPECT_EQ(0, chargesOf(run, counter)) << n;
		}
		EXPECT_EQ(0, engine->engineState.sparkOutOfOrderCounter) << n;
		expectSettled(eth, "skipped");
	}
	EXPECT_GT(skippedSparks, 0u) << (int)lateLimit << " " << dwellMs;
}

TEST(crankTooth, toothBeforeChargeSkipsSparkCleanly) {
	// off
	expectSkipsCleanly(0, 4.5f);
	// 2 degrees at 230 rpm is 1.45 ms, less than the shortest late dwell
	expectSkipsCleanly(2, 4.5f);
	// the late dwell of a 26 ms dwell is 13 ms, more than 15 degrees at 230 rpm
	expectSkipsCleanly(15, 26);
}

// a late charge that bails (a later spark already fired on the coil) is skipped at once: no second attempt, no repeated
// rescheduling of the fire, and the next cycle prepares as usual
TEST(crankTooth, lateBailIsOneShot) {
	size_t n;
	int k;
	uint32_t counter;
	ASSERT_TRUE(findLate(n, k, counter));

	EngineTestHelper eth(engine_type_e::TEST_ENGINE);
	setUpCrank(eth, true);
	Capture run;
	IgnitionOutputPin& coil = enginePins.coils[0];
	uint32_t saved = 0;
	// the late charge is made while time moves to the next tooth, after before(k + 1)
	play(eth, run, slowThenRecover(n), [&](size_t t) {
		if ((int)t == k + 1) {
			saved = coil.signalFallSparkId;
			coil.signalFallSparkId = UINT32_MAX;
		} else if ((int)t == k + 2) {
			coil.signalFallSparkId = saved;
		}
	});
	// the bailed spark is the only one lost: one out-of-order, one spark edge without a charge, no repeats
	EXPECT_EQ(1, run.outOfOrder);
	EXPECT_EQ(0, chargesOf(run, counter));
	EXPECT_EQ(1, sparksOf(run, counter));
	for (const Edge& e : run.edges) {
		EXPECT_EQ(1, sparksOf(run, e.counter));
		EXPECT_LE(chargesOf(run, e.counter), 1);
	}
	EXPECT_GE(sparksAfter(run, k + 2).size(), 3u);
	expectSettled(eth, "bail");
	EXPECT_EQ(1, engine->engineState.sparkOutOfOrderCounter);
}

// a trigger error or an engine stop right as a late charge starts: its fire is time scheduled, so the coil is released
// once, and nothing is left charging
TEST(crankTooth, lateTriggerErrorDuringDwell) {
	size_t n;
	int k;
	uint32_t counter;
	ASSERT_TRUE(findLate(n, k, counter));
	for (int what = 0; what < 2; what++) {
		EngineTestHelper eth(engine_type_e::TEST_ENGINE);
		setUpCrank(eth, true);
		Capture run;
		bool injected = false;
		play(eth, run, slowThenRecover(n), nullptr, [&](IgnitionEvent* e, bool charge) {
			if (charge && e->sparkCounter == counter) {
				EXPECT_TRUE(enginePins.coils[0].getLogicValue());
				injected = true;
				if (what == 0) {
					engine->triggerCentral.triggerState.onTriggerError();
				} else {
					engine->rpmCalculator.setStopSpinning();
				}
			}
		});
		ASSERT_TRUE(injected);
		EXPECT_EQ(1, chargesOf(run, counter)) << what;
		EXPECT_EQ(1, sparksOf(run, counter)) << what;
		EXPECT_EQ(0, run.outOfOrder) << what;
		for (const Edge& e : run.edges) {
			EXPECT_LE(chargesOf(run, e.counter), 1) << what;
			EXPECT_LE(sparksOf(run, e.counter), 1) << what;
		}
		expectSettled(eth, what ? "stop" : "error");
	}
}

// the late spark limit is a setting: with 0 (off) or a limit the late dwell exceeds there is no late spark, and the
// live bit follows the late sparks
TEST(crankTooth, lateSparkLimitSetting) {
	struct Result {
		size_t lateSparks = 0;
		bool bit = false;
	};
	// limit < 0 leaves the default
	auto run = [](int limit, bool toothLocked) {
		Result r;
		for (size_t n = 60; n < 72; n++) {
			EngineTestHelper eth(engine_type_e::TEST_ENGINE);
			setUpCrank(eth, toothLocked);
			if (limit >= 0) {
				engineConfiguration->crankingToothLateSparkMaxDeg = limit;
			}
			Capture c;
			play(eth, c, slowThenRecover(n), [&](size_t) {
				r.bit |= engine->ignitionState.crankSparkLate;
			});
			for (const Edge& e : c.edges) {
				for (const Edge& ch : c.edges) {
					r.lateSparks += !e.charge && ch.charge && ch.counter == e.counter
						&& NT2US((double)(e.nt - ch.nt)) < 3000;
				}
			}
		}
		return r;
	};
	Result def = run(-1, true);
	EXPECT_GT(def.lateSparks, 0u);
	EXPECT_TRUE(def.bit);
	// the default is 15 degrees
	EXPECT_EQ(def.lateSparks, run(15, true).lateSparks);
	for (int limit : {0, 2}) {
		Result r = run(limit, true);
		EXPECT_EQ(0u, r.lateSparks) << limit;
		EXPECT_FALSE(r.bit) << limit;
	}
	// the setting off: the bit never rises
	EXPECT_FALSE(run(-1, false).bit);
}

// the late dwell is judged at the newest tooth interval, and there is none after the window is reset
TEST(crankTooth, lateDwellFollowsNewestInterval) {
	EngineTestHelper eth(engine_type_e::TEST_ENGINE);
	setUpCrank(eth, true);
	auto& module = *engine->module<CrankToothSpark>();
	Capture slow, fast;
	play(eth, slow, steady(230, 40));
	// 15 degrees at 230 rpm is 10.9 ms
	EXPECT_NEAR(2.25, module.lateDwellMs(4.5f), 1e-3);
	// 15 degrees at 1200 rpm is 2.1 ms, less than the late dwell
	play(eth, fast, steady(1200, 3));
	EXPECT_EQ(0, module.lateDwellMs(4.5f));
	engine->rpmCalculator.setStopSpinning();
	EXPECT_EQ(0, module.lateDwellMs(4.5f));
}

// faster spin-up on, the crank goes 200 -> 900 rpm in a few teeth: nothing early, no tooth-lock while spinning up, and
// every late spark within its bound
TEST(crankTooth, lateDuringSpinUp) {
	for (size_t start : {40u, 55u, 70u}) {
		EngineTestHelper eth(engine_type_e::TEST_ENGINE);
		setUpCrank(eth, true);
		engineConfiguration->isFasterEngineSpinUpEnabled = true;
		std::vector<double> rpm = steady(200, start);
		for (double r : {260.0, 340.0, 450.0, 600.0, 750.0}) {
			rpm.push_back(r);
		}
		rpm.resize(rpm.size() + 60, 900);
		Capture run;
		play(eth, run, rpm, [&](size_t) {
			if (engine->rpmCalculator.getState() == SPINNING_UP) {
				EXPECT_FALSE(engine->ignitionState.crankSparkToothLocked);
				EXPECT_FALSE(engine->ignitionState.crankSparkLate);
			}
		});
		for (const Spark& s : sparksAfter(run, 10)) {
			EXPECT_GT(s.errDeg, -0.5) << start;
			// a late spark is within the 5 degree reach plus the 15 degree late bound
			if (s.dwellMs >= 0 && s.dwellMs < 3.0) {
				EXPECT_LT(s.errDeg, 20.0) << start;
			}
		}
		EXPECT_EQ(0, run.outOfOrder) << start;
		EXPECT_TRUE(chargesAndSparksPair(run)) << start;
	}
}

// a flip of the setting applies when an event next decides, so never to a charge already scheduled
TEST(crankTooth, modeChangeNoDoubleCharge) {
	for (bool wide : {false, true}) {
		for (bool enable : {false, true}) {
			for (size_t n = 60; n < 84; n++) {
				EngineTestHelper eth(engine_type_e::TEST_ENGINE);
				setUpCrank(eth, !enable);
				if (wide) {
					setUpWideCommand();
				}
				Capture run;
				play(eth, run, steady(210, 160), [=](size_t i) {
					if (i == n) {
						engineConfiguration->crankingToothLockedSpark = enable;
					}
				});
				EXPECT_TRUE(chargesAndSparksPair(run)) << wide << enable << n;
				EXPECT_EQ(0, run.outOfOrder) << wide << enable << n;
			}
		}
	}
}

// a sync loss or re-sync resets the window but keeps each event's decided path: no second charge, never early
TEST(crankTooth, syncLossKeepsDecision) {
	for (bool triggerError : {true, false}) {
		EngineTestHelper eth(engine_type_e::TEST_ENGINE);
		setUpCrank(eth, true);
		setUpWideCommand();
		bool injected = false;
		Capture run;
		play(eth, run, steady(210, 200), [&](size_t n) {
			if (!injected && n > 70 && anyPending()) {
				injected = true;
				if (triggerError) {
					engine->triggerCentral.triggerState.onTriggerError();
				} else {
					// a cam sync that disagrees with the assumed phase
					auto& ts = engine->triggerCentral.triggerState;
					int remainder = (ts.getSynchronizationCounter() + 1) % 2;
					EXPECT_NE(0, engine->triggerCentral.syncEnginePhaseAndReport(2, remainder));
				}
				EXPECT_EQ(0, engine->module<CrankToothSpark>()->window().lastUsPerDeg());
				EXPECT_TRUE(anyPending());
			}
		});
		ASSERT_TRUE(injected);
		EXPECT_TRUE(chargesAndSparksPair(run));
		EXPECT_EQ(0, run.outOfOrder);
		for (const Spark& s : sparksAfter(run, 60)) {
			EXPECT_GT(s.errDeg, -0.1);
		}
	}
}

// a spark that outlives the mode: switched off mid-crank, at most one more cycle of tooth-locked charges
TEST(crankTooth, bitOffTailIsOneCycle) {
	for (size_t n = 60; n < 84; n++) {
		EngineTestHelper eth(engine_type_e::TEST_ENGINE);
		setUpCrank(eth, true);
		Capture run;
		play(eth, run, steady(210, 160), [=](size_t i) {
			if (i == n) {
				engineConfiguration->crankingToothLockedSpark = false;
			}
		});
		size_t tail = 0;
		for (const Charge& c : run.charges) {
			if (c.toothLocked && c.tooth > n) {
				tail++;
			}
		}
		EXPECT_LE(tail, 4u) << n;
		EXPECT_TRUE(chargesAndSparksPair(run)) << n;
		for (const Spark& s : sparksAfter(run, 40)) {
			EXPECT_GT(s.errDeg, -3.0) << n;
		}
	}
}

// the first spark after the switch to RUNNING is still on the tooth, with a charge that has had time to build
TEST(crankTooth, carriedIntoRunningStaysLocked) {
	EngineTestHelper eth(engine_type_e::TEST_ENGINE);
	setUpCrank(eth, true);
	Capture run;
	play(eth, run, slowFlash());
	ASSERT_EQ(RUNNING, engine->rpmCalculator.getState());
	auto sparks = sparksAfter(run, 40);
	size_t i = 0;
	for (const Edge& e : run.edges) {
		if (e.charge || e.nt <= run.teeth[40].nt) {
			continue;
		}
		if (e.running) {
			EXPECT_NEAR(2.0, sparks[i].errDeg, 0.1);
			EXPECT_GT(sparks[i].dwellMs, 3.0);
			return;
		}
		i++;
	}
	FAIL() << "no spark fired while RUNNING";
}

// the window is a tooth history of this run only
TEST(crankTooth, engineStopClearsWindow) {
	EngineTestHelper eth(engine_type_e::TEST_ENGINE);
	setUpCrank(eth, true);
	Capture run;
	play(eth, run, steady(210, 100));
	EXPECT_GT(engine->module<CrankToothSpark>()->window().lastUsPerDeg(), 0);
	engine->rpmCalculator.setStopSpinning();
	EXPECT_EQ(0, engine->module<CrankToothSpark>()->window().lastUsPerDeg());
}

// the wheel check follows the cylinder layout, also when it is not burned
TEST(crankTooth, geometryFollowsLayout) {
	EngineTestHelper eth(engine_type_e::TEST_ENGINE);
	setUpCrank(eth, true);
	auto& module = *engine->module<CrankToothSpark>();
	EXPECT_TRUE(module.isGeometryValid());
	// 5 cylinders: 144 degrees is not a whole number of 30 degree teeth
	engineConfiguration->cylindersCount = 5;
	engineConfiguration->firingOrder = FO_1_2_4_5_3;
	EXPECT_FALSE(module.isGeometryValid());
	module.onSlowCallback();
	EXPECT_FALSE(module.isGeometryValid());
	engineConfiguration->cylindersCount = 4;
	engineConfiguration->firingOrder = FO_1_3_4_2;
	module.onSlowCallback();
	EXPECT_TRUE(module.isGeometryValid());
	// an uneven engine
	engineConfiguration->timing_offset_cylinder[1] = 20;
	EXPECT_FALSE(module.isGeometryValid());
}

// the coil stays charged until the backup releases it when the crank stops before the spark tooth
TEST(crankTooth, overDwellBackup) {
	// 0 reads as the default, 40
	for (double limitMs : {0.0, 10.0, 20.0, 40.0}) {
		double longest = 0;
		for (size_t stopAfter = 72; stopAfter < 96; stopAfter++) {
			EngineTestHelper eth(engine_type_e::TEST_ENGINE);
			setUpCrank(eth, true);
			engineConfiguration->crankingToothLockBackupDwellMs = limitMs;
			Capture run;
			play(eth, run, steady(210, stopAfter));
			eth.moveTimeForwardAndInvokeEventsUs(300000);
			for (const Spark& s : sparksAfter(run, 30)) {
				longest = std::max(longest, s.dwellMs);
			}
		}
		// while cranking: the larger of the limit and 3 x the 4.5 ms dwell
		EXPECT_NEAR(std::max(limitMs == 0 ? 40.0 : limitMs, 13.5), longest, 0.3) << limitMs;
	}
}

// a charge carried over the switch to RUNNING is released by the shorter of the limit and 3 x dwell
TEST(crankTooth, overDwellBackupRunning) {
	for (double limitMs : {40.0, 100.0}) {
		auto setUp = [=](EngineTestHelper& eth) {
			setUpCrank(eth, true, 222);
			setUpWideCommand();
			engineConfiguration->crankingToothLockBackupDwellMs = limitMs;
		};
		Capture probe;
		{
			EngineTestHelper eth(engine_type_e::TEST_ENGINE);
			setUp(eth);
			play(eth, probe, flash());
		}
		ASSERT_GT(probe.runningLockedAt, 0);

		// the same run, stopped right after that charge was scheduled
		EngineTestHelper eth(engine_type_e::TEST_ENGINE);
		setUp(eth);
		std::vector<double> rpm = flash();
		rpm.resize(probe.runningLockedAt + 1);
		Capture run;
		play(eth, run, rpm);
		eth.moveTimeForwardAndInvokeEventsUs(300000);
		EXPECT_NEAR(27, sparksAfter(run, 30).back().dwellMs, 0.5) << limitMs;
	}
}

// a command farther before its tooth than the reach setting keeps normal scheduling; 0 reads as the default, 5
TEST(crankTooth, maxRetardSetting) {
	for (int reach : {0, 5, 19, 20}) {
		EngineTestHelper eth(engine_type_e::TEST_ENGINE);
		setUpCrank(eth, true);
		// 20 degrees before the 12 BTDC tooth
		engineConfiguration->fixedTiming = 32;
		engineConfiguration->crankingToothLockMaxRetardDeg = reach;
		bool locked = false;
		bool notLockable = false;
		Capture run;
		play(eth, run, steady(210, 100), [&](size_t) {
			locked |= engine->ignitionState.crankSparkToothLocked;
			notLockable |= engine->ignitionState.crankSparkNotLockable;
		});
		EXPECT_EQ(reach == 20, locked) << reach;
		EXPECT_EQ(reach != 20, notLockable) << reach;
		for (const Spark& s : sparksAfter(run, 60)) {
			EXPECT_NEAR(reach == 20 ? 20.0 : 0.0, s.errDeg, 0.1) << reach;
		}
	}
}

TEST(crankTooth, geometryCheckByWheel) {
	struct Row {
		trigger_type_e wheel;
		int cylinders;
		bool valid;
		bool unevenFire = false;
	};
	const Row rows[] = {
		{trigger_type_e::TT_12_TOOTH_CRANK, 4, true},
		{trigger_type_e::TT_12_TOOTH_CRANK, 6, true},
		{trigger_type_e::TT_12_TOOTH_CRANK, 8, true},
		{trigger_type_e::TT_12_TOOTH_CRANK, 5, false},	// 144 degrees is not a whole number of teeth
		{trigger_type_e::TT_12_TOOTH_CRANK, 1, true},	// every tooth is its own twin
		{trigger_type_e::TT_12_TOOTH_CRANK, 4, false, true},	// the stroke is not cycle / cylinders
		{trigger_type_e::TT_6_TOOTH_CRANK, 4, true},
		{trigger_type_e::TT_6_TOOTH_CRANK, 6, false},	// 2 teeth per stroke
		{trigger_type_e::TT_3_TOOTH_CRANK, 4, false},
		{trigger_type_e::TT_MITSU_4G63_CRANK, 4, false},	// 1 tooth per stroke
		{trigger_type_e::TT_TOOTHED_WHEEL_36_1, 4, false},	// missing tooth
		{trigger_type_e::TT_TOOTHED_WHEEL_60_2, 4, false},
		{trigger_type_e::TT_MAZDA_MIATA_NA, 4, false},	// falling edges
		{trigger_type_e::TT_FORD_ASPIRE, 4, false},	// two channels, event angles not sorted
		{trigger_type_e::TT_FORD_ASPIRE, 1, false},	// the twin check would pass, the sort check refuses
	};
	for (const Row& r : rows) {
		EngineTestHelper eth(engine_type_e::TEST_ENGINE);
		engineConfiguration->cylindersCount = r.cylinders;
		switch (r.cylinders) {
		case 1:
			engineConfiguration->firingOrder = FO_1;
			break;
		case 4:
			engineConfiguration->firingOrder = FO_1_3_4_2;
			break;
		case 5:
			engineConfiguration->firingOrder = FO_1_2_4_5_3;
			break;
		case 6:
			engineConfiguration->firingOrder = FO_1_5_3_6_2_4;
			break;
		default:
			engineConfiguration->firingOrder = FO_1_8_4_3_6_5_7_2;
		}
		if (r.unevenFire) {
			engineConfiguration->timing_offset_cylinder[1] = 20;
		}
		eth.setTriggerType(r.wheel);
		EXPECT_EQ(r.valid, engine->module<CrankToothSpark>()->isGeometryValid())
			<< getTrigger_type_e(r.wheel) << r.cylinders;
	}
}

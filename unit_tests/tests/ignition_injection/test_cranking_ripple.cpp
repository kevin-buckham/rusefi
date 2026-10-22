/**
 * @file test_cranking_ripple.cpp
 *
 * Spark angle and dwell while cranking a 12 tooth crank whose speed swings within a cycle (compression strokes).
 * Pins what normal cranking scheduling delivers: it times each spark from the RPM stored when the cycle began.
 */

#include "pch.h"

namespace {

struct Tooth {
	efitick_t nt;
	float phase;
};

struct Charge {
	efitick_t nt;
	uint32_t counter;
};

struct Spark {
	double errDeg;	// delivered minus commanded, positive is later
	double dwellMs;
};

struct Cranked {
	std::vector<Tooth> teeth;
	std::vector<Charge> charges;
	std::vector<Spark> sparks;
};

// 12 tooth crank, 4 cylinders, one coil, 4.5 ms cranking dwell, 14 degrees BTDC (the tooth at 12 BTDC is 2 later)
void setUpCrank(EngineTestHelper& eth) {
	engineConfiguration->isFasterEngineSpinUpEnabled = false;
	engineConfiguration->ignitionMode = IM_ONE_COIL;
	engineConfiguration->cylindersCount = 4;
	engineConfiguration->firingOrder = FO_1_3_4_2;
	engineConfiguration->isInjectionEnabled = false;
	engineConfiguration->isIgnitionEnabled = true;
	engineConfiguration->cranking.rpm = 500;
	engineConfiguration->ignitionDwellForCrankingMs = 4.5f;
	engineConfiguration->globalTriggerAngleOffset = 162;
	engineConfiguration->timingMode = TM_FIXED;
	engineConfiguration->fixedTiming = 14;
	eth.setTriggerType(trigger_type_e::TT_12_TOOTH_CRANK);
}

// one tooth (30 degrees) per entry at that crank rpm
Cranked crank(const std::vector<double>& rpm) {
	EngineTestHelper eth(engine_type_e::TEST_ENGINE);
	setUpCrank(eth);

	struct Fire {
		efitick_t nt;
		uint32_t counter;
		float commanded;
	};
	std::vector<Fire> fires;
	Cranked run;
	engine->onIgnitionEvent = [&](IgnitionEvent* e, bool charge) {
		if (charge) {
			run.charges.push_back({getTimeNowNt(), e->sparkCounter});
		} else {
			fires.push_back({getTimeNowNt(), e->sparkCounter, e->sparkAngle});
		}
	};
	for (double r : rpm) {
		eth.moveTimeForwardAndInvokeEventsUs((int)std::lround(5.0 / r * 1e6));
		eth.firePrimaryTriggerRise();
		run.teeth.push_back({getTimeNowNt(), engine->triggerCentral.currentEngineDecodedPhase});
	}

	// where the crank was at each spark, interpolated between the two teeth around it
	for (const Fire& f : fires) {
		size_t k = 0;
		while (k + 2 < run.teeth.size() && run.teeth[k + 1].nt <= f.nt) {
			k++;
		}
		double span = std::fmod(run.teeth[k + 1].phase - run.teeth[k].phase + 1440.0, 720.0);
		double frac = (double)(f.nt - run.teeth[k].nt) / (double)(run.teeth[k + 1].nt - run.teeth[k].nt);
		double err = std::fmod(run.teeth[k].phase + span * frac - f.commanded + 1440.0, 720.0);
		double dwellMs = -1;
		for (const Charge& c : run.charges) {
			if (c.counter == f.counter) {
				dwellMs = NT2US((double)(f.nt - c.nt)) / 1000.0;
			}
		}
		run.sparks.push_back({err > 360 ? err - 720 : err, dwellMs});
	}
	return run;
}

// teeth cranked at rpm, with the speed of tooth n scaled by pattern[n % pattern.size()]
std::vector<double> rippled(double rpm, const std::vector<double>& pattern, size_t teeth) {
	std::vector<double> out;
	for (size_t n = 0; n < teeth; n++) {
		out.push_back(rpm * pattern[n % pattern.size()]);
	}
	return out;
}

} // namespace

std::vector<double> steady(double rpm, size_t teeth) {
	return std::vector<double>(teeth, rpm);
}

// a ramp over `teeth` teeth after 60 steady ones at `from`
std::vector<double> ramp(double from, double to, size_t teeth) {
	std::vector<double> out = steady(from, 60);
	for (size_t n = 0; n < teeth; n++) {
		out.push_back(from + (to - from) * (n + 1) / teeth);
	}
	return out;
}

// every spark in order must be the expected delivered angle error and dwell
void expectSparks(const Cranked& run, const std::vector<Spark>& expected) {
	ASSERT_EQ(expected.size(), run.sparks.size());
	for (size_t i = 0; i < expected.size(); i++) {
		EXPECT_NEAR(expected[i].errDeg, run.sparks[i].errDeg, 0.015) << i;
		EXPECT_NEAR(expected[i].dwellMs, run.sparks[i].dwellMs, 0.015) << i;
	}
}

// a steady crank is on time: the RPM stored at the start of the cycle is the RPM of the whole cycle
TEST(crankingRipple, steady) {
	expectSparks(crank(steady(230, 120)), std::vector<Spark>(15, {0.0, 4.5}));
}

// a speed pattern that repeats every stroke gives every spark the same error (3.6 degrees late here): the timing
// follows the stored average speed, not the speed where the spark happens
TEST(crankingRipple, rippleRepeatingEveryStroke) {
	expectSparks(crank(rippled(230, {0.8, 0.9, 1.1, 1.2, 1.1, 0.9}, 120)), std::vector<Spark>(15, {3.57, 4.5}));
}

// the spark is timed from the speed of the previous cycle, so a crank that speeds up fires later and later and one
// that slows down fires earlier and earlier, up to a dozen degrees off the command
TEST(crankingRipple, speedUpFiresLate) {
	expectSparks(crank(ramp(200, 330, 40)), {
		{0, 4.5}, {0, 4.5}, {0, 4.5}, {0, 4.5}, {0, 4.5}, {1.36, 4.5},
		{4.13, 4.5}, {6.89, 4.5}, {6.67, 4.15}, {9.21, 4.15}, {11.75, 4.15}, {14.28, 4.15}});
}

TEST(crankingRipple, slowDownFiresEarly) {
	expectSparks(crank(ramp(260, 170, 40)), {
		{0, 4.5}, {0, 4.5}, {0, 4.5}, {0, 4.5}, {0, 4.5}, {-0.73, 4.5},
		{-2.18, 4.5}, {-3.63, 4.5}, {-3.85, 4.74}, {-5.38, 4.74}, {-6.91, 4.74}, {-8.44, 4.74}});
}

// one slow tooth (half the speed) just before a spark is timed makes that spark fire 14 degrees early; the sparks
// after it stay 1.2 degrees late
TEST(crankingRipple, oneSlowToothFiresEarly) {
	std::vector<double> rpm = steady(230, 62);
	rpm.push_back(115);
	rpm.resize(62 + 41, 230);
	expectSparks(crank(rpm), {
		{0, 4.5}, {0, 4.5}, {0, 4.5}, {0, 4.5}, {0, 4.5}, {-14, 4.5},
		{0, 4.5}, {0, 4.5}, {1.17, 4.69}, {1.17, 4.69}, {1.17, 4.69}, {1.17, 4.69}});
}

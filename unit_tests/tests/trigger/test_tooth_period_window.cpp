#include "pch.h"

#include "tooth_period_window.h"

namespace {

constexpr float kCycle = 720;
// the same values as edgedwell::kInstabilityRatio / kInstabilityRpm (Task 3 pins that they stay equal)
constexpr float kRatio = 0.1875f;
constexpr float kRpmLimit = 1400.0f;

// feeds one tooth of a 12 tooth wheel: tooth k sits at engine phase 18 + 30k, arriving after the given us/deg
struct Feeder {
	ToothPeriodWindow w;
	double tUs = 1000000;
	int k = 0;

	void tooth(double usPerDeg) {
		tUs += usPerDeg * 30;
		w.onTooth(US2NT((efitick_t)std::llround(tUs)), std::fmod(18.0f + 30.0f * k, kCycle), kCycle);
		k++;
	}
};

} // namespace

TEST(toothPeriodWindow, emptyWindowReportsNothing) {
	ToothPeriodWindow w;
	w.reset();
	EXPECT_EQ(0, w.lastUsPerDeg());
	EXPECT_EQ(0, w.slowUsPerDeg(360));
}

TEST(toothPeriodWindow, windowIsValidOnlyAfterAFull360Degrees) {
	Feeder f;
	f.w.reset();
	for (int i = 0; i < 12; i++) {
		f.tooth(1000); // first tooth only anchors, so 12 teeth = 11 intervals = 330 deg
	}
	EXPECT_EQ(0, f.w.slowUsPerDeg(360));
	EXPECT_NEAR(1000, f.w.lastUsPerDeg(), 0.01);
	f.tooth(1000); // 12 intervals = 360 deg
	EXPECT_NEAR(1000, f.w.slowUsPerDeg(360), 0.01);
}

TEST(toothPeriodWindow, slowestIntervalWinsUntilItLeavesTheWindow) {
	Feeder f;
	f.w.reset();
	for (int i = 0; i < 14; i++) {
		f.tooth(1000);
	}
	f.tooth(2000); // one slow tooth
	for (int i = 0; i < 11; i++) {
		f.tooth(1000);
	}
	// the slow interval is still among the newest 12
	EXPECT_NEAR(2000, f.w.slowUsPerDeg(360), 0.5);
	f.tooth(1000); // and now it is the 13th newest: gone
	EXPECT_NEAR(1000, f.w.slowUsPerDeg(360), 0.5);
}

TEST(toothPeriodWindow, repeatedPhaseAndStalledClockCarryNoSpeed) {
	ToothPeriodWindow w;
	w.reset();
	w.onTooth(US2NT(1000000), 18, kCycle);
	w.onTooth(US2NT(1030000), 18, kCycle); // same phase: zero span
	EXPECT_EQ(0, w.lastUsPerDeg());
	w.onTooth(US2NT(1030000), 48, kCycle); // same instant: zero time
	EXPECT_EQ(0, w.lastUsPerDeg());
	w.onTooth(US2NT(1060000), 78, kCycle);
	EXPECT_NEAR(1000, w.lastUsPerDeg(), 0.01);
}

TEST(toothPeriodWindow, resetForgetsEverything) {
	Feeder f;
	f.w.reset();
	for (int i = 0; i < 20; i++) {
		f.tooth(1000);
	}
	ASSERT_GT(f.w.slowUsPerDeg(360), 0);
	f.w.reset();
	EXPECT_EQ(0, f.w.slowUsPerDeg(360));
	EXPECT_EQ(0, f.w.lastUsPerDeg());
}

// compression ripple repeats every 180 deg (6 teeth): intervals ending at phase 18, 48, ... have these us/deg
static const double kPattern[6] = {1300, 1000, 800, 900, 1000, 1100};

static void feedPattern(Feeder& f, int teeth) {
	for (int i = 0; i < teeth; i++) {
		f.tooth(kPattern[f.k % 6]);
	}
}

TEST(toothPeriodWindow, predictsTheNextIntervalFromOneStrokeEarlier) {
	Feeder f;
	f.w.reset();
	feedPattern(f, 20); // newest tooth is k=19; the next interval is pattern[20 % 6]
	float now = std::fmod(18.0f + 30.0f * 19, kCycle);
	float one = f.w.predictTimeUs(now, now + 30, 180, kCycle, 0.05f);
	EXPECT_NEAR(kPattern[20 % 6] * 30, one, 1.0);
	float two = f.w.predictTimeUs(now, now + 60, 180, kCycle, 0.05f);
	EXPECT_NEAR((kPattern[20 % 6] + kPattern[21 % 6]) * 30, two, 1.0);
}

TEST(toothPeriodWindow, predictionIsScaledByASpeedChangeAlreadyVisible) {
	Feeder f;
	f.w.reset();
	feedPattern(f, 19);
	f.tooth(kPattern[19 % 6] * 0.5); // newest interval is twice as fast as its twin 180 deg earlier
	float now = std::fmod(18.0f + 30.0f * 19, kCycle);
	float one = f.w.predictTimeUs(now, now + 30, 180, kCycle, 0.05f);
	EXPECT_NEAR(kPattern[20 % 6] * 30 * 0.5, one, 1.0);
}

TEST(toothPeriodWindow, predictionGivesUpWhenAnIntervalIsMissing) {
	Feeder f;
	f.w.reset();
	f.tooth(1000);
	f.tooth(1000);
	f.tooth(1000);
	float now = std::fmod(18.0f + 30.0f * 2, kCycle);
	// nothing was recorded a stroke earlier than the next interval
	EXPECT_EQ(0, f.w.predictTimeUs(now, now + 30, 180, kCycle, 0.05f));
	EXPECT_EQ(0, f.w.predictTimeUs(now, now, 180, kCycle, 0.05f)); // zero distance
}

TEST(toothPeriodWindow, predictionExtrapolatesASlowdownThatIsStillInProgress) {
	Feeder f;
	f.w.reset();
	feedPattern(f, 19);
	f.tooth(kPattern[19 % 6] * 2.0); // newest interval is twice as SLOW as its twin 180 deg earlier
	float now = std::fmod(18.0f + 30.0f * 19, kCycle);
	float one = f.w.predictTimeUs(now, now + 30, 180, kCycle, 0.05f);
	// twin x trend 2, and the slowdown continues for (30 + 30) / 2 = 30 deg more: x 2^(30/180)
	EXPECT_NEAR(kPattern[20 % 6] * 30 * 2.0 * std::pow(2.0, 30.0 / 180.0), one, 2.0);
}

TEST(toothPeriodWindow, periodUsSumsExactlyOneStrokeAndRefusesAnythingElse) {
	Feeder f;
	f.w.reset();
	for (int i = 0; i < 20; i++) {
		f.tooth(1000);
	}
	size_t used = 0;
	EXPECT_NEAR(180000, f.w.periodUs(180, 0.05f, 0, &used), 1);
	EXPECT_EQ(6u, used);
	size_t used2 = 0;
	EXPECT_NEAR(180000, f.w.periodUs(180, 0.05f, used, &used2), 1);
	EXPECT_EQ(0, f.w.periodUs(100, 0.05f, 0, &used)); // 30 deg intervals cannot add up to 100 deg
	// not enough history for two strokes after a reset plus 8 teeth
	f.w.reset();
	for (int i = 0; i < 8; i++) {
		f.tooth(1000);
	}
	EXPECT_NEAR(180000, f.w.periodUs(180, 0.05f, 0, &used), 1);
	EXPECT_EQ(0, f.w.periodUs(180, 0.05f, used, &used2));
}

namespace {

// us/deg of a 12 tooth wheel at 'rpm', with ripple (fraction) repeating every 180 deg and per-interval jitter (fraction)
double usPerDegAt(double rpm, int k, double ripple, double jitter, uint32_t& seed) {
	double base = 60e6 / (rpm * 360.0);
	double mid = 18.0 + 30.0 * k - 15.0;
	seed = seed * 1664525u + 1013904223u;
	double j = ((seed >> 8) & 0xFFFF) / 65535.0 * 2 - 1;
	return base * (1.0 - ripple * std::cos(2 * M_PI * mid / 180.0)) * (1.0 + jitter * j);
}

struct InstabilityRun {
	Feeder f;
	std::vector<int> latched;
	uint32_t seed = 11;
	void tooth(double rpm, double ripple = 0.3, double jitter = 0.03) {
		if (!f.w.postStartActive(1e9f)) {
			f.w.markPostStart(1e9f); // these tests exercise the detector itself: keep the post-start window open
		}
		f.tooth(usPerDegAt(rpm, f.k, ripple, jitter, seed));
		f.w.evaluateInstability(180, kCycle, kRatio, kRpmLimit);
		latched.push_back(f.w.instabilityLatched(kCycle));
	}
};

} // namespace

TEST(toothPeriodWindow, steadyRunningWithRippleAndJitterNeverTripsTheInstabilityDetector) {
	for (double rpm : {700.0, 1000.0, 1300.0}) {
		for (double ripple : {0.1, 0.3, 0.5}) {
			for (double jitter : {0.0, 0.03, 0.05}) {
				InstabilityRun r;
				r.f.w.reset();
				for (int i = 0; i < 400; i++) {
					r.tooth(rpm, ripple, jitter);
				}
				EXPECT_EQ(0, std::count(r.latched.begin(), r.latched.end(), 1)) << rpm << " " << ripple << " " << jitter;
			}
		}
	}
}

TEST(toothPeriodWindow, aSpeedStepBelowTheLimitTripsAndLatchesForOneFullCycle) {
	InstabilityRun r;
	r.f.w.reset();
	for (int i = 0; i < 60; i++) {
		r.tooth(1000);
	}
	ASSERT_EQ(0, std::count(r.latched.begin(), r.latched.end(), 1));
	const int stepAt = (int)r.latched.size();
	for (int i = 0; i < 80; i++) {
		r.tooth(650);	// 1000 -> 650 rpm: the next 180 deg takes 54 percent longer
	}
	EXPECT_EQ(1, r.latched[stepAt + 2]);	// trips within the first stroke after the step
	EXPECT_EQ(1, r.latched[stepAt + 30]);	// still latched: the step leaves the comparison after 12 teeth, then 24 more teeth (720 deg)
	EXPECT_EQ(0, r.latched[stepAt + 45]);	// and released after that, although the engine is still running at 650
	EXPECT_EQ(0, r.latched.back());
}

TEST(toothPeriodWindow, aStepAboveTheRpmLimitNeverTripsButTheSameStepBelowItDoes) {
	InstabilityRun above;
	above.f.w.reset();
	for (int i = 0; i < 40; i++) {
		above.tooth(3000);
	}
	for (int i = 0; i < 60; i++) {
		above.tooth(1800);	// 40 percent slower, both above 1400 rpm: a clutch dump or tip-in, not an unstable crank
	}
	EXPECT_EQ(0, std::count(above.latched.begin(), above.latched.end(), 1));

	InstabilityRun below;
	below.f.w.reset();
	for (int i = 0; i < 40; i++) {
		below.tooth(1300);
	}
	for (int i = 0; i < 60; i++) {
		below.tooth(780);
	}
	EXPECT_GT(std::count(below.latched.begin(), below.latched.end(), 1), 0);
}

TEST(toothPeriodWindow, aSpeedUpTripsToo) {
	InstabilityRun r;
	r.f.w.reset();
	for (int i = 0; i < 40; i++) {
		r.tooth(300);
	}
	for (int i = 0; i < 40; i++) {
		r.tooth(800);	// first fire flash
	}
	EXPECT_GT(std::count(r.latched.begin(), r.latched.end(), 1), 0);
}

TEST(toothPeriodWindow, resetClearsTheInstabilityLatch) {
	InstabilityRun r;
	r.f.w.reset();
	for (int i = 0; i < 40; i++) {
		r.tooth(1000);
	}
	for (int i = 0; i < 6; i++) {
		r.tooth(500);
	}
	ASSERT_EQ(1, r.latched.back());
	r.f.w.reset();
	EXPECT_FALSE(r.f.w.instabilityLatched(kCycle));
}

TEST(toothPeriodWindow, aTipInThatStartsBelowTheLimitTripsBrieflyAndReleasesAfterOneCycle) {
	// the E931 rule looks at the newest stroke: 1100 -> 2500 rpm is already 23 percent faster two teeth into the step
	// while the newest stroke still averages 1350 rpm, so it trips; Design question 8
	InstabilityRun r;
	r.f.w.reset();
	for (int i = 0; i < 40; i++) {
		r.tooth(1100);
	}
	const int stepAt = (int)r.latched.size();
	for (int i = 0; i < 80; i++) {
		r.tooth(2500);
	}
	EXPECT_GT(std::count(r.latched.begin() + stepAt, r.latched.end(), 1), 0);
	EXPECT_EQ(0, r.latched.back());
}

TEST(toothPeriodWindow, postStartWindowOpensOnMarkAndClosesAfterTheRequestedDegrees) {
	Feeder f;
	f.w.reset();
	for (int i = 0; i < 20; i++) {
		f.tooth(1000);
	}
	EXPECT_FALSE(f.w.postStartActive(1440));	// never marked
	f.w.markPostStart();
	EXPECT_TRUE(f.w.postStartActive(1440));
	for (int i = 0; i < 47; i++) {
		f.tooth(1000);	// 47 intervals = 1410 deg
	}
	EXPECT_TRUE(f.w.postStartActive(1440));
	f.tooth(1000);		// 48 intervals = 1440 deg = 8 cylinder events
	EXPECT_FALSE(f.w.postStartActive(1440));
}

TEST(toothPeriodWindow, resetClosesThePostStartWindow) {
	Feeder f;
	f.w.reset();
	f.w.markPostStart();
	EXPECT_TRUE(f.w.postStartActive(1440));
	f.w.reset();
	EXPECT_FALSE(f.w.postStartActive(1440));
}

TEST(toothPeriodWindow, latchCannotOutliveThePostStartWindow) {
	InstabilityRun r;
	r.f.w.reset();
	for (int i = 0; i < 40; i++) {
		r.tooth(1000);
	}
	r.f.w.markPostStart(1440);
	// like the firmware wiring: evaluate only while the window is open
	auto step = [&](double rpm) {
		r.f.tooth(usPerDegAt(rpm, r.f.k, 0.3, 0.03, r.seed));
		if (r.f.w.postStartActive(1440)) {
			r.f.w.evaluateInstability(180, kCycle, kRatio, kRpmLimit);
		}
	};
	for (int i = 0; i < 36; i++) {
		step(1000);
	}
	for (int i = 0; i < 6; i++) {
		step(500); // trips in the last cycle of the window (teeth 36..42 of 48)
	}
	ASSERT_TRUE(r.f.w.postStartActive(1440));
	ASSERT_TRUE(r.f.w.instabilityLatched(kCycle));
	for (int i = 0; i < 6; i++) {
		step(500);
	}
	EXPECT_FALSE(r.f.w.postStartActive(1440));
	EXPECT_FALSE(r.f.w.instabilityLatched(kCycle)); // would stay true until the next reset without the window gate
	for (int i = 0; i < 60; i++) {
		step(500);
		EXPECT_FALSE(r.f.w.instabilityLatched(kCycle));
	}
}

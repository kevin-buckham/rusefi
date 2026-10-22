#include "pch.h"

#include "tooth_period_window.h"

namespace {

// a 12 tooth wheel: tooth k sits at engine phase 18 + 30k and arrives after the given us per degree
struct Feeder {
	ToothPeriodWindow window;
	double timeUs = 1000000;
	int k = 0;

	void tooth(double usPerDeg) {
		timeUs += usPerDeg * 30;
		window.onTooth(US2NT((efitick_t)std::llround(timeUs)), std::fmod(18.0f + 30.0f * k, 720.0f), 720);
		k++;
	}

	// phase of the newest tooth
	float now() const {
		return std::fmod(18.0f + 30.0f * (k - 1), 720.0f);
	}
};

// intervals one stroke (6 teeth) apart repeat, like compression ripple
const double ripple[6] = {1300, 1000, 800, 900, 1000, 1100};

} // namespace

TEST(toothPeriodWindow, slowestOver360) {
	Feeder f;
	for (int i = 0; i < 13; i++) {
		f.tooth(1000);
	}
	EXPECT_NEAR(1000, f.window.slowUsPerDeg(), 0.01);
	f.tooth(2000);
	for (int i = 0; i < 11; i++) {
		f.tooth(1000);
	}
	EXPECT_NEAR(2000, f.window.slowUsPerDeg(), 0.5);
	// the slow interval is now the 13th newest
	f.tooth(1000);
	EXPECT_NEAR(1000, f.window.slowUsPerDeg(), 0.5);
}

TEST(toothPeriodWindow, reset) {
	Feeder f;
	for (int i = 0; i < 20; i++) {
		f.tooth(1000);
	}
	ASSERT_GT(f.window.slowUsPerDeg(), 0);
	f.window.reset();
	EXPECT_EQ(0, f.window.slowUsPerDeg());
	EXPECT_EQ(0, f.window.lastUsPerDeg());
}

TEST(toothPeriodWindow, predictFromStrokeTwin) {
	Feeder f;
	for (int i = 0; i < 20; i++) {
		f.tooth(ripple[f.k % 6]);
	}
	// the next two intervals repeat the ones one stroke earlier
	EXPECT_NEAR(ripple[20 % 6] * 30, f.window.predictTimeUs(f.now(), f.now() + 30, 180, 720), 1);
	EXPECT_NEAR((ripple[20 % 6] + ripple[21 % 6]) * 30, f.window.predictTimeUs(f.now(), f.now() + 60, 180, 720), 1);
	// a newest interval twice as slow as its twin scales the prediction and the slowdown continues over the span
	Feeder g;
	for (int i = 0; i < 19; i++) {
		g.tooth(ripple[g.k % 6]);
	}
	g.tooth(ripple[19 % 6] * 2);
	double expected = ripple[20 % 6] * 30 * 2 * std::pow(2.0, 30.0 / 180.0);
	EXPECT_NEAR(expected, g.window.predictTimeUs(g.now(), g.now() + 30, 180, 720), 2);
}

TEST(toothPeriodWindow, predictGivesUpWithoutTwin) {
	Feeder f;
	for (int i = 0; i < 3; i++) {
		f.tooth(1000);
	}
	EXPECT_EQ(0, f.window.predictTimeUs(f.now(), f.now() + 30, 180, 720));
}

TEST(toothPeriodWindow, forwardDistanceMatchesModulo) {
	// the in-range fast path gives the same floats as the modulo it avoids
	for (float from = -720; from <= 1440; from += 18.25f) {
		for (float to = -720; to <= 1440; to += 30.5f) {
			float expected = fmodf(to - from, 720);
			if (expected < 0) {
				expected += 720;
			}
			if (expected >= 720) {
				expected = 0;
			}
			EXPECT_EQ(expected, phaseForwardDistance(from, to, 720));
		}
	}
	EXPECT_EQ(0, phaseForwardDistance(10, 10, 720));
	EXPECT_EQ(0, phaseForwardDistance(0, -1e-9f, 720));
}

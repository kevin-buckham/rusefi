#include "pch.h"

#include "edge_dwell.h"

#include <array>
#include <limits>

using namespace edgedwell;

namespace {

// our wheel at globalTriggerAngleOffset 162: teeth every 30 deg at 18, 48, ... 708 (12 BTDC), each listed twice
// because rise-only shapes carry a rise and a fall entry at the same angle
std::array<float, 48> twelveToothPhases() {
	std::array<float, 48> a;
	for (size_t i = 0; i < a.size(); i++) {
		a[i] = std::fmod(18.0f + 30.0f * (float)(i / 2), 720.0f);
	}
	return a;
}

Plan plan(float commanded, float dwellDeg) {
	auto phases = twelveToothPhases();
	return planForEvent([&](size_t i) { return phases[i]; }, phases.size(), 720, commanded, dwellDeg);
}

float wrap720(float a) {
	while (a < 0) {
		a += 720;
	}
	while (a >= 720) {
		a -= 720;
	}
	return a;
}

} // namespace

TEST(edgeDwellMath, fourteenBtdcCommandLocksToTheTwelveBtdcTooth) {
	// cylinder 1: 14 BTDC = angle 706; a latched dwell of 7.6 deg (6 ms at 210 rpm; any value under one 30 deg tooth spacing behaves the same)
	Plan p = plan(706, 7.6f);
	ASSERT_TRUE(p.valid);
	EXPECT_EQ(708.0f, p.sparkPhase);	// exact tooth, bit for bit
	EXPECT_EQ(678.0f, p.dwellPhase);	// the tooth right before it: 42 BTDC
	EXPECT_NEAR(2.0f, p.retardDeg, 1e-4);
}

TEST(edgeDwellMath, aCommandExactlyOnAToothStaysOnIt) {
	Plan p = plan(708, 7.6f);
	ASSERT_TRUE(p.valid);
	EXPECT_EQ(708.0f, p.sparkPhase);
	EXPECT_NEAR(0.0f, p.retardDeg, 1e-4);
}

TEST(edgeDwellMath, floatNoiseJustPastAToothDoesNotFlipToTheNextOne) {
	// 0.03 deg after the tooth is inside the 0.05 tolerance: still the same tooth, 0.03 deg "advanced"
	Plan p = plan(708.03f, 7.6f);
	ASSERT_TRUE(p.valid);
	EXPECT_EQ(708.0f, p.sparkPhase);
	EXPECT_NEAR(-0.03f, p.retardDeg, 1e-3);
}

TEST(edgeDwellMath, aCommandJustBeyondTheToleranceTakesTheNextToothAndRetardsAlmostAFullSpacing) {
	Plan p = plan(708.2f, 7.6f);
	ASSERT_TRUE(p.valid);
	EXPECT_EQ(18.0f, p.sparkPhase);	// wraps past 720
	EXPECT_EQ(708.0f, p.dwellPhase);
	EXPECT_NEAR(29.8f, p.retardDeg, 1e-3);
}

TEST(edgeDwellMath, theBarePlannerStillRetardsABelowTwelveCommandToTheNextToothWhichIsWhyTheEligibilityRuleExists) {
	// 11 BTDC = 709 -> 18 ATDC: a 29 deg retard. edgeEligible() keeps such events off the planner altogether
	Plan p = plan(709, 7.6f);
	ASSERT_TRUE(p.valid);
	EXPECT_EQ(18.0f, p.sparkPhase);
	EXPECT_NEAR(29.0f, p.retardDeg, 1e-3);
}

TEST(edgeDwellMath, longerDwellStartsFromAnEarlierTooth) {
	EXPECT_EQ(678.0f, plan(706, 30.0f).dwellPhase);	// exactly one spacing still fits the next tooth back
	EXPECT_EQ(648.0f, plan(706, 30.5f).dwellPhase);
	EXPECT_EQ(648.0f, plan(706, 40.0f).dwellPhase);
	EXPECT_EQ(678.0f, plan(706, 0.0f).dwellPhase);	// zero dwell still starts strictly before the spark tooth
}

TEST(edgeDwellMath, dwellLongerThanACycleHasNoPlan) {
	EXPECT_FALSE(plan(706, 720).valid);
	EXPECT_FALSE(plan(706, NAN).valid);
}

TEST(edgeDwellMath, neverMoreAdvancedThanCommandedAndAlwaysOnAToothForEveryCommand) {
	auto phases = twelveToothPhases();
	for (float c = 0; c < 720; c += 0.37f) {
		Plan p = plan(c, 7.6f);
		ASSERT_TRUE(p.valid) << c;
		bool onTooth = false;
		for (float ph : phases) {
			onTooth |= (ph == p.sparkPhase);
		}
		EXPECT_TRUE(onTooth) << c;
		EXPECT_GE(p.retardDeg, -kToothEpsDeg) << c;
		EXPECT_LT(p.retardDeg, 30.0f) << c;
		EXPECT_NE(p.sparkPhase, p.dwellPhase) << c;
	}
}

TEST(edgeDwellMath, dwellDelayFromThePrediction) {
	// predicted 30 ms to the spark tooth, 4.5 ms dwell: charge 25.5 ms after the dwell tooth
	EXPECT_NEAR(25500, dwellStartDelayUs(30, 4.5f, 30000, 0, 0, 800), 1);
	// a prediction well inside twice the slowest tooth (1000 us/deg * 30 deg = 30 ms) is used as is
	EXPECT_NEAR(40500, dwellStartDelayUs(30, 4.5f, 45000, 1000, 0, 800), 1);
	// the prediction says the spark is sooner than the dwell: start immediately, never negative
	EXPECT_EQ(0, dwellStartDelayUs(30, 4.5f, 3000, 0, 0, 800));
}

TEST(edgeDwellMath, theSanityCapStopsAnAbsurdPredictionAtTwiceTheSlowestTooth) {
	// predicted 150 ms against a slowest tooth of 30 ms: capped at 60 ms, charge 55.5 ms after the dwell tooth
	EXPECT_NEAR(55500, dwellStartDelayUs(30, 4.5f, 150000, 1000, 0, 800), 1);
}

TEST(edgeDwellMath, dwellDelayFromTheSlowMaximumIsGuardedByTheNewestInterval) {
	// no prediction: slow 1133 us/deg would delay 27.99 ms, but the newest interval ran at 700 us/deg
	// so at least 3 ms (half the dwell) must remain at that speed: delay <= 30*700 - 3000
	EXPECT_NEAR(18000, dwellStartDelayUs(30, 6, 0, 1133, 700, 800), 1);
	// when the newest interval is as slow as the maximum the guard is not the limit
	EXPECT_NEAR(27990, dwellStartDelayUs(30, 6, 0, 1133, 1133, 800), 1);
}

TEST(edgeDwellMath, dwellDelayFallsBackToTheStoredRpm) {
	// window not full and no newest interval: today's arithmetic, remaining * stored - dwell
	EXPECT_NEAR(30 * 793.6f - 6000, dwellStartDelayUs(30, 6, 0, 0, 0, 793.6f), 1);
	EXPECT_EQ(0, dwellStartDelayUs(5, 6, 0, 0, 0, 793.6f));
}

// cylinder whose TDC is at engine phase tdc, commanded 'advance' deg BTDC
static bool eligible(float tdc, float advance) {
	return edgeEligible(wrap720(tdc - advance), tdc, 720);
}

TEST(edgeDwellMath, anEventIsEdgeEligibleOnlyWhenItsCommandedAdvanceIsAtLeastTwelveDegreesBtdc) {
	for (float tdc : {0.0f, 180.0f, 360.0f, 540.0f}) {
		for (float advance = -60; advance < 90; advance += 0.07f) {
			bool expect = advance >= 12.0f - kToothEpsDeg && advance < 360;
			if (std::fabs(advance - (12.0f - kToothEpsDeg)) < 0.01f) {
				continue; // exactly on the tolerance edge: float noise decides, nothing to pin
			}
			EXPECT_EQ(expect, eligible(tdc, advance)) << tdc << " " << advance;
		}
	}
	// the boundary itself: 12.0 qualifies, 11.9 does not, an ATDC command never does
	EXPECT_TRUE(eligible(0, 12.0f));
	EXPECT_TRUE(eligible(180, 12.0f));
	EXPECT_FALSE(eligible(0, 11.9f));
	EXPECT_FALSE(eligible(0, 0.0f));
	EXPECT_FALSE(eligible(360, -18.0f));
	EXPECT_TRUE(eligible(360, 14.0f));
}

TEST(edgeDwellMath, anEligibleCommandNeverProducesAnAtdcSpark) {
	// the reason for the eligibility rule: from 12 BTDC up the tooth at or after the command is 12 BTDC or earlier, never 18 ATDC
	for (float tdc : {0.0f, 180.0f, 360.0f, 540.0f}) {
		for (float advance = 12.0f; advance < 70; advance += 0.13f) {
			ASSERT_TRUE(eligible(tdc, advance)) << tdc << " " << advance;
			Plan p = plan(wrap720(tdc - advance), 7.6f);
			ASSERT_TRUE(p.valid) << tdc << " " << advance;
			float deliveredBtdc = forwardDistance(p.sparkPhase, tdc, 720);
			EXPECT_GE(deliveredBtdc, 12.0f - kToothEpsDeg) << tdc << " " << advance;	// never an ATDC tooth
			EXPECT_LE(deliveredBtdc, advance + kToothEpsDeg) << tdc << " " << advance;	// never more advanced than commanded
		}
	}
}

TEST(edgeDwellMath, theInstabilityConstantsAreTheMitsubishiOnesAndMatchTheWindowTest) {
	// test_tooth_period_window.cpp repeats these two literals so that it needs no engine header
	EXPECT_EQ(0.1875f, kInstabilityRatio);
	EXPECT_EQ(1400.0f, kInstabilityRpm);
	EXPECT_EQ(3.0f, kEdgeOverdwellFactor);
	EXPECT_EQ(1.5f, kLegacyOverdwellFactor);
	EXPECT_EQ(40.0f, kEdgeBackupMinMs); // test_edge_dwell.cpp / edge_dwell_sweep.h repeat this literal as kBackupMinMs
	EXPECT_EQ(12.0f, kEdgeMinAdvanceDeg);
	EXPECT_EQ(8, kPostStartWindowEvents);
}

TEST(edgeDwellMath, forwardDistanceNeverHangsOnNonFiniteInputOrANonPositiveCycle) {
	const float nan = std::nanf("");
	const float inf = std::numeric_limits<float>::infinity();
	EXPECT_EQ(0, forwardDistance(nan, 10, 720));
	EXPECT_EQ(0, forwardDistance(10, inf, 720));
	EXPECT_EQ(0, forwardDistance(10, 20, 0));
	EXPECT_EQ(0, forwardDistance(10, 20, -720));
	EXPECT_EQ(0, forwardDistance(10, 20, nan));
	EXPECT_EQ(0, forwardDistance(10, 20, inf));
	EXPECT_NEAR(710, forwardDistance(20, 10, 720), 1e-3);
	EXPECT_NEAR(10, forwardDistance(10, 20, 720), 1e-3);
}

TEST(edgeDwellMath, nonFiniteInputsRefuseTheEdgeChargeInsteadOfChargingNow) {
	const float nan = std::nanf("");
	// a NaN delay used to fall through to 0 = charge at the dwell tooth (up to ~28 ms lead-in at 210 rpm: early backup spark)
	EXPECT_EQ(kRefuseCharge, dwellStartDelayUs(nan, 4.5f, 0, 1000, 800, 800));
	EXPECT_EQ(kRefuseCharge, dwellStartDelayUs(30, nan, 30000, 1000, 800, 800));
	EXPECT_EQ(kRefuseCharge, dwellStartDelayUs(-5, 4.5f, 0, 1000, 800, 800));
	// no usable speed at all
	EXPECT_EQ(kRefuseCharge, dwellStartDelayUs(30, 4.5f, 0, 0, 0, nan));
	EXPECT_EQ(kRefuseCharge, dwellStartDelayUs(30, 4.5f, 0, nan, nan, nan));
	EXPECT_EQ(kRefuseCharge, dwellStartDelayUs(30, 4.5f, nan, 0, 0, 0));
	// a NaN OPTIONAL input is "unavailable": the next tier answers
	EXPECT_NEAR(30 * 800 - 4500, dwellStartDelayUs(30, 4.5f, nan, nan, nan, 800), 1);
	EXPECT_NEAR(30 * 1000 - 4500, dwellStartDelayUs(30, 4.5f, 0, 1000, nan, 800), 1);
	// the pre-check the scheduler uses agrees
	EXPECT_FALSE(chargeInputsUsable(nan, 4.5f, 1000, 800, 800));
	EXPECT_FALSE(chargeInputsUsable(30, 4.5f, 0, 0, 0));
	EXPECT_TRUE(chargeInputsUsable(30, 4.5f, 0, 0, 800));
	// good inputs are never refused
	EXPECT_GE(dwellStartDelayUs(30, 4.5f, 3000, 0, 0, 800), 0);
}

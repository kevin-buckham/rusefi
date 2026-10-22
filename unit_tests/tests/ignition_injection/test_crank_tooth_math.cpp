#include "pch.h"

#include "crank_tooth_spark.h"

namespace {

// 12 tooth crank at trigger offset 162: teeth every 30 degrees at 18, 48 ... 708 (12 BTDC), each listed twice
// because a rising-edge-only shape carries a rise and a fall entry
struct Wheel {
	float angles[48];

	Wheel() {
		for (size_t i = 0; i < 48; i++) {
			angles[i] = 18 + 30 * (float)(i / 2);
		}
	}
};

// one stroke is the twin
bool usable(const float* angles, size_t count, int cylinders) {
	return isCrankToothWheelUsable(angles, count, 720, 720.0f / cylinders);
}

CrankToothPlan plan(float spark, float dwellDeg, float tdcPhase = 0, float maxRetard = 5) {
	Wheel w;
	return planCrankToothEvent(w.angles, 48, 0, 720, spark, spark - dwellDeg, tdcPhase, maxRetard);
}

} // namespace

TEST(crankToothMath, planPicksToothAtOrAfterCommand) {
	EngineTestHelper eth(engine_type_e::TEST_ENGINE);
	// cylinder 1, 14 BTDC: the 12 BTDC tooth, with the dwell tooth right before it
	CrankToothPlan p = plan(706, 7.6f);
	ASSERT_TRUE(p.valid);
	EXPECT_EQ(708, p.sparkPhase);
	EXPECT_EQ(678, p.dwellPhase);
	// a command exactly on a tooth, and float noise just past it, keep that tooth
	EXPECT_EQ(708, plan(708, 7.6f).sparkPhase);
	EXPECT_EQ(708, plan(708.03f, 7.6f).sparkPhase);
	// a longer dwell starts from an earlier tooth
	EXPECT_EQ(678, plan(706, 30).dwellPhase);
	EXPECT_EQ(648, plan(706, 30.5f).dwellPhase);
}

TEST(crankToothMath, planRejectsOutOfReachAndAtdc) {
	EngineTestHelper eth(engine_type_e::TEST_ENGINE);
	// 17 BTDC is exactly the default 5 degrees from the tooth, 17.5 is not, and 11 BTDC has only an ATDC tooth after it
	EXPECT_TRUE(plan(703, 7.6f).valid);
	EXPECT_FALSE(plan(702.5f, 7.6f).valid);
	EXPECT_FALSE(plan(709, 7.6f).valid);
	// the limit decides
	EXPECT_TRUE(plan(702.5f, 7.6f, 0, 6).valid);
	EXPECT_FALSE(plan(706, 7.6f, 0, 1).valid);
	// an ATDC command (cylinder 1 TDC is phase 0, so 5 is after it) is never planned
	EXPECT_FALSE(plan(5, 7.6f, 0, 20).valid);
}

// the planner on a dense wheel: 60 teeth, every 12 degrees from 0
TEST(crankToothMath, planOnDenseWheel) {
	EngineTestHelper eth(engine_type_e::TEST_ENGINE);
	float angles[120];
	for (size_t i = 0; i < 120; i++) {
		angles[i] = 12.0f * (float)(i / 2);
	}
	CrankToothPlan p = planCrankToothEvent(angles, 120, 0, 720, 705, 705 - 7.6f, 0, 5);
	ASSERT_TRUE(p.valid);
	EXPECT_EQ(708, p.sparkPhase);
	EXPECT_EQ(696, p.dwellPhase);
	// the tooth after the command is across the end of the cycle, at TDC
	p = planCrankToothEvent(angles, 120, 0, 720, 715, 715 - 7.6f, 0, 5);
	ASSERT_TRUE(p.valid);
	EXPECT_EQ(0, p.sparkPhase);
	EXPECT_EQ(708, p.dwellPhase);
}

// a dwell longer than any lead-in takes the farthest tooth, the one after the spark tooth
TEST(crankToothMath, planDwellLongerThanLeadIn) {
	EngineTestHelper eth(engine_type_e::TEST_ENGINE);
	// 690 degrees is the longest lead-in on this wheel: the tooth after the spark tooth is that far before it
	CrankToothPlan p = plan(706, 690);
	ASSERT_TRUE(p.valid);
	EXPECT_EQ(708, p.sparkPhase);
	EXPECT_EQ(18, p.dwellPhase);
	p = plan(706, 700);
	ASSERT_TRUE(p.valid);
	EXPECT_EQ(18, p.dwellPhase);
}

TEST(crankToothMath, strokeTwinGeometry) {
	Wheel w;
	// 12 teeth, 4 cylinders: a stroke is 6 teeth
	EXPECT_TRUE(usable(w.angles, 48, 4));
	// 5 cylinders: 144 degrees is not a whole number of 30 degree teeth
	EXPECT_FALSE(usable(w.angles, 48, 5));
	// a missing tooth breaks the repeat
	float gap[46];
	for (size_t i = 0; i < 46; i++) {
		gap[i] = w.angles[i < 4 ? i : i + 2];
	}
	EXPECT_FALSE(usable(gap, 46, 4));
	// too few teeth per stroke: 12 teeth on 6 cylinders is 2
	float six[24];
	for (size_t i = 0; i < 24; i++) {
		six[i] = 60 * (float)(i / 2);
	}
	EXPECT_FALSE(usable(six, 24, 6));
	EXPECT_TRUE(usable(six, 24, 4));
	// angles that are not in order (two channel shapes) are refused, the searches need them sorted
	float unsorted[48];
	std::copy(w.angles, w.angles + 48, unsorted);
	unsorted[10] = unsorted[3];
	EXPECT_FALSE(usable(unsorted, 48, 4));
	EXPECT_FALSE(usable(unsorted, 48, 1));
}

TEST(crankToothMath, twinStride) {
	Wheel w;
	// a revolution twin: 12 teeth every 360 degrees
	EXPECT_TRUE(isCrankToothWheelUsable(w.angles, 48, 720, 360));
	// the first revolution differs from the second
	float shifted[48];
	std::copy(w.angles, w.angles + 48, shifted);
	for (size_t i = 0; i < 24; i++) {
		shifted[i] += 7;
	}
	EXPECT_FALSE(isCrankToothWheelUsable(shifted, 48, 720, 360));
	// the window holds the twin of at most 60 teeth: a single cylinder on a 62 tooth wheel does not fit
	float dense[124];
	for (size_t n : {60u, 62u}) {
		for (size_t i = 0; i < 2 * n; i++) {
			dense[i] = 720.0f / n * (float)(i / 2);
		}
		EXPECT_EQ(n == 60, usable(dense, 2 * n, 1)) << n;
	}
}

TEST(crankToothMath, dwellDelayTiers) {
	// prediction: 20 ms to the spark tooth less 4.5 ms of dwell
	EXPECT_NEAR(15500, crankToothChargeDelayUs(30, 4.5f, 20000, 800, 700, 900), 0.1);
	// capped at 2 x remaining x the slowest interval
	EXPECT_NEAR(60000 - 4500, crankToothChargeDelayUs(30, 4.5f, 500000, 1000, 700, 900), 0.1);
	// no prediction: the slowest interval, limited so half the dwell remains at the newest speed
	EXPECT_NEAR(30 * 800 - 4500, crankToothChargeDelayUs(30, 4.5f, 0, 800, 0, 900), 0.1);
	EXPECT_NEAR(30 * 400 - 2250, crankToothChargeDelayUs(30, 4.5f, 0, 800, 400, 900), 0.1);
	// no window at all: the stored speed
	EXPECT_NEAR(30 * 900 - 4500, crankToothChargeDelayUs(30, 4.5f, 0, 0, 0, 900), 0.1);
	// already due starts now, never negative
	EXPECT_EQ(0, crankToothChargeDelayUs(1, 4.5f, 0, 800, 700, 900));
	// nothing usable
	EXPECT_FALSE(isCrankToothChargeUsable(30, 4.5f, 0));
	EXPECT_FALSE(isCrankToothChargeUsable(NAN, 4.5f, 700));
}

TEST(crankToothMath, lateDwell) {
	// half the dwell, at least 1.5 ms
	EXPECT_NEAR(2.25f, crankToothLateDwellMs(4.5f, 725, 15), 1e-4);
	EXPECT_NEAR(1.5f, crankToothLateDwellMs(2, 725, 15), 1e-4);
	// 2.25 ms is 15 degrees at 150 us per degree (1000 rpm): no later spark than that
	EXPECT_NEAR(2.25f, crankToothLateDwellMs(4.5f, 150, 15), 1e-4);
	EXPECT_EQ(0, crankToothLateDwellMs(4.5f, 140, 15));
	// no tooth interval
	EXPECT_EQ(0, crankToothLateDwellMs(4.5f, 0, 15));
	// a smaller limit refuses sooner, 0 is off
	EXPECT_NEAR(2.25f, crankToothLateDwellMs(4.5f, 725, 4), 1e-4);
	EXPECT_EQ(0, crankToothLateDwellMs(4.5f, 725, 3));
	EXPECT_EQ(0, crankToothLateDwellMs(4.5f, 725, 0));
}

TEST(crankToothMath, backupMs) {
	EXPECT_EQ(40, crankToothBackupMs(4.5f, true, 40));
	EXPECT_EQ(60, crankToothBackupMs(20, true, 40));
	EXPECT_EQ(27, crankToothBackupMs(9, false, 40));
	EXPECT_EQ(20, crankToothBackupMs(9, false, 20));
}

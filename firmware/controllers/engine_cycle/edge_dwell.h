/**
 * @file edge_dwell.h
 *
 * Pure helpers for the cranking edge-dwell scheduler (engineConfiguration->crankingEdgeDwell).
 * No engine state is touched here, so every function is unit-testable on its own.
 *
 * Angles are engine phase in degrees [0, cycleDeg), the same space as IgnitionEvent::sparkAngle
 * and the currentPhase/nextPhase passed to onTriggerEventSparkLogic().
 */
#pragma once

#include <stddef.h>
#include <math.h>
#include <cmath>

namespace edgedwell {

// A tooth within this many degrees BEFORE the commanded angle still counts as "at" it (float noise
// between the spark-angle arithmetic and the tooth-angle arithmetic). 0.05 deg = ~40 us at 210 rpm.
constexpr float kToothEpsDeg = 0.05f;

// over-dwell backup: legacy firing is at 1.5x dwell. In edge mode the backup would deliver an EARLY spark
// (opening a charged coil always sparks), so it is pushed out to 3x dwell (13.5 ms at the tune's 4.5 ms cranking dwell).
constexpr float kLegacyOverdwellFactor = 1.5f;
constexpr float kEdgeOverdwellFactor = 3.0f;

// Edge mode is used for an ignition event only when its COMMANDED advance is at least this many degrees BTDC. Below it
// the tooth at or after the command would be the 18 ATDC tooth (up to 29 deg of retard), so the event keeps the legacy
// time-scheduled path (owner decision 2026-10-17, replaces the earlier 12 BTDC floor). With crankingAdvance 0/14/16/16
// at 0/200/400/1000 rpm the command reaches 12 deg at about 171 rpm.
constexpr float kEdgeMinAdvanceDeg = 12.0f;

// Mitsubishi E931/E932 style instability DETECTOR (borrowed idea only, not their scheme): below kInstabilityRpm a change
// of the per-cylinder period of kInstabilityRatio (18.75 percent) or more latches edge mode for one more engine cycle,
// but ONLY during the first kPostStartWindowEvents cylinder events after the switch to RUNNING (bit
// edgeDwellPostStartLatch). 8 events = 2 engine cycles: the stored rpm is refreshed once per cycle, so the stale-rpm
// sparks of the logged 18:00 start last at most one cycle after the switch, and the detector needs 360 deg of history
// to compare two 180 deg windows.
// Compile-time constants: the config struct has no spare scalar slot that can be taken without a TunerStudio
// field and a tune default, so promoting them to settings is deferred until the car shows they need tuning.
constexpr float kInstabilityRatio = 0.1875f;
constexpr float kInstabilityRpm = 1400.0f;
constexpr int kPostStartWindowEvents = 8;

// never start charging so late that, at the speed of the most recent tooth interval, less than this
// fraction of the planned dwell would be left before the spark tooth
constexpr float kMinDwellFraction = 0.5f;

// stretch applied to the stroke-twin prediction, kept at 1.0. A late charge start only shortens dwell (weak spark, or a lost
// spark when it is later than the spark itself) while an early one lengthens it toward the over-dwell backup (early spark).
// 4.5 ms is only 19 percent of a 30 deg tooth at 210 rpm, so any stretch costs a visible share of the dwell: measured on the
// earlier 6 ms design a 10 percent stretch lost 36 and 20 percent lost 78 of 504 sparks.
constexpr float kSlowBias = 1.0f;

// tier-1 sanity cap, in multiples of (remaining degrees x slowest us/deg of the last 360 deg)
constexpr float kTier1CapFactor = 2.0f;

inline float forwardDistance(float from, float to, float cycleDeg) {
	// guarded like ToothPeriodWindow::forwardDistance: non-finite input or a non-positive cycle returns 0 (no loop to hang)
	if (!(cycleDeg > 0) || !std::isfinite(cycleDeg) || !std::isfinite(from) || !std::isfinite(to)) {
		return 0;
	}
	float d = fmodf(to - from, cycleDeg);
	if (d < 0) {
		d += cycleDeg;
	}
	if (d >= cycleDeg) {
		d = 0;
	}
	return d;
}

struct Plan {
	bool valid = false;
	// exact tooth phases (bit-identical to the currentPhase/nextPhase values that trigger_central hands to
	// mainTriggerCallback, so isPhaseInRange() inclusive-start selects the tooth with zero delay)
	float sparkPhase = 0;
	float dwellPhase = 0;
	// how far the spark tooth is later than the command, degrees, >= -kToothEpsDeg by construction
	float retardDeg = 0;
};

/**
 * Is this event's commanded advance (degrees BTDC, from the latched spark angle and the cylinder's TDC phase) at least
 * kEdgeMinAdvanceDeg? ATDC commands (advance negative, i.e. the forward distance spark -> TDC is more than half a cycle)
 * are never eligible. kToothEpsDeg of float noise is tolerated so that a command of exactly 12.0 qualifies.
 */
inline bool edgeEligible(float sparkAngle, float tdcPhase, float cycleDeg) {
	float advance = forwardDistance(sparkAngle, tdcPhase, cycleDeg);
	return advance < cycleDeg / 2 && advance >= kEdgeMinAdvanceDeg - kToothEpsDeg;
}

/**
 * @param phaseAt  callable: size_t index -> float engine phase of trigger event 'index' (duplicates allowed)
 * @param count    number of trigger events per engine cycle
 * @param commandedSpark latched IgnitionEvent::sparkAngle
 * @param dwellDurationDeg latched dwell duration in degrees (sparkAngle - dwellAngle, wrapped); only used to
 *        choose WHICH tooth starts dwell, never to time it - so the choice cannot change between teeth
 */
template <typename PhaseAt>
Plan planForEvent(PhaseAt phaseAt, size_t count, float cycleDeg, float commandedSpark, float dwellDurationDeg) {
	Plan p;
	if (count == 0 || !(cycleDeg > 0) || !(dwellDurationDeg >= 0) || dwellDurationDeg >= cycleDeg) {
		return p;
	}

	// 1) spark tooth: smallest forward distance from (commanded - eps); i.e. the tooth at or after the command
	float bestSparkDist = cycleDeg;
	size_t sparkIdx = count;
	for (size_t i = 0; i < count; i++) {
		float d = forwardDistance(commandedSpark - kToothEpsDeg, phaseAt(i), cycleDeg);
		if (d < bestSparkDist) {
			bestSparkDist = d;
			sparkIdx = i;
		}
	}
	if (sparkIdx == count) {
		return p;
	}
	p.sparkPhase = phaseAt(sparkIdx);
	p.retardDeg = bestSparkDist - kToothEpsDeg;

	// 2) dwell tooth: the nearest tooth that is strictly before the spark tooth and at least dwellDurationDeg behind it
	float bestDwellDist = cycleDeg;
	size_t dwellIdx = count;
	for (size_t i = 0; i < count; i++) {
		float dist = forwardDistance(phaseAt(i), p.sparkPhase, cycleDeg);
		if (dist <= kToothEpsDeg) {
			continue; // the spark tooth itself (or a duplicate of it)
		}
		if (dist + kToothEpsDeg < dwellDurationDeg) {
			continue; // too close: would leave less dwell than latched
		}
		if (dist < bestDwellDist) {
			bestDwellDist = dist;
			dwellIdx = i;
		}
	}
	if (dwellIdx == count) {
		// dwell is longer than any available lead-in: take the farthest tooth that is still before the spark
		float farthest = 0;
		for (size_t i = 0; i < count; i++) {
			float dist = forwardDistance(phaseAt(i), p.sparkPhase, cycleDeg);
			if (dist > kToothEpsDeg && dist > farthest) {
				farthest = dist;
				dwellIdx = i;
			}
		}
		if (dwellIdx == count) {
			return p;
		}
	}
	p.dwellPhase = phaseAt(dwellIdx);
	p.valid = true;
	return p;
}

/**
 * Delay from the dwell tooth to the start of coil charge. Three tiers, best first:
 *  1. predictedUs > 0: the 180-deg stroke-twin prediction (ToothPeriodWindow::predictTimeUs with the 180-deg twin, scaled by
 *     the newest interval's change and extrapolated for a decelerating crank), sanity-capped at 2 x the slowest
 *     tooth of the last 360 deg;
 *  2. slowUsPerDeg > 0: the slow-biased 360-deg maximum, guarded by the newest interval so the charge cannot start
 *     so late that less than kMinDwellFraction of the dwell remains at the newest speed;
 *  3. otherwise the stored 720-deg RPM (today's behaviour), same guard.
 *
 * @param remainingDeg degrees from the dwell tooth to the spark tooth
 * @param predictedUs predicted microseconds from the dwell tooth to the spark tooth, 0 if unavailable
 * @param slowUsPerDeg largest us/deg over the last 360 deg, 0 if that window is not full
 * @param lastUsPerDeg us/deg of the most recent tooth interval, 0 if unknown
 * @param storedUsPerDeg us/deg from the stored 720-deg RPM
 *
 * The result is never negative unless refused: when the arithmetic says the charge should already have started, it
 * starts right at the dwell tooth (shortest safe dwell that still has a chance to spark) rather than being skipped.
 *
 * Refusal (kRefuseCharge, -1): the inputs cannot produce a trustworthy finite delay (NaN/inf remaining degrees or dwell,
 * or no usable speed at all, or a non-finite result). A NaN used to fall through to 0 = "charge now", i.e. up to ~28 ms
 * of lead-in at 210 rpm against a 13.5 ms over-dwell backup (an early spark). The caller must then keep the event on the
 * legacy path (see chargeInputsUsable, checked before the edge plan is accepted). Non-finite OPTIONAL inputs (predictedUs,
 * slowUsPerDeg, lastUsPerDeg) are treated as "unavailable" (0) and the next tier is used.
 */
constexpr float kRefuseCharge = -1.0f;

inline bool chargeInputsUsable(float remainingDeg, float dwellMs, float slowUsPerDeg, float lastUsPerDeg, float storedUsPerDeg) {
	if (!std::isfinite(remainingDeg) || remainingDeg < 0 || !std::isfinite(dwellMs) || dwellMs < 0) {
		return false;
	}
	auto ok = [](float v) { return std::isfinite(v) && v > 0; };
	return ok(slowUsPerDeg) || ok(lastUsPerDeg) || ok(storedUsPerDeg);
}

inline float dwellStartDelayUs(float remainingDeg, float dwellMs, float predictedUs, float slowUsPerDeg, float lastUsPerDeg, float storedUsPerDeg) {
	if (!std::isfinite(predictedUs)) {
		predictedUs = 0;
	}
	if (!std::isfinite(slowUsPerDeg)) {
		slowUsPerDeg = 0;
	}
	if (!std::isfinite(lastUsPerDeg)) {
		lastUsPerDeg = 0;
	}
	if (!std::isfinite(storedUsPerDeg)) {
		storedUsPerDeg = 0;
	}
	if (!chargeInputsUsable(remainingDeg, dwellMs, slowUsPerDeg, lastUsPerDeg, storedUsPerDeg)) {
		return kRefuseCharge;
	}
	float dwellUs = dwellMs * 1000.0f;
	if (predictedUs > 0) {
		// stroke-twin prediction (already extrapolated for a decelerating crank, see ToothPeriodWindow::predictTimeUs).
		// Sanity cap only: never longer than kTier1CapFactor x the slowest tooth of the last 360 deg. A cap at 1.0 x (the
		// locked MAX predictor) pulled the charge start early exactly when the crank was slowing: removing it took the
		// die-back sweep from 24 backup fires to 0.
		float t = predictedUs * kSlowBias;
		if (slowUsPerDeg > 0 && t > kTier1CapFactor * remainingDeg * slowUsPerDeg) {
			t = kTier1CapFactor * remainingDeg * slowUsPerDeg;
		}
		// The dwell is t - delay = dwellUs at the estimate by construction (so at most 1 x configured when the estimate is
		// right; the REAL dwell is bounded by the sweeps). A separate 2 x dwell lower bound on the delay was removed: it is
		// arithmetically unreachable (t - dwellUs >= t - 2 x dwellUs for any dwellUs >= 0).
		float d = t - dwellUs;
		return d > 0 ? d : (d == d ? 0 : kRefuseCharge);
	}
	float usPerDeg = slowUsPerDeg > 0 ? slowUsPerDeg : storedUsPerDeg;
	float delay = remainingDeg * usPerDeg - dwellUs;
	if (lastUsPerDeg > 0) {
		float guard = remainingDeg * lastUsPerDeg - kMinDwellFraction * dwellUs;
		if (delay > guard) {
			delay = guard;
		}
	}
	if (!std::isfinite(delay)) {
		return kRefuseCharge;
	}
	return delay > 0 ? delay : 0;
}

} // namespace edgedwell

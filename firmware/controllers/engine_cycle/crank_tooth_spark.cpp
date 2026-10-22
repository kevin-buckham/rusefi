/**
 * @file crank_tooth_spark.cpp
 */

#include "pch.h"

#include "crank_tooth_spark.h"

#if EFI_ENGINE_CONTROL && EFI_SHAFT_POSITION_INPUT

// ATDC advances are more than half a cycle forward of the cylinder's TDC
static bool isBtdc(float advanceDeg, float cycleDeg) {
	return advanceDeg < cycleDeg / 2 || advanceDeg >= cycleDeg - ToothPeriodWindow::toothEpsDeg;
}

// the phase mainTriggerCallback receives for this trigger event
static float toothPhase(const float* eventAngles, size_t index, float tdcPosition) {
	return wrapAngleMethod(eventAngles[index] - tdcPosition, "toothPhase", ObdCode::CUSTOM_ERR_6555);
}

CrankToothPlan planCrankToothEvent(const float* eventAngles, size_t count, float tdcPosition, float cycleDeg,
		float sparkAngle, float dwellAngle, float tdcPhase, float maxRetardDeg) {
	constexpr float eps = ToothPeriodWindow::toothEpsDeg;
	const float dwellDeg = phaseForwardDistance(dwellAngle, sparkAngle, cycleDeg);
	CrankToothPlan plan;

	if (count == 0 || !isBtdc(phaseForwardDistance(sparkAngle, tdcPhase, cycleDeg), cycleDeg)) {
		return plan;
	}

	// spark tooth: nearest at or after the command; eps absorbs rounding between the two angle frames
	const float from = sparkAngle - eps;
	float sparkDist = cycleDeg;
	size_t sparkIdx = count;
	for (size_t i = 0; i < count; i++) {
		float d = phaseForwardDistance(from, toothPhase(eventAngles, i, tdcPosition), cycleDeg);
		if (d < sparkDist) {
			sparkDist = d;
			sparkIdx = i;
		}
	}
	if (sparkIdx == count) {
		return plan;
	}
	plan.sparkPhase = toothPhase(eventAngles, sparkIdx, tdcPosition);
	bool tooFar = sparkDist - eps > maxRetardDeg + eps;
	if (tooFar || !isBtdc(phaseForwardDistance(plan.sparkPhase, tdcPhase, cycleDeg), cycleDeg)) {
		return plan;
	}

	// dwell tooth: the latest tooth at least the latched dwell before the spark tooth
	float dwellDist = cycleDeg;
	float farthestDist = 0;
	size_t dwellIdx = count;
	size_t farthestIdx = count;
	for (size_t i = 0; i < count; i++) {
		float dist = phaseForwardDistance(toothPhase(eventAngles, i, tdcPosition), plan.sparkPhase, cycleDeg);
		if (dist > eps && dist + eps >= dwellDeg && dist < dwellDist) {
			dwellDist = dist;
			dwellIdx = i;
		}
		if (dist > eps && dist > farthestDist) {
			farthestDist = dist;
			farthestIdx = i;
		}
	}
	if (dwellIdx == count) {
		// no tooth leaves the dwell: the farthest one, the tooth after the spark tooth, which the window holds
		dwellIdx = farthestIdx;
	}
	if (dwellIdx == count) {
		return plan;
	}
	plan.dwellPhase = toothPhase(eventAngles, dwellIdx, tdcPosition);
	plan.valid = true;
	return plan;
}

// is there an event within toothEpsDeg of target (sorted angles, target may be up to one cycle past the end)
static bool hasEventNear(const float* angles, size_t count, float cycleDeg, float target) {
	constexpr float eps = ToothPeriodWindow::toothEpsDeg;
	const float wrapped = target >= cycleDeg ? target - cycleDeg : target;
	const size_t k = std::lower_bound(angles, angles + count, wrapped - eps) - angles;
	// the neighbours of k absorb rounding in the comparison, both ends cover the wrap at 0
	const size_t candidates[] = { k > 0 ? k - 1 : 0, k < count ? k : count - 1, k + 1 < count ? k + 1 : count - 1,
		0, count - 1 };
	for (size_t j : candidates) {
		float d = phaseForwardDistance(target, angles[j], cycleDeg);
		if (d <= eps || d >= cycleDeg - eps) {
			return true;
		}
	}
	return false;
}

bool isCrankToothWheelUsable(const float* eventAngles, size_t eventCount, float cycleDeg, float twinDeg) {
	if (eventCount == 0 || !(cycleDeg > 0) || !(twinDeg > 0)) {
		return false;
	}
	// the twin search needs the angles in order; the two channel shapes that break it are refused
	size_t teeth = 1;
	for (size_t i = 1; i < eventCount; i++) {
		if (eventAngles[i] < eventAngles[i - 1]) {
			return false;
		}
		// the distinct angles are the teeth: a rising-edge tooth is listed for the rise and for the fall at one angle
		teeth += eventAngles[i] != eventAngles[i - 1];
	}
	// fewer teeth per twin interval and one noisy interval is a large share of it; more than the window holds
	// and the twin would have been forgotten
	const float teethPerTwin = teeth * twinDeg / cycleDeg;
	if (teethPerTwin < 3 - 0.001f || teethPerTwin > ToothPeriodWindow::maxTeethPerTwin) {
		return false;
	}
	if (twinDeg >= cycleDeg) {
		// every tooth is its own twin
		return true;
	}
	for (size_t i = 0; i < eventCount; i++) {
		if (!hasEventNear(eventAngles, eventCount, cycleDeg, eventAngles[i] + twinDeg)) {
			return false;
		}
	}
	return true;
}

float crankToothBackupMs(float dwellMs, bool cranking, float backupLimitMs) {
	const float threeDwells = 3 * dwellMs;
	return cranking ? std::max(backupLimitMs, threeDwells) : std::min(threeDwells, backupLimitMs);
}

// the stored speed is not validated elsewhere: non-finite or non-positive counts as unavailable
static float usableUsPerDeg(float v) {
	return (std::isfinite(v) && v > 0) ? v : 0;
}

bool isCrankToothChargeUsable(float remainingDeg, float dwellMs, float lastUsPerDeg) {
	if (!std::isfinite(remainingDeg) || remainingDeg < 0 || !std::isfinite(dwellMs) || dwellMs < 0) {
		return false;
	}
	return lastUsPerDeg > 0;
}

float crankToothChargeDelayUs(float remainingDeg, float dwellMs, float predictedUs, float slowUsPerDeg,
		float lastUsPerDeg, float storedUsPerDeg) {
	const float dwellUs = dwellMs * 1000.0f;
	if (predictedUs > 0) {
		// capped at 2 x the slowest interval; a tighter cap pulls the charge early exactly while the crank slows
		float t = slowUsPerDeg > 0 ? std::min(predictedUs, 2.0f * remainingDeg * slowUsPerDeg) : predictedUs;
		return std::max(t - dwellUs, 0.0f);
	}
	float delay = remainingDeg * (slowUsPerDeg > 0 ? slowUsPerDeg : usableUsPerDeg(storedUsPerDeg)) - dwellUs;
	if (lastUsPerDeg > 0) {
		// keep at least half the dwell at the newest speed
		delay = std::min(delay, remainingDeg * lastUsPerDeg - 0.5f * dwellUs);
	}
	return std::max(delay, 0.0f);
}

// a setting of 0 is what a tune saved before the setting existed reads as
static constexpr float defaultMaxRetardDeg = 5;
static constexpr float defaultBackupDwellMs = 40;

float crankToothLateDwellMs(float dwellMs, float lastUsPerDeg, float maxDeg) {
	const float ms = std::max(1.5f, 0.5f * dwellMs);
	return (maxDeg > 0 && lastUsPerDeg > 0 && ms * 1000 <= maxDeg * lastUsPerDeg) ? ms : 0;
}

void CrankToothSpark::onTooth(efitick_t edgeTimestamp, angle_t currentPhase) {
	const bool enabled = engineConfiguration->crankingToothLockedSpark;
	if (enabled && !m_settingWasOn) {
		// stale intervals from before the setting was enabled
		m_window.reset();
	}
	m_settingWasOn = enabled;
	if (enabled) {
		m_window.onTooth(edgeTimestamp, currentPhase, engine->engineState.engineCycle);
	}
}

void CrankToothSpark::onEngineStop() {
	// the trigger callback may be feeding the window
	chibios_rt::CriticalSectionLocker csl;
	m_window.reset();
}

void CrankToothSpark::resetFromTrigger() {
	m_window.reset();
}

void CrankToothSpark::onConfigurationChange(engine_configuration_s const* /*previousConfig*/) {
	updateGeometry();
}

void CrankToothSpark::onSlowCallback() {
	if (!m_geometryKnown || m_geometryKey != layoutKey()) {
		updateGeometry();
	}
}

uint32_t CrankToothSpark::layoutKey() const {
	uint32_t key = engine->globalConfigurationVersion;
	auto mix = [&key](uint32_t v) {
		key = key * 16777619u ^ v;
	};
	const TriggerCentral& tc = engine->triggerCentral;
	mix(engineConfiguration->cylindersCount);
	mix(tc.engineCycleEventCount);
	mix((uint32_t)engine->engineState.engineCycle);
	mix(tc.triggerShape.useOnlyRisingEdges + 2 * tc.triggerShape.shapeDefinitionError);
	for (size_t i = 0; i < engineConfiguration->cylindersCount; i++) {
		uint32_t bits;
		memcpy(&bits, &engineConfiguration->timing_offset_cylinder[i], sizeof(bits));
		mix(bits);
	}
	return key;
}

void CrankToothSpark::updateGeometry() {
	m_geometryKey = layoutKey();
	m_geometryKnown = true;
	m_geometryValid = false;

	const TriggerCentral& tc = engine->triggerCentral;
	const float cycle = engine->engineState.engineCycle;
	const size_t cylinders = engineConfiguration->cylindersCount;
	if (cylinders == 0 || tc.triggerShape.shapeDefinitionError || !tc.triggerShape.useOnlyRisingEdges || cycle != 720) {
		return;
	}
	// uneven firing: the stroke is not cycle / cylinders for every cylinder
	for (size_t i = 0; i < cylinders; i++) {
		if (engineConfiguration->timing_offset_cylinder[i] != 0) {
			return;
		}
	}
	const float twinDeg = cycle / cylinders;
	m_geometryValid = isCrankToothWheelUsable(tc.triggerFormDetails.eventAngles, tc.engineCycleEventCount, cycle,
		twinDeg);
	m_twinDeg = twinDeg;
}

bool CrankToothSpark::isGeometryValid() const {
	return m_geometryValid && m_geometryKey == layoutKey();
}

bool CrankToothSpark::isAllowed() const {
	return engineConfiguration->crankingToothLockedSpark
		&& engine->rpmCalculator.getState() == CRANKING
		&& !(engine->engineState.useOddFireWastedSpark && getCurrentIgnitionMode() == IM_WASTED_SPARK)
		&& isGeometryValid();
}

CrankToothPlan CrankToothSpark::plan(const IgnitionEvent& event) const {
	const TriggerCentral& tc = engine->triggerCentral;
	const float configured = engineConfiguration->crankingToothLockMaxRetardDeg;
	return planCrankToothEvent(tc.triggerFormDetails.eventAngles, tc.engineCycleEventCount, tdcPosition(),
		engine->engineState.engineCycle, event.sparkAngle, event.dwellAngle,
		getPerCylinderFiringOrderOffset(event.cylinderIndex, event.coilIndex),
		configured == 0 ? defaultMaxRetardDeg : clampF(0, configured, 20));
}

bool CrankToothSpark::isChargeUsable(float currentPhase, float sparkPhase, floatms_t dwellMs) const {
	return isCrankToothChargeUsable(phaseForwardDistance(currentPhase, sparkPhase, engine->engineState.engineCycle),
		dwellMs, m_window.lastUsPerDeg());
}

efitick_t CrankToothSpark::chargeTime(efitick_t edgeTimestamp, float currentPhase, float sparkPhase,
		floatms_t dwellMs) const {
	const float cycle = engine->engineState.engineCycle;
	const float remainingDeg = phaseForwardDistance(currentPhase, sparkPhase, cycle);
	const float slow = m_window.slowUsPerDeg();
	// the twin prediction needs a full 360 degree window
	float predictedUs = 0;
	if (slow > 0) {
		predictedUs = m_window.predictTimeUs(currentPhase, sparkPhase, m_twinDeg, cycle);
	}
	const float delayUs = crankToothChargeDelayUs(remainingDeg, dwellMs, predictedUs, slow, m_window.lastUsPerDeg(),
		engine->rpmCalculator.oneDegreeUs);
	return sumTickAndFloat(edgeTimestamp, USF2NT(delayUs));
}

floatms_t CrankToothSpark::overDwellMs(floatms_t dwellMs) const {
	const float configured = engineConfiguration->crankingToothLockBackupDwellMs;
	return crankToothBackupMs(dwellMs, engine->rpmCalculator.getState() == CRANKING,
		configured == 0 ? defaultBackupDwellMs : clampF(10, configured, 100));
}

floatms_t CrankToothSpark::lateDwellMs(floatms_t dwellMs) const {
	return crankToothLateDwellMs(dwellMs, m_window.lastUsPerDeg(), engineConfiguration->crankingToothLateSparkMaxDeg);
}

#endif // EFI_ENGINE_CONTROL && EFI_SHAFT_POSITION_INPUT

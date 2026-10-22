/**
 * @file crank_tooth_spark.h
 *
 * crankingToothLockedSpark: while cranking, fire each spark on a trigger tooth and time the coil charge from the
 * recent tooth intervals. Angles are engine phase in degrees, [0, cycleDeg).
 */
#pragma once

#include <stddef.h>

#include "crank_tooth_plan.h"
#include "engine_module.h"
#include "tooth_period_window.h"

class IgnitionEvent;

// Spark tooth: first tooth at or after the command, within maxRetardDeg and BTDC; dwell tooth: nearest one at least the
// latched dwell before it. tdcPhase is the engine phase of this cylinder's TDC.
CrankToothPlan planCrankToothEvent(const float* eventAngles, size_t eventCount, float tdcPosition, float cycleDeg,
	float sparkAngle, float dwellAngle, float tdcPhase, float maxRetardDeg);

// Does the twin-based charge prediction fit this wheel: a tooth exactly twinDeg after every tooth, 3 to
// ToothPeriodWindow::maxTeethPerTwin teeth per twin interval. twinDeg is one stroke (cycleDeg / cylinders) here.
bool isCrankToothWheelUsable(const float* eventAngles, size_t eventCount, float cycleDeg, float twinDeg);

// Ms after the start of charge at which a coil whose spark tooth has not arrived is released: at least backupLimitMs
// while cranking, at most backupLimitMs once RUNNING.
float crankToothBackupMs(float dwellMs, bool cranking, float backupLimitMs);

// Is there enough speed information to time a tooth-locked charge: an interval since the window was reset.
bool isCrankToothChargeUsable(float remainingDeg, float dwellMs, float lastUsPerDeg);

// Delay from the dwell tooth to the start of charge: twin prediction, else slowest recent interval, else stored
// RPM. Never negative.
float crankToothChargeDelayUs(float remainingDeg, float dwellMs, float predictedUs, float slowUsPerDeg,
	float lastUsPerDeg, float storedUsPerDeg);

#if EFI_ENGINE_CONTROL && EFI_SHAFT_POSITION_INPUT

class CrankToothSpark : public EngineModule {
public:
	void onEngineStop() override;
	// The trigger shape and cylinder layout are current by the time modules are told about a burn.
	void onConfigurationChange(engine_configuration_s const* previousConfig) override;
	// Covers boot and layout changes that are not burned.
	void onSlowCallback() override;

	// Records a decoded tooth while the setting is on. Called for every tooth, also before the RPM is known, so that
	// the first sparks are timed from a full window.
	void onTooth(efitick_t edgeTimestamp, angle_t currentPhase);

	// Start the tooth window over; call from trigger decoding context (a trigger error or a phase re-sync).
	void resetFromTrigger();

	// Setting on, CRANKING, and a layout the charge prediction fits.
	bool isAllowed() const;
	// Spark and dwell tooth for an event whose angles are latched; not valid when no tooth is within reach.
	CrankToothPlan plan(const IgnitionEvent& event) const;
	// Is there a tooth interval to time the charge from.
	bool isChargeUsable(float currentPhase, float sparkPhase, floatms_t dwellMs) const;
	// When to start charging, given the tooth at edgeTimestamp is the dwell tooth.
	efitick_t chargeTime(efitick_t edgeTimestamp, float currentPhase, float sparkPhase, floatms_t dwellMs) const;
	// Delay from the start of charge to the release of a coil whose spark tooth has not arrived.
	floatms_t overDwellMs(floatms_t dwellMs) const;

	const ToothPeriodWindow& window() const {
		return m_window;
	}

	// The wheel and engine suit the charge prediction, as of the current layout. It is computed off the trigger path:
	// a burn recomputes it (onConfigurationChange), a layout change that is not burned is picked up by the next slow
	// callback, and meanwhile a changed layout reads as not valid, so events keep normal scheduling.
	bool isGeometryValid() const;

private:
	// what the geometry depends on, cheap enough to compare on every tooth
	uint32_t layoutKey() const;
	void updateGeometry();

	ToothPeriodWindow m_window;
	bool m_geometryValid = false;
	// the twin distance the geometry was validated with (one stroke for now), used by the charge prediction
	float m_twinDeg = 0;
	// the layout the geometry was computed for
	uint32_t m_geometryKey = 0;
	bool m_geometryKnown = false;
	bool m_settingWasOn = false;
};

#elif EFI_ENGINE_CONTROL

// No trigger input, so no teeth to lock to: the callers in spark_logic.cpp and the trigger code compile against this.
class CrankToothSpark : public EngineModule {
public:
	void resetFromTrigger() {}
	bool isAllowed() const {
		return false;
	}
	CrankToothPlan plan(const IgnitionEvent&) const {
		return {};
	}
	bool isChargeUsable(float, float, floatms_t) const {
		return false;
	}
	efitick_t chargeTime(efitick_t edgeTimestamp, float, float, floatms_t) const {
		return edgeTimestamp;
	}
	floatms_t overDwellMs(floatms_t dwellMs) const {
		return dwellMs;
	}
};

#endif // EFI_ENGINE_CONTROL && EFI_SHAFT_POSITION_INPUT

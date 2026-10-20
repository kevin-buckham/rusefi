/*
 * @file spark_logic.cpp
 *
 * @date Sep 15, 2016
 * @author Andrey Belomutskiy, (c) 2012-2020
 */

#include "pch.h"

#include "spark_logic.h"

#include "utlist.h"
#include "event_queue.h"

#include "knock_logic.h"
#include "edge_dwell.h"

#if EFI_ENGINE_CONTROL

#if EFI_UNIT_TEST
extern bool verboseMode;
#endif /* EFI_UNIT_TEST */

#if EFI_PRINTF_FUEL_DETAILS || FUEL_MATH_EXTREME_LOGGING
	extern bool printFuelDebug;
#endif // EFI_PRINTF_FUEL_DETAILS

#if !EFI_UNIT_TEST
// only consumed by the skipped-spark detection in startDwellByTurningSparkPinHigh()
static const char *prevSparkName = nullptr;
#endif // EFI_UNIT_TEST

static void fireSparkBySettingPinLow(IgnitionEvent *event, IgnitionOutputPin *output) {
#if SPARK_EXTREME_LOGGING
	efiPrintf("[%s] %d spark goes low revolution %d tick %d current value %d",
		event->getOutputForLoggins()->getName(), event->sparkCounter,
		getRevolutionCounter(),  time2print(getTimeNowUs()),
		output->currentLogicValue);
#endif /* SPARK_EXTREME_LOGGING */

	/**
	 * there are two kinds of 'out-of-order'
	 * 1) low goes before high, everything is fine afterwards
	 *
	 * 2) we have an un-matched low followed by legit pairs
	 */
	output->signalFallSparkId = event->sparkCounter;

	if (!output->currentLogicValue && !event->wasSparkLimited && !event->wasSparkCanceled) {
#if SPARK_EXTREME_LOGGING
		efiPrintf("out-of-order coil off %s", output->getName());
#endif /* SPARK_EXTREME_LOGGING */
		warning(ObdCode::CUSTOM_OUT_OF_ORDER_COIL, "out-of-order coil off %s", output->getName());
	}
	output->setLow();
}

static void assertPinAssigned(IgnitionOutputPin* output) {
	if (!output->isInitialized()) {
		warning(ObdCode::CUSTOM_OBD_COIL_PIN_NOT_ASSIGNED, "Pin Not Assigned check configuration #%s", output->getName()); \
	}
}

/**
 * @param cylinderIndex from 0 to cylinderCount, not cylinder number
 */
static int getIgnitionPinForIndex(int cylinderIndex, ignition_mode_e ignitionMode) {
	switch (ignitionMode) {
	case IM_ONE_COIL:
		return 0;
	case IM_WASTED_SPARK: {
		if (engineConfiguration->cylindersCount == 1) {
			// we do not want to divide by zero
			return 0;
		}
		return cylinderIndex % (engineConfiguration->cylindersCount / 2);
	}
	case IM_INDIVIDUAL_COILS:
		return cylinderIndex;
	case IM_TWO_COILS:
		return cylinderIndex % 2;

	default:
		firmwareError(ObdCode::CUSTOM_OBD_IGNITION_MODE, "Invalid ignition mode getIgnitionPinForIndex(): %d", engineConfiguration->ignitionMode);
		return 0;
	}
}

static void prepareCylinderIgnitionSchedule(angle_t dwellAngleDuration, floatms_t sparkDwell, IgnitionEvent *event) {
	// todo: clean up this implementation? does not look too nice as is.

	// let's save planned duration so that we can later compare it with reality
	event->sparkDwell = sparkDwell;
	event->edgeDwellCycleState = 0;

	auto ignitionMode = getCurrentIgnitionMode();

	// On an odd cylinder (or odd fire) wasted spark engine, map outputs as if in sequential.
	// During actual scheduling, the events just get scheduled every 360 deg instead
	// of every 720 deg.
	if (ignitionMode == IM_WASTED_SPARK && engine->engineState.useOddFireWastedSpark) {
		ignitionMode = IM_INDIVIDUAL_COILS;
	}

	const int index = getIgnitionPinForIndex(event->cylinderIndex, ignitionMode);
	const int coilIndex = getCylinderNumberAtIndex(index);
	angle_t finalIgnitionTiming =	getEngineState()->timingAdvance[coilIndex];
	// Stash which cylinder we're scheduling so that knock sensing knows which
	// cylinder just fired
	event->coilIndex = coilIndex;

	// 10 ATDC ends up as 710, convert it to -10 so we can log and clamp correctly
	if (finalIgnitionTiming > 360) {
		finalIgnitionTiming -= 720;
	}

	// Clamp the final ignition timing to the configured limits
	// finalIgnitionTiming is deg BTDC
	// minimumIgnitionTiming limits maximum retard
	// maximumIgnitionTiming limits maximum advance
	/*
	https://github.com/rusefi/rusefi/issues/5894 disabling feature for now
	finalIgnitionTiming = clampF(engineConfiguration->minimumIgnitionTiming, finalIgnitionTiming, engineConfiguration->maximumIgnitionTiming);
	*/

    engine->outputChannels.ignitionAdvanceCyl[event->cylinderIndex] = finalIgnitionTiming;

	angle_t sparkAngle =
		// Negate because timing *before* TDC, and we schedule *after* TDC
		- finalIgnitionTiming
		// Offset by this cylinder's position in the cycle
		+ getPerCylinderFiringOrderOffset(event->cylinderIndex, coilIndex);

	efiAssertVoid(ObdCode::CUSTOM_SPARK_ANGLE_1, !std::isnan(sparkAngle), "sparkAngle#1");
	wrapAngle(sparkAngle, "findAngle#2", ObdCode::CUSTOM_ERR_6550);
	event->sparkAngle = sparkAngle;

	engine->outputChannels.currentIgnitionMode = static_cast<uint8_t>(ignitionMode);

	IgnitionOutputPin *output = &enginePins.coils[coilIndex];
	event->outputs[0] = output;
	IgnitionOutputPin *secondOutput;

	// We need two outputs if:
	//  - we are running wasted spark, and have "two wire" mode enabled
	//  - We are running sequential mode, but we're cranking, so we should run in two wire wasted mode (not one wire wasted)
	bool isTwoWireWasted = engineConfiguration->twoWireBatchIgnition || (engineConfiguration->ignitionMode == IM_INDIVIDUAL_COILS);
	if (ignitionMode == IM_WASTED_SPARK && isTwoWireWasted) {
		int secondIndex = index + engineConfiguration->cylindersCount / 2;
		int secondCoilIndex = getCylinderNumberAtIndex(secondIndex);
		secondOutput = &enginePins.coils[secondCoilIndex];
		assertPinAssigned(secondOutput);
	} else {
		secondOutput = nullptr;
	}

	assertPinAssigned(output);

	event->outputs[1] = secondOutput;


	angle_t dwellStartAngle = sparkAngle - dwellAngleDuration;
	efiAssertVoid(ObdCode::CUSTOM_ERR_6590, !std::isnan(dwellStartAngle), "findAngle#5");

	assertAngleRange(dwellStartAngle, "findAngle dwellStartAngle", ObdCode::CUSTOM_ERR_6550);
	wrapAngle(dwellStartAngle, "findAngle#7", ObdCode::CUSTOM_ERR_6550);
	event->dwellAngle = dwellStartAngle;

#if FUEL_MATH_EXTREME_LOGGING
	if (printFuelDebug) {
		efiPrintf("addIgnitionEvent %s angle=%.1f", output->getName(), dwellStartAngle);
	}
	//	efiPrintf("addIgnitionEvent %s ind=%d", output->name, event->dwellPosition->eventIndex);
#endif /* FUEL_MATH_EXTREME_LOGGING */
}

static void chargeTrailingSpark(IgnitionOutputPin* pin) {
#if SPARK_EXTREME_LOGGING
	efiPrintf("chargeTrailingSpark %s", pin->getName());
#endif /* SPARK_EXTREME_LOGGING */
	pin->setHigh();
}

static void fireTrailingSpark(IgnitionOutputPin* pin) {
#if SPARK_EXTREME_LOGGING
	efiPrintf("fireTrailingSpark %s", pin->getName());
#endif /* SPARK_EXTREME_LOGGING */
	pin->setLow();
}

static void overFireSparkAndPrepareNextSchedule(IgnitionEvent *event) {
#if SPARK_EXTREME_LOGGING
	efiPrintf("[%s] %d %s",
		event->getOutputForLoggins()->getName(), event->sparkCounter,
		__func__);
#endif /* SPARK_EXTREME_LOGGING */
	float actualDwellMs = event->actualDwellTimer.getElapsedSeconds() * 1e3;

	warning((ObdCode)((int)ObdCode::CUSTOM_Ignition_Coil_Overcharge_1 + event->cylinderIndex),
		"cylinder %d %s overcharge %f ms",
		event->cylinderIndex + 1, event->outputs[0]->getName(), actualDwellMs);

	// kill pending fire
	engine->module<TriggerScheduler>()->cancel(&event->sparkEvent);

	engine->engineState.overDwellCanceledCounter++;
	event->wasSparkCanceled = true;
	fireSparkAndPrepareNextSchedule(event);
}

/**
 * TL,DR: each IgnitionEvent is in charge of it's own scheduling forever, we plant next event while finishing handling of the current one
 */
void fireSparkAndPrepareNextSchedule(IgnitionEvent *event) {
#if EFI_UNIT_TEST
	if (engine->onIgnitionEvent) {
		engine->onIgnitionEvent(event, false);
	}
#endif

	for (int i = 0; i< MAX_OUTPUTS_FOR_IGNITION;i++) {
		IgnitionOutputPin *output = event->outputs[i];

		if (output) {
			fireSparkBySettingPinLow(event, output);
		}
	}

	efitick_t nowNt = getTimeNowNt();

#if EFI_TOOTH_LOGGER
	LogTriggerCoilState(nowNt, event->coilIndex, false);
#endif // EFI_TOOTH_LOGGER
	if (!event->wasSparkLimited) {
		/**
		 * ratio of desired dwell duration to actual dwell duration gives us some idea of how good is input trigger jitter
		 */
		float actualDwellMs = event->actualDwellTimer.getElapsedSeconds(nowNt) * 1e3;
		float ratio = actualDwellMs / event->sparkDwell;

		if (ratio > 1.2) {
			engine->engineState.dwellOverChargeCounter++;
		} else if (ratio < 0.8) {
			engine->engineState.dwellUnderChargeCounter++;
		}
		engine->engineState.dwellActualRatio = ratio;
	}

	// now that we've just fired a coil let's prepare the new schedule for the next engine revolution

	angle_t dwellAngleDuration = engine->ignitionState.dwellDurationAngle;
	floatms_t sparkDwell = engine->ignitionState.getDwell();
	if (std::isnan(dwellAngleDuration) || std::isnan(sparkDwell)) {
		// we are here if engine has just stopped
		return;
	}

	// If there are more sparks to fire, schedule them
	if (event->sparksRemaining > 0) {
		event->sparksRemaining--;

		// each restrike is its own spark: grab a fresh id, otherwise the out-of-order
		// protection in startDwellByTurningSparkPinHigh bails restrike dwell since
		// signalFallSparkId already matches this event's sparkCounter
		event->sparkCounter = engine->engineState.globalSparkCounter++;

		efitick_t nextDwellStart = nowNt + engine->engineState.multispark.delay;
		efitick_t nextFiring = nextDwellStart + engine->engineState.multispark.dwell;
#if SPARK_EXTREME_LOGGING
		efiPrintf("schedule multispark");
#endif /* SPARK_EXTREME_LOGGING */

		// We can schedule both of these right away, since we're going for "asap" not "particular angle"
		engine->scheduler.schedule("dwell", &event->dwellStartTimer, nextDwellStart, action_s::make<turnSparkPinHighStartCharging>( event ));
		engine->scheduler.schedule("firing", &event->sparkEvent.eventScheduling, nextFiring, action_s::make<fireSparkAndPrepareNextSchedule>( event ));
	} else {
		if (engineConfiguration->enableTrailingSparks) {
#if SPARK_EXTREME_LOGGING
			efiPrintf("scheduleByAngle TrailingSparks");
#endif /* SPARK_EXTREME_LOGGING */

			// Trailing sparks are enabled - schedule an event for the corresponding trailing coil
			scheduleByAngle(
				&event->trailingSparkFire, nowNt, engine->ignitionState.trailingSparkAngle,
				action_s::make<fireTrailingSpark>( &enginePins.trailingCoils[event->coilIndex] )
			);
		}

		// If all events have been scheduled, prepare for next time.
		prepareCylinderIgnitionSchedule(dwellAngleDuration, sparkDwell, event);
	}

	engine->onSparkFireKnockSense(event->cylinderIndex, nowNt);
}

static bool startDwellByTurningSparkPinHigh(IgnitionEvent *event, IgnitionOutputPin *output) {
	// todo: no reason for this to be disabled in unit_test mode?!
#if ! EFI_UNIT_TEST

	if (Sensor::getOrZero(SensorType::Rpm) > 2 * engineConfiguration->cranking.rpm) {
		const char *outputName = output->getName();
		if (prevSparkName == outputName && getCurrentIgnitionMode() != IM_ONE_COIL) {
			warning(ObdCode::CUSTOM_OBD_SKIPPED_SPARK, "looks like skipped spark event revolution=%d [%s]", getRevolutionCounter(), outputName);
		}
		prevSparkName = outputName;
	}
#endif /* EFI_UNIT_TEST */


#if SPARK_EXTREME_LOGGING
	efiPrintf("[%s] %d spark goes high revolution %d tick %d current value %d",
		event->getOutputForLoggins()->getName(), event->sparkCounter,
		getRevolutionCounter(), time2print(getTimeNowUs()),
		output->currentLogicValue, event->sparkCounter);
#endif /* SPARK_EXTREME_LOGGING */

	// Reset error flag(s)
	event->wasSparkCanceled = false;

	if (output->signalFallSparkId >= event->sparkCounter) {
	  /**
	   * fact: we schedule both start of dwell and spark firing using a combination of time and trigger event domain
	   * in case of bad/noisy signal we can get unexpected trigger events and a small time delay for spark firing before
	   * we even start dwell if it scheduled with a longer time-only delay with fewer trigger events
	   *
	   * here we are detecting such out-of-order processing and choose the safer route of not even starting dwell
	   * [tag] #6349
	   */

#if SPARK_EXTREME_LOGGING
		efiPrintf("[%s] bail spark dwell\n", output->getName());
#endif /* SPARK_EXTREME_LOGGING */
		// let's save this coil if things do not look right
		engine->engineState.sparkOutOfOrderCounter++;
		return true;
	}

	output->setHigh();
	return false;
}

void turnSparkPinHighStartCharging(IgnitionEvent *event) {
	efitick_t nowNt = getTimeNowNt();

	event->actualDwellTimer.reset(nowNt);

  bool skippedDwellDueToTriggerNoised = false;
	for (int i = 0; i< MAX_OUTPUTS_FOR_IGNITION;i++) {
		IgnitionOutputPin *output = event->outputs[i];
		if (output != NULL) {
		  // at the moment we have a funny xor as if outputs could have different destiny. That's probably an over exaggeration,
		  // realistically it should be enough to check the sequencing of only the first output but that would be less elegant
		  //
		  // maybe it would have need nicer if instead of an array of outputs we had a linked list of outputs? but that's just daydreaming.
			skippedDwellDueToTriggerNoised |= startDwellByTurningSparkPinHigh(event, output);
		}
	}

#if EFI_UNIT_TEST
	engine->incrementBailedOnDwellCount();
#endif


  if (!skippedDwellDueToTriggerNoised) {

#if EFI_UNIT_TEST
  	if (engine->onIgnitionEvent) {
  		engine->onIgnitionEvent(event, true);
  	}
#endif

#if EFI_TOOTH_LOGGER
		LogTriggerCoilState(nowNt, event->coilIndex, true);
#endif // EFI_TOOTH_LOGGER
  }


	if (engineConfiguration->enableTrailingSparks) {
		IgnitionOutputPin *output = &enginePins.trailingCoils[event->coilIndex];
		// Trailing sparks are enabled - schedule an event for the corresponding trailing coil
		scheduleByAngle(
			&event->trailingSparkCharge, nowNt, engine->ignitionState.trailingSparkAngle,
			action_s::make<chargeTrailingSpark>( output )
		);
	}
}


/**
 * crankingEdgeDwell, tooth-locked branch: the dwell tooth and the spark tooth are exact tooth phases
 * (see edgedwell::planForEvent), so the charge start is the only thing that needs a time estimate. It comes from
 * the recent tooth intervals (ToothPeriodWindow, see edgedwell::dwellStartDelayUs for the three tiers) and falls
 * back to the stored RPM until a full 360 deg has been observed.
 */
static efitick_t scheduleEdgeDwellCharge(IgnitionEvent *event, efitick_t edgeTimestamp,
		float sparkPhase, float currentPhase, floatms_t dwellMs) {
	const ToothPeriodWindow &window = getTriggerCentral()->toothWindow;
	const float cycle = engine->engineState.engineCycle;

	float remainingDeg = edgedwell::forwardDistance(currentPhase, sparkPhase, cycle);
	float slow = window.slowUsPerDeg(360);
	// the 180-deg stroke-twin prediction is only trusted once the locked 360-deg window is full
	float predictedUs = 0;
	if (slow > 0 && engineConfiguration->cylindersCount > 0) {
		predictedUs = window.predictTimeUs(currentPhase, sparkPhase, cycle / engineConfiguration->cylindersCount, cycle, edgedwell::kToothEpsDeg);
	}
	float delayUs = edgedwell::dwellStartDelayUs(remainingDeg, dwellMs, predictedUs,
		slow, window.lastUsPerDeg(), engine->rpmCalculator.oneDegreeUs);
	if (delayUs < 0) {
		// unreachable: the plan is only accepted after chargeInputsUsable(); never turn a refusal into "charge now"
		delayUs = 0;
	}

	// the planned dwell is the one actually being used: keeps 'Ignition: Dwell deviation' honest
	event->sparkDwell = dwellMs;

	efitick_t chargeTime = sumTickAndFloat(edgeTimestamp, USF2NT(delayUs));
	engine->scheduler.schedule("dwell", &event->dwellStartTimer, chargeTime, action_s::make<turnSparkPinHighStartCharging>( event ));
	return chargeTime;
}

static void scheduleSparkEvent(bool limitedSpark, IgnitionEvent *event,
		float rpm, float dwellMs, float dwellAngle, float sparkAngle, efitick_t edgeTimestamp, float currentPhase, float nextPhase,
		bool edgeEvent) {
	UNUSED(rpm);

	float angleOffset = dwellAngle - currentPhase;
	if (angleOffset < 0) {
		angleOffset += engine->engineState.engineCycle;
	}

	// For single-tooth triggers (currentPhase == nextPhase), all dwell angles map to the
	// same trigger tooth. When the dwell angle is just below the current phase, the offset
	// wraps to nearly a full engine cycle, scheduling the dwell far in the future
	// Clamp to 0 so the spark starts immediately.
	if (currentPhase == nextPhase && angleOffset > engine->engineState.engineCycle / 2) {
		angleOffset = 0;
#if SPARK_EXTREME_LOGGING
	    efiPrintf("Clamping spark dwell to current phase due to single-tooth trigger");
#endif /* SPARK_EXTREME_LOGGING */
	}

	/**
	 * By the way 32-bit value should hold at least 400 hours of events at 6K RPM x 12 events per revolution
	 * [tag:duration_limit]
	 */
	event->sparkCounter = engine->engineState.globalSparkCounter++;
	event->wasSparkLimited = limitedSpark;
	if (engineConfiguration->crankingEdgeDwell || engineConfiguration->edgeDwellPostStartLatch) {
		// per-spark log bit: this spark was scheduled by the edge-locked path (0 = legacy path, even with edge mode on)
		engine->ignitionState.edgeDwellSparkEdge = edgeEvent;
	}

	efitick_t chargeTime = 0;

	/**
	 * The start of charge is always within the current trigger event range, so just plain time-based scheduling
	 */
	if (!limitedSpark) {
#if SPARK_EXTREME_LOGGING
		efiPrintf("[%s] %d sparkUp scheduling revolution %d angle %.1f (+%.1f) later",
			event->getOutputForLoggins()->getName(), event->sparkCounter,
			getRevolutionCounter(), dwellAngle, angleOffset);
#endif /* SPARK_EXTREME_LOGGING */

		/**
		 * Note how we do not check if spark is limited or not while scheduling 'spark down'
		 * This way we make sure that coil dwell started while spark was enabled would fire and not burn
		 * the coil.
		 */
		if (edgeEvent) {
			// angleOffset is 0 here: dwellAngle IS the current tooth
			chargeTime = scheduleEdgeDwellCharge(event, edgeTimestamp, sparkAngle, currentPhase, dwellMs);
		} else if (engineConfiguration->dwellFixOnRpmUpdate && currentPhase != nextPhase && !std::isnan(engine->rpmCalculator.oneDegreeUs)) {
			/**
			 * E4: dwellAngle was latched at the previous fire from the RPM of THAT moment, but scheduleByAngle()
			 * converts it with the CURRENT oneDegreeUs: when RPM was updated in between, the dwell in time scales
			 * by oldRpm/newRpm (logged 'Ignition: Dwell deviation' -58% at the 230->547 update).
			 * Keep the tooth selection (dwellAngle) but re-derive the delay so that the dwell is dwellMs at the
			 * current speed: start (dwellMs worth of degrees) before the spark, never before this tooth.
			 */
			float sparkAhead = angleOffset + edgedwell::forwardDistance(dwellAngle, sparkAngle, engine->engineState.engineCycle);
			float dwellDegNow = dwellMs * 1000.0f / engine->rpmCalculator.oneDegreeUs;
			angleOffset = std::max(0.0f, sparkAhead - dwellDegNow);
			event->sparkDwell = dwellMs;
			chargeTime = scheduleByAngle(&event->dwellStartTimer, edgeTimestamp, angleOffset, action_s::make<turnSparkPinHighStartCharging>( event ));
		} else {
			chargeTime = scheduleByAngle(&event->dwellStartTimer, edgeTimestamp, angleOffset, action_s::make<turnSparkPinHighStartCharging>( event ));
		}

#if EFI_UNIT_TEST
		// edge events: the charge is scheduled by time from this tooth, the offset in degrees is not used
		engine->onScheduleTurnSparkPinHighStartCharging(*event, edgeTimestamp, edgeEvent ? 0.0f : angleOffset, chargeTime);
#endif

#if SPARK_EXTREME_LOGGING
		efitimeus_t chargeTimeUs = NT2US(chargeTime);
		efiPrintf("[%s] %d sparkUp scheduled at %d ticks (%d.%06d)",
			event->getOutputForLoggins()->getName(), event->sparkCounter,
			time2print(chargeTime), time2print(chargeTimeUs / (1000 * 1000)), time2print(chargeTimeUs % (1000 * 1000)));
#endif /* SPARK_EXTREME_LOGGING */

		event->sparksRemaining = engine->engineState.multispark.count;
	} else {
		// don't fire multispark if spark is cut completely!
		event->sparksRemaining = 0;

#if SPARK_EXTREME_LOGGING
		efiPrintf("[%s] %d sparkUp NOT scheduled because of limitedSpark",
			event->getOutputForLoggins()->getName(), event->sparkCounter);
#endif /* SPARK_EXTREME_LOGGING */
	}

	/**
	 * Spark event is often happening during a later trigger event timeframe
	 */

	efiAssertVoid(ObdCode::CUSTOM_ERR_6591, !std::isnan(sparkAngle), "findAngle#4");
	assertAngleRange(sparkAngle, "findAngle#a5", ObdCode::CUSTOM_ERR_6549);

#if SPARK_EXTREME_LOGGING
	efiPrintf("[%s] %d sparkDown scheduling revolution %d angle %.1f",
		event->getOutputForLoggins()->getName(), event->sparkCounter,
		getRevolutionCounter(), sparkAngle);
#endif /* FUEL_MATH_EXTREME_LOGGING */


	bool isTimeScheduled = engine->module<TriggerScheduler>()->scheduleOrQueue(
		"spark",
		&event->sparkEvent, edgeTimestamp, sparkAngle,
		action_s::make<fireSparkAndPrepareNextSchedule>( event ),
		currentPhase, nextPhase);

#if SPARK_EXTREME_LOGGING
	efiPrintf("[%s] %d sparkDown scheduled %s",
		event->getOutputForLoggins()->getName(), event->sparkCounter,
		isTimeScheduled ? "later" : "to queue");
#endif /* FUEL_MATH_EXTREME_LOGGING */

	if (isTimeScheduled) {
		// event was scheduled by time, we expect it to happen reliably
	} else {
		// event was queued in relation to some expected tooth event in the future which might just never come so we shall protect from over-dwell
		if (!limitedSpark) {
			// auto fire spark at 1.5x nominal dwell (3x in tooth-locked cranking mode: an over-dwell fire there is an EARLY spark)
			float overdwellMs = edgeEvent
				? std::max(edgedwell::kEdgeOverdwellFactor * dwellMs, edgedwell::kEdgeBackupMinMs)
				: edgedwell::kLegacyOverdwellFactor * dwellMs;
			efitick_t fireTime = sumTickAndFloat(chargeTime, MSF2NT(overdwellMs));

#if SPARK_EXTREME_LOGGING
			efitimeus_t fireTimeUs = NT2US(fireTime);
			efiPrintf("[%s] %d overdwell scheduling at %d ticks (%d.%06d)",
				event->getOutputForLoggins()->getName(), event->sparkCounter,
				time2print(fireTime), time2print(fireTimeUs / (1000 * 1000)), time2print(fireTimeUs % (1000 * 1000)));
#endif /* SPARK_EXTREME_LOGGING */

			/**
			* todo: can we please comprehend/document how this even works? we seem to be reusing 'sparkEvent.scheduling' instance
			* and it looks like current (smart?) re-queuing is effectively cancelling out the overdwell? is that the way this was intended to work?
			* [tag:overdwell]
			*/
			engine->scheduler.schedule("overdwell", &event->sparkEvent.eventScheduling, fireTime, action_s::make<overFireSparkAndPrepareNextSchedule>( event ));

#if EFI_UNIT_TEST
			engine->onScheduleOverFireSparkAndPrepareNextSchedule(*event, fireTime);
#endif
		} else {
		  engine->engineState.overDwellNotScheduledCounter++;
		}
	}

#if EFI_UNIT_TEST
	if (verboseMode) {
		efiPrintf("spark dwell@ %.1f spark@ %.2f id=%d sparkCounter=%d", event->dwellAngle,
			event->sparkEvent.getAngle(),
			event->coilIndex,
			event->sparkCounter);
	}
#endif
}

void initializeIgnitionActions() {
	IgnitionEventList *list = &engine->ignitionEvents;
	angle_t dwellAngle = engine->ignitionState.dwellDurationAngle;
	floatms_t sparkDwell = engine->ignitionState.getDwell();
	if (std::isnan(engine->engineState.timingAdvance[0]) || std::isnan(dwellAngle)) {
		// error should already be reported
		// need to invalidate previous ignition schedule
		list->isReady = false;
		return;
	}
	efiAssertVoid(ObdCode::CUSTOM_ERR_6592, engineConfiguration->cylindersCount > 0, "cylindersCount");

	for (size_t cylinderIndex = 0; cylinderIndex < engineConfiguration->cylindersCount; cylinderIndex++) {
		list->elements[cylinderIndex].cylinderIndex = cylinderIndex;
		prepareCylinderIgnitionSchedule(dwellAngle, sparkDwell, &list->elements[cylinderIndex]);
	}
	list->isReady = true;
}

static void prepareIgnitionSchedule() {
	ScopePerf perf(PE::PrepareIgnitionSchedule);

	operation_mode_e operationMode = getEngineRotationState()->getOperationMode();
	float maxAllowedDwellAngle;

	if (getCurrentIgnitionMode() == IM_ONE_COIL) {
		maxAllowedDwellAngle = getEngineCycle(operationMode) / engineConfiguration->cylindersCount / 1.1;
	} else {
	  maxAllowedDwellAngle = (int) (getEngineCycle(operationMode) / 2); // the cast is about making Coverity happy
	}

	if (engine->ignitionState.dwellDurationAngle == 0) {
		warning(ObdCode::CUSTOM_ZERO_DWELL, "dwell is zero?");
	}
	if (engine->ignitionState.dwellDurationAngle > maxAllowedDwellAngle) {
		warning(ObdCode::CUSTOM_DWELL_TOO_LONG, "dwell angle too long: %.2f", engine->ignitionState.dwellDurationAngle);
	}

	// todo: add some check for dwell overflow? like 4 times 6 ms while engine cycle is less then that

	initializeIgnitionActions();
}

void onTriggerEventSparkLogic(float rpm, efitick_t edgeTimestamp, float currentPhase, float nextPhase) {
	ScopePerf perf(PE::OnTriggerEventSparkLogic);

	if (!engineConfiguration->isIgnitionEnabled) {
		return;
	}

	LimpState limitedSparkState = getLimpManager()->allowIgnition();

	// todo: eliminate state copy logic by giving limpManager it's owm limp_manager.txt and leveraging LiveData
	engine->outputChannels.sparkCutReason = (int8_t)limitedSparkState.reason;
	bool limitedSpark = !limitedSparkState.value;

	const floatms_t dwellMs = engine->ignitionState.getDwell();
	if (std::isnan(dwellMs) || dwellMs <= 0) {
		warning(ObdCode::CUSTOM_DWELL, "invalid dwell to handle: %.2f", dwellMs);
		return;
	}

	if (!engine->ignitionEvents.isReady) {
		prepareIgnitionSchedule();
	}


	/**
	 * Ignition schedule is defined once per revolution
	 * See initializeIgnitionActions()
	 */


	// Only apply odd cylinder count wasted logic if:
	// - odd cyl count
	// - current mode is wasted spark
	// - four stroke
	bool enableOddCylinderWastedSpark =
		engine->engineState.useOddFireWastedSpark
		&& getCurrentIgnitionMode() == IM_WASTED_SPARK;

	// Cranking edge-dwell (default off, three independent bits):
	//  crankingEdgeDwell        - tooth-locked spark + fresh-speed charge start, only in the CRANKING state (isCranking() also
	//                             covers SPINNING_UP: edge mode stays out of it) and per event only when the commanded
	//                             advance is >= edgedwell::kEdgeMinAdvanceDeg (12 BTDC); other events keep the legacy path;
	//  edgeDwellPostStartLatch  - the same edge path in RUNNING, only during the first kPostStartWindowEvents cylinder events
	//                             after the switch to RUNNING and only after the per-cylinder period jumped by 18.75 percent or
	//                             more below 1400 rpm (E931 style detector, see ToothPeriodWindow::instabilityLatched);
	//  dwellFixOnRpmUpdate      - E4, legacy path only (see scheduleSparkEvent).
	// Odd-fire wasted spark and wheels that deliver falling edges are not covered: they keep today's behaviour.
	// With both of the first two bits off nothing below is evaluated or written (byte-for-byte today's behaviour).
	const bool edgeBitsSet = engineConfiguration->crankingEdgeDwell || engineConfiguration->edgeDwellPostStartLatch;
	const float engineCycleDeg = engine->engineState.engineCycle;
	bool edgeModeAllowed = false;
	if (edgeBitsSet) {
		const spinning_state_e rpmState = engine->rpmCalculator.getState();
		const bool crankingEdgeNow = engineConfiguration->crankingEdgeDwell && rpmState == CRANKING;
		const float postStartWindowDeg = engineConfiguration->cylindersCount > 0
			? edgedwell::kPostStartWindowEvents * engineCycleDeg / engineConfiguration->cylindersCount : 0;
		const bool unstableLatched = engineConfiguration->edgeDwellPostStartLatch && rpmState == RUNNING
			&& getTriggerCentral()->toothWindow.postStartActive(postStartWindowDeg)
			&& getTriggerCentral()->toothWindow.instabilityLatched(engineCycleDeg);
		edgeModeAllowed = (crankingEdgeNow || unstableLatched)
			&& !enableOddCylinderWastedSpark
			&& getTriggerCentral()->triggerShape.useOnlyRisingEdges;
		engine->ignitionState.edgeDwellActive = false;
		engine->ignitionState.edgeDwellUnstable = unstableLatched;
		engine->ignitionState.edgeDwellWindowValid = edgeModeAllowed && getTriggerCentral()->toothWindow.slowUsPerDeg(360) > 0;
	} else if (engine->ignitionState.edgeDwellActive || engine->ignitionState.edgeDwellUnstable || engine->ignitionState.edgeDwellWindowValid || engine->ignitionState.edgeDwellSparkEdge) {
		// a bit was switched off while the live bits were up: lower them (no behavioural effect)
		engine->ignitionState.edgeDwellActive = false;
		engine->ignitionState.edgeDwellUnstable = false;
		engine->ignitionState.edgeDwellWindowValid = false;
		engine->ignitionState.edgeDwellSparkEdge = false;
	}

	if (engine->ignitionEvents.isReady) {
		for (size_t i = 0; i < engineConfiguration->cylindersCount; i++) {
			IgnitionEvent *event = &engine->ignitionEvents.elements[i];

			angle_t dwellAngle = event->dwellAngle;

			angle_t sparkAngle = event->sparkAngle;
			if (std::isnan(sparkAngle)) {
				warning(ObdCode::CUSTOM_ADVANCE_SPARK, "NaN advance");
				continue;
			}

			bool edgeEvent = false;
			const angle_t legacyDwellAngle = dwellAngle;
			// the decision is per event per cycle (see IgnitionEvent::edgeDwellCycleState): an event already scheduled by the
			// legacy path is never rescheduled by the edge path (double charge), and an event whose edge dwell tooth is
			// still ahead is completed on the edge path even if the mode gate dropped in between (skipped spark)
			const bool edgePending = event->edgeDwellCycleState == 2;
			if ((edgeModeAllowed || edgePending) && event->edgeDwellCycleState != 1) {
				const float cycle = engineCycleDeg;
				const float tdcPhase = getPerCylinderFiringOrderOffset(event->cylinderIndex, event->coilIndex);
				// a command below kEdgeMinAdvanceDeg BTDC keeps the legacy path for this event (never snapped to 18 ATDC)
				if (edgedwell::edgeEligible(sparkAngle, tdcPhase, cycle)) {
					// tooth choice depends only on the latched event angles + the fixed tooth table, so it cannot
					// change between two teeth of the same cycle (no double / missed dwell schedule)
					edgedwell::Plan plan = edgedwell::planForEvent(
						[](size_t index) { return getTriggerCentral()->getToothEnginePhase(index); },
						getTriggerCentral()->engineCycleEventCount, cycle, sparkAngle,
						edgedwell::forwardDistance(dwellAngle, sparkAngle, cycle));
					// NaN/unusable speed or dwell inputs would turn into "charge now" (early backup spark): stay on the legacy path
					const TriggerCentral* tc = getTriggerCentral();
					if (plan.valid && !edgedwell::chargeInputsUsable(
							edgedwell::forwardDistance(currentPhase, plan.sparkPhase, cycle), dwellMs,
							tc->toothWindow.slowUsPerDeg(360), tc->toothWindow.lastUsPerDeg(), engine->rpmCalculator.oneDegreeUs)) {
						plan.valid = false;
					}
					if (plan.valid) {
						dwellAngle = plan.dwellPhase;
						sparkAngle = plan.sparkPhase;
						edgeEvent = true;
					}
				}
			}

			bool isOddCylWastedEvent = false;
			if (enableOddCylinderWastedSpark) {
				auto dwellAngleWastedEvent = dwellAngle + 360;
				if (dwellAngleWastedEvent > 720) {
					dwellAngleWastedEvent -= 720;
				}

				// Check whether this event hits 360 degrees out from now (ie, wasted spark),
				// and if so, twiddle the dwell and spark angles so it happens now instead
				isOddCylWastedEvent = isPhaseInRange(dwellAngleWastedEvent, currentPhase, nextPhase);

				if (isOddCylWastedEvent) {
					dwellAngle = dwellAngleWastedEvent;

					sparkAngle += 360;
					if (sparkAngle > 720) {
						sparkAngle -= 720;
					}
				}
			}

			if (!isOddCylWastedEvent && !isPhaseInRange(dwellAngle, currentPhase, nextPhase)) {
				if (edgeEvent && isPhaseInRange(legacyDwellAngle, currentPhase, nextPhase)) {
					// the legacy dwell tooth: the charge belongs to the later edge dwell tooth
					event->edgeDwellCycleState = 2;
				}
				continue;
			}
			if (edgeEvent && getTriggerCentral()->toothWindow.lastUsPerDeg() <= 0) {
				// No fresh tooth interval since the last window reset (sync loss / trigger error / phase re-sync / stop): the
				// charge would fall to the stale 720-deg stored RPM (tier 3), which is up to ~3x too fast right after a crank
				// slowdown, i.e. an EARLY spark. Refuse the charge for this event: nothing is scheduled (no charge, no spark =
				// one lost spark, the safe direction), and the pending state is cleared so nothing is stranded. The legacy path
				// is NOT used instead: it times by the same stale RPM and fires early the same way. The next cycle plans normally
				// because the window has an interval again by then.
				event->edgeDwellCycleState = 0;
				continue;
			}
			if (edgePending) {
				event->edgeDwellCycleState = 0;
			}

			if (i == 0 && engineConfiguration->artificialTestMisfire && (getRevolutionCounter() % ((int)engineConfiguration->scriptSetting[5]) == 0)) {
				// artificial misfire on cylinder #1 for testing purposes
				// enable artificialMisfire
				warning(ObdCode::CUSTOM_ARTIFICIAL_MISFIRE, "artificial misfire on cylinder #1 for testing purposes %d", engine->engineState.globalSparkCounter);
				continue;
			}
#if ROTATIONAL_IDLE_CONTROLLER
		if (engine->rotationalIdleController.shouldSkipSparkRotationalIdle()) {
			continue;
		}
#endif // ROTATIONAL_IDLE_CONTROLLER

#if EFI_LAUNCH_CONTROL
            bool sparkLimited = engine->softSparkLimiter.shouldSkip() || engine->hardSparkLimiter.shouldSkip();
            engine->ignitionState.luaIgnitionSkip = sparkLimited;
			if (sparkLimited) {
				continue;
			}
#endif // EFI_LAUNCH_CONTROL

#if EFI_ANTILAG_SYSTEM && EFI_LAUNCH_CONTROL
/*
       if (engine->antilagController.isAntilagCondition) {
			if (engine->ALSsoftSparkLimiter.shouldSkip()) {
				continue;
			}
		}
		float throttleIntent = Sensor::getOrZero(SensorType::DriverThrottleIntent);
		engine->antilagController.timingALSSkip = interpolate3d(
			config->ALSIgnSkipTable,
			config->alsIgnSkipLoadBins, throttleIntent,
			config->alsIgnSkiprpmBins, rpm
		);

			auto ALSSkipRatio = engine->antilagController.timingALSSkip;
            engine->ALSsoftSparkLimiter.setTargetSkipRatio(ALSSkipRatio/100);
*/
#endif // EFI_ANTILAG_SYSTEM

			if (edgeEvent) {
				// means "scheduled on the edge path" for this tooth (a skipped/limited event does not raise it)
				engine->ignitionState.edgeDwellActive = true;
			} else {
				event->edgeDwellCycleState = 1;
			}
			scheduleSparkEvent(limitedSpark, event, rpm, dwellMs, dwellAngle, sparkAngle, edgeTimestamp, currentPhase, nextPhase, edgeEvent);
		}
	}
}

/**
 * Number of sparks per physical coil
 * @see getNumberOfInjections
 */
int getNumberOfSparks(ignition_mode_e mode) {
	switch (mode) {
	case IM_ONE_COIL:
		return engineConfiguration->cylindersCount;
	case IM_TWO_COILS:
		return engineConfiguration->cylindersCount / 2;
	case IM_INDIVIDUAL_COILS:
		return 1;
	case IM_WASTED_SPARK:
		return 2;
	default:
		firmwareError(ObdCode::CUSTOM_ERR_IGNITION_MODE, "Unexpected ignition_mode_e %d", mode);
		return 1;
	}
}

/**
 * @see getInjectorDutyCycle
 */
percent_t getCoilDutyCycle(float rpm) {
	floatms_t totalPerCycle = engine->ignitionState.getDwell() * getNumberOfSparks(getCurrentIgnitionMode());
	floatms_t engineCycleDuration = getCrankshaftRevolutionTimeMs(rpm) * (getEngineRotationState()->getOperationMode() == TWO_STROKE ? 1 : 2);
	return 100 * totalPerCycle / engineCycleDuration;
}

#endif // EFI_ENGINE_CONTROL

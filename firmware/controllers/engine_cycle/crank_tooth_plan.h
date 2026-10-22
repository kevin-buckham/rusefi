/**
 * @file crank_tooth_plan.h
 *
 * The spark and dwell teeth chosen for one ignition event by crankingToothLockedSpark.
 */
#pragma once

struct CrankToothPlan {
	bool valid = false;
	// tooth phases exactly as mainTriggerCallback receives them
	float sparkPhase = 0;
	float dwellPhase = 0;
};

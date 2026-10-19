/**
 * @file edge_dwell_harness.h
 *
 * Shared harness for the cranking edge-dwell tests: drives the 12-tooth crank trigger with a per-tooth speed
 * profile that has a real compression slowdown, records every coil edge and every tooth, and reports for each
 * spark (a) the angle it was actually delivered at relative to the commanded angle and (b) the real dwell.
 *
 * Physical model: crank speed = nominal * (1 - ripple * cos(2*pi*(angle - tdcShift)/180)) sampled at the
 * midpoint of each 30-degree tooth interval, so the slowest interval is the one containing compression TDC.
 * The ECU's own tooth phases (TriggerCentral::currentEngineDecodedPhase) are used to place the profile, so the
 * ripple is anchored to the engine phase the ECU believes in; tdcShiftDeg models a wrong phase guess.
 */
#pragma once

#include "pch.h"

#include <cmath>
#include <cstdint>
#include <functional>
#include <vector>

namespace edgetest {

constexpr double kPi = 3.14159265358979323846; // M_PI is not defined by strict -std=c++20 on every host

struct Tooth {
	efitick_t nt;
	float phase;
};

struct Coil {
	int cyl;
	bool charge; // true = dwell start (coil goes high), false = spark (coil goes low)
	efitick_t nt;
	float commandedAngle;
	float plannedDwellMs;
	uint32_t counter;
};

struct Trace {
	std::vector<Tooth> teeth;
	std::vector<Coil> coils;
	int sparkOutOfOrder = 0;
	int finalState = 0;
};

struct Spark {
	int cyl;
	double tUs;
	double errDeg;   // delivered minus commanded: positive = later (retarded), negative = EARLIER than commanded
	double dwellMs;  // -1 if the coil was never charged for this spark
	double plannedMs;
};

struct Profile {
	std::vector<double> rpm;   // nominal rpm of the interval ending at each tooth
	float tdcShiftDeg = 0;
	float ripple = 0.3f;       // compression ripple amplitude (fraction of nominal speed): THE single sweep parameter
	float impulse = 0;         // per-stroke combustion-impulse amplitude (fraction of nominal speed), see impulseMode
	int impulseMode = 0;       // 0 none, 1 alternating (stroke fires, next stroke does not, ...), 2 random per stroke
	float noise = 0;           // +- fraction of random interval jitter (deterministic LCG)
	uint32_t seed = 1;
	int missedTooth = -1;      // index in rpm[]: the clock advances but no tooth is delivered
	int noiseToothBefore = -1; // index in rpm[]: an extra pulse arrives 25% of an interval early
	std::function<void(size_t)> beforeTooth; // called before interval n is played
	std::function<void(size_t)> afterTooth;  // called after interval n has been played
};

inline double jitter(uint32_t& seed) {
	seed = seed * 1664525u + 1013904223u;
	return ((seed >> 8) & 0xFFFF) / 65535.0 * 2 - 1;
}

/** 12 tooth crank, 4 cyl, single coil, cranking dwell 4.5 ms (the tune's value since 2026-10-17), fixed 14 deg advance, offset 162: our car's geometry */
inline void configureCrank(EngineTestHelper& eth) {
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

/** deterministic per-stroke combustion gain: +-impulse around 1 (alternating or hashed-random per stroke index) */
inline double strokeGain(const Profile& p, long strokeIdx) {
	if (p.impulseMode == 0) {
		return 1.0;
	}
	if (p.impulseMode == 1) {
		return 1.0 + p.impulse * ((strokeIdx & 1) ? 1.0 : -1.0);
	}
	uint32_t h = (uint32_t)(strokeIdx * 2654435761u) ^ (p.seed * 40503u);
	h = h * 1664525u + 1013904223u;
	h = h * 1664525u + 1013904223u;
	return 1.0 + p.impulse * (((h >> 8) & 0xFFFF) / 65535.0 * 2 - 1);
}

inline void fireTooth(EngineTestHelper& eth, Trace& t) {
	eth.firePrimaryTriggerRise();
	t.teeth.push_back({getTimeNowNt(), engine->triggerCentral.currentEngineDecodedPhase});
}

inline void drive(EngineTestHelper& eth, Trace& t, const Profile& p) {
	engine->onIgnitionEvent = [&t](IgnitionEvent* e, bool state) {
		t.coils.push_back({e->cylinderIndex, state, getTimeNowNt(), e->sparkAngle, (float)e->sparkDwell, e->sparkCounter});
	};
	uint32_t seed = p.seed;
	float phase = 0;
	long cumDeg = 0; // degrees played so far, only used to number the strokes for the impulse model
	for (size_t n = 0; n < p.rpm.size(); n++) {
		if (p.beforeTooth) {
			p.beforeTooth(n);
		}
		float nextPhase = t.teeth.empty() ? 0.0f : std::fmod(phase + 30.0f, 720.0f);
		double mid = nextPhase - 15.0 - p.tdcShiftDeg;
		double mult = 1.0 - p.ripple * std::cos(2 * kPi * mid / 180.0);
		if (p.impulseMode != 0) {
			mult *= strokeGain(p, cumDeg / 180);
		}
		cumDeg += 30;
		double dtUs = 5.0 / (p.rpm[n] * mult) * 1e6 * (1.0 + p.noise * jitter(seed));
		if ((int)n == p.noiseToothBefore) {
			eth.moveTimeForwardAndInvokeEventsUs((int)std::lround(dtUs * 0.75));
			fireTooth(eth, t);
			eth.moveTimeForwardAndInvokeEventsUs((int)std::lround(dtUs * 0.25));
			fireTooth(eth, t);
			phase = t.teeth.back().phase;
			if (p.afterTooth) {
				p.afterTooth(n);
			}
			continue;
		}
		eth.moveTimeForwardAndInvokeEventsUs((int)std::lround(dtUs));
		if ((int)n == p.missedTooth) {
			phase = nextPhase;
			if (p.afterTooth) {
				p.afterTooth(n);
			}
			continue;
		}
		fireTooth(eth, t);
		phase = t.teeth.back().phase;
		if (p.afterTooth) {
			p.afterTooth(n);
		}
	}
	t.sparkOutOfOrder = engine->engineState.sparkOutOfOrderCounter;
	t.finalState = (int)engine->rpmCalculator.getState();
}

/** delivered-vs-commanded angle and actual dwell for every spark, using the ECU's own tooth phases */
inline std::vector<Spark> analyze(const Trace& r) {
	std::vector<double> unwrapped(r.teeth.size());
	for (size_t k = 0; k < r.teeth.size(); k++) {
		if (k == 0) {
			unwrapped[k] = r.teeth[k].phase;
			continue;
		}
		double d = std::fmod(r.teeth[k].phase - r.teeth[k - 1].phase + 1440.0, 720.0);
		unwrapped[k] = unwrapped[k - 1] + d;
	}
	std::vector<Spark> out;
	for (size_t i = 0; i < r.coils.size(); i++) {
		const Coil& lo = r.coils[i];
		if (lo.charge) {
			continue;
		}
		double dwell = -1;
		for (size_t j = 0; j < i; j++) {
			if (r.coils[j].charge && r.coils[j].counter == lo.counter) {
				dwell = (NT2US((double)lo.nt) - NT2US((double)r.coils[j].nt)) / 1000.0;
			}
		}
		double t = NT2US((double)lo.nt);
		if (r.teeth.size() < 3 || t < NT2US((double)r.teeth[0].nt)) {
			continue;
		}
		size_t k = 0;
		while (k + 2 < r.teeth.size() && NT2US((double)r.teeth[k + 1].nt) <= t) {
			k++;
		}
		double t0 = NT2US((double)r.teeth[k].nt), t1 = NT2US((double)r.teeth[k + 1].nt);
		double pos = unwrapped[k] + (unwrapped[k + 1] - unwrapped[k]) * (t - t0) / (t1 - t0);
		double err = std::fmod(pos - lo.commandedAngle, 720.0);
		if (err > 360) {
			err -= 720;
		}
		if (err <= -360) {
			err += 720;
		}
		out.push_back({lo.cyl, t, err, dwell, lo.plannedDwellMs});
	}
	return out;
}

/** FNV-1a over every coil edge (cylinder, edge, microsecond time): the flag-off identity fingerprint */
inline uint64_t coilFingerprint(const Trace& t) {
	uint64_t h = 1469598103934665603ull;
	auto mix = [&h](uint64_t v) {
		for (int i = 0; i < 8; i++) {
			h ^= (v >> (8 * i)) & 0xFF;
			h *= 1099511628211ull;
		}
	};
	for (const Coil& c : t.coils) {
		mix((uint64_t)c.cyl);
		mix(c.charge ? 1 : 0);
		mix((uint64_t)std::llround((double)NT2US(c.nt)));
	}
	mix(t.coils.size());
	return h;
}

/** Scenario library shared by the pin test (flag absent) and the edge-dwell tests */
inline Profile steady(double rpm, int teeth) {
	Profile p;
	p.rpm.assign(teeth, rpm);
	return p;
}

/** steady cranking, then a 3-tooth flash to 800 that holds */
inline Profile flash(double crankRpm = 210) {
	Profile p = steady(crankRpm, 60);
	for (double r : {400.0, 600.0, 800.0}) {
		p.rpm.push_back(r);
	}
	for (int i = 0; i < 40; i++) {
		p.rpm.push_back(800);
	}
	return p;
}

/** flash to 800 for 9 teeth then die back to 200 within the same 720 deg (stored rpm stays below 500) */
inline Profile dieBack() {
	Profile p = steady(210, 60);
	for (double r : {400.0, 600.0, 800.0, 800.0, 800.0, 800.0, 800.0, 800.0, 800.0}) {
		p.rpm.push_back(r);
	}
	for (double r : {650.0, 500.0, 400.0, 320.0, 260.0, 220.0}) {
		p.rpm.push_back(r);
	}
	for (int i = 0; i < 40; i++) {
		p.rpm.push_back(200);
	}
	return p;
}

} // namespace edgetest

/**
 * @file tooth_period_window.h
 *
 * Recent trigger tooth intervals in microseconds per crank degree, for crankingToothLockedSpark only.
 */
#pragma once

#include <stddef.h>
#include <cmath>

#include "rusefi/rusefi_math.h"
#include "rusefi/rusefi_time_math.h"
#include "efitime.h"

// degrees from 'from' forward to 'to', in [0, cycleDeg)
inline float phaseForwardDistance(float from, float to, float cycleDeg) {
	float d = to - from;
	// the callers are within one cycle, so the divide is only for an out-of-range distance
	if (d >= cycleDeg || d <= -cycleDeg) {
		d = fmodf(d, cycleDeg);
	}
	if (d < 0) {
		d += cycleDeg;
	}
	// a tiny negative remainder can round up to the whole cycle
	return d >= cycleDeg ? 0 : d;
}

class ToothPeriodWindow {
public:
	// float noise between spark-angle and tooth-angle arithmetic
	static constexpr float toothEpsDeg = 0.05f;

	// most teeth the twin of an interval may be away: a revolution on a 60-2 wheel
	static constexpr size_t maxTeethPerTwin = 60;

	void reset() {
		m_count = 0;
		m_head = 0;
		m_hasPrev = false;
	}

	void onTooth(efitick_t nowNt, float phase, float cycleDeg) {
		if (m_hasPrev) {
			float span = phaseForwardDistance(m_prevPhase, phase, cycleDeg);
			float dtUs = NT2USF(nowNt - m_prevNt);
			// a repeated phase or a non-advancing clock carries no speed information
			if (span > 0.001f && dtUs > 0) {
				m_us[m_head] = dtUs / span;
				m_span[m_head] = span;
				m_endPhase[m_head] = phase;
				m_head = (m_head + 1) % capacity;
				if (m_count < capacity) {
					m_count++;
				}
			}
		}
		m_prevNt = nowNt;
		m_prevPhase = phase;
		m_hasPrev = true;
	}

	// us/deg of the newest interval, 0 if there is none
	float lastUsPerDeg() const {
		return m_count == 0 ? 0 : m_us[newest(0)];
	}

	// largest us/deg over the newest 360 degrees, 0 until that much has been recorded
	float slowUsPerDeg() const {
		float acc = 0;
		float worst = 0;
		for (size_t i = 0; i < m_count; i++) {
			size_t idx = newest(i);
			if (m_us[idx] > worst) {
				worst = m_us[idx];
			}
			acc += m_span[idx];
			if (acc >= 360) {
				return worst;
			}
		}
		return 0;
	}

	/**
	 * Time to rotate from fromPhase to toPhase, replaying the intervals recorded one stroke earlier and scaling
	 * them by how much the newest interval differs from its own twin. Returns 0 when an interval is missing.
	 */
	float predictTimeUs(float fromPhase, float toPhase, float strokeDeg, float cycleDeg) const {
		if (m_count == 0 || !(strokeDeg > 0)) {
			return 0;
		}
		float wanted = phaseForwardDistance(fromPhase, toPhase, cycleDeg);
		if (wanted <= toothEpsDeg) {
			return 0;
		}
		// only the newest entries spanning one cycle can hold a twin
		size_t scan = 0;
		for (float acc = 0; scan < m_count && acc < cycleDeg; scan++) {
			acc += m_span[newest(scan)];
		}
		if (scan < m_count) {
			scan++;
		}
		float covered = 0;
		float sumUs = 0;
		float cursor = fromPhase;
		size_t guard = 0;
		while (covered < wanted - toothEpsDeg && guard++ < scan) {
			float twinStart = cursor - strokeDeg;
			size_t twin = capacity;
			for (size_t i = 0; i < scan; i++) {
				size_t idx = newest(i);
				if (phaseNear(m_endPhase[idx] - m_span[idx], twinStart, cycleDeg)) {
					twin = idx;
					break;
				}
			}
			if (twin == capacity) {
				return 0;
			}
			sumUs += m_us[twin] * m_span[twin];
			covered += m_span[twin];
			cursor += m_span[twin];
		}
		if (covered < wanted - toothEpsDeg || covered > wanted + toothEpsDeg) {
			return 0;
		}
		float trend = 1.0f;
		float last = lastUsPerDeg();
		float lastTwin = usPerDegEndingAt(m_prevPhase - strokeDeg, cycleDeg);
		if (last > 0 && lastTwin > 0) {
			trend = clampF(0.25f, last / lastTwin, 4.0f);
		}
		if (trend > 1.0f) {
			// slowing: the slowdown seen over the last stroke continues over the span being predicted
			trend *= powf(trend, 0.5f * (m_span[newest(0)] + wanted) / strokeDeg);
		}
		return sumUs * trend;
	}

private:
	// the twin interval and the span being predicted must both be in the window
	static constexpr size_t capacity = maxTeethPerTwin + 4;
	static_assert(maxTeethPerTwin >= 58 && capacity > maxTeethPerTwin, "a revolution twin on a 60-2 wheel must fit");

	size_t newest(size_t i) const {
		return (m_head + capacity - 1 - i) % capacity;
	}

	static bool phaseNear(float a, float b, float cycleDeg) {
		float d = phaseForwardDistance(a, b, cycleDeg);
		return d <= toothEpsDeg || d >= cycleDeg - toothEpsDeg;
	}

	// us/deg of the newest interval that ended at endPhase, 0 if none
	float usPerDegEndingAt(float endPhase, float cycleDeg) const {
		for (size_t i = 0; i < m_count; i++) {
			size_t idx = newest(i);
			if (phaseNear(m_endPhase[idx], endPhase, cycleDeg)) {
				return m_us[idx];
			}
		}
		return 0;
	}

	float m_us[capacity];
	float m_span[capacity];
	float m_endPhase[capacity];
	size_t m_count = 0;
	size_t m_head = 0;
	bool m_hasPrev = false;
	efitick_t m_prevNt = 0;
	float m_prevPhase = 0;
};

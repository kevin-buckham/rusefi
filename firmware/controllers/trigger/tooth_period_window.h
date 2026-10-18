/**
 * @file tooth_period_window.h
 *
 * Rolling record of the most recent trigger tooth intervals, expressed as microseconds per
 * crankshaft degree. Used ONLY by the cranking edge-dwell scheduler (engineConfiguration->crankingEdgeDwell):
 * it never feeds RPM, the cranking/running state machine, fuel or any table lookup.
 *
 * Second job (Mitsubishi E931 style instability detector, see evaluateInstability()): compare the time of the newest
 * 180 deg (one cylinder event on a four cylinder) with the one before it and latch for one cycle when it jumped.
 *
 * The estimate is deliberately SLOW-BIASED: slowUsPerDeg() is the largest us/deg seen over the most recent
 * windowDeg degrees of rotation (360 deg = two compression strokes on a four cylinder), so it contains the
 * previous compression minimum.
 */
#pragma once

#include <stddef.h>
#include <math.h>

class ToothPeriodWindow {
public:
	// 64 intervals covers 360 deg on every wheel up to 60-2 (58 teeth per revolution)
	static constexpr size_t kCapacity = 64;

	void reset() {
		m_count = 0;
		m_head = 0;
		m_hasPrev = false;
		m_added = 0;
		m_evaluated = 0;
		m_unstableSeen = false;
		m_degSinceUnstable = 0;
		m_postStartArmed = false;
		m_degSincePostStart = 0;
	}

	/**
	 * Arm the post-start window (called on the switch to RUNNING, only when edgeDwellPostStartLatch is set): the
	 * instability detector acts only until postStartWindowDeg of rotation have passed since this call.
	 */
	void markPostStart() {
		m_postStartArmed = true;
		m_degSincePostStart = 0;
		m_unstableSeen = false;
	}

	bool postStartActive(float windowDeg) const {
		return m_postStartArmed && m_degSincePostStart < windowDeg;
	}

	/**
	 * @param nowNt tooth timestamp in ticks (only differences are used)
	 * @param phase engine phase of this tooth in degrees [0, cycleDeg)
	 */
	void onTooth(efitick_t nowNt, float phase, float cycleDeg) {
		if (m_hasPrev) {
			float span = forwardDistance(m_prevPhase, phase, cycleDeg);
			float dtUs = NT2USF(nowNt - m_prevNt);
			// a repeated phase (zero span) or a non-advancing clock carries no speed information
			if (span > 0.001f && dtUs > 0) {
				m_us[m_head] = dtUs / span;
				m_span[m_head] = span;
				m_endPhase[m_head] = phase;
				m_head = (m_head + 1) % kCapacity;
				if (m_count < kCapacity) {
					m_count++;
				}
				m_added++;
				if (m_postStartArmed && m_degSincePostStart < 100000.0f) {
					m_degSincePostStart += span;
				}
			}
		}
		m_prevNt = nowNt;
		m_prevPhase = phase;
		m_hasPrev = true;
	}

	/** most recent interval, us/deg; 0 if no interval since reset */
	float lastUsPerDeg() const {
		if (m_count == 0) {
			return 0;
		}
		return m_us[(m_head + kCapacity - 1) % kCapacity];
	}

	/**
	 * Largest us/deg over the newest intervals that together span at least windowDeg.
	 * @return 0 when fewer than windowDeg degrees have been observed since reset (window not full)
	 */
	float slowUsPerDeg(float windowDeg) const {
		float acc = 0;
		float worst = 0;
		for (size_t i = 0; i < m_count; i++) {
			size_t idx = (m_head + kCapacity - 1 - i) % kCapacity;
			if (m_us[idx] > worst) {
				worst = m_us[idx];
			}
			acc += m_span[idx];
			if (acc >= windowDeg) {
				return worst;
			}
		}
		return 0;
	}

	/** us/deg of the most recent interval that ENDED at endPhase (within epsDeg), 0 if none recorded */
	float usPerDegEndingAt(float endPhase, float cycleDeg, float epsDeg) const {
		for (size_t i = 0; i < m_count; i++) {
			size_t idx = (m_head + kCapacity - 1 - i) % kCapacity;
			if (phaseNear(m_endPhase[idx], endPhase, cycleDeg, epsDeg)) {
				return m_us[idx];
			}
		}
		return 0;
	}

	/**
	 * Predict the time to rotate from fromPhase to toPhase from what the crank did exactly strokeDeg earlier
	 * (compression ripple repeats every stroke: 180 deg on a four cylinder), scaled by how much the newest
	 * interval differs from its own one-stroke-earlier twin (captures a speed change that already happened).
	 *
	 * @return 0 when any needed interval is missing (caller falls back to a coarser estimate)
	 */
	float predictTimeUs(float fromPhase, float toPhase, float strokeDeg, float cycleDeg, float epsDeg) const {
		if (m_count == 0 || !(strokeDeg > 0)) {
			return 0;
		}
		float wanted = forwardDistance(fromPhase, toPhase, cycleDeg);
		if (wanted <= epsDeg) {
			return 0;
		}
		// walk the future intervals tooth by tooth: each one starts where some recorded interval started a stroke ago
		float covered = 0;
		float sumUs = 0;
		float cursor = fromPhase;
		int guard = 0;
		while (covered < wanted - epsDeg && guard++ < (int)kCapacity) {
			float twinStart = cursor - strokeDeg;
			size_t twin = kCapacity;
			for (size_t i = 0; i < m_count; i++) { // newest first: the most recent time the crank was at that angle
				size_t idx = (m_head + kCapacity - 1 - i) % kCapacity;
				if (phaseNear(m_endPhase[idx] - m_span[idx], twinStart, cycleDeg, epsDeg)) {
					twin = idx;
					break;
				}
			}
			if (twin == kCapacity) {
				return 0;
			}
			sumUs += m_us[twin] * m_span[twin];
			covered += m_span[twin];
			cursor += m_span[twin];
		}
		if (covered < wanted - epsDeg || covered > wanted + epsDeg) {
			return 0;
		}
		float trend = 1.0f;
		float last = lastUsPerDeg();
		float lastTwin = usPerDegEndingAt(m_prevPhase - strokeDeg, cycleDeg, epsDeg);
		if (last > 0 && lastTwin > 0) {
			trend = last / lastTwin;
			if (trend < 0.25f) {
				trend = 0.25f;
			}
			if (trend > 4.0f) {
				trend = 4.0f;
			}
		}
		if (trend > 1.0f) {
			// the crank is slowing: the middle of the span we predict is (lastSpan + wanted) / 2 degrees later than the
			// middle of the newest interval, so the slowdown seen over the last stroke continues for that much more. Only for slowing: a later charge start after a
			// speed-up merely shortens dwell, while an early start after a slow-down is the long-dwell hazard.
			trend *= powf(trend, 0.5f * (m_span[(m_head + kCapacity - 1) % kCapacity] + wanted) / strokeDeg);
		}
		return sumUs * trend;
	}

	/**
	 * Time in us to rotate the newest spanDeg degrees after skipping the newest skipIntervals intervals.
	 * @param used (out) number of intervals consumed, so a second call can continue where this one stopped
	 * @return 0 when there is not enough history or the intervals do not add up to spanDeg within epsDeg
	 */
	float periodUs(float spanDeg, float epsDeg, size_t skipIntervals, size_t* used) const {
		float acc = 0;
		float us = 0;
		size_t i = skipIntervals;
		for (; i < m_count && acc < spanDeg - epsDeg; i++) {
			size_t idx = (m_head + kCapacity - 1 - i) % kCapacity;
			acc += m_span[idx];
			us += m_us[idx] * m_span[idx];
		}
		if (acc < spanDeg - epsDeg || acc > spanDeg + epsDeg) {
			return 0;
		}
		*used = i - skipIntervals;
		return us;
	}

	/**
	 * Mitsubishi E931 / E932 style "unstable speed" detector (their ECU, below about 1464 rpm, falls back to
	 * edge-locked timing when the per-cylinder period changed by 18.75 percent or more against the previous one).
	 * Call once per tooth, after onTooth(). The newest strokeDeg of rotation (180 deg on a four cylinder) is compared
	 * with the strokeDeg before it; compression ripple repeats every stroke, so any 180 deg window holds exactly one
	 * ripple period and steady running does not trip it, while a real speed step does.
	 * A trip latches for cycleDeg of further rotation (one full cycle = 4 cylinder events), re-armed by every
	 * further trip.
	 *
	 * @param ratioLimit trip when |newest - previous| >= ratioLimit * newest
	 * @param rpmLimit   only trips while the newest stroke period corresponds to less than this rpm
	 */
	void evaluateInstability(float strokeDeg, float cycleDeg, float ratioLimit, float rpmLimit) {
		if (m_added == m_evaluated || m_count == 0) {
			return; // no new interval since the last call
		}
		m_evaluated = m_added;
		float newestSpan = m_span[(m_head + kCapacity - 1) % kCapacity];
		if (m_degSinceUnstable < 4 * cycleDeg) {
			m_degSinceUnstable += newestSpan;
		}
		size_t used1 = 0;
		size_t used2 = 0;
		float p1 = periodUs(strokeDeg, 0.05f, 0, &used1);
		if (p1 <= 0) {
			return;
		}
		float p2 = periodUs(strokeDeg, 0.05f, used1, &used2);
		if (p2 <= 0) {
			return;
		}
		float rpm = (strokeDeg / 360.0f) * 60000000.0f / p1;
		if (rpm < rpmLimit && fabsf(p1 - p2) >= ratioLimit * p1) {
			m_unstableSeen = true;
			m_degSinceUnstable = 0;
		}
	}

	/** true for cycleDeg of rotation after the last trip of evaluateInstability() */
	bool instabilityLatched(float cycleDeg) const {
		return m_unstableSeen && m_degSinceUnstable < cycleDeg;
	}

	static bool phaseNear(float a, float b, float cycleDeg, float epsDeg) {
		float d = forwardDistance(a, b, cycleDeg);
		return d <= epsDeg || d >= cycleDeg - epsDeg;
	}

	static float forwardDistance(float from, float to, float cycleDeg) {
		float d = to - from;
		while (d < 0) {
			d += cycleDeg;
		}
		while (d >= cycleDeg) {
			d -= cycleDeg;
		}
		return d;
	}

private:
	float m_us[kCapacity];
	float m_span[kCapacity];
	float m_endPhase[kCapacity];
	size_t m_count = 0;
	size_t m_head = 0;
	bool m_hasPrev = false;
	efitick_t m_prevNt = 0;
	float m_prevPhase = 0;
	unsigned m_added = 0;      // intervals recorded since reset
	unsigned m_evaluated = 0;  // value of m_added at the last evaluateInstability()
	bool m_unstableSeen = false;
	float m_degSinceUnstable = 0;
	bool m_postStartArmed = false;
	float m_degSincePostStart = 0;
};

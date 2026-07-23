/**
 * @file dash_warning.h
 *
 * Debounced-hysteresis latch + rpm-stepped threshold selector for the NMEA2000
 * dashboard safety-warning flags (can_dash.cpp, PGN 127489). Pure logic, no
 * engine/sensor/CAN dependencies -> unit-testable in isolation.
 *
 * Terminology (matches the project's "DTC" wording): assert == failed/set,
 * clear == passed/reset.
 */

#pragma once

#include <cstdint>

/**
 * One warning flag's latched state, combining
 *   (a) hysteresis  — independent assert / clear thresholds, and
 *   (b) debounce    — N consecutive samples to assert, M consecutive to clear.
 *
 * updateHigh() trips when the value rises ABOVE a limit (over-temperature);
 * updateLow() trips when it falls BELOW (low oil/fuel/water pressure). The
 * thresholds are passed on every call, so one instance can serve a dynamic
 * (rpm-/MAP-scheduled) threshold — no stored state depends on the level value.
 *
 * The *Gated() variants take an `enabled` predicate (sensor validity AND/OR a
 * post-start grace window): when disabled the latch is reset and held clear, so
 * an absent sensor (reads 0) or the start-up pressure ramp cannot raise a
 * phantom fault.
 *
 * Intended cadence: one update per 1 Hz dashboard cycle, so one count == 1 s.
 */
class DashWarning {
public:
	/** trips when value > assertLevel; clears when value < clearLevel (clearLevel <= assertLevel) */
	bool updateHigh(float value, float assertLevel, float clearLevel,
	                int assertCount, int clearCount) {
		return update(value > assertLevel, value < clearLevel, assertCount, clearCount);
	}

	/** trips when value < assertLevel; clears when value > clearLevel (clearLevel >= assertLevel) */
	bool updateLow(float value, float assertLevel, float clearLevel,
	               int assertCount, int clearCount) {
		return update(value < assertLevel, value > clearLevel, assertCount, clearCount);
	}

	/** updateHigh(), but reset+hold-clear while !enabled (invalid sensor / within grace) */
	bool updateHighGated(bool enabled, float value, float assertLevel, float clearLevel,
	                     int assertCount, int clearCount) {
		if (!enabled) { reset(); return false; }
		return updateHigh(value, assertLevel, clearLevel, assertCount, clearCount);
	}

	/** updateLow(), but reset+hold-clear while !enabled (invalid sensor / within grace) */
	bool updateLowGated(bool enabled, float value, float assertLevel, float clearLevel,
	                    int assertCount, int clearCount) {
		if (!enabled) { reset(); return false; }
		return updateLow(value, assertLevel, clearLevel, assertCount, clearCount);
	}

	bool state() const { return m_state; }
	void reset() { m_state = false; m_counter = 0u; }

private:
	bool update(bool fault, bool recovered, int assertCount, int clearCount) {
		if (!m_state) {
			if (fault) {
				if (m_counter < 65535u) { m_counter++; }
				if (m_counter >= assertCount) { m_state = true; m_counter = 0u; }
			} else {
				m_counter = 0u;
			}
		} else {
			if (recovered) {
				if (m_counter < 65535u) { m_counter++; }
				if (m_counter >= clearCount) { m_state = false; m_counter = 0u; }
			} else {
				m_counter = 0u;
			}
		}
		return m_state;
	}

	bool     m_state   = false;
	uint16_t m_counter = 0u;   // consecutive-sample count; uint16 headroom (<=65535 == ~18 h @1Hz)
	                           // so a large assert/clearCount can never silently saturate and fail.
};

/**
 * Selects a threshold from an ascending rpm-stepped schedule, with hysteresis on
 * the rpm axis so an rpm hovering near a step does not switch bands (and thus
 * threshold) every cycle.
 *   boundaries[N]  ascending rpm break points
 *   levels[N+1]    threshold per band (band 0 below boundaries[0], band N above boundaries[N-1])
 *   rpmMargin      half-width dead-zone: climb only above (boundary+margin), drop only below (boundary-margin)
 */
template <int N>
class SteppedThreshold {
public:
	float get(float rpm, const float (&boundaries)[N], const float (&levels)[N + 1], float rpmMargin) {
		while (m_band < N && rpm > boundaries[m_band] + rpmMargin) { m_band++; }
		while (m_band > 0 && rpm < boundaries[m_band - 1] - rpmMargin) { m_band--; }
		return levels[m_band];
	}

	int  band() const { return m_band; }
	void reset()      { m_band = 0; }

private:
	int m_band = 0;
};

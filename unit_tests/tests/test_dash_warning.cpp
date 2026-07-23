/*
 * @file test_dash_warning.cpp
 *
 * Unit tests for the NMEA2000 dashboard warning helpers (dash_warning.h):
 *   DashWarning      — debounced-hysteresis latch (assert=failed / clear=passed)
 *   SteppedThreshold — rpm-stepped threshold selector with rpm hysteresis
 */

#include "pch.h"

#include "dash_warning.h"

// High-side (over-temp): trip needs assertCount consecutive over-threshold samples.
TEST(DashWarning, highSideTripNeedsDebounce) {
	DashWarning w;
	EXPECT_FALSE(w.updateHigh(120.0f, 125.0f, 122.0f, 2, 3)); // below trip
	EXPECT_FALSE(w.updateHigh(130.0f, 125.0f, 122.0f, 2, 3)); // c1 (need 2)
	EXPECT_TRUE (w.updateHigh(130.0f, 125.0f, 122.0f, 2, 3)); // c2 -> assert
}

// A single good sample restarts the assert debounce (samples must be consecutive).
TEST(DashWarning, highSideAssertDebounceResetsOnGoodSample) {
	DashWarning w;
	EXPECT_FALSE(w.updateHigh(130.0f, 125.0f, 122.0f, 3, 3)); // c1
	EXPECT_FALSE(w.updateHigh(130.0f, 125.0f, 122.0f, 3, 3)); // c2
	EXPECT_FALSE(w.updateHigh(120.0f, 125.0f, 122.0f, 3, 3)); // good -> reset
	EXPECT_FALSE(w.updateHigh(130.0f, 125.0f, 122.0f, 3, 3)); // c1
	EXPECT_FALSE(w.updateHigh(130.0f, 125.0f, 122.0f, 3, 3)); // c2
	EXPECT_TRUE (w.updateHigh(130.0f, 125.0f, 122.0f, 3, 3)); // c3 -> assert
}

// Once asserted: no clear inside the hysteresis band; clear needs the recovery
// margin AND clearCount consecutive samples; a blip into the band restarts it.
TEST(DashWarning, highSideHysteresisAndClearDebounce) {
	DashWarning w;
	EXPECT_TRUE (w.updateHigh(130.0f, 125.0f, 122.0f, 1, 3)); // immediate assert
	EXPECT_TRUE (w.updateHigh(123.0f, 125.0f, 122.0f, 1, 3)); // in band -> stays
	EXPECT_TRUE (w.updateHigh(121.0f, 125.0f, 122.0f, 1, 3)); // c1
	EXPECT_TRUE (w.updateHigh(123.0f, 125.0f, 122.0f, 1, 3)); // band -> restart clear debounce
	EXPECT_TRUE (w.updateHigh(121.0f, 125.0f, 122.0f, 1, 3)); // c1
	EXPECT_TRUE (w.updateHigh(121.0f, 125.0f, 122.0f, 1, 3)); // c2
	EXPECT_FALSE(w.updateHigh(121.0f, 125.0f, 122.0f, 1, 3)); // c3 -> clear
}

// Low-side (oil-pressure): immediate assert (count 1), slow clear, hysteresis band.
TEST(DashWarning, lowSideImmediateAssertSlowClear) {
	DashWarning w;
	EXPECT_FALSE(w.updateLow(200.0f, 150.0f, 165.0f, 1, 3)); // healthy
	EXPECT_TRUE (w.updateLow(140.0f, 150.0f, 165.0f, 1, 3)); // immediate assert
	EXPECT_TRUE (w.updateLow(160.0f, 150.0f, 165.0f, 1, 3)); // in band (150..165) -> stays
	EXPECT_TRUE (w.updateLow(170.0f, 150.0f, 165.0f, 1, 3)); // c1
	EXPECT_TRUE (w.updateLow(170.0f, 150.0f, 165.0f, 1, 3)); // c2
	EXPECT_FALSE(w.updateLow(170.0f, 150.0f, 165.0f, 1, 3)); // c3 -> clear
}

// Gated: while disabled (invalid sensor / within post-start grace) the latch is reset and
// held clear even if the value would trip — the phantom-immunity path. Discriminating: it
// checks state() and a HEALTHY re-enable, so it would FAIL if reset() left a stale m_state.
TEST(DashWarning, gatedDisabledHoldsClear) {
	DashWarning w;
	EXPECT_TRUE (w.updateLowGated(true,  140.0f, 150.0f, 165.0f, 1, 3)); // valid -> assert
	EXPECT_FALSE(w.updateLowGated(false,   0.0f, 150.0f, 165.0f, 1, 3)); // 0 would trip, but gated off
	EXPECT_FALSE(w.state());                                             // latch is truly cleared
	EXPECT_FALSE(w.updateLowGated(true,  200.0f, 150.0f, 165.0f, 1, 3)); // healthy re-enable stays clear
	EXPECT_TRUE (w.updateLowGated(true,  140.0f, 150.0f, 165.0f, 1, 3)); // re-asserts from scratch on a fault
}

// The over-temp warnings use the High-gated path — verify gating + reset there too.
TEST(DashWarning, highGatedDisabledHoldsClear) {
	DashWarning w;
	EXPECT_TRUE (w.updateHighGated(true,  130.0f, 125.0f, 122.0f, 1, 3)); // valid -> assert
	EXPECT_FALSE(w.updateHighGated(false, 999.0f, 125.0f, 122.0f, 1, 3)); // invalid: reset + hold clear
	EXPECT_FALSE(w.state());
	EXPECT_FALSE(w.updateHighGated(true,  120.0f, 125.0f, 122.0f, 1, 3)); // healthy -> stays clear
}

TEST(DashWarning, resetClearsStateAndCounter) {
	DashWarning w;
	EXPECT_TRUE (w.updateHigh(130.0f, 125.0f, 122.0f, 1, 1));
	w.reset();
	EXPECT_FALSE(w.state());
	EXPECT_FALSE(w.updateHigh(130.0f, 125.0f, 122.0f, 2, 2)); // must re-debounce from scratch
	EXPECT_TRUE (w.updateHigh(130.0f, 125.0f, 122.0f, 2, 2));
}

// SteppedThreshold: band selection with rpm hysteresis (oil-pressure schedule).
TEST(SteppedThreshold, oilBandsWithHysteresis) {
	SteppedThreshold<3> s;
	const float b[3] = { 750.0f, 1400.0f, 2250.0f };
	const float l[4] = { 150.0f, 190.0f,  275.0f, 325.0f };
	const float m = 150.0f;
	EXPECT_FLOAT_EQ(150.0f, s.get(400.0f,  b, l, m)); // band 0
	EXPECT_FLOAT_EQ(150.0f, s.get(800.0f,  b, l, m)); // 800 < 750+150 -> no change
	EXPECT_FLOAT_EQ(190.0f, s.get(950.0f,  b, l, m)); // 950 > 900 -> band 1
	EXPECT_FLOAT_EQ(190.0f, s.get(650.0f,  b, l, m)); // 650 > 750-150 -> stays band 1
	EXPECT_FLOAT_EQ(150.0f, s.get(590.0f,  b, l, m)); // 590 < 600 -> band 0
	EXPECT_FLOAT_EQ(325.0f, s.get(2500.0f, b, l, m)); // climbs multiple bands
}

// A hovering rpm near a step must NOT flip the band (and threshold) every cycle.
TEST(SteppedThreshold, waterNoFlipWhenHovering) {
	SteppedThreshold<1> s;
	const float b[1] = { 1000.0f };
	const float l[2] = {   15.0f, 25.0f };
	const float m = 100.0f;
	EXPECT_FLOAT_EQ(25.0f, s.get(1200.0f, b, l, m)); // climb to band 1
	EXPECT_FLOAT_EQ(25.0f, s.get(1050.0f, b, l, m)); // hover within +/-100 dead-zone
	EXPECT_FLOAT_EQ(25.0f, s.get( 950.0f, b, l, m));
	EXPECT_FLOAT_EQ(25.0f, s.get( 910.0f, b, l, m)); // >900 -> stays
	EXPECT_FLOAT_EQ(15.0f, s.get( 890.0f, b, l, m)); // <900 -> drops
}

// The production oil-pressure schedule is a SteppedThreshold<6> (6 boundaries / 7 levels);
// exercise the production instantiation directly: first-call multi-band climb, full descent,
// and each band cleanly past its boundary + hysteresis margin.
TEST(SteppedThreshold, oilSchedule6Bands) {
	SteppedThreshold<6> s;
	const float b[6] = { 750.0f, 1400.0f, 2250.0f, 2900.0f, 3700.0f, 4400.0f };
	const float l[7] = { 140.0f, 145.0f,  275.0f,  320.0f,  335.0f,  345.0f,  365.0f };
	const float m = 100.0f;
	EXPECT_FLOAT_EQ(365.0f, s.get(4600.0f, b, l, m)); // first call at high rpm climbs to top band
	EXPECT_FLOAT_EQ(140.0f, s.get( 500.0f, b, l, m)); // full descent to band 0
	EXPECT_FLOAT_EQ(145.0f, s.get( 900.0f, b, l, m)); // > 750 + 100
	EXPECT_FLOAT_EQ(275.0f, s.get(1550.0f, b, l, m)); // > 1400 + 100
	EXPECT_FLOAT_EQ(320.0f, s.get(2400.0f, b, l, m)); // > 2250 + 100
	EXPECT_FLOAT_EQ(335.0f, s.get(3050.0f, b, l, m)); // > 2900 + 100
	EXPECT_FLOAT_EQ(345.0f, s.get(3850.0f, b, l, m)); // > 3700 + 100
	EXPECT_FLOAT_EQ(365.0f, s.get(4550.0f, b, l, m)); // > 4400 + 100
}

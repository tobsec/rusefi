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

// Gated: while disabled (invalid sensor / within post-start grace) the latch is
// reset and held clear even if the value would trip — the phantom-immunity path.
TEST(DashWarning, gatedDisabledHoldsClear) {
	DashWarning w;
	EXPECT_TRUE (w.updateLowGated(true,  140.0f, 150.0f, 165.0f, 1, 3)); // valid -> assert
	EXPECT_FALSE(w.updateLowGated(false,   0.0f, 150.0f, 165.0f, 1, 3)); // 0 would trip, but gated off
	EXPECT_FALSE(w.updateLowGated(false,   0.0f, 150.0f, 165.0f, 1, 3));
	EXPECT_TRUE (w.updateLowGated(true,  140.0f, 150.0f, 165.0f, 1, 3)); // re-enabled -> re-assert
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

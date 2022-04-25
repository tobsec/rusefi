/*
 * @file	mazda_miata_vvt.cpp
 *
 * Miata NB2, also known as MX-5 Mk2.5
 *
 * Frankenso MAZDA_MIATA_2003
 * set engine_type 47
 *
 * coil1/4          (p1 +5 VP)    GPIOE_14
 * coil2/2          (p1 +5 VP)    GPIOC_9
 * tachometer +5 VP (p3 +12 VP)   GPIOE_8
 * alternator +5 VP (p3 +12 VP)   GPIOE_10
 * ETB PWM                        GPIOE_6 inverted low-side with pull-up
 * ETB dir1                       GPIOE_12
 * ETB dir2                       GPIOC_7
 *
 * COP ion #1                     GPIOD_8
 * COP ion #3                     GPIOD_9
 *
 * @date Oct 4, 2016
 * @author Andrey Belomutskiy, (c) 2012-2020
 * http://rusefi.com/forum/viewtopic.php?f=3&t=1095
 *
 *
 * See also TT_MAZDA_MIATA_VVT_TEST for trigger simulation
 *
 * Based on http://rusefi.com/wiki/index.php?title=Manual:Hardware_Frankenso_board#Default_Pinout
 *
 * board #70 - red car, hunchback compatible
 * set engine_type 55
 *
 * Crank   primary trigger        PA5 (3E in Miata board)       white
 * Cam     vvt input              PC6 (3G in Miata board)       blue
 * Wideband input                 PA3 (3J in Miata board)
 *
 * coil1/4          (p1 +5 VP)    GPIOE_14
 * coil2/2          (p1 +5 VP)    GPIOC_7
 *
 * tachometer +5 VP (p3 +12 VP)   GPIOE_8
 * alternator +5 VP (p3 +12 VP)   GPIOE_10
 *
 * VVT solenoid on aux PID#1      GPIOE_3
 * warning light                  GPIOE_6
 *
 *
 * idle solenoid                  PC13 on middle harness plug. diodes seem to be in the harness
 */

#include "pch.h"

#include "mazda_miata_vvt.h"
#include "custom_engine.h"
#include "mazda_miata_base_maps.h"
#include "hip9011_logic.h"


#if HW_PROTEUS
#include "proteus_meta.h"
#endif

#include "mre_meta.h"

static const float injectorLagBins[VBAT_INJECTOR_CURVE_SIZE] = {
        6.0,         8.0,        10.0,        11.0,
        12.0,        13.0,  14.0,        15.0
};

static const float injectorLagCorrection[VBAT_INJECTOR_CURVE_SIZE] = {
        4.0 ,        3.0 ,        2.0 ,        1.7,
        1.5 ,        1.35,        1.25 ,        1.20
};

static const float vvt18fsioRpmBins[SCRIPT_TABLE_8] =
{700.0, 1000.0, 2000.0, 3000.0, 3500.0, 4500.0, 5500.0, 6500.0}
;

static const float vvt18fsioLoadBins[SCRIPT_TABLE_8] =
{30.0, 40.0, 50.0, 60.0, 70.0, 75.0, 82.0, 85.0}
;

static const uint8_t SCRIPT_TABLE_vvt_target[SCRIPT_TABLE_8][SCRIPT_TABLE_8] = {
		/* Generated by TS2C on Mon Feb 13 19:11:32 EST 2017*/
		{/* 0 30	*//* 0 700.0*/1,	/* 1 1000.0*/3,	/* 2 2000.0*/10,	/* 3 3000.0*/20,	/* 4 3500.0*/27,	/* 5 4500.0*/28,	/* 6 5500.0*/11,	/* 7 6500.0*/5,	},
		{/* 1 40	*//* 0 700.0*/3,	/* 1 1000.0*/10,	/* 2 2000.0*/19,	/* 3 3000.0*/26,	/* 4 3500.0*/30,	/* 5 4500.0*/28,	/* 6 5500.0*/11,	/* 7 6500.0*/5,	},
		{/* 2 50	*//* 0 700.0*/7,	/* 1 1000.0*/16,	/* 2 2000.0*/24,	/* 3 3000.0*/28,	/* 4 3500.0*/30,	/* 5 4500.0*/28,	/* 6 5500.0*/11,	/* 7 6500.0*/5,	},
		{/* 3 60	*//* 0 700.0*/11,	/* 1 1000.0*/20,	/* 2 2000.0*/27,	/* 3 3000.0*/28,	/* 4 3500.0*/30,	/* 5 4500.0*/28,	/* 6 5500.0*/11,	/* 7 6500.0*/5,	},
		{/* 4 70	*//* 0 700.0*/13,	/* 1 1000.0*/24,	/* 2 2000.0*/31,	/* 3 3000.0*/28,	/* 4 3500.0*/30,	/* 5 4500.0*/28,	/* 6 5500.0*/11,	/* 7 6500.0*/5,	},
		{/* 5 75	*//* 0 700.0*/15,	/* 1 1000.0*/27,	/* 2 2000.0*/33,	/* 3 3000.0*/28,	/* 4 3500.0*/30,	/* 5 4500.0*/28,	/* 6 5500.0*/11,	/* 7 6500.0*/5,	},
		{/* 6 82	*//* 0 700.0*/17,	/* 1 1000.0*/28,	/* 2 2000.0*/33,	/* 3 3000.0*/28,	/* 4 3500.0*/30,	/* 5 4500.0*/28,	/* 6 5500.0*/11,	/* 7 6500.0*/5,	},
		{/* 7 85	*//* 0 700.0*/17,	/* 1 1000.0*/28,	/* 2 2000.0*/33,	/* 3 3000.0*/28,	/* 4 3500.0*/30,	/* 5 4500.0*/28,	/* 6 5500.0*/11,	/* 7 6500.0*/5,	},
};

const float mazda_miata_nb2_RpmBins[FUEL_RPM_COUNT] = {700.0, 820.0, 950.0, 1100.0,
		1300.0, 1550.0, 1800.0, 2150.0,
		2500.0, 3000.0, 3500.0, 4150.0,
		4900.0, 5800.0, 6800.0, 8000.0}
;

const float mazda_miata_nb2_LoadBins[FUEL_LOAD_COUNT] = {20.0, 25.0, 30.0, 35.0,
		40.0, 46.0, 54.0, 63.0,
		73.0, 85.0, 99.0, 116.0,
		135.0, 158.0, 185.0, 220.0}
;

static const  float ignition18vvtRpmBins[FUEL_RPM_COUNT] = {
		700.0, 		         850.0 ,		         943.0 ,
		         1112.0 ,		         1310.0 ,		         1545.0 ,
		         1821.0, 		         2146.0, 		         2530.0,
		         2982.0, 		         3515.0 ,		         4144.0 ,
		         4884.0 ,		         5757.0 ,		         6787.0, 		         8000.0};

static const float ignition18vvtLoadBins[FUEL_LOAD_COUNT] = {
		25.0 ,		         29.10009765625 ,		         34.0 ,		         39.60009765625 ,
		         46.2001953125 ,		         53.89990234375 ,		         62.7998046875 ,
				 73.2001953125 ,		         85.400390625 ,		         99.5 ,		         116.0 ,
		         135.30078125 ,		         157.69921875 ,		         183.900390625 ,		         214.400390625 ,
		         250.0};

static const int8_t mapBased18vvtVeTable_NB_fuel_rail[16][16] = {
		/* Generated by TS2C on Tue Apr 18 21:46:03 EDT 2017*/
		{/* 0 20	*//* 0 700.0*/35,	/* 1 820.0*/36,	/* 2 950.0*/37,	/* 3 1100.0*/35,	/* 4 1300.0*/36,	/* 5 1550.0*/37,	/* 6 1800.0*/33,	/* 7 2150.0*/31,	/* 8 2500.0*/25,	/* 9 3000.0*/24,	/* 10 3500.0*/24,	/* 11 4150.0*/25,	/* 12 4900.0*/26,	/* 13 5800.0*/29,	/* 14 6800.0*/33,	/* 15 8000.0*/36,	},
		{/* 1 25	*//* 0 700.0*/35,	/* 1 820.0*/37,	/* 2 950.0*/38,	/* 3 1100.0*/37,	/* 4 1300.0*/36,	/* 5 1550.0*/37,	/* 6 1800.0*/41,	/* 7 2150.0*/39,	/* 8 2500.0*/40,	/* 9 3000.0*/37,	/* 10 3500.0*/35,	/* 11 4150.0*/36,	/* 12 4900.0*/37,	/* 13 5800.0*/35,	/* 14 6800.0*/38,	/* 15 8000.0*/40,	},
		{/* 2 30	*//* 0 700.0*/37,	/* 1 820.0*/40,	/* 2 950.0*/39,	/* 3 1100.0*/37,	/* 4 1300.0*/38,	/* 5 1550.0*/41,	/* 6 1800.0*/45,	/* 7 2150.0*/47,	/* 8 2500.0*/54,	/* 9 3000.0*/48,	/* 10 3500.0*/47,	/* 11 4150.0*/55,	/* 12 4900.0*/55,	/* 13 5800.0*/49,	/* 14 6800.0*/50,	/* 15 8000.0*/51,	},
		{/* 3 35	*//* 0 700.0*/39,	/* 1 820.0*/44,	/* 2 950.0*/42,	/* 3 1100.0*/40,	/* 4 1300.0*/45,	/* 5 1550.0*/48,	/* 6 1800.0*/48,	/* 7 2150.0*/52,	/* 8 2500.0*/56,	/* 9 3000.0*/53,	/* 10 3500.0*/52,	/* 11 4150.0*/58,	/* 12 4900.0*/62,	/* 13 5800.0*/57,	/* 14 6800.0*/58,	/* 15 8000.0*/58,	},
		{/* 4 40	*//* 0 700.0*/45,	/* 1 820.0*/56,	/* 2 950.0*/49,	/* 3 1100.0*/45,	/* 4 1300.0*/54,	/* 5 1550.0*/53,	/* 6 1800.0*/55,	/* 7 2150.0*/54,	/* 8 2500.0*/57,	/* 9 3000.0*/55,	/* 10 3500.0*/57,	/* 11 4150.0*/59,	/* 12 4900.0*/62,	/* 13 5800.0*/59,	/* 14 6800.0*/63,	/* 15 8000.0*/62,	},
		{/* 5 46	*//* 0 700.0*/54,	/* 1 820.0*/61,	/* 2 950.0*/56,	/* 3 1100.0*/52,	/* 4 1300.0*/53,	/* 5 1550.0*/58,	/* 6 1800.0*/57,	/* 7 2150.0*/59,	/* 8 2500.0*/58,	/* 9 3000.0*/58,	/* 10 3500.0*/60,	/* 11 4150.0*/64,	/* 12 4900.0*/66,	/* 13 5800.0*/64,	/* 14 6800.0*/65,	/* 15 8000.0*/63,	},
		{/* 6 54	*//* 0 700.0*/60,	/* 1 820.0*/67,	/* 2 950.0*/66,	/* 3 1100.0*/60,	/* 4 1300.0*/59,	/* 5 1550.0*/59,	/* 6 1800.0*/61,	/* 7 2150.0*/63,	/* 8 2500.0*/63,	/* 9 3000.0*/60,	/* 10 3500.0*/62,	/* 11 4150.0*/69,	/* 12 4900.0*/71,	/* 13 5800.0*/67,	/* 14 6800.0*/65,	/* 15 8000.0*/63,	},
		{/* 7 63	*//* 0 700.0*/65,	/* 1 820.0*/70,	/* 2 950.0*/71,	/* 3 1100.0*/67,	/* 4 1300.0*/62,	/* 5 1550.0*/61,	/* 6 1800.0*/65,	/* 7 2150.0*/63,	/* 8 2500.0*/63,	/* 9 3000.0*/64,	/* 10 3500.0*/66,	/* 11 4150.0*/69,	/* 12 4900.0*/73,	/* 13 5800.0*/71,	/* 14 6800.0*/67,	/* 15 8000.0*/65,	},
		{/* 8 73	*//* 0 700.0*/70,	/* 1 820.0*/74,	/* 2 950.0*/73,	/* 3 1100.0*/75,	/* 4 1300.0*/71,	/* 5 1550.0*/66,	/* 6 1800.0*/66,	/* 7 2150.0*/65,	/* 8 2500.0*/67,	/* 9 3000.0*/69,	/* 10 3500.0*/68,	/* 11 4150.0*/72,	/* 12 4900.0*/76,	/* 13 5800.0*/75,	/* 14 6800.0*/66,	/* 15 8000.0*/65,	},
		{/* 9 85	*//* 0 700.0*/71,	/* 1 820.0*/75,	/* 2 950.0*/76,	/* 3 1100.0*/74,	/* 4 1300.0*/73,	/* 5 1550.0*/72,	/* 6 1800.0*/71,	/* 7 2150.0*/70,	/* 8 2500.0*/72,	/* 9 3000.0*/72,	/* 10 3500.0*/74,	/* 11 4150.0*/76,	/* 12 4900.0*/78,	/* 13 5800.0*/76,	/* 14 6800.0*/68,	/* 15 8000.0*/64,	},
		{/* 10 99	*//* 0 700.0*/75,	/* 1 820.0*/76,	/* 2 950.0*/78,	/* 3 1100.0*/76,	/* 4 1300.0*/73,	/* 5 1550.0*/74,	/* 6 1800.0*/74,	/* 7 2150.0*/74,	/* 8 2500.0*/77,	/* 9 3000.0*/76,	/* 10 3500.0*/77,	/* 11 4150.0*/76,	/* 12 4900.0*/77,	/* 13 5800.0*/76,	/* 14 6800.0*/69,	/* 15 8000.0*/65,	},
		{/* 11 116	*//* 0 700.0*/80,	/* 1 820.0*/80,	/* 2 950.0*/80,	/* 3 1100.0*/80,	/* 4 1300.0*/80,	/* 5 1550.0*/80,	/* 6 1800.0*/80,	/* 7 2150.0*/80,	/* 8 2500.0*/80,	/* 9 3000.0*/80,	/* 10 3500.0*/80,	/* 11 4150.0*/80,	/* 12 4900.0*/80,	/* 13 5800.0*/80,	/* 14 6800.0*/80,	/* 15 8000.0*/80,	},
		{/* 12 135	*//* 0 700.0*/80,	/* 1 820.0*/80,	/* 2 950.0*/80,	/* 3 1100.0*/80,	/* 4 1300.0*/80,	/* 5 1550.0*/80,	/* 6 1800.0*/80,	/* 7 2150.0*/80,	/* 8 2500.0*/80,	/* 9 3000.0*/80,	/* 10 3500.0*/80,	/* 11 4150.0*/80,	/* 12 4900.0*/80,	/* 13 5800.0*/80,	/* 14 6800.0*/80,	/* 15 8000.0*/80,	},
		{/* 13 158	*//* 0 700.0*/80,	/* 1 820.0*/80,	/* 2 950.0*/80,	/* 3 1100.0*/80,	/* 4 1300.0*/80,	/* 5 1550.0*/80,	/* 6 1800.0*/80,	/* 7 2150.0*/80,	/* 8 2500.0*/80,	/* 9 3000.0*/80,	/* 10 3500.0*/80,	/* 11 4150.0*/80,	/* 12 4900.0*/80,	/* 13 5800.0*/80,	/* 14 6800.0*/80,	/* 15 8000.0*/80,	},
		{/* 14 185	*//* 0 700.0*/80,	/* 1 820.0*/80,	/* 2 950.0*/80,	/* 3 1100.0*/80,	/* 4 1300.0*/80,	/* 5 1550.0*/80,	/* 6 1800.0*/80,	/* 7 2150.0*/80,	/* 8 2500.0*/80,	/* 9 3000.0*/80,	/* 10 3500.0*/80,	/* 11 4150.0*/80,	/* 12 4900.0*/80,	/* 13 5800.0*/80,	/* 14 6800.0*/80,	/* 15 8000.0*/80,	},
		{/* 15 220	*//* 0 700.0*/80,	/* 1 820.0*/80,	/* 2 950.0*/80,	/* 3 1100.0*/80,	/* 4 1300.0*/80,	/* 5 1550.0*/80,	/* 6 1800.0*/80,	/* 7 2150.0*/80,	/* 8 2500.0*/80,	/* 9 3000.0*/80,	/* 10 3500.0*/80,	/* 11 4150.0*/80,	/* 12 4900.0*/80,	/* 13 5800.0*/80,	/* 14 6800.0*/80,	/* 15 8000.0*/80,	},
};

#if IGN_LOAD_COUNT == DEFAULT_IGN_LOAD_COUNT
static const uint8_t mapBased18vvtTimingTable[16][16] = {
		/* Generated by TS2C on Tue Apr 18 21:43:57 EDT 2017*/
		{/* 0 25	*//* 0 700.0*/14,	/* 1 850.0*/13,	/* 2 943.0*/13,	/* 3 1112.0*/16,	/* 4 1310.0*/21,	/* 5 1545.0*/25,	/* 6 1821.0*/28,	/* 7 2146.0*/31,	/* 8 2530.0*/34,	/* 9 2982.0*/36,	/* 10 3515.0*/38,	/* 11 4144.0*/39,	/* 12 4884.0*/40,	/* 13 5757.0*/40,	/* 14 6787.0*/40,	/* 15 8000.0*/41,	},
		{/* 1 29.100	*//* 0 700.0*/14,	/* 1 850.0*/13,	/* 2 943.0*/13,	/* 3 1112.0*/16,	/* 4 1310.0*/21,	/* 5 1545.0*/25,	/* 6 1821.0*/28,	/* 7 2146.0*/31,	/* 8 2530.0*/34,	/* 9 2982.0*/36,	/* 10 3515.0*/38,	/* 11 4144.0*/39,	/* 12 4884.0*/40,	/* 13 5757.0*/40,	/* 14 6787.0*/40,	/* 15 8000.0*/40,	},
		{/* 2 34	*//* 0 700.0*/14,	/* 1 850.0*/13,	/* 2 943.0*/13,	/* 3 1112.0*/16,	/* 4 1310.0*/21,	/* 5 1545.0*/24,	/* 6 1821.0*/27,	/* 7 2146.0*/30,	/* 8 2530.0*/33,	/* 9 2982.0*/35,	/* 10 3515.0*/37,	/* 11 4144.0*/38,	/* 12 4884.0*/39,	/* 13 5757.0*/40,	/* 14 6787.0*/40,	/* 15 8000.0*/40,	},
		{/* 3 39.600	*//* 0 700.0*/15,	/* 1 850.0*/13,	/* 2 943.0*/13,	/* 3 1112.0*/17,	/* 4 1310.0*/21,	/* 5 1545.0*/24,	/* 6 1821.0*/27,	/* 7 2146.0*/30,	/* 8 2530.0*/33,	/* 9 2982.0*/35,	/* 10 3515.0*/36,	/* 11 4144.0*/38,	/* 12 4884.0*/38,	/* 13 5757.0*/39,	/* 14 6787.0*/39,	/* 15 8000.0*/39,	},
		{/* 4 46.200	*//* 0 700.0*/15,	/* 1 850.0*/13,	/* 2 943.0*/13,	/* 3 1112.0*/18,	/* 4 1310.0*/21,	/* 5 1545.0*/24,	/* 6 1821.0*/26,	/* 7 2146.0*/29,	/* 8 2530.0*/32,	/* 9 2982.0*/33,	/* 10 3515.0*/36,	/* 11 4144.0*/37,	/* 12 4884.0*/38,	/* 13 5757.0*/38,	/* 14 6787.0*/38,	/* 15 8000.0*/39,	},
		{/* 5 53.900	*//* 0 700.0*/15,	/* 1 850.0*/14,	/* 2 943.0*/14,	/* 3 1112.0*/18,	/* 4 1310.0*/21,	/* 5 1545.0*/24,	/* 6 1821.0*/26,	/* 7 2146.0*/28,	/* 8 2530.0*/30,	/* 9 2982.0*/32,	/* 10 3515.0*/34,	/* 11 4144.0*/36,	/* 12 4884.0*/37,	/* 13 5757.0*/37,	/* 14 6787.0*/38,	/* 15 8000.0*/38,	},
		{/* 6 62.800	*//* 0 700.0*/15,	/* 1 850.0*/15,	/* 2 943.0*/14,	/* 3 1112.0*/19,	/* 4 1310.0*/21,	/* 5 1545.0*/23,	/* 6 1821.0*/25,	/* 7 2146.0*/27,	/* 8 2530.0*/29,	/* 9 2982.0*/31,	/* 10 3515.0*/33,	/* 11 4144.0*/34,	/* 12 4884.0*/35,	/* 13 5757.0*/36,	/* 14 6787.0*/36,	/* 15 8000.0*/37,	},
		{/* 7 73.200	*//* 0 700.0*/16,	/* 1 850.0*/16,	/* 2 943.0*/15,	/* 3 1112.0*/19,	/* 4 1310.0*/21,	/* 5 1545.0*/23,	/* 6 1821.0*/24,	/* 7 2146.0*/26,	/* 8 2530.0*/28,	/* 9 2982.0*/30,	/* 10 3515.0*/31,	/* 11 4144.0*/32,	/* 12 4884.0*/33,	/* 13 5757.0*/34,	/* 14 6787.0*/34,	/* 15 8000.0*/35,	},
		{/* 8 85.400	*//* 0 700.0*/16,	/* 1 850.0*/17,	/* 2 943.0*/16,	/* 3 1112.0*/19,	/* 4 1310.0*/20,	/* 5 1545.0*/22,	/* 6 1821.0*/23,	/* 7 2146.0*/24,	/* 8 2530.0*/26,	/* 9 2982.0*/28,	/* 10 3515.0*/29,	/* 11 4144.0*/31,	/* 12 4884.0*/31,	/* 13 5757.0*/32,	/* 14 6787.0*/33,	/* 15 8000.0*/33,	},
		{/* 9 99.500	*//* 0 700.0*/16,	/* 1 850.0*/16,	/* 2 943.0*/17,	/* 3 1112.0*/18,	/* 4 1310.0*/19,	/* 5 1545.0*/20,	/* 6 1821.0*/21,	/* 7 2146.0*/22,	/* 8 2530.0*/23,	/* 9 2982.0*/25,	/* 10 3515.0*/26,	/* 11 4144.0*/28,	/* 12 4884.0*/28,	/* 13 5757.0*/29,	/* 14 6787.0*/30,	/* 15 8000.0*/31,	},
		{/* 10 116	*//* 0 700.0*/15,	/* 1 850.0*/15,	/* 2 943.0*/16,	/* 3 1112.0*/16,	/* 4 1310.0*/17,	/* 5 1545.0*/18,	/* 6 1821.0*/19,	/* 7 2146.0*/20,	/* 8 2530.0*/21,	/* 9 2982.0*/23,	/* 10 3515.0*/24,	/* 11 4144.0*/25,	/* 12 4884.0*/26,	/* 13 5757.0*/27,	/* 14 6787.0*/28,	/* 15 8000.0*/29,	},
		{/* 11 135.301	*//* 0 700.0*/13,	/* 1 850.0*/13,	/* 2 943.0*/14,	/* 3 1112.0*/14,	/* 4 1310.0*/15,	/* 5 1545.0*/15,	/* 6 1821.0*/17,	/* 7 2146.0*/17,	/* 8 2530.0*/19,	/* 9 2982.0*/20,	/* 10 3515.0*/22,	/* 11 4144.0*/23,	/* 12 4884.0*/24,	/* 13 5757.0*/25,	/* 14 6787.0*/26,	/* 15 8000.0*/27,	},
		{/* 12 157.699	*//* 0 700.0*/11,	/* 1 850.0*/11,	/* 2 943.0*/11,	/* 3 1112.0*/12,	/* 4 1310.0*/12,	/* 5 1545.0*/13,	/* 6 1821.0*/14,	/* 7 2146.0*/15,	/* 8 2530.0*/16,	/* 9 2982.0*/17,	/* 10 3515.0*/19,	/* 11 4144.0*/20,	/* 12 4884.0*/21,	/* 13 5757.0*/22,	/* 14 6787.0*/24,	/* 15 8000.0*/25,	},
		{/* 13 183.900	*//* 0 700.0*/8,	/* 1 850.0*/8,	/* 2 943.0*/9,	/* 3 1112.0*/9,	/* 4 1310.0*/9,	/* 5 1545.0*/10,	/* 6 1821.0*/11,	/* 7 2146.0*/12,	/* 8 2530.0*/13,	/* 9 2982.0*/14,	/* 10 3515.0*/16,	/* 11 4144.0*/17,	/* 12 4884.0*/18,	/* 13 5757.0*/19,	/* 14 6787.0*/21,	/* 15 8000.0*/22,	},
		{/* 14 214.400	*//* 0 700.0*/5,	/* 1 850.0*/5,	/* 2 943.0*/5,	/* 3 1112.0*/5,	/* 4 1310.0*/6,	/* 5 1545.0*/7,	/* 6 1821.0*/7,	/* 7 2146.0*/8,	/* 8 2530.0*/9,	/* 9 2982.0*/10,	/* 10 3515.0*/12,	/* 11 4144.0*/13,	/* 12 4884.0*/14,	/* 13 5757.0*/16,	/* 14 6787.0*/17,	/* 15 8000.0*/18,	},
		{/* 15 250	*//* 0 700.0*/1,	/* 1 850.0*/1,	/* 2 943.0*/1,	/* 3 1112.0*/2,	/* 4 1310.0*/2,	/* 5 1545.0*/3,	/* 6 1821.0*/3,	/* 7 2146.0*/4,	/* 8 2530.0*/5,	/* 9 2982.0*/6,	/* 10 3515.0*/7,	/* 11 4144.0*/9,	/* 12 4884.0*/10,	/* 13 5757.0*/12,	/* 14 6787.0*/13,	/* 15 8000.0*/14,	},
};
#endif


/*
#define MAF_TRANSFER_SIZE 8

static const float mafTransferVolts[MAF_TRANSFER_SIZE] = {1.365,
		1.569,
		2.028,
		2.35,
		2.611,
		2.959,
		3.499,
		4.011,
};


according to internet this should be the Miata NB transfer function but in reality it seems off
this could be related to us not using proper signal conditioning hardware
static const float mafTransferKgH[MAF_TRANSFER_SIZE] = {
		0,
		3.9456,
		18.7308,
		45.4788,
		82.278,
		154.4328,
		329.8104,
		594.2772
};


*/

#define MAF_TRANSFER_SIZE 10

// this transfer function somehow works with 1K pull-down
static const float mafTransferVolts[MAF_TRANSFER_SIZE] = {
		0.50,
		0.87,
		1.07,
		1.53,
		1.85,
		2.11,
		2.46,
		3.00,
		3.51,
		4.50
};

static const float mafTransferKgH[MAF_TRANSFER_SIZE] = {
		0.00,
		0.00,
		1.00,
		3.00,
		8.00,
		19.00,
		45.00,
		100.00,
		175.00,
		350.00
};


static void setMAFTransferFunction() {
	memcpy(config->mafDecoding, mafTransferKgH, sizeof(mafTransferKgH));
	memcpy(config->mafDecodingBins, mafTransferVolts, sizeof(mafTransferVolts));
	for (int i = MAF_TRANSFER_SIZE;i<MAF_DECODING_COUNT;i++) {
		config->mafDecodingBins[i] = config->mafDecodingBins[MAF_TRANSFER_SIZE - 1] + i * 0.01;
		config->mafDecoding[i] = config->mafDecoding[MAF_TRANSFER_SIZE - 1];
	}
}

void setMazdaMiataNbInjectorLag() {
	copyArray(engineConfiguration->injector.battLagCorr, injectorLagCorrection);
	copyArray(engineConfiguration->injector.battLagCorrBins, injectorLagBins);
}

void setMazdaNB2VVTSettings() {
	copyArray(config->vvtTable1RpmBins, vvt18fsioRpmBins);
	copyArray(config->vvtTable1LoadBins, vvt18fsioLoadBins);
	copyTable(config->vvtTable1, SCRIPT_TABLE_vvt_target);

	// VVT closed loop
	engineConfiguration->auxPid[0].pFactor = 2;
	engineConfiguration->auxPid[0].iFactor = 0.005;
	engineConfiguration->auxPid[0].dFactor = 0.002;
	engineConfiguration->auxPid[0].offset = 33;
	engineConfiguration->auxPid[0].minValue = 20;
	engineConfiguration->auxPid[0].maxValue = 90;
}

static void set4EC_AT() {
	engineConfiguration->totalGearsCount = 4;
	// http://www.new-cars.com/2003/mazda/mazda-miata-specs.html
	engineConfiguration->gearRatio[0] = 2.45;
	engineConfiguration->gearRatio[1] = 1.45;
	engineConfiguration->gearRatio[2] = 1.0;
	engineConfiguration->gearRatio[3] = 0.73;
}

/**
 * stuff common between NA1 and NB2
 */
static void setCommonMazdaNB() {
	engineConfiguration->displayLogicLevelsInEngineSniffer = true;
	engineConfiguration->useOnlyRisingEdgeForTrigger = true;
	engineConfiguration->trigger.type = TT_MIATA_VVT;

	engineConfiguration->ignitionDwellForCrankingMs = 4;
	// set cranking_fuel 27.5
	engineConfiguration->cranking.baseFuel = 27.5; // this value for return-less NB miata fuel system, higher pressure

	engineConfiguration->cranking.rpm = 400;
	engineConfiguration->idle.solenoidFrequency = 500;
	engineConfiguration->rpmHardLimit = 7200;
	engineConfiguration->useInstantRpmForIdle = true;
	engineConfiguration->enableFan1WithAc = true;

	engineConfiguration->isAlternatorControlEnabled = true;
	// enable altdebug
	engineConfiguration->targetVBatt = 13.8;
	engineConfiguration->alternatorControl.offset = 20;
	engineConfiguration->alternatorControl.pFactor = 16;
	engineConfiguration->alternatorControl.iFactor = 8;
	engineConfiguration->alternatorControl.dFactor = 0.1;
	engineConfiguration->alternatorControl.periodMs = 10;

	copyArray(config->veRpmBins, mazda_miata_nb2_RpmBins);
	copyArray(config->veLoadBins, mazda_miata_nb2_LoadBins);
	copyTable(config->veTable, mapBased18vvtVeTable_NB_fuel_rail);

	copyArray(config->ignitionRpmBins, ignition18vvtRpmBins);
	copyArray(config->ignitionLoadBins, ignition18vvtLoadBins);
#if IGN_LOAD_COUNT == DEFAULT_IGN_LOAD_COUNT
	copyTable(config->ignitionTable, mapBased18vvtTimingTable);
#endif
	// set_whole_ve_map 80
	setMazdaMiataNbInjectorLag();

	engineConfiguration->idleMode = IM_AUTO;
	engineConfiguration->tachPulsePerRev = 2;

	engineConfiguration->specs.displacement = 1.839;
	engineConfiguration->cylinderBore = 83;
	strcpy(engineConfiguration->engineMake, ENGINE_MAKE_MAZDA);

	setCommonNTCSensor(&engineConfiguration->clt, 2700);
	setCommonNTCSensor(&engineConfiguration->iat, 2700);
	setMAFTransferFunction();

    // second harmonic (aka double) is usually quieter background noise
    // 13.8
	engineConfiguration->knockBandCustom = 2 * HIP9011_BAND(engineConfiguration->cylinderBore);

	// set tps_min 90
	engineConfiguration->tpsMin = 100; // convert 12to10 bit (ADC/4)
	// set tps_max 540
	engineConfiguration->tpsMax = 650; // convert 12to10 bit (ADC/4)

	// set idle_position 20
	engineConfiguration->manIdlePosition = 20;
	engineConfiguration->iacByTpsTaper = 6;
	engineConfiguration->acIdleExtraOffset = 15;

	engineConfiguration->useIdleTimingPidControl = true;
	engineConfiguration->idlePidRpmUpperLimit = 350;
	engineConfiguration->idlePidRpmDeadZone = 100;

	engineConfiguration->crankingIACposition = 36;
	engineConfiguration->afterCrankingIACtaperDuration = 189;

	engineConfiguration->wwaeTau = 0.1;
	engineConfiguration->targetVBatt = 14.2;

	engineConfiguration->vehicleWeight = 1070;
	engineConfiguration->specs.cylindersCount = 4;
	engineConfiguration->specs.firingOrder = FO_1_3_4_2;

	engineConfiguration->injectionMode = IM_SEQUENTIAL;
	engineConfiguration->ignitionMode = IM_WASTED_SPARK;

	//set idle_offset 30
	engineConfiguration->idleRpmPid.pFactor = 0.0065;
	engineConfiguration->idleRpmPid.iFactor = 0.3;
	engineConfiguration->idle_derivativeFilterLoss = 0.08;
	engineConfiguration->idle_antiwindupFreq = 0.03;
	engineConfiguration->idleRpmPid.dFactor = 0.002;
	engineConfiguration->idleRpmPid.offset = 9;
	engineConfiguration->idleRpmPid.minValue = -8;
	engineConfiguration->idleRpmPid.minValue = 76;
	engineConfiguration->idlerpmpid_iTermMin = -15;
	engineConfiguration->idlerpmpid_iTermMax =  30;

	// is this used?
	engineConfiguration->idleRpmPid.periodMs = 10;

	miataNA_setCltIdleCorrBins();
	miataNA_setCltIdleRpmBins();
	miataNA_setIacCoastingBins();
	set4EC_AT();
}

static void setMazdaMiataEngineNB1Defaults() {
	setCommonMazdaNB();
	strcpy(engineConfiguration->engineCode, "NB1");
}

static void setMazdaMiataEngineNB2Defaults() {
	strcpy(engineConfiguration->engineCode, "NB2");

	engineConfiguration->map.sensor.type = MT_GM_3_BAR;
	setEgoSensor(ES_Innovate_MTX_L);

	/**
	 * http://miataturbo.wikidot.com/fuel-injectors
	 * 01-05 (purple) - #195500-4060
	 *
	 * NB2 Miata has an absolute pressure fuel system - NB2 fuel rail regulator has no vacuum line.
	 *
	 * Theoretically we shall have injectorFlow(MAP) curve, practically VE gets artificially high as MAP increases and
	 * accounts for flow change.
	 *
	 * Wall wetting AE could be an argument for honest injectorFlow(MAP)
	 */
	engineConfiguration->injector.flow = 265;
	engineConfiguration->fuelReferencePressure = 400; // 400 kPa, 58 psi
	engineConfiguration->injectorCompensationMode = ICM_FixedRailPressure;

	engineConfiguration->crankingIACposition = 60;
	engineConfiguration->afterCrankingIACtaperDuration = 250;


	engineConfiguration->vvtCamSensorUseRise = true;
	// set vvt_mode 3
	engineConfiguration->vvtMode[0] = VVT_MIATA_NB;
	engineConfiguration->vvtOffsets[0] = 98; // 2003 red car value

	setCommonMazdaNB();

	setMazdaNB2VVTSettings();
} // end of setMazdaMiataEngineNB2Defaults

// MAZDA_MIATA_2003
void setMazdaMiata2003EngineConfiguration() {
	setFrankensoConfiguration();

	setMazdaMiataEngineNB2Defaults();

//	engineConfiguration->triggerInputPins[0] = GPIOA_8; // custom Frankenso wiring in order to use SPI1 for accelerometer
	engineConfiguration->triggerInputPins[0] = GPIOA_5; // board still not modified
	engineConfiguration->triggerInputPins[1] = GPIO_UNASSIGNED;
	engineConfiguration->camInputs[0] = GPIOC_6;

//	engineConfiguration->is_enabled_spi_1 = true;

	engineConfiguration->twoWireBatchInjection = true; // this is needed for #492 testing

	engineConfiguration->alternatorControlPin = GPIOE_10;
	engineConfiguration->alternatorControlPinMode = OM_OPENDRAIN;

//	engineConfiguration->vehicleSpeedSensorInputPin = GPIOA_8;

	engineConfiguration->vvtPins[0] = GPIOE_3; // VVT solenoid control

	// high-side driver with +12v VP jumper
	engineConfiguration->tachOutputPin = GPIOE_8; // tachometer

	// set global_trigger_offset_angle 0
	engineConfiguration->globalTriggerAngleOffset = 0;

	// enable trigger_details
	engineConfiguration->verboseTriggerSynchDetails = false;

	// set cranking_timing_angle 10
	engineConfiguration->crankingTimingAngle = 10;

/**
 * Saab attempt
 * Saab  coil on #1 PD8 extra blue wire
 * Miata coil on #2 PC9  - orange ECU wire "2&3"
 * Saab  coil on #3 PD9 extra white wire
 * Miata coil on #4 PE14 - white ECU wire "1&4"
 */

	engineConfiguration->ignitionPins[0] = GPIOE_14;
	engineConfiguration->ignitionPins[1] = GPIO_UNASSIGNED;
	engineConfiguration->ignitionPins[2] = GPIOC_9;
	engineConfiguration->ignitionPins[3] = GPIO_UNASSIGNED;



	engineConfiguration->malfunctionIndicatorPin = GPIOD_5;


//	engineConfiguration->malfunctionIndicatorPin = GPIOD_9;
//	engineConfiguration->malfunctionIndicatorPinMode = OM_INVERTED;

	// todo: blue jumper wire - what is it?!
	// Frankenso analog #6 pin 3R, W56 (5th lower row pin from the end) top <> W45 bottom jumper, not OEM


	// see setFrankensoConfiguration
	// map.sensor.hwChannel = EFI_ADC_0; W53

	/**
	 * PA4 Wideband O2 Sensor
	 */
	// todo: re-wire the board to use "Frankenso analog #7 pin 3J, W48 top <>W48 bottom jumper, not OEM"
	//engineConfiguration->afr.hwChannel = EFI_ADC_3; // PA3
	engineConfiguration->afr.hwChannel = EFI_ADC_4;

	//
	/**
	 * Combined RPM, CLT and VBATT warning light
	 *
	 * to test
	 * set_fsio_setting 2 1800
	 * set_fsio_setting 3 65
	 * set_fsio_setting 4 15
	 */
	engineConfiguration->scriptSetting[1] = 6500; // #2 RPM threshold
	engineConfiguration->scriptSetting[2] = 105; // #3 CLT threshold
	engineConfiguration->scriptSetting[3] = 12.0; // #4 voltage threshold

	// enable auto_idle
	// enable verbose_idle
	engineConfiguration->isVerboseIAC = false;
	// set idle_p 0.05
	// set idle_i 0
	// set idle_d 0
	// set debug_mode 3
	// set idle_rpm 1700
	// see setDefaultIdleParameters

	engineConfiguration->adcVcc = 3.3f;
	engineConfiguration->vbattDividerCoeff = 8.80f;

	// by the way NB2 MAF internal diameter is about 2.5 inches / 63mm
	// 1K pull-down to read current from this MAF
	engineConfiguration->mafAdcChannel = EFI_ADC_6; // PA6 W46 <> W46

	engineConfiguration->throttlePedalUpVoltage = 0.65f;


	// TLE7209 two-wire ETB control
	// PWM
	engineConfiguration->etb_use_two_wires = true;

	engineConfiguration->etbIo[0].controlPin = GPIO_UNASSIGNED;

	//
	engineConfiguration->etbIo[0].directionPin1 = GPIOE_12; // orange
	//
	engineConfiguration->etbIo[0].directionPin2 = GPIOC_7; // white/blue

	// set_analog_input_pin tps PC3
	engineConfiguration->tps1_1AdcChannel = EFI_ADC_13; // PC3 blue

	// set_analog_input_pin pps PA2
/* a step back - Frankenso does not use ETB
	engineConfiguration->throttlePedalPositionAdcChannel = EFI_ADC_2;
*/

	//set etb_p 12
	engineConfiguration->etb.pFactor = 12; // a bit lower p-factor seems to work better on TLE9201? MRE?
	engineConfiguration->etb.iFactor = 	0;
	engineConfiguration->etb.dFactor = 0;
	engineConfiguration->etb.offset = 0;
	engineConfiguration->etb.minValue = -60;
	engineConfiguration->etb.maxValue = 50;

	config->crankingFuelCoef[0] = 2.8; // base cranking fuel adjustment coefficient
	config->crankingFuelBins[0] = -20; // temperature in C
	config->crankingFuelCoef[1] = 2.2;
	config->crankingFuelBins[1] = -10;
	config->crankingFuelCoef[2] = 1.8;
	config->crankingFuelBins[2] = 5;
	config->crankingFuelCoef[3] = 1.5;
	config->crankingFuelBins[3] = 30;

	config->crankingFuelCoef[4] = 1.0;
	config->crankingFuelBins[4] = 35;
	config->crankingFuelCoef[5] = 1.0;
	config->crankingFuelBins[5] = 50;
	config->crankingFuelCoef[6] = 1.0;
	config->crankingFuelBins[6] = 65;
	config->crankingFuelCoef[7] = 1.0;
	config->crankingFuelBins[7] = 90;
}

/**
 * red car setting with default 1991/1995 miata harness
 * board #70 - closer to default miata NA6 harness
 *
 */
void setMazdaMiata2003EngineConfigurationBoardTest() {
	setMazdaMiata2003EngineConfiguration();

	engineConfiguration->ignitionPins[2] = GPIOC_7;

	// Frankenso analog #7 pin 3J, W48 top <>W48 bottom jumper, not OEM. Make sure 500K pull-down on Frankenso
	engineConfiguration->afr.hwChannel = EFI_ADC_3; // PA3

	engineConfiguration->mafAdcChannel = EFI_ADC_4; // PA4 - W47 top <>W47
}

static void setMiataNB2_MRE_common() {
	setMazdaMiataEngineNB2Defaults();
#if (BOARD_TLE8888_COUNT > 0)

	// MRE has a special main relay control low side pin - rusEfi firmware is totally not involved with main relay control
	//
	// fuelPumpPin output is inherited from boards/microrusefi/board_configuration.cpp
	// fanPin output is inherited from boards/microrusefi/board_configuration.cpp
	//
	// crank trigger input is inherited from boards/microrusefi/board_configuration.cpp
	// map.sensor.hwChannel input is inherited from boards/microrusefi/board_configuration.cpp
	// tps1_1AdcChannel input is inherited from boards/microrusefi/board_configuration.cpp
	// afr.hwChannel input is inherited from boards/microrusefi/board_configuration.cpp

	engineConfiguration->ignitionPins[1] = GPIO_UNASSIGNED;
	engineConfiguration->ignitionPins[3] = GPIO_UNASSIGNED;

	engineConfiguration->camInputs[0] = GPIOA_5;
	/**
	 * By default "auto detection mode for VR sensor signals" is used
	 * We know that for short & strange Hall (?) signals like Miata NB2 crank sensor this does not work well above certain RPM.
	 */
	engineConfiguration->tle8888mode = TL_MANUAL;

	// GPIOD_6: "13 - GP Out 6" - selected to +12v
	engineConfiguration->alternatorControlPin = GPIOD_6;
	// GPIOD_7: "14 - GP Out 5" - selected to +12v
	engineConfiguration->tachOutputPin = GPIOD_7; // tachometer
	engineConfiguration->tachPulsePerRev = 2;

	engineConfiguration->isSdCardEnabled = true;

	engineConfiguration->ignitionDwellForCrankingMs = 8;

	engineConfiguration->vvtOffsets[0] = 97;


	//   # TLE8888 high current low side: VVT1 IN10 / OUT6
	// TLE8888_PIN_6:  "7 - Lowside 1"
	engineConfiguration->vvtPins[0] = TLE8888_PIN_6; // VVT solenoid control

	// TLE8888_PIN_23: "33 - GP Out 3"
	engineConfiguration->malfunctionIndicatorPin = TLE8888_PIN_23;


	// todo: alternator warn
	// ?

	// todo: AC fan
	// TLE8888_PIN_24: "43 - GP Out 4"

	// set_analog_input_pin pps PA7
	// EFI_ADC_7: "31 - AN volt 3" - PA7
// disabled for now since only allowed with ETB
//	engineConfiguration->throttlePedalPositionAdcChannel = EFI_ADC_7;

	// set tps_min 90
	engineConfiguration->tpsMin = 90;

	// set tps_max 540
	engineConfiguration->tpsMax = 870;

	// 0.3#4 has wrong R139? TODO: fix that custom board to match proper value!!!
	// set vbatt_divider 10.956
	// 56k high side/10k low side multiplied by analogInputDividerCoefficient
	// vbattDividerCoeff = 10.956 (66.0f / 10.0f) * engineConfiguration->analogInputDividerCoefficient;
#endif /* BOARD_TLE8888_COUNT */
}


/**
 * Pretty much OEM 2003 Miata with ETB
 * set engine_type 13
 */
void setMiataNB2_MRE_ETB() {
	setMiataNB2_MRE_common();

	engineConfiguration->useETBforIdleControl = true;


	engineConfiguration->useETBforIdleControl = true;
	engineConfiguration->throttlePedalUpVoltage = 1;
	// WAT? that's an interesting value, how come it's above 5v?
	engineConfiguration->throttlePedalWOTVoltage = 5.47;

	engineConfiguration->etb.pFactor = 12; // a bit lower p-factor seems to work better on TLE9201? MRE?
	engineConfiguration->etb.iFactor = 	0;
	engineConfiguration->etb.dFactor = 0;
	engineConfiguration->etb.offset = 0;
}

/**
 * Normal mechanical throttle body
 * set engine_type 11
 */
void setMiataNB2_MRE_MAP() {
	setMiataNB2_MRE_common();

	// somehow MRE72 adapter 0.2 has TPS routed to pin 26?
	engineConfiguration->tps1_1AdcChannel = EFI_ADC_6; // PA6


	// 1K pull-down to read current from this MAF
	engineConfiguration->mafAdcChannel = EFI_ADC_13; // J30 AV5
}

void setMiataNB2_MRE_MAF() {
	setMiataNB2_MRE_MAP();

	engineConfiguration->fuelAlgorithm = LM_REAL_MAF;
}

/**
 * https://github.com/rusefi/rusefi/wiki/HOWTO-TCU-A42DE-on-Proteus
 */
#if HW_PROTEUS
void setMiataNB2_Proteus_TCU() {
	engineConfiguration->tcuEnabled = true;

	engineConfiguration->trigger.type = TT_TOOTHED_WHEEL;
	engineConfiguration->trigger.customTotalToothCount = 10;
	engineConfiguration->trigger.customSkippedToothCount = 0;


	engineConfiguration->triggerInputPins[0] = GPIO_UNASSIGNED;
	engineConfiguration->tcuInputSpeedSensorPin = PROTEUS_VR_1;

	engineConfiguration->vehicleSpeedSensorInputPin = PROTEUS_VR_2;

	engineConfiguration->driveWheelRevPerKm = 544;	// 205/50R15
	engineConfiguration->vssGearRatio = 4.3;
	engineConfiguration->vssToothCount = 22;

	// "Highside 2"
	engineConfiguration->tcu_solenoid[0] = GPIOA_8;
	// "Highside 1"
	engineConfiguration->tcu_solenoid[1] = GPIOA_9;

	// "Digital 1" green
	engineConfiguration->tcuUpshiftButtonPin = GPIOC_6;
	engineConfiguration->tcuUpshiftButtonPinMode = PI_PULLUP;
	// "Digital 6" white
	engineConfiguration->tcuDownshiftButtonPin = GPIOE_15;
	engineConfiguration->tcuDownshiftButtonPinMode = PI_PULLUP;

	// R
	config->tcuSolenoidTable[0][0] = 1;
	config->tcuSolenoidTable[0][1] = 0;
	// P/N
	config->tcuSolenoidTable[1][0] = 1;
	config->tcuSolenoidTable[1][1] = 0;
	// 1
	config->tcuSolenoidTable[2][0] = 1;
	config->tcuSolenoidTable[2][1] = 0;
	// 2
	config->tcuSolenoidTable[3][0] = 1;
	config->tcuSolenoidTable[3][1] = 1;
	// 3
	config->tcuSolenoidTable[4][0] = 0;
	config->tcuSolenoidTable[4][1] = 1;
	// 4
	config->tcuSolenoidTable[5][0] = 0;
	config->tcuSolenoidTable[5][1] = 0;

}

/**
 * https://github.com/rusefi/rusefi/wiki/HOWTO-Miata-NB2-on-Proteus
 */
void setMiataNB2_ProteusEngineConfiguration() {
    setMazdaMiataEngineNB2Defaults();

    engineConfiguration->triggerInputPins[0] = GPIOC_6;                     // pin 10/black23
    engineConfiguration->triggerInputPins[1] = GPIO_UNASSIGNED;
    engineConfiguration->camInputs[0] = GPIOE_11;                           // pin  1/black23

    engineConfiguration->alternatorControlPin = GPIOA_8;  // "Highside 2"    # pin 1/black35

    engineConfiguration->vvtPins[0] = GPIOB_5; // VVT solenoid control # pin 8/black35

    // high-side driver with +12v VP jumper
    engineConfiguration->tachOutputPin = GPIOA_9; // tachometer
    engineConfiguration->tachPulsePerRev = 2;

    engineConfiguration->ignitionMode = IM_WASTED_SPARK;

    #if EFI_PROD_CODE
    engineConfiguration->ignitionPins[0] = PROTEUS_IGN_1;
    engineConfiguration->ignitionPins[1] = GPIO_UNASSIGNED;
    engineConfiguration->ignitionPins[2] = PROTEUS_IGN_3;
    engineConfiguration->ignitionPins[3] = GPIO_UNASSIGNED;

    engineConfiguration->crankingInjectionMode = IM_SIMULTANEOUS;
    engineConfiguration->injectionMode = IM_SEQUENTIAL;


    engineConfiguration->injectionPins[0] = PROTEUS_LS_1;  // BLU  # pin 3/black35
    engineConfiguration->injectionPins[1] = PROTEUS_LS_2;  // BLK
    engineConfiguration->injectionPins[2] = PROTEUS_LS_3; // GRN
    engineConfiguration->injectionPins[3] = PROTEUS_LS_4; // WHT
    engineConfiguration->injectionPinMode = OM_DEFAULT;


    engineConfiguration->enableSoftwareKnock = true;

    engineConfiguration->malfunctionIndicatorPin = PROTEUS_LS_10;

    engineConfiguration->map.sensor.hwChannel = PROTEUS_IN_MAP;


    engineConfiguration->afr.hwChannel = EFI_ADC_11;

    engineConfiguration->mafAdcChannel = EFI_ADC_13; // PA6 W46 <> W46

    engineConfiguration->tps1_1AdcChannel = EFI_ADC_12;

    engineConfiguration->clt.adcChannel =  PROTEUS_IN_ANALOG_TEMP_1;
    engineConfiguration->iat.adcChannel = PROTEUS_IN_ANALOG_TEMP_3;

    engineConfiguration->fuelPumpPin = PROTEUS_LS_6;

    engineConfiguration->idle.solenoidPin = PROTEUS_LS_7;


    engineConfiguration->fanPin = GPIOB_7;

	engineConfiguration->mainRelayPin = GPIOG_12;
#endif // EFI_PROD_CODE


}
#endif // HW_PROTEUS

#if HW_HELLEN
void setHellenNB1() {
	setMazdaMiataEngineNB1Defaults();

	engineConfiguration->injector.flow = 256;
}

void setMiataNB2_Hellen72() {
    setMazdaMiataEngineNB2Defaults();
	strcpy(engineConfiguration->vehicleName, "H72 test");


	// set tps_min 90
	engineConfiguration->tpsMin = 110; // convert 12to10 bit (ADC/4)

}

void setMiataNB2_Hellen72_36() {
	setMiataNB2_Hellen72();

	engineConfiguration->trigger.type = TT_TOOTHED_WHEEL_36_1;
	engineConfiguration->globalTriggerAngleOffset = 76;
}

#endif // HW_HELLEN

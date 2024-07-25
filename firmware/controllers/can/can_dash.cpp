/**
 * @file	can_dash.cpp
 *
 * This file handles transmission of ECU data to various OE dashboards.
 *
 * @date Mar 19, 2020
 * @author Matthew Kennedy, (c) 2020
 */

#include "pch.h"

// #if EFI_CAN_SUPPORT
#include "can_dash.h"
#include "can_msg_tx.h"
#include "can_bmw.h"
#include "can_vag.h"

#include "malfunction_central.h"

// ----------------------------------NMEA2000----------------------------------------
#define USE_N2K_CAN USE_N2K_ESP32_CAN
#include <NMEA2000_CAN.h>  // This will automatically choose right CAN library and create suitable NMEA2000 object
#include <N2kMessages.h>

// List here messages your device will transmit.
const unsigned long TransmitMessages[] PROGMEM={130310L,130311L,130312L,0};

// Define schedulers for messages. Define schedulers here disabled. Schedulers will be enabled
// on OnN2kOpen so they will be synchronized with system.
// We use own scheduler for each message so that each can have different offset and period.
// Setup periods according PGN definition (see comments on IsDefaultSingleFrameMessage and
// IsDefaultFastPacketMessage) and message first start offsets. Use a bit different offset for
// each message so they will not be sent at same time.
// tN2kSyncScheduler TemperatureScheduler(false,2000,500);
// tN2kSyncScheduler EnvironmentalScheduler(false,500,510);
// tN2kSyncScheduler OutsideEnvironmentalScheduler(false,500,520);
// ----------------------------------------------------------------------------------

#include "rusefi_types.h"
#include "rtc_helper.h"
#include "fuel_math.h"

// CAN Bus ID for broadcast
#define CAN_FIAT_MOTOR_INFO           0x561
#define CAN_MAZDA_RX_RPM_SPEED        0x201
#define CAN_MAZDA_RX_STEERING_WARNING 0x300
#define CAN_MAZDA_RX_STATUS_1         0x212
#define CAN_MAZDA_RX_STATUS_2         0x420

//w202 DASH
#define W202_STAT_1	     0x308 /* _20ms cycle */
#define W202_STAT_2      0x608 /* _100ms cycle */
#define W202_ALIVE	     0x210 /* _200ms cycle */
#define W202_STAT_3      0x310 /* _200ms cycle */

//BMW E90 DASH
#define E90_ABS_COUNTER      0x0C0
#define E90_SEATBELT_COUNTER 0x0D7
#define E90_T15	             0x130
#define E90_RPM              0x175
#define E90_BRAKE_COUNTER    0x19E
#define E90_SPEED            0x1A6
#define E90_TEMP             0x1D0
#define E90_GEAR             0x1D2
#define E90_FUEL             0x349
#define E90_EBRAKE           0x34F
#define E90_TIME             0x39E

#define HONDA_SPEED_158 0x158
#define HONDA_TACH_1DC 0x1DC

static time_msecs_t mph_timer;
static time_msecs_t mph_ctr;

/**
 * https://docs.google.com/spreadsheets/d/1IkP05ODpjNt-k4YQLYl58_TNlN9U4IBu5z7i0BPVEM4
 */
#define GENESIS_COUPLE_RPM_316 0x316
#define GENESIS_COUPLE_COOLANT_329 0x329
#define GENESIS_COUPLE_SENSORS_382 0x382
// when A/C compressor is allowed to be on, these values need to be sent so the A/C panel activates the compressor
#define GENESIS_COUPLE_AC_ENABLE_18F 0x18F

//https://www.drive2.ru/b/500679938089681452/
#define NISSAN_STEERING_WHEEL 0x002

// 505
#define NISSAN_RPM_1F9 0x1F9

// 561
#define NISSAN_ENGINE_2 0x231
// 563
#define NISSAN_UNKNOWN_2 0x233
// Nissan z33 350Z and else
// 0x23d = 573
#define NISSAN_RPM_CLT       0x23D
// 574
#define NISSAN_UNKNOWN_3 0x23E

#define NISSAN_TCU_1 0x251
#define NISSAN_TCU_2 0x253

// 640
#define NISSAN_VEHICLE_SPEED_280 0x280
// wheel speed see "102 CAN Communication decoded"
// 19500 value would be 100 kph
// 644
#define NISSAN_WHEEL_SPEED1 0x284
// 645
#define NISSAN_WHEEL_SPEED2 0x285

// 670
#define NISSAN_UNKNOWN_4 0x29E

#define NISSAN_ABS 0x2A0

// 833 doors
#define NISSAN_BCM 0x341

// https://www.drive2.com/l/530057789272229668/
// 852
#define NISSAN_VEHICLE_SPEED 0x354

// 1361
#define NISSAN_CLT_551 0x551
// 1408
#define NISSAN_RPM_AGAIN 0x580
#define NISSAN_ODOMETER 0x5C5
// 1549
#define NISSAN_BCM_2 0x60D

static uint8_t rpmcounter;
static uint8_t seatbeltcnt;
static uint8_t abscounter = 0xF0;
static uint8_t brakecnt_1 = 0xF0, brakecnt_2 = 0xF0;
static uint8_t mph_a, mph_2a, mph_last, tmp_cnt, gear_cnt;
static uint16_t mph_counter = 0xF000;
static bool cluster_time_set;

constexpr uint8_t e90_temp_offset = 49;

// todo: those forward declarations are out of overall code style
void canDashboardBMW(CanCycle cycle);
void canDashboardFiat(CanCycle cycle);
void canMazdaRX8(CanCycle cycle);
void canDashboardW202(CanCycle cycle);
void canDashboardBMWE90(CanCycle cycle);
void canDashboardVagMqb(CanCycle cycle);
void canDashboardNissanVQ(CanCycle cycle);
void canDashboardGenesisCoupe(CanCycle cycle);
void canDashboardAim(CanCycle cycle);
void canDashboardHaltech(CanCycle cycle);
void canDashboardNMEA2000(CanCycle cycle);

void updateDash(CanCycle cycle) {

	// Transmit dash data, if enabled
	switch (engineConfiguration->canNbcType) {
	case CAN_BUS_NBC_NONE:
		break;
	case CAN_BUS_NBC_BMW:
		canDashboardBMW(cycle);
		break;
	case CAN_BUS_Haltech:
		canDashboardHaltech(cycle);
		break;
	case CAN_BUS_NBC_FIAT:
		canDashboardFiat(cycle);
		break;
	case CAN_BUS_NBC_VAG:
		canDashboardVAG(cycle);
		break;
	case CAN_BUS_MAZDA_RX8:
		canMazdaRX8(cycle);
		break;
	case CAN_BUS_W202_C180:
		canDashboardW202(cycle);
		break;
	case CAN_BUS_BMW_E90:
		canDashboardBMWE90(cycle);
		break;
	case CAN_BUS_MQB:
		canDashboardVagMqb(cycle);
		break;
	case CAN_BUS_NISSAN_VQ:
		canDashboardNissanVQ(cycle);
		break;
	case CAN_BUS_GENESIS_COUPE:
		canDashboardGenesisCoupe(cycle);
		break;
	case CAN_AIM_DASH:
		canDashboardAim(cycle);
		break;
	case CAN_BUS_NBC_NMEA2K:
		canDashboardNMEA2000(cycle);
		break;
	default:
		firmwareError(OBD_PCM_Processor_Fault, "Nothing for canNbcType %s", getCan_nbc_e(engineConfiguration->canNbcType));
		break;
	}
}

//BMW Dashboard
//todo: we use 50ms fixed cycle, trace is needed to check for correct period
void canDashboardBMW(CanCycle cycle) {
	
	if (cycle.isInterval(CI::_50ms)) {
		{
			CanTxMessage msg(CanCategory::NBC, CAN_BMW_E46_SPEED);
			msg.setShortValue(10 * 8, 1);
		}

		{
			CanTxMessage msg(CanCategory::NBC, CAN_BMW_E46_RPM);
			msg.setShortValue((int) (Sensor::getOrZero(SensorType::Rpm) * 6.4), 2);
		}

		{
			CanTxMessage msg(CanCategory::NBC, CAN_BMW_E46_DME2);
			msg.setShortValue((int) ((Sensor::getOrZero(SensorType::Clt) + 48.373) / 0.75), 1);
		}
	}
}

//todo: we use 50ms fixed cycle, trace is needed to check for correct period
void canMazdaRX8(CanCycle cycle) {
	if (cycle.isInterval(CI::_50ms)) {
		{
			CanTxMessage msg(CanCategory::NBC, CAN_MAZDA_RX_STEERING_WARNING);
			// todo: something needs to be set here? see http://rusefi.com/wiki/index.php?title=Vehicle:Mazda_Rx8_2004
		}

		{
			CanTxMessage msg(CanCategory::NBC, CAN_MAZDA_RX_RPM_SPEED);

			float kph = Sensor::getOrZero(SensorType::VehicleSpeed);

			msg.setShortValue(SWAP_UINT16(Sensor::getOrZero(SensorType::Rpm) * 4), 0);
			msg.setShortValue(0xFFFF, 2);
			msg.setShortValue(SWAP_UINT16((int )(100 * kph + 10000)), 4);
			msg.setShortValue(0, 6);
		}

		{
			CanTxMessage msg(CanCategory::NBC, CAN_MAZDA_RX_STATUS_1);
			msg[0] = 0xFE; //Unknown
			msg[1] = 0xFE; //Unknown
			msg[2] = 0xFE; //Unknown
			msg[3] = 0x34; //DSC OFF in combo with byte 5 Live data only seen 0x34
			msg[4] = 0x00; // B01000000; // Brake warning B00001000;  //ABS warning
			msg[5] = 0x40; // TCS in combo with byte 3
			msg[6] = 0x00; // Unknown
			msg[7] = 0x00; // Unused
		}

		{
			CanTxMessage msg(CanCategory::NBC, CAN_MAZDA_RX_STATUS_2);
			auto clt = Sensor::get(SensorType::Clt);
			msg[0] = (uint8_t)(clt.value_or(0) + 69); //temp gauge //~170 is red, ~165 last bar, 152 centre, 90 first bar, 92 second bar
			// TODO: fixme!
			//msg[1] = ((int16_t)(engine->engineState.vssEventCounter*(engineConfiguration->vehicleSpeedCoef*0.277*2.58))) & 0xff;
			msg[2] = 0x00; // unknown
			msg[3] = 0x00; //unknown
			msg[4] = 0x01; //Oil Pressure (not really a gauge)
			msg[5] = 0x00; //check engine light
			msg[6] = 0x00; //Coolant, oil and battery
			if ((Sensor::getOrZero(SensorType::Rpm)>0) && (Sensor::get(SensorType::BatteryVoltage).value_or(VBAT_FALLBACK_VALUE)<13)) {
				msg.setBit(6, 6); // battery light
			}
			if (!clt.Valid || clt.Value > 105) {
				// coolant light, 101 - red zone, light means its get too hot
				// Also turn on the light in case of sensor failure
				msg.setBit(6, 1);
			}
			//oil pressure warning lamp bit is 7
			msg[7] = 0x00; //unused
		}
	}

}

void canDashboardFiat(CanCycle cycle) {
	if (cycle.isInterval(CI::_50ms)) {
		{
			//Fiat Dashboard
			CanTxMessage msg(CanCategory::NBC, CAN_FIAT_MOTOR_INFO);
			msg.setShortValue((int) (Sensor::getOrZero(SensorType::Clt) - 40), 3); //Coolant Temp
			msg.setShortValue(Sensor::getOrZero(SensorType::Rpm) / 32, 6); //RPM
		}
	}
}

void canDashboardVAG(CanCycle cycle) {
	if (cycle.isInterval(CI::_10ms)) {
		{
			// https://github.com/commaai/opendbc/blob/57c8340a180dd8c75139b18050eb17c72c9cb6e4/vw_golf_mk4.dbc#L394
			//VAG Dashboard
			CanTxMessage msg(CanCategory::NBC, CAN_VAG_Motor_1);
			msg.setShortValue(Sensor::getOrZero(SensorType::Rpm) * 4, 2); //RPM
		}

		float clt = Sensor::getOrZero(SensorType::Clt);

		{
			CanTxMessage msg(CanCategory::NBC, CAN_VAG_Motor_2);
			msg.setShortValue((int) ((clt + 48.373) / 0.75), 1); //Coolant Temp
		}

		{
			CanTxMessage msg(CanCategory::NBC, CAN_VAG_CLT_V2);
			msg.setShortValue((int) ((clt + 48.373) / 0.75), 4); //Coolant Temp
		}

		{
			CanTxMessage msg(CanCategory::NBC, CAN_VAG_IMMO);
			msg.setShortValue(0x80, 1);
		}
	}
}

void canDashboardW202(CanCycle cycle) {
	if (cycle.isInterval(CI::_20ms)) {
		{
			CanTxMessage msg(CanCategory::NBC, W202_STAT_1);
			uint16_t tmp = Sensor::getOrZero(SensorType::Rpm);
			msg[0] = 0x08; // Unknown
			msg[1] = (tmp >> 8); //RPM
			msg[2] = (tmp & 0xff); //RPM
			msg[3] = 0x00; // 0x01 - tank blink, 0x02 - EPC
			msg[4] = 0x00; // Unknown
			msg[5] = 0x00; // Unknown
			msg[6] = 0x00; // Unknown - oil info
			msg[7] = 0x00; // Unknown - oil info
		}
	}

	if (cycle.isInterval(CI::_100ms)) {
		{
			CanTxMessage msg(CanCategory::NBC, W202_STAT_2); //dlc 7
			msg[0] = (int)(Sensor::getOrZero(SensorType::Clt) + 40); // CLT -40 offset
			msg[1] = 0x3D; // TBD
			msg[2] = 0x63; // Const
			msg[3] = 0x41; // Const
			msg[4] = 0x00; // Unknown
			msg[5] = 0x05; // Const
			msg[6] = 0x50; // TBD
			msg[7] = 0x00; // Unknown
		}
	}

	if (cycle.isInterval(CI::_200ms)) {
		{
			CanTxMessage msg(CanCategory::NBC, W202_ALIVE);
			msg[0] = 0x0A; // Const
			msg[1] = 0x18; // Const
			msg[2] = 0x00; // Const
			msg[3] = 0x00; // Const
			msg[4] = 0xC0; // Const
			msg[5] = 0x00; // Const
			msg[6] = 0x00; // Const
			msg[7] = 0x00; // Const
		}	

		{
			CanTxMessage msg(CanCategory::NBC, W202_STAT_3);
			msg[0] = 0x00; // Const
			msg[1] = 0x00; // Const
			msg[2] = 0x6D; // TBD
			msg[3] = 0x7B; // Const
			msg[4] = 0x21; // TBD
			msg[5] = 0x07; // Const
			msg[6] = 0x33; // Const
			msg[7] = 0x05; // Const
		}
	}
}

static int rollingId = 0;

void canDashboardGenesisCoupe(CanCycle cycle) {
	if (cycle.isInterval(CI::_50ms)) {
		{
			CanTxMessage msg(CanCategory::NBC, GENESIS_COUPLE_RPM_316, 8);
			int rpm8 = Sensor::getOrZero(SensorType::Rpm) * 4;
			msg[3] = rpm8 >> 8;
			msg[4] = rpm8 & 0xFF;
		}
		{
			CanTxMessage msg(CanCategory::NBC, GENESIS_COUPLE_COOLANT_329, 8);
			int clt = Sensor::getOrZero(SensorType::Clt) * 2;
			msg[1] = clt;
		}
	}
}

void canDashboardNissanVQ(CanCycle cycle) {
	if (cycle.isInterval(CI::_50ms)) {
		{
			CanTxMessage msg(CanCategory::NBC, NISSAN_RPM_1F9, 8);
			msg[0] = 0x20;
			int rpm8 = (int)(Sensor::getOrZero(SensorType::Rpm) * 8);
			msg[2] = rpm8 >> 8;
			msg[3] = rpm8 & 0xFF;
		}

		{
			CanTxMessage msg(CanCategory::OBD, NISSAN_CLT_551, 8);

			int clt = Sensor::getOrZero(SensorType::Clt);
			msg[0] = clt + 45;
		}


		{
			CanTxMessage msg(CanCategory::NBC, NISSAN_RPM_CLT, 8);

			rollingId = (rollingId + 1) % 4;
			const uint8_t magicByte[4] = {0x03, 0x23, 0x42, 0x63};

			msg[0] = magicByte[rollingId];
			msg[1] = (int)(Sensor::getOrZero(SensorType::AcceleratorPedal) * 255 / 100);

			// thank you "102 CAN Communication decoded"
#define CAN_23D_RPM_MULT 3.15
			int rpm315 = (int)(Sensor::getOrZero(SensorType::Rpm) / CAN_23D_RPM_MULT);
			msg[3] = rpm315 & 0xFF;
			msg[4] = rpm315 >> 8;

			msg[7] = 0x70; // todo: CLT decoding?
		}
	}
}

/**
 * https://docs.google.com/spreadsheets/d/1XMfeGlhgl0lBL54lNtPdmmFd8gLr2T_YTriokb30kJg
 */
void canDashboardVagMqb(CanCycle cycle) {
	if (cycle.isInterval(CI::_50ms)) {

		{ // 'turn-on'
			CanTxMessage msg(CanCategory::NBC, 0x3C0, 4);
			// ignition ON
			msg[2] = 3;
		}
	
		{ //RPM
			CanTxMessage msg(CanCategory::NBC, 0x107, 8);
			msg[3] = ((int)(Sensor::getOrZero(SensorType::Rpm) / 3.5)) & 0xFF;
			msg[4] = ((int)(Sensor::getOrZero(SensorType::Rpm) / 3.5)) >> 8;
		}
	}
}

void canDashboardBMWE90(CanCycle cycle)
{

	if (cycle.isInterval(CI::_50ms)) {
		
		{ //T15 'turn-on'
			CanTxMessage msg(CanCategory::NBC, E90_T15, 5);
			msg[0] = 0x45;
			msg[1] = 0x41;
			msg[2] = 0x61;
			msg[3] = 0x8F;
			msg[4] = 0xFC;
		}

		{ //Ebrake light
			CanTxMessage msg(CanCategory::OBD, E90_EBRAKE, 2);
			msg[0] = 0xFD;
			msg[1] = 0xFF;
		}

		{ //RPM
			rpmcounter++;
			if (rpmcounter > 0xFE)
				rpmcounter = 0xF0;
			CanTxMessage msg(CanCategory::OBD, E90_RPM, 3);
			msg[0] = rpmcounter;
			msg[1] = ((int)(Sensor::getOrZero(SensorType::Rpm)) * 4) & 0xFF;
			msg[2] = ((int)(Sensor::getOrZero(SensorType::Rpm)) * 4) >> 8;
		}

		{ //oil & coolant temp (all in C, despite gauge being F)
			tmp_cnt++;
			if (tmp_cnt >= 0x0F)
				tmp_cnt = 0x00;
			CanTxMessage msg(CanCategory::OBD, E90_TEMP, 8);
			msg[0] = (int)(Sensor::getOrZero(SensorType::Clt) + e90_temp_offset); //coolant
			msg[1] = (int)(Sensor::getOrZero(SensorType::AuxTemp1) + e90_temp_offset); //oil (AuxTemp1)
			msg[2] = tmp_cnt;
			msg[3] = 0xC8;
			msg[4] = 0xA7;
			msg[5] = 0xD3;
			msg[6] = 0x0D;
			msg[7] = 0xA8;
		}
	}

	if (cycle.isInterval(CI::_100ms)) {
		{
			//Seatbelt counter
			seatbeltcnt++;
			if (seatbeltcnt > 0xFE)
				seatbeltcnt = 0x00;
			CanTxMessage msg(CanCategory::NBC, E90_SEATBELT_COUNTER, 2);
			msg[0] = seatbeltcnt;
			msg[1] = 0xFF;
		}
	
		{
			//Brake counter 100ms
			brakecnt_1 += 16;
			brakecnt_2 += 16;
			if (brakecnt_1 > 0xEF)
				brakecnt_1 = 0x0F;
			if (brakecnt_2 > 0xF0)
				brakecnt_2 = 0xA0;
			CanTxMessage msg(CanCategory::NBC, E90_BRAKE_COUNTER, 8);
			msg[0] = 0x00;
			msg[1] = 0xE0;
			msg[2] = brakecnt_1;
			msg[3] = 0xFC;
			msg[4] = 0xFE;
			msg[5] = 0x41;
			msg[6] = 0x00;
			msg[7] = brakecnt_2;
		}

		{ //ABS counter
			abscounter++;
			if (abscounter > 0xFE)
				abscounter = 0xF0;
			CanTxMessage msg(CanCategory::NBC, E90_ABS_COUNTER, 2);
			msg[0] = abscounter;
			msg[1] = 0xFF;
		}

		{ //Fuel gauge
			CanTxMessage msg(CanCategory::NBC, E90_FUEL, 5); //fuel gauge
			msg[0] = 0x76;
			msg[1] = 0x0F;
			msg[2] = 0xBE;
			msg[3] = 0x1A;
			msg[4] = 0x00;
		}

		{ //Gear indicator/counter
			gear_cnt++;
			if (gear_cnt >= 0x0F)
				gear_cnt = 0x00;
			CanTxMessage msg(CanCategory::NBC, E90_GEAR, 6);
			msg[0] = 0x78;
			msg[1] = 0x0F;
			msg[2] = 0xFF;
			msg[3] = (gear_cnt << 4) | 0xC;
			msg[4] = 0xF1;
			msg[5] = 0xFF;
		}

		{ //E90_SPEED
			auto vehicleSpeed = Sensor::getOrZero(SensorType::VehicleSpeed); 
			float mph = vehicleSpeed * 0.6213712;
			mph_ctr = ((TIME_I2MS(chVTGetSystemTime()) - mph_timer) / 50);
			mph_a = (mph_ctr * mph / 2);
			mph_2a = mph_a + mph_last;
			mph_last = mph_2a;
			mph_counter += mph_ctr * 100;
			if(mph_counter >= 0xFFF0)
				mph_counter = 0xF000;
			mph_timer = TIME_I2MS(chVTGetSystemTime());
			CanTxMessage msg(CanCategory::NBC, E90_SPEED, 8);
			msg[0] = mph_2a & 0xFF;
			msg[1] = mph_2a >> 8;
			msg[2] = mph_2a & 0xFF;
			msg[3] = mph_2a >> 8;
			msg[4] = mph_2a & 0xFF;
			msg[5] = mph_2a >> 8;
			msg[6] = mph_counter & 0xFF;
			msg[7] = (mph_counter >> 8) | 0xF0;
		}
	}

	{
		if (!cluster_time_set) {
#if EFI_RTC
			efidatetime_t dateTime = getRtcDateTime();
#else // EFI_RTC
			efidatetime_t dateTime = {
				.year = 0, .month = 0, .day = 0,
				.hour = 0, .minute = 0, .second = 0,
			};
#endif // EFI_RTC
			CanTxMessage msg(CanCategory::NBC, E90_TIME, 8);
			msg[0] = dateTime.hour;
			msg[1] = dateTime.minute;
			msg[2] = dateTime.second;
			msg[3] = dateTime.day;
			msg[4] = (dateTime.month << 4) | 0x0F;
			msg[5] = dateTime.year & 0xFF;
			msg[6] = (dateTime.year >> 8) | 0xF0; // collides CAN dash at 4096!
			msg[7] = 0xF2;
			cluster_time_set = 1;
		}
	}
}

void canDashboardHaltech(CanCycle cycle) {
	
	uint16_t tmp;

	if (cycle.isInterval(CI::_20ms)) {
		/* 0x360 - 50Hz rate */
		{
			CanTxMessage msg(CanCategory::NBC, 0x360, 8);
			tmp = Sensor::getOrZero(SensorType::Rpm);
			/* RPM */
			msg[0] = (tmp >> 8);
			msg[1] = (tmp & 0x00ff);
			/* MAP */
			tmp = (((uint16_t)(Sensor::getOrZero(SensorType::Map))) * 10); 
			msg[2] = (tmp >> 8);
			msg[3] = (tmp & 0x00ff);
			/* TPS  y = x/10 */
			tmp = (uint16_t)((float)(Sensor::getOrZero(SensorType::Tps1)) * 10);
			msg[4] = (tmp >> 8);
			msg[5] = (tmp & 0x00ff);
			/* Coolant pressure */
			msg[6] = 0;
			msg[7] = 0;
		}

		/* 0x361 - 50Hz rate */
		{ 
			CanTxMessage msg(CanCategory::NBC, 0x361, 8);
			/* Fuel pressure */
			tmp =  (uint16_t)((Sensor::getOrZero(SensorType::FuelPressureLow) + 101.3) * 10);
			msg[0] = (tmp >> 8);
			msg[1] = (tmp&0x00ff);
			/* Oil pressure */
			tmp =  (uint16_t)((Sensor::getOrZero(SensorType::OilPressure) + 101.3) * 10);
			msg[2] = (tmp >> 8);
			msg[3] = (tmp & 0x00ff);
			/* Engine Demand */
			tmp =  (uint16_t)(Sensor::getOrZero(SensorType::Map));
			msg[4] = (tmp >> 8);
			msg[5] = (tmp & 0x00ff);
			/* Wastegate Pressure */
			msg[6] = 0;
			msg[7] = 0;			
		}

		/* 0x362 - 50Hz rate */
		{ 
			CanTxMessage msg(CanCategory::NBC, 0x362, 6);
			/* Injection Stage 1 Duty Cycle - y = x/10 */
			uint16_t rpm = Sensor::getOrZero(SensorType::Rpm);
			tmp = (uint16_t)( getInjectorDutyCycle(rpm) * 10) ;
			msg[0] = (tmp >> 8);
			msg[1] = (tmp & 0x00ff);
			/* Injcetion Stage 2 Duty Cycle */
			msg[2] = 0x00;
			msg[3] = 0x00;
			/* Ignition Angle (Leading) - y = x/10 */
			float timing = engine->engineState.timingAdvance[0];
			int16_t ignAngle = ((timing > 360 ? timing - 720 : timing) * 10);
			msg[4] = (ignAngle >> 8);			
			msg[5] = (ignAngle & 0x00ff);
		}

		/* todo: 0x3E5 = 50Hz rate */
		{ 
			CanTxMessage msg(CanCategory::NBC, 0x3E5, 8);
			msg[0] = 0x00; 
			msg[1] = 0x00;
			msg[2] = 0x00;
			msg[3] = 0x00;
			msg[4] = 0x00;
			msg[5] = 0x00;
			msg[6] = 0x00;
			msg[7] = 0x00;
		}

		/* todo: 0x3EA = 50Hz rate */
		{ 
			CanTxMessage msg(CanCategory::NBC, 0x3EA, 8);
			msg[0] = 0x00; 
			msg[1] = 0x00;
			msg[2] = 0x00;
			msg[3] = 0x00;
			msg[4] = 0x00;
			msg[5] = 0x00;
			msg[6] = 0x00;
			msg[7] = 0x00;
		}

		/* todo: 0x3EB = 50Hz rate */
		{ 
			CanTxMessage msg(CanCategory::NBC, 0x3EB, 8);
			msg[0] = 0x00; 
			msg[1] = 0x00;
			msg[2] = 0x00;
			msg[3] = 0x00;
			msg[4] = 0x00;
			msg[5] = 0x00;
			msg[6] = 0x00;
			msg[7] = 0x00;
		}

		/* todo: 0x3EC = 50Hz rate */
		{ 
			CanTxMessage msg(CanCategory::NBC, 0x3EC, 8);
			msg[0] = 0x00; 
			msg[1] = 0x00;
			msg[2] = 0x00;
			msg[3] = 0x00;
			msg[4] = 0x00;
			msg[5] = 0x00;
			msg[6] = 0x00;
			msg[7] = 0x00;
		}

		/* todo: 0x3ED = 50Hz rate */
		{ 
			CanTxMessage msg(CanCategory::NBC, 0x3ED, 2);
			msg[0] = 0x00; 
			msg[1] = 0x00;
		}

		/* todo: 0x471 = 50Hz rate */
		{ 
			CanTxMessage msg(CanCategory::NBC, 0x471, 2);
			msg[0] = 0x00; 
			msg[1] = 0x00;
			msg[2] = 0x00;
			msg[3] = 0x00;
		}
	}

	if (cycle.isInterval(CI::_50ms)) {
	
		/* 0x363 - 20Hz rate */
		{ 
			CanTxMessage msg(CanCategory::NBC, 0x363, 4);
			/* Wheel Slip */
			msg[0] = 0x00;
			msg[1] = 0x00;
			/* Wheel Diff */
			msg[2] = 0x00;
			msg[3] = 0x00 ;
		}

		/* 0x368 - 20Hz rate */
		{ 
			CanTxMessage msg(CanCategory::NBC, 0x368, 8);
			/* Wideband Sensor 1 */
			tmp = (uint16_t)(Sensor::getOrZero(SensorType::Lambda1) * 1000 * 14.7);
			msg[0] = (tmp >> 8);
			msg[1] = (tmp & 0x00ff);
			/* Wideband Sensor 2 */
			tmp =  (uint16_t)(Sensor::getOrZero(SensorType::Lambda2) * 1000);
			msg[2] = (tmp >> 8);
			msg[3] = (tmp & 0x00ff);
			/* Wideband Sensor 3 */
			msg[4] = 0x00;
			msg[5] = 0x00;
			/* Wideband Sensor 4 */			
			msg[6] = 0x00;
			msg[7] = 0x00;
		}

#if EFI_SHAFT_POSITION_INPUT
		/* 0x369 - 20Hz rate */
		{ 
			CanTxMessage msg(CanCategory::NBC, 0x369, 8);
			/* Trigger System Error Count */
			tmp = engine->triggerCentral.triggerState.totalTriggerErrorCounter;
			msg[0] = (tmp >> 8);
			msg[1] = (tmp & 0x00ff);
			/* Trigger Counter ?? */
			tmp =  engine->triggerCentral.getHwEventCounter((int)SHAFT_PRIMARY_FALLING);
			msg[2] = (tmp >> 8);
			msg[3] = (tmp & 0x00ff);
			/* unused */
			msg[4] = 0x00;
			msg[5] = 0x00;
			/* Trigger Sync Level ?? */
			msg[6] = 0x00;			
			msg[7] = 0x00;
		}
#endif // EFI_SHAFT_POSITION_INPUT

		/* 0x36A - 20Hz rate */
		/* todo: one day we should split this */
		{ 
			CanTxMessage msg(CanCategory::NBC, 0x36A, 4);
			/* Knock Level 1 */
			tmp = (engine->outputChannels.knockLevel * 100);
			msg[0] = (tmp >> 8);
			msg[1] = (tmp & 0x00ff);
			/* Knock Level 2 */
			msg[2] = (tmp >> 8);
			msg[3] = (tmp * 0x00ff);
		}

		/* 0x36B - 20Hz rate */
		{ 
			CanTxMessage msg(CanCategory::NBC, 0x36B, 8);
			/* Break Pressure */
			msg[0] = 0x00;
			msg[1] = 0x00;
			/* NOS pressure Sensor 1 */
			msg[2] = 0x00;
			msg[3] = 0x00;
			/* Turbo Speed Sensor 1 */
			msg[4] = 0x00;
			msg[5] = 0x00;
			/* Lateral G */
			msg[6] = 0x00;
			msg[7] = 0x00;
		}

		/* 0x36C = 20Hz rate */
		{ 
			CanTxMessage msg(CanCategory::NBC, 0x36C, 8);
			/* Wheel Speed Front Left */
			auto vehicleSpeed = Sensor::getOrZero(SensorType::VehicleSpeed);
			tmp = (vehicleSpeed * 10 );
			msg[0] = (tmp >> 8);
			msg[1] = (tmp & 0x00ff);
			/* Wheel Speed Front Right */
			msg[2] = (tmp >> 8);
			msg[3] = (tmp & 0x00ff);
			/* Wheel Speed Read Left */
			msg[4] = (tmp >> 8);
			msg[5] = (tmp & 0x00ff);
			/* Wheel Speed Read Right */
			msg[6] = (tmp >> 8);
			msg[7] = (tmp & 0x00ff);
		}

		/* 0x36D = 20Hz rate */
		{ 
			CanTxMessage msg(CanCategory::NBC, 0x36D, 8);
			/* Unused */
			msg[0] = 0x00;
			msg[1] = 0x00;
			msg[2] = 0x00;
			msg[3] = 0x00;
			/* Exhaust Cam Angle 1 */
			msg[4] = 0x00;
			msg[5] = 0x00;
			/* Exhaust Cam Angle 2 */
			msg[6] = 0x00;
			msg[7] = 0x00;
		}	

		/* 0x36E = 20Hz rate */
		{ 
			CanTxMessage msg(CanCategory::NBC, 0x36E, 8);
			/* Engine Limiting Active 0 = off/1=on*/
			msg[0] = 0x00;
			msg[1] = 0x00;
			/* Launch Control Ignition Retard */
			msg[2] = 0x00;
			msg[3] = 0x00;
			/* Launch Control Fuel Enrich */
			msg[4] = 0x00;
			msg[5] = 0x00;
			/* Longitudinal G */
			msg[6] = 0x00;
			msg[7] = 0x00;
		}

		/* 0x36F = 20Hz rate */
		{ 
			CanTxMessage msg(CanCategory::NBC, 0x36F, 4);
			/* Generic Output 1 Duty Cycle */
			msg[0] = 0x00;
			msg[1] = 0x00;
			/* Boost Control Output */
			msg[2] = 0x00;
			msg[3] = 0x00;
		}

		/* 0x370 = 20Hz rate */
		{ 
			CanTxMessage msg(CanCategory::NBC, 0x370, 8);
			/* Vehicle Speed */
			auto vehicleSpeed = Sensor::getOrZero(SensorType::VehicleSpeed);
			tmp = (vehicleSpeed * 10 );
			msg[0] = (tmp >> 8);
			msg[1] = (tmp & 0x00ff);
			/* unused */
			msg[2] = 0x00;
			msg[3] = 0x00;
			/* Intake Cam Angle 1 */
			msg[4] = 0x00;
			msg[5] = 0x00;
			/* Intake Cam Angle 2 */
			msg[6] = 0x00;
			msg[7] = 0x00;
		}

		/* todo: 0x3E6 = 20Hz rate */
		{ 
			CanTxMessage msg(CanCategory::NBC, 0x3E6, 8);
			msg[0] = 0x00; 
			msg[1] = 0x00;
			msg[2] = 0x00;
			msg[3] = 0x00;
			msg[4] = 0x00;
			msg[5] = 0x00;
			msg[6] = 0x00;
			msg[7] = 0x00;
		}

		/* todo: 0x3E7 = 20Hz rate */
		{ 
			CanTxMessage msg(CanCategory::NBC, 0x3E7, 8);
			msg[0] = 0x00; 
			msg[1] = 0x00;
			msg[2] = 0x00;
			msg[3] = 0x00;
			msg[4] = 0x00;
			msg[5] = 0x00;
			msg[6] = 0x00;
			msg[7] = 0x00;
		}
	
		/* todo: 0x3E8 = 20Hz rate */
		{ 
			CanTxMessage msg(CanCategory::NBC, 0x3E8, 8);
			msg[0] = 0x00; 
			msg[1] = 0x00;
			msg[2] = 0x00;
			msg[3] = 0x00;
			msg[4] = 0x00;
			msg[5] = 0x00;
			msg[6] = 0x00;
			msg[7] = 0x00;
		}

		/* todo: 0x3E9 = 20Hz rate */
		{ 
			CanTxMessage msg(CanCategory::NBC, 0x3E9, 8);
			msg[0] = 0x00; 
			msg[1] = 0x00;
			msg[2] = 0x00;
			msg[3] = 0x00;
			msg[4] = 0x00;
			msg[5] = 0x00;
			msg[6] = 0x00;
			msg[7] = 0x00;
		}

		/* todo: 0x3EE = 20Hz rate */
		{ 
			CanTxMessage msg(CanCategory::NBC, 0x3EE, 8);
			msg[0] = 0x00; 
			msg[1] = 0x00;
			msg[2] = 0x00;
			msg[3] = 0x00;
			msg[4] = 0x00;
			msg[5] = 0x00;
			msg[6] = 0x00;
			msg[7] = 0x00;
		}

		/* todo: 0x3EF = 20Hz rate */
		{ 
			CanTxMessage msg(CanCategory::NBC, 0x3EF, 8);
			msg[0] = 0x00; 
			msg[1] = 0x00;
			msg[2] = 0x00;
			msg[3] = 0x00;
			msg[4] = 0x00;
			msg[5] = 0x00;
			msg[6] = 0x00;
			msg[7] = 0x00;
		}

		/* todo: 0x470 = 20Hz rate */
		{ 
			CanTxMessage msg(CanCategory::NBC, 0x470, 8);
			msg[0] = 0x00; 
			msg[1] = 0x00;
			msg[2] = 0x00;
			msg[3] = 0x00;
			msg[4] = 0x00;
			msg[5] = 0x00;
			msg[6] = 0x00;
			msg[7] = 0x00;
		}

		/* todo: 0x472 = 20Hz rate */
		{ 
			CanTxMessage msg(CanCategory::NBC, 0x472, 8);
			msg[0] = 0x00; 
			msg[1] = 0x00;
			msg[2] = 0x00;
			msg[3] = 0x00;
			msg[4] = 0x00;
			msg[5] = 0x00;
			msg[6] = 0x00;
			msg[7] = 0x00;
		}		
	}

	if (cycle.isInterval(CI::_100ms)) {
		
		/* 0x371 = 10Hz rate */
		{ 
			CanTxMessage msg(CanCategory::NBC, 0x371, 4);
			/* Fuel Flow */
			msg[0] = 0x00;
			msg[1] = 0x00;
			/* Fuel Flow Return */
			msg[2] = 0x00;
			msg[3] = 0x00;
		}

		/* 0x372 = 10Hz rate */
		{ 
			CanTxMessage msg(CanCategory::NBC, 0x372, 8);
			/* Battery Voltage */
			tmp =  (uint16_t)(Sensor::getOrZero(SensorType::BatteryVoltage) * 10);
			msg[0] = (tmp >> 8);
			msg[1] = (tmp & 0x00ff);
			/* unused */
			msg[2] = 0x00;
			msg[3] = 0x00;
			/* Target Boost Level todo */
			msg[4] = 0x00;
			msg[5] = 0x00;
			/* Barometric pressure */
			tmp = (uint16_t)(Sensor::getOrZero(SensorType::BarometricPressure) * 10);
			msg[6] = (tmp >> 8);
			msg[7] = (tmp & 0x00ff);
		}
	
		/* 0x373 = 10Hz rate */
		{ 
			CanTxMessage msg(CanCategory::NBC, 0x373, 8);
			/* EGT1 */
			msg[0] = 0x00;
			msg[1] = 0x00;
			/* EGT2 */
			msg[2] = 0x00;
			msg[3] = 0x00;
			/* EGT3 */
			msg[4] = 0x00;
			msg[5] = 0x00;
			/* EGT4 */
			msg[6] = 0x00;
			msg[7] = 0x00;
		}

		/* 0x374 = 10Hz rate */
		{ 
			CanTxMessage msg(CanCategory::NBC, 0x374, 8);
			/* EGT5 */
			msg[0] = 0x00;
			msg[1] = 0x00;
			/* EGT6 */
			msg[2] = 0x00;
			msg[3] = 0x00;
			/* EGT7 */
			msg[4] = 0x00;
			msg[5] = 0x00;
			/* EGT8 */
			msg[6] = 0x00;
			msg[7] = 0x00;
		}

		/* 0x375 = 10Hz rate */
		{ 
			CanTxMessage msg(CanCategory::NBC, 0x375, 8);
			/* EGT9 */
			msg[0] = 0x00;
			msg[1] = 0x00;
			/* EGT10 */
			msg[2] = 0x00;
			msg[3] = 0x00;
			/* EGT11 */
			msg[4] = 0x00;
			msg[5] = 0x00;
			/* EGT12 */
			msg[6] = 0x00;
			msg[7] = 0x00;
		}

		/* 0x376 = 10Hz rate */
		{ 
			CanTxMessage msg(CanCategory::NBC, 0x376, 8);
			/* Ambient Air Temperature */
			msg[0] = 0x00;
			msg[1] = 0x00;
			/* Relative Humidity */
			msg[2] = 0x00;
			msg[3] = 0x00;
			/* Specific Humidity */
			msg[4] = 0x00;
			msg[5] = 0x00;
			/* Absolute Humidity */
			msg[6] = 0x00;
			msg[7] = 0x00;
		}
	}

	if (cycle.isInterval(CI::_200ms)) {
		/* 0x3E0 = 5Hz rate */
		{ 
			CanTxMessage msg(CanCategory::NBC, 0x3E0, 8);
			/* Coolant temperature in K y = x/10 */
			tmp = ((Sensor::getOrZero(SensorType::Clt) + 273.15) * 10);
			msg[0] = (tmp >> 8);
			msg[1] = (tmp & 0x00ff);
			/* Air Temperature */
			tmp = ((Sensor::getOrZero(SensorType::Iat) + 273.15) * 10);
			msg[2] = (tmp >> 8);
			msg[3] = (tmp & 0x00ff);
			/* Fuel Temperature */
			msg[4] = 0x00;
			msg[5] = 0x00;
			/* Oil Temperature */
			msg[6] = 0x00;
			msg[7] = 0x00;
		}

		/* 0x3E1 = 5Hz rate */
		{ 
			CanTxMessage msg(CanCategory::NBC, 0x3E1, 6);
			/* Gearbox Oil Temperature */
			msg[0] = 0x00;
			msg[1] = 0x00;
			/* Diff oil Temperature */
			msg[2] = 0x00;
			msg[3] = 0x00;
			/* Fuel Composition */
			msg[4] = 0x00;
			msg[5] = 0x00;
		}

		/* 0x3E2 = 5Hz rate */
		{ 
			CanTxMessage msg(CanCategory::NBC, 0x3E2, 2);
			/* Fuel Level in Liters */
			tmp = (Sensor::getOrZero(SensorType::FuelLevel)* 10);
			msg[0] = (tmp >> 8);
			msg[1] = (tmp & 0x00ff);
		}

		/* 0x3E3 = 5Hz rate */
		{ 
			CanTxMessage msg(CanCategory::NBC, 0x3E3, 8);
			/* Fuel Trim Short Term Bank 1*/
			msg[0] = 0x00;
			msg[1] = 0x00;
			/* Fuel Trim Short Term Bank 2*/
			msg[2] = 0x00;
			msg[3] = 0x00;
			/* Fuel Trim Long Term Bank 1*/
			msg[4] = 0x00;
			msg[5] = 0x00;
			/* Fuel Trim Long Term Bank 2*/
			msg[6] = 0x00;
			msg[7] = 0x00;
		}

		/* todo: 0x3E4 = 5Hz rate */
		{ 
			CanTxMessage msg(CanCategory::NBC, 0x3E4, 8);
			msg[0] = 0x00; //unused
			/* Switch status */
			msg[1] = 0x00;
			/* Switch status */
			msg[2] = 0x00;
			msg[3] = 0x00;
			msg[4] = 0x00;
			msg[5] = 0x00;
			msg[6] = 0x00;
			msg[7] = 0x00;
		}

	}
}

//Based on AIM can protocol
//https://www.aimtechnologies.com/support/racingecu/AiM_CAN_101_eng.pdf

struct Aim5f0 {
	scaled_channel<uint16_t, 1> Rpm;
	scaled_channel<uint16_t, 650> Tps;
	scaled_channel<uint16_t, 650> Pps;
	scaled_channel<uint16_t, 100> Vss;
};

static void populateFrame(Aim5f0& msg) {
	msg.Rpm = Sensor::getOrZero(SensorType::Rpm);
	msg.Tps = Sensor::getOrZero(SensorType::Tps1);
	msg.Pps = Sensor::getOrZero(SensorType::AcceleratorPedal);
	msg.Vss = Sensor::getOrZero(SensorType::VehicleSpeed);
}

struct Aim5f1 {
	scaled_channel<uint16_t, 10> WheelSpeedFR;
	scaled_channel<uint16_t, 10> WheelSpeedFL;
	scaled_channel<uint16_t, 10> WheelSpeedRR;
	scaled_channel<uint16_t, 10> WheelSpeedRL;
};

static void populateFrame(Aim5f1& msg) {
	// We don't handle wheel speed, just set to 0?
	msg.WheelSpeedFR = 0;
	msg.WheelSpeedFL = 0;
	msg.WheelSpeedRR = 0;
	msg.WheelSpeedRL = 0;
}

struct Aim5f2 {
	scaled_channel<uint16_t, 190> Iat;
	scaled_channel<uint16_t, 190> Ect;
	scaled_channel<uint16_t, 190> FuelT;
	scaled_channel<uint16_t, 190> OilT;
};

static void populateFrame(Aim5f2& msg) {
	msg.Iat = Sensor::getOrZero(SensorType::Iat) + 45;
	msg.Ect = Sensor::getOrZero(SensorType::Clt) + 45;
	msg.FuelT = Sensor::getOrZero(SensorType::AuxTemp1) + 45;
	msg.OilT = Sensor::getOrZero(SensorType::AuxTemp2) + 45;
}

struct Aim5f3 {
	scaled_channel<uint16_t, 10> Map;
	scaled_channel<uint16_t, 10> Baro;
	scaled_channel<uint16_t, 1000> OilP;
	scaled_channel<uint16_t, 20> FuelP;
};

static void populateFrame(Aim5f3& msg) {
	// MAP/Baro are sent in millibar -> 10 millibar per kpa
	msg.Map = 10 * Sensor::getOrZero(SensorType::Map);
	msg.Baro = 10 * Sensor::getOrZero(SensorType::BarometricPressure);

	// Oil/Fuel P use bar -> 100 kpa per bar
	msg.OilP = Sensor::getOrZero(SensorType::OilPressure) / 100;
	msg.FuelP = Sensor::getOrZero(SensorType::FuelPressureInjector) / 100;
}

struct Aim5f4 {
	scaled_channel<uint16_t, 10000> Boost;
	scaled_channel<uint16_t, 3200> Vbat;
	scaled_channel<uint16_t, 10> FuelUse;
	scaled_channel<uint16_t, 10> Gear;
};

static void populateFrame(Aim5f4& msg) {
	float deltaKpa = Sensor::getOrZero(SensorType::Map) 
		- Sensor::get(SensorType::BarometricPressure).value_or(101.325);
	float boostBar = deltaKpa / 100;

	msg.Boost = boostBar;
	msg.Vbat = Sensor::getOrZero(SensorType::BatteryVoltage);
	msg.FuelUse = 0;
	msg.Gear = 0;
}

struct Aim5f5 {
	scaled_channel<uint16_t, 1> ShiftFlag;
	scaled_channel<uint16_t, 1> GearTime;
	scaled_channel<uint16_t, 1> TpsV;
	scaled_channel<uint16_t, 100> FuelLevel;
};

static void populateFrame(Aim5f5& msg) {
	msg.FuelLevel = Sensor::getOrZero(SensorType::FuelLevel);

	// Dunno what to do with these
	msg.ShiftFlag = 0;
	msg.GearTime = 0;
	msg.TpsV = 0;
}

struct Aim5f6 {
	scaled_channel<uint16_t, 2000> Lambda1;
	scaled_channel<uint16_t, 2000> Lambda2;
	scaled_channel<uint16_t, 10> LambdaTemp1;
	scaled_channel<uint16_t, 10> LambdaTemp2;
};

static void populateFrame(Aim5f6& msg) {
	msg.Lambda1 = Sensor::getOrZero(SensorType::Lambda1);
	msg.Lambda2 = Sensor::getOrZero(SensorType::Lambda2);
	msg.LambdaTemp1 = 0;
	msg.LambdaTemp2 = 0;
}

struct Aim5f7 {
	scaled_channel<uint16_t, 10> LambdaErr1;
	scaled_channel<uint16_t, 10> LambdaErr2;
	scaled_channel<uint16_t, 2000> LambdaTarget1;
	scaled_channel<uint16_t, 2000> LambdaTarget2;
};

static void populateFrame(Aim5f7& msg) {
	// We don't handle wheel speed, just set to 0?
	msg.LambdaErr1 = 0;
	msg.LambdaErr2 = 0;
	// both targets are the same for now
	msg.LambdaTarget1 = (float)engine->fuelComputer.targetLambda;
	msg.LambdaTarget2 = (float)engine->fuelComputer.targetLambda;
}

void canDashboardAim(CanCycle cycle) {
	if (!cycle.isInterval(CI::_10ms)) {
		return;
	}

	transmitStruct<Aim5f0>(CanCategory::NBC, 0x5f0, false);
	transmitStruct<Aim5f1>(CanCategory::NBC, 0x5f1, false);
	transmitStruct<Aim5f2>(CanCategory::NBC, 0x5f2, false);
	transmitStruct<Aim5f3>(CanCategory::NBC, 0x5f3, false);
	transmitStruct<Aim5f4>(CanCategory::NBC, 0x5f4, false);
	transmitStruct<Aim5f5>(CanCategory::NBC, 0x5f5, false);
	transmitStruct<Aim5f6>(CanCategory::NBC, 0x5f6, false);
	transmitStruct<Aim5f7>(CanCategory::NBC, 0x5f7, false);

	// there are more, but less important for us
	// transmitStruct<Aim5f8>(0x5f8, false);
	// transmitStruct<Aim5f9>(0x5f9, false);
	// transmitStruct<Aim5fa>(0x5fa, false);
	// transmitStruct<Aim5fb>(0x5fb, false);
	// transmitStruct<Aim5fc>(0x5fc, false);
	// transmitStruct<Aim5fd>(0x5fd, false);
}

// *****************************************************************************
// Call back for NMEA2000 open. This will be called, when library starts bus communication.
// See NMEA2000.SetOnOpen(OnN2kOpen); on setup()
void OnN2kOpen() {
  // Start schedulers now.
//   TemperatureScheduler.UpdateNextTime();
//   EnvironmentalScheduler.UpdateNextTime();
//   OutsideEnvironmentalScheduler.UpdateNextTime();
}

// double ReadCabinTemp()
// {
// 	static double temp = 21.0;
// 	return temp += 0.01;
// }

// double ReadWaterTemp()
// {
// 	static double temp = 11.0;
// 	return temp += 0.01;
// }
	// static volatile uint16_t cntTemperatureScheduler = 0u;
	// static volatile uint16_t cntEnvironmentalScheduler = 0u;
	// static volatile uint16_t cntOutsideEnvironmentalScheduler = 0u;

//NMEA2000 Dashboard
void canDashboardNMEA2000(CanCycle cycle) {

	static bool initDone = false;

	static bool doOnce = false;

	if (false == initDone)
	{
		// Set Product information
		NMEA2000.SetProductInformation("00000001", // Manufacturer's Model serial code
										100, // Manufacturer's product code
										"rusEFI Eidothea",  // Manufacturer's Model ID
										"1.1.0.0 (2024-07-10)",  // Manufacturer's Software version code
										"1.0.0.0 (2023-04-01)" // Manufacturer's Model version
										);
		// Set device information
		NMEA2000.SetDeviceInformation(112233, // Unique number. Use e.g. Serial number.
										140, // Device function=Engine. See codes on https://web.archive.org/web/20190531120557/https://www.nmea.org/Assets/20120726%20nmea%202000%20class%20&%20function%20codes%20v%202.00.pdf
										50, // Device class=Propulsion. See codes on https://web.archive.org/web/20190531120557/https://www.nmea.org/Assets/20120726%20nmea%202000%20class%20&%20function%20codes%20v%202.00.pdf
										2040 // Just choosen free from code list on https://web.archive.org/web/20190529161431/http://www.nmea.org/Assets/20121020%20nmea%202000%20registration%20list.pdf
									);
		// Uncomment 2 rows below to see, what device will send to bus. Use e.g. OpenSkipper or Actisense NMEA Reader                           
		//Serial.begin(115200);
		//NMEA2000.SetForwardStream(&Serial);
		// If you want to use simple ascii monitor like Arduino Serial Monitor, uncomment next line
		//NMEA2000.SetForwardType(tNMEA2000::fwdt_Text); // Show in clear text. Leave uncommented for default Actisense format.

		// If you also want to see all traffic on the bus use N2km_ListenAndNode instead of N2km_NodeOnly below
		NMEA2000.SetMode(tNMEA2000::N2km_NodeOnly,22);
		//NMEA2000.SetDebugMode(tNMEA2000::dm_Actisense); // Uncomment this, so you can test code without CAN bus chips on Arduino Mega
		NMEA2000.EnableForward(false); // Disable all msg forwarding to USB (=Serial)
		// Here we tell library, which PGNs we transmit
		NMEA2000.ExtendTransmitMessages(TransmitMessages);
		// Define OnOpen call back. This will be called, when CAN is open and system starts address claiming.
		NMEA2000.SetOnOpen(OnN2kOpen);
		NMEA2000.Open();

		initDone = true;
	}

	// warning(CUSTOM_ERR_CAN_CONFIGURATION, "canDashboardNMEA2000");

	tN2kMsg N2kMsg;

	// if (TemperatureScheduler.IsTime())
	// {
	// 	TemperatureScheduler.UpdateNextTime();
	// 	SetN2kTemperature(N2kMsg, 1, 1, N2kts_MainCabinTemperature, CToKelvin(ReadCabinTemp()));
	// 	NMEA2000.SendMsg(N2kMsg);
	// 	cntTemperatureScheduler++;
	// }

	// if (EnvironmentalScheduler.IsTime())
	// {
	// 	EnvironmentalScheduler.UpdateNextTime();
	// 	// SetN2kEnvironmentalParameters(N2kMsg, 1, N2kts_MainCabinTemperature, ReadCabinTemp());
	// 	SetN2kEnvironmentalParameters(N2kMsg, 1, N2kts_OutsideTemperature, CToKelvin(ReadCabinTemp()));

	// 	NMEA2000.SendMsg(N2kMsg);
	// 	cntEnvironmentalScheduler++;
	// }

	// if (OutsideEnvironmentalScheduler.IsTime())
	// {
	// 	OutsideEnvironmentalScheduler.UpdateNextTime();
	// 	// SetN2kOutsideEnvironmentalParameters(N2kMsg, 1, ReadWaterTemp(), CToKelvin(ReadCabinTemp()+10.0), 850.0 );
	// 	SetN2kOutsideEnvironmentalParameters(N2kMsg, 1, N2kDoubleNA, N2kDoubleNA, 850.0 );
	// 	NMEA2000.SendMsg(N2kMsg);
	// 	cntOutsideEnvironmentalScheduler++;
	// 	// Serial.print(millis()); Serial.println(", Temperature send ready");
	// }

	if (cycle.isInterval(CI::_100ms))
	{
		SetN2kPGN127488(N2kMsg, 0 /* EngineInstance */, Sensor::getOrZero(SensorType::Rpm) /* EngineSpeed */,
                     Sensor::getOrZero(SensorType::Map) /* EngineBoostPressure */);
		NMEA2000.SendMsg(N2kMsg);

		/* Lambda */
		uint16_t lambdaVal[2u] = {
			(uint16_t)(Sensor::getOrZero(SensorType::Lambda1)/0.0001),
			(uint16_t)(Sensor::getOrZero(SensorType::Lambda2)/0.0001) };
		double outputLambda = 0.0;

		{
			CanTxMessage msg(CanCategory::NBC, 0x180);
			msg.busIndex = 1;
			msg[0] = (uint8_t)(lambdaVal[0u] >> 8);
			msg[1] = (uint8_t)(lambdaVal[0u] & 0xFF);
		}

		{
			CanTxMessage msg(CanCategory::NBC, 0x181);
			msg.busIndex = 1;
			msg[0] = (uint8_t)(lambdaVal[1u] >> 8);
			msg[1] = (uint8_t)(lambdaVal[1u] & 0xFF);
		}

		/* Use outside Atmospheric Pressure as Lambda */
   		// Check all sensors for invalid value or timeout
		for (uint8_t i=0u; i<2u; i++)
		{
			if (lambdaVal[i] != 0)
			{
				// Sensor is valid, take highest (leanest) value for output
				if (lambdaVal[i] > outputLambda)
				{
					outputLambda = lambdaVal[i]; // Take highest (leanest) value for output
				}
			}
		}

		SetN2kOutsideEnvironmentalParameters(N2kMsg, 1, N2kDoubleNA, N2kDoubleNA, outputLambda );
		NMEA2000.SendMsg(N2kMsg);
	}

	if (cycle.isInterval(CI::_1000ms))
	{
		/*inline void SetN2kPGN127489(tN2kMsg &N2kMsg, unsigned char EngineInstance, double EngineOilPress, double EngineOilTemp, double EngineCoolantTemp, double AltenatorVoltage,
                       double FuelRate, double EngineHours, double EngineCoolantPress=N2kDoubleNA, double EngineFuelPress=N2kDoubleNA,
                       int8_t EngineLoad=N2kInt8NA, int8_t EngineTorque=N2kInt8NA,
                       bool flagCheckEngine=false,       bool flagOverTemp=false,         bool flagLowOilPress=false,         bool flagLowOilLevel=false,
                       bool flagLowFuelPress=false,      bool flagLowSystemVoltage=false, bool flagLowCoolantLevel=false,     bool flagWaterFlow=false,
                       bool flagWaterInFuel=false,       bool flagChargeIndicator=false,  bool flagPreheatIndicator=false,    bool flagHighBoostPress=false,
                       bool flagRevLimitExceeded=false,  bool flagEgrSystem=false,        bool flagTPS=false,                 bool flagEmergencyStopMode=false,
                       bool flagWarning1=false,          bool flagWarning2=false,         bool flagPowerReduction=false,      bool flagMaintenanceNeeded=false,
                       bool flagEngineCommError=false,   bool flagSubThrottle=false,      bool flagNeutralStartProtect=false, bool flagEngineShuttingDown=false) */

		double EngineOilPress = Sensor::getOrZero(SensorType::OilPressure);
		double EngineOilTemp = CToKelvin(Sensor::getOrZero(SensorType::AuxTemp1));
		double EngineCoolantTemp = CToKelvin(Sensor::getOrZero(SensorType::Clt));
		double AlternatorVoltage = Sensor::getOrZero(SensorType::BatteryVoltage);

		double FuelRate = 0.0;
		/*
		 * Calculate current fuel rate
		 * From Lua:
		 * local gPerSecond= getOutput('fuelFlowRate') --g/s
		 * local gPerHour=gPerSecond*3600
		 * local FuelRate=gPerHour/720 -- l/h
		 * local FuelRateRes = math.floor(FuelRate/0.1) --0.0001 m3/h
		*/
		float gPerSecond = engine->engineState.fuelConsumption.getConsumptionGramPerSecond();
		float gPerHour = gPerSecond * 3600.0;
		FuelRate = gPerHour / 720.0; // -- l/h

		double EngineHours = 0.0;
		double EngineCoolantPress = Sensor::getOrZero(SensorType::AuxLinear1);
		float EngineFuelPress = Sensor::getOrZero(SensorType::AuxLinear2);

		// warning(CUSTOM_ERR_CAN_CONFIGURATION, "canDashboardNMEA2000 fuel pressure %f", EngineFuelPress);

        int8_t EngineLoad=N2kInt8NA;
		int8_t EngineTorque=N2kInt8NA;

		bool flagOverTemp = false;
		bool flagLowOilPress = false;
		bool flagLowFuelPress = false;
		bool flagLowSystemVoltage = false;
		bool flagWaterFlow = false;
        bool flagCheckEngine = false;
		bool flagLowOilLevel = false;
		bool flagLowCoolantLevel = false;
        bool flagWaterInFuel=false;
		bool flagChargeIndicator=false;
		bool flagPreheatIndicator=false;
		bool flagHighBoostPress=false;
        bool flagRevLimitExceeded=false;
		bool flagEgrSystem=false;
		bool flagTPS=false;
		bool flagEmergencyStopMode=false;
        bool flagWarning1= false;
		bool flagWarning2=false;
		bool flagPowerReduction=false;
		bool flagMaintenanceNeeded=false;
        bool flagEngineCommError=false;
		bool flagSubThrottle=false;
		bool flagNeutralStartProtect=false;
		bool flagEngineShuttingDown=false;

		float mapValue = Sensor::getOrZero(SensorType::Map);
		float battVoltage = Sensor::getOrZero(SensorType::BatteryVoltage);

		/* Check if MAP sensor is valid */
		if ( (battVoltage > 7.0f) && /* ensure 5V sensor supply */
		    ((mapValue == 0.0f) || (mapValue >= 101.0f))) /* MAP sensor is neither 0 (invalid) nor above 100 kPa atmospheric pressure */
		{
			flagEmergencyStopMode = true;
		}

		/* If engine is off a voltage below 11.7V will trigger a warning,
		 * if engine is running, below 750 RPM a voltage threshold of 12.5V and above 13.0V is used */
		float battVoltageThreshold = 11.7f;
		static uint8_t debounceCounterBattVoltage = 0u;
		if (Sensor::getOrZero(SensorType::Rpm) >= 750.0f)
		{
			battVoltageThreshold = 13.0f;
		}
		else if (Sensor::getOrZero(SensorType::Rpm) > 400.0f)
		{
			battVoltageThreshold = 12.5f;
		}
		flagLowSystemVoltage = (battVoltage < battVoltageThreshold) ? true : false;

		/* debounce voltage errors for 5s because of trim pump or anchor winch drawing a lot of current */
		if (true == flagLowSystemVoltage)
		{
			/* error is active - debounce it if present less then 5s */
			if (debounceCounterBattVoltage < 5u)
			{
				debounceCounterBattVoltage++;
				flagLowSystemVoltage = false;
			}
		}
		else
		{
			/* No voltage error - reset debounce counter */
			debounceCounterBattVoltage = 0u;
		}

		/* Check if motor is running */
		if (Sensor::getOrZero(SensorType::Rpm) > 400.0f)
		{
			float oilPress = Sensor::getOrZero(SensorType::OilPressure);
			float coolantTemp = Sensor::getOrZero(SensorType::Clt);
			float fuelPress = Sensor::getOrZero(SensorType::AuxLinear2);
			float oilTemp = Sensor::getOrZero(SensorType::AuxTemp1);

			/* Check if water or oil temperature is too hot */
			flagOverTemp =  ((oilTemp > 125.0f) || (coolantTemp > 77.0f)) ? true : false;

			/* 285kPa is the normal fuel pressure under full load
			 * set threshold to -0.1 Bar -> 275 kPa
			 * decrement by inverse MAP (100 kPa - current MAP value) */
		    flagLowFuelPress = (fuelPress < (275.0f - (100.0f - mapValue))) ? true : false;

			/* Below 1000 RPM use a water pressure threshold of 15.0 kPa and above use 25.0 kPa */
			float waterPressThreshold = 15.0f;
			if (Sensor::getOrZero(SensorType::Rpm) >= 1000.0f)
			{
				waterPressThreshold = 25.0f;
			}
			flagWaterFlow = (Sensor::getOrZero(SensorType::AuxLinear1) < waterPressThreshold) ? true : false;

			/* Oil pressure based on RPM. Measured values from log file:
			 * At idle we have a minimum 180 kPa
			 * above 750 RPM we have a minimum 240 kPa
			 * above 1400 RPM we have a minimum 327 kPa
			 * above 2250 RPM we have a minimum 375 kPa
			 * we decrement these values with a margin of 50 kPa
			 */
			float oilPressThreshold = 150.0f;
			if (Sensor::getOrZero(SensorType::Rpm) >= 2250.0f)
			{
				oilPressThreshold = 325.0f;
			}
			else if (Sensor::getOrZero(SensorType::Rpm) >= 1400.0f)
			{
				oilPressThreshold = 275.0f;
			}
			else if (Sensor::getOrZero(SensorType::Rpm) >= 750.0f)
			{
				oilPressThreshold = 190.0f;
			}
			flagLowOilPress = (oilPress < oilPressThreshold) ? true : false;

		}

		/* summarize critical errors */
		// flagCheckEngine = (flagOverTemp || flagLowOilPress || flagLowFuelPress);
		// this has higher prio and the root cause is not directly visible

		/* Enable beeper in case of critical error */
		setError((flagOverTemp || flagLowOilPress || flagLowFuelPress), (obd_code_e)1 /* OBD_Manifold_Absolute_Pressure_Circuit_Malfunction blinks 9 times */);

		/* Set the TS warning as indicator */
		flagWarning1 = engine->engineState.warnings.isWarningNow();

		/* check if we can validate the TPS and pedal sensor for errors */
		// static volatile float tps1prim = Sensor::getOrZero(SensorType::Tps1Primary);
		// static volatile float tps1sec = Sensor::getOrZero(SensorType::Tps1Secondary);
		// static volatile float pedalComb = Sensor::getOrZero(SensorType::AcceleratorPedal);
		// static volatile float pedalPrim = Sensor::getOrZero(SensorType::Tps1Primary);
		// static volatile float pedalSec = Sensor::getOrZero(SensorType::Tps1Secondary);
		// flagTPS = true;

		SetN2kPGN127489(N2kMsg,                    0 /* EngineInstance */,   (EngineOilPress*1000),   EngineOilTemp,
		               EngineCoolantTemp,          AlternatorVoltage,        FuelRate,                EngineHours,
                       (EngineCoolantPress*1000),  (EngineFuelPress*1000),   EngineLoad,              EngineTorque,
                       flagCheckEngine,            flagOverTemp,             flagLowOilPress,         flagLowOilLevel,
                       flagLowFuelPress,           flagLowSystemVoltage,     flagLowCoolantLevel,     flagWaterFlow,
                       flagWaterInFuel,            flagChargeIndicator,      flagPreheatIndicator,    flagHighBoostPress,
                       flagRevLimitExceeded,       flagEgrSystem,            flagTPS,                 flagEmergencyStopMode,
                       flagWarning1,               flagWarning2,             flagPowerReduction,      flagMaintenanceNeeded,
                       flagEngineCommError,        flagSubThrottle,          flagNeutralStartProtect, flagEngineShuttingDown);
		NMEA2000.SendMsg(N2kMsg);


        double IntakeAirTemp = CToKelvin(Sensor::getOrZero(SensorType::Iat));
		SetN2kTransmissionParameters(N2kMsg, 0 /* EngineInstance */, N2kTG_Unknown /* TransmissionGear */,
		                            N2kDoubleNA /* OilPressure */, IntakeAirTemp /* OilTemperature */); // use transmission oil temperature for IAT
		NMEA2000.SendMsg(N2kMsg);
	}

	if (cycle.isInterval(CI::_50ms))
	{
		// {
		// 	CanTxMessage msg(CanCategory::NBC, CAN_BMW_E46_SPEED);
		// 	msg.setShortValue(10 * 8, 1);
		// }

		// {
		// 	CanTxMessage msg(CanCategory::NBC, CAN_BMW_E46_RPM);
		// 	msg.setShortValue((int)(Sensor::getOrZero(SensorType::Rpm) * 6.4), 2);
		// }

		// {
		// 	CanTxMessage msg(CanCategory::NBC, CAN_BMW_E46_DME2);
		// 	msg.setShortValue((int)((Sensor::getOrZero(SensorType::Clt) + 48.373) / 0.75), 1);
		// }


	// if (false == doOnce)
	// {
	// 	SetN2kEnvironmentalParameters(N2kMsg, 1, N2kts_MainCabinTemperature, ReadCabinTemp());
	// 	NMEA2000.SendMsg(N2kMsg);
	// 	doOnce = true;
	// }
	// static uint8_t counter = 0;
	// if (counter > 10) // after start wait some time to avoid timeout? was just a test, i think not needed...
	// {
		NMEA2000.ParseMessages();
	// }
	// else counter++;
	}

	
  }

// #endif // EFI_CAN_SUPPORT

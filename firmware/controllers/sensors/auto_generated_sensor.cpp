#include "global.h"
#include "sensor_type.h"
// was generated automatically by rusEFI tool  from sensor_type.h // by enum2string.jar tool on Mon Mar 21 00:17:32 UTC 2022
// see also gen_config_and_enums.bat



const char *getSensorType(SensorType value){
switch(value) {
case SensorType::AcceleratorPedal:
  return "AcceleratorPedal";
case SensorType::AcceleratorPedalPrimary:
  return "AcceleratorPedalPrimary";
case SensorType::AcceleratorPedalSecondary:
  return "AcceleratorPedalSecondary";
case SensorType::Aux1:
  return "Aux1";
case SensorType::Aux2:
  return "Aux2";
case SensorType::Aux3:
  return "Aux3";
case SensorType::Aux4:
  return "Aux4";
case SensorType::Aux5:
  return "Aux5";
case SensorType::Aux6:
  return "Aux6";
case SensorType::Aux7:
  return "Aux7";
case SensorType::Aux8:
  return "Aux8";
case SensorType::AuxLinear1:
  return "AuxLinear1";
case SensorType::AuxLinear2:
  return "AuxLinear2";
case SensorType::AuxTemp1:
  return "AuxTemp1";
case SensorType::AuxTemp2:
  return "AuxTemp2";
case SensorType::BarometricPressure:
  return "BarometricPressure";
case SensorType::BatteryVoltage:
  return "BatteryVoltage";
case SensorType::Clt:
  return "Clt";
case SensorType::DriverThrottleIntent:
  return "DriverThrottleIntent";
case SensorType::FuelEthanolPercent:
  return "FuelEthanolPercent";
case SensorType::FuelLevel:
  return "FuelLevel";
case SensorType::FuelPressureHigh:
  return "FuelPressureHigh";
case SensorType::FuelPressureInjector:
  return "FuelPressureInjector";
case SensorType::FuelPressureLow:
  return "FuelPressureLow";
case SensorType::Iat:
  return "Iat";
case SensorType::IdlePosition:
  return "IdlePosition";
case SensorType::Invalid:
  return "Invalid";
case SensorType::Lambda1:
  return "Lambda1";
case SensorType::Lambda2:
  return "Lambda2";
case SensorType::Maf:
  return "Maf";
case SensorType::Map:
  return "Map";
case SensorType::MapFast:
  return "MapFast";
case SensorType::MapSlow:
  return "MapSlow";
case SensorType::OilPressure:
  return "OilPressure";
case SensorType::PlaceholderLast:
  return "PlaceholderLast";
case SensorType::Rpm:
  return "Rpm";
case SensorType::Tps1:
  return "Tps1";
case SensorType::Tps1Primary:
  return "Tps1Primary";
case SensorType::Tps1Secondary:
  return "Tps1Secondary";
case SensorType::Tps2:
  return "Tps2";
case SensorType::Tps2Primary:
  return "Tps2Primary";
case SensorType::Tps2Secondary:
  return "Tps2Secondary";
case SensorType::TurbochargerSpeed:
  return "TurbochargerSpeed";
case SensorType::VehicleSpeed:
  return "VehicleSpeed";
case SensorType::WastegatePosition:
  return "WastegatePosition";
  }
 return NULL;
}

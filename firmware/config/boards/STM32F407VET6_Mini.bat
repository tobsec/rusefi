
echo "Compiling for https://rusefi.com/forum/viewtopic.php?f=4&t=1498"
cd ../..
rem TODO: somehow this -DDUMMY is helping us to not mess up the parameters, why?!
rem https://github.com/rusefi/rusefi/issues/684
rem this board has only 512K flash so using custom FLASH_ADDR
rem You probably want "flash0  : org = 0x08000000, len = 450K" in the .ld file
set EXTRA_PARAMS=-DDUMMY -DEFI_COMMUNICATION_PIN=GPIOB_9 -DSTM32_HSECLK=25000000U -DSTM32_PLLM_VALUE=25 -DSTM32_RTCPRE_VALUE=25 -DDEFAULT_ENGINE_TYPE=MINIMAL_PINS^
 -DEFI_CDM_INTEGRATION=FALSE ^
 -DEFI_WAVE_ANALYZER=FALSE ^
 -DEFI_ICU_INPUTS=FALSE ^
 -DEFI_INTERNAL_FLASH=FALSE ^
 -DEFI_SHAFT_POSITION_INPUT=FALSE ^
 -DEFI_MAX_31855=FALSE ^
 -DEFI_HIP_9011=FALSE ^
 -DEFI_CJ125=FALSE ^
 -DEFI_INTERNAL_ADC=FALSE ^
 -DEFI_CAN_SUPPORT=FALSE ^
 -DEFI_HD44780_LCD=FALSE ^
 -DEFI_IDLE_CONTROL=FALSE ^
 -DEFI_PWM=FALSE ^
 -DEFI_VEHICLE_SPEED=FALSE ^
 -DEFI_ENGINE_EMULATOR=FALSE ^
 -DEFI_EMULATE_POSITION_SENSORS=FALSE ^
 -DEFI_FILE_LOGGING=FALSE ^
 -DEFI_ENGINE_SNIFFER=FALSE ^
 -DHAL_USE_ICU=FALSE ^
 -DHAL_USE_ADC=FALSE ^
 -DHAL_USE_CAN=FALSE ^
 -DHAL_USE_I2C=FALSE ^
 -DHAL_USE_PWM=FALSE ^
 -DHAL_USE_RTC=FALSE ^
 -DFLASH_ADDR=0x08075000

make -j8 clean
make -j8
rem rm -f build/rusefi.dfu
rem dir build
rem ..\misc\hex2dfu\HEX2DFU.exe build/rusefi.hex   -out build/rusefi.dfu

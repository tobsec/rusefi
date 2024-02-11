# Combine the related files for a specific platform and MCU.

# Target ECU board design
BOARDCPPSRC = $(BOARD_DIR)/board_configuration.cpp
DDEFS += -DEFI_MAIN_RELAY_CONTROL=TRUE

# Add them all together
DDEFS += -DEFI_SOFTWARE_KNOCK=TRUE -DSTM32_ADC_USE_ADC3=TRUE

# MM176_GP9
DDEFS += -DADC_MUX_PIN=Gpio::F2

LED_CRITICAL_ERROR_BRAIN_PIN = -DLED_CRITICAL_ERROR_BRAIN_PIN=H176_MCU_MEGA_LED1_RED
include $(BOARDS_DIR)/hellen/hellen-common176.mk

ifeq ($(PROJECT_CPU),ARCH_STM32F7)
	# TODO: why do I struggle to fit into flash? compare with Proteus
	DDEFS += -DCH_DBG_ENABLE_ASSERTS=FALSE
	DDEFS += -DENABLE_PERF_TRACE=FALSE
    USE_OPT += -Wl,--defsym=FLASH_SIZE=768k
else ifeq ($(PROJECT_CPU),ARCH_STM32F4)
    # This board has trigger scope hardware!
    DDEFS += -DTRIGGER_SCOPE
    # serial ports only on F4
	DDEFS += $(PRIMARY_COMMUNICATION_PORT_USART2)
else
$(error Unsupported PROJECT_CPU [$(PROJECT_CPU)])
endif
DDEFS += -DSTATIC_BOARD_ID=STATIC_BOARD_ID_ALPHAX_8CHAN

DDEFS += -DHW_HELLEN_8CHAN=1

ONBOARD_MEMS_TYPE=LIS2DH12

set PATH=%PATH%C:\tools\arm-gnu-toolchain-12.2.rel1-mingw-w64-i686-arm-none-eabi\bin;
arm-none-eabi-gcc.exe



make PROJECT_BOARD=proteus PROJECT_CPU=ARCH_STM32F4 -j8


make PROJECT_BOARD=proteus PROJECT_CPU=ARCH_STM32F4 -j8 clean


gen_config_board.bat proteus proteus_f4
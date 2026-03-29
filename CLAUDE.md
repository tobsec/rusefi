# CLAUDE.md

This file provides guidance to Claude Code (claude.ai/code) when working with code in this repository.

## Project Overview

rusEFI is an open-source engine control unit (ECU) firmware for STM32 microcontrollers. The firmware runs on ChibiOS RTOS. A PC simulator and Java-based tuning console are also included.

Development model is "stable master" — no release branches, only master with CI and limited automated testing. Each PR should be complete and not break existing functionality. WIP PRs must be prefixed with "[WIP]".

## Build Commands

### Prerequisites
- GCC ARM toolchain version 11 or 12 (see `firmware/gcc_version_check.c`)
- Java 11+ (for Gradle/Java tools)
- Git submodules: `git submodule update --init`

### Firmware
```bash
cd firmware
# Select board interactively:
./bin/compile.sh
# Or specify a board directly:
./bin/compile.sh config/boards/f407-discovery/meta-info.env

# Make targets (from firmware/):
make docs-enums     # Generate docs and enums only
make config         # Generate docs, enums, and configs
make                # Generate all + build firmware
make bundle         # Full bundle (bootloader + firmware + packaging)
```

Board selection uses `BOARD_DIR`, `SHORT_BOARD_NAME`, and `PROJECT_BOARD` env vars. Source a meta-info file to set them:
```bash
source config/boards/common_script_read_meta_env.inc config/boards/f407-discovery/meta-info.env
```

### Unit Tests
```bash
cd unit_tests
make -j$(nproc)
build/rusefi_test

# Run a single test:
build/rusefi_test --gtest_filter=*TEST_NAME*

# With coverage:
make -j$(nproc) COVERAGE=yes
```

### Simulator
```bash
# From firmware/:
make ../simulator/build/rusefi_simulator.exe    # Windows
```

### Java Tools & Console
```bash
cd java_tools
./gradlew :ui:shadowJar          # Build console JAR
./gradlew test                    # Run Java tests
```

## Architecture

### Firmware (`firmware/`)
C/C++ embedded firmware on ChibiOS RTOS for STM32F4/F7/H7.
- `controllers/` — Engine control algorithms: fuel, ignition, idle, ETB, boost, sensors, CAN, Lua scripting
- `hw_layer/` — MCU-specific hardware drivers
- `config/boards/` — Board-specific configurations (30+ boards). Each board has a `meta-info*.env` file
- `libfirmware/` — Shared firmware library (git submodule)
- `ChibiOS/` — RTOS kernel (git submodule)

### Code Generation
Extensive auto-generated code — do not edit generated files directly:
- `gen_config.sh` / `gen_config_board.sh` — Generates C headers from configuration definitions
- `gen_enum_to_string.sh` — Generates enum-to-string converters
- `gen_live_documentation.sh` — API documentation generation
- Java tools in `java_tools/config_definition/` and `java_tools/enum_to_string/` drive the generation

Auto-generated files are committed to the repo. CI commits include "Auto-generated configs and docs" or "Auto-generated default tune" in the message.

### Simulator (`simulator/`)
Same firmware codebase compiled for desktop (Win32/POSIX) with mocked sensors/outputs. Used for testing and generating default tunes (MSQ files).

### Java Console (`java_console/`)
Native tuning/configuration GUI with modular architecture (ui, io, logging, models, trigger visualization). Integrates with TunerStudio.

### Java Tools (`java_tools/`)
Gradle multi-project build with 20+ subprojects for code generation, configuration management, and development utilities.

### Unit Tests (`unit_tests/`)
Google Test framework. Plain C/C++ compiled for desktop — no ChibiOS or ARM dependencies. Tests live in `unit_tests/tests/`.

## CI Notes

- Firmware CI builds use a matrix of 30+ board variants
- Commit messages with `only:<board>` restrict firmware CI to that board only (e.g., `only:uaefi`)
- `only:docs` or similar non-board values skip all firmware builds
- Board `meta-info.env` files can specify `skip_rate` to probabilistically skip CI builds

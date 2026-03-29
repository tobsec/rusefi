# Build Setup Guide — NMEA2000 Branch (Proteus F4)

This documents everything needed to go from a fresh Windows machine to a
complete firmware build, step by step.

## 1. Install WSL2 with Debian

From an elevated PowerShell or Command Prompt:

```powershell
wsl --install -d Debian
```

Reboot if prompted. On first launch, Debian will ask you to create a
username and password. Remember the password — you'll need it for `sudo`.

Verify WSL2 is running:

```powershell
wsl --list --verbose
```

Should show `Debian` with `VERSION 2`.

## 2. Fix Windows PATH in WSL

WSL2 inherits the Windows PATH which contains spaces and breaks bash
commands. Disable it:

```bash
echo -e '[interop]\nappendWindowsPath = false' | sudo tee /etc/wsl.conf
```

Then from Windows PowerShell, restart WSL:

```powershell
wsl --shutdown
```

Reopen WSL. Verify with `echo $PATH` — it should only show Linux paths.

## 3. Install system packages

```bash
sudo apt-get update && sudo apt-get install -y make openjdk-21-jdk-headless zip dosfstools mtools xxd gzip wget
```

What each package is for:
- `make` — build system
- `openjdk-21-jdk-headless` — Java runtime for config generation (jar files are pre-built in repo)
- `zip` — packaging ini files into the ramdisk FAT image
- `dosfstools` — `mkfs.fat` and `fatlabel` for creating FAT filesystem
- `mtools` — `mcopy` for copying files into FAT image without mounting
- `xxd` — converts ramdisk binary to C header
- `gzip` — compresses the ramdisk image (Proteus F4 uses compressed variant)
- `wget` — for downloading the ARM toolchain

## 4. Install ARM GCC 12.2

The firmware requires GCC 11 or 12 — it rejects anything below 11.3.1
or above 13 at compile time (see `firmware/gcc_version_check.c`).

```bash
cd /tmp && wget -q https://developer.arm.com/-/media/Files/downloads/gnu/12.2.rel1/binrel/arm-gnu-toolchain-12.2.rel1-x86_64-arm-none-eabi.tar.xz && sudo mkdir -p /opt/arm-gcc && sudo tar xf arm-gnu-toolchain-12.2.rel1-x86_64-arm-none-eabi.tar.xz -C /opt/arm-gcc --strip-components=1 && echo 'export PATH=/opt/arm-gcc/bin:$PATH' >> ~/.bashrc && source ~/.bashrc
```

Verify:

```bash
arm-none-eabi-gcc --version
```

Should show `arm-none-eabi-gcc (Arm GNU Toolchain 12.2.Rel1 ...) 12.2.1 20221205`.

## 5. Clone the repository

```bash
cd /mnt/c/Data
git clone https://github.com/tobsec/rusefi.git
cd rusefi
git checkout NMEA2000
git submodule update --init
```

If the repo is already cloned on the Windows side (e.g. at `C:\Data\rusefi`),
it's accessible from WSL at `/mnt/c/Data/rusefi` — no need to clone again.

## 6. Build

All commands from the `firmware/` directory:

```bash
cd /mnt/c/Data/rusefi/firmware
```

### Full build (config generation + compile)

```bash
./gen_enum_to_string.sh && ./gen_config_board.sh proteus proteus_f4 && make PROJECT_BOARD=proteus PROJECT_CPU=ARCH_STM32F4 -j$(nproc)
```

This runs three steps:

**Step 1: `gen_enum_to_string.sh`** — generates enum-to-string converter
source files from `rusefi_enums.h` and `rusefi_hw_enums.h`.

**Step 2: `gen_config_board.sh proteus proteus_f4`** — runs `ConfigDefinition.jar`
and produces:
- `tunerstudio/generated/rusefi_proteus_f4.ini` — TunerStudio configuration
- `controllers/generated/rusefi_generated.h` — C defines for config fields
- `controllers/generated/engine_configuration_generated_structures.h` — C structs
- `controllers/generated/signature_proteus_f4.h` — build signature
- `hw_layer/mass_storage/ramdisk_image_compressed.h` — compressed FAT image as C array
- `ramdisk.image` — raw FAT filesystem with the ini file inside

**Step 3: `make`** — compiles the firmware. Produces in `build/`:
- `rusefi.elf` — ELF with debug symbols
- `rusefi.bin` — raw binary for flashing
- `rusefi.hex` — Intel HEX format
- `rusefi.dfu` — DFU format for USB bootloader flashing
- `rusefi.map` — linker map

### Compile only (skip config generation)

If you only changed C/C++ source files and not configuration definitions:

```bash
make PROJECT_BOARD=proteus PROJECT_CPU=ARCH_STM32F4 -j$(nproc)
```

### Clean build

```bash
make PROJECT_BOARD=proteus PROJECT_CPU=ARCH_STM32F4 clean
make PROJECT_BOARD=proteus PROJECT_CPU=ARCH_STM32F4 -j$(nproc)
```

## Flashing

### Via DFU (USB bootloader)

Put the ECU into DFU mode (boot0 pin or firmware command), then:

```bash
./flash_dfu.sh
```

### Via OpenOCD (SWD/JTAG debugger)

```bash
./flash_openocd407.sh
```

## Ramdisk / USB Mass Storage

The ECU exposes a USB mass storage device so TunerStudio can auto-detect
the ini file. Proteus F4 (STM32F429) uses `EFI_USE_COMPRESSED_INI_MSD` —
the compressed FAT image is embedded in firmware rodata (~107KB).

The image is generated automatically by `gen_config_board.sh` using
`mkfs.fat`, `mcopy`, `gzip`, and `xxd`. If these tools are missing,
the build will fail at the config generation step.

## Board Selection

Build parameters for Proteus F4:
- `PROJECT_BOARD=proteus`
- `PROJECT_CPU=ARCH_STM32F4`

Board configuration lives in `config/boards/proteus/`. The meta-info file
`config/boards/proteus/meta-info-proteus_f4.env` defines these values.

## Notes

- **NTFS performance**: Building on `/mnt/c/` (Windows filesystem) is ~5x
  slower than on a native ext4 filesystem. For faster builds, clone the repo
  inside WSL (e.g. `~/rusefi`) and copy the output binary back.
- **Java tools**: `ConfigDefinition.jar` and `gcc_map_reader.jar` are
  pre-built and committed to `java_tools/`. No Gradle build needed.
- **Submodules**: The build requires ChibiOS, NMEA2000, and other submodules.
  Always run `git submodule update --init` after cloning or switching branches.

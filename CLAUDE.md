# CLAUDE.md

This file provides guidance to Claude Code (claude.ai/code) when working with code in this repository.

## Project Overview

This is an advanced inverted pendulum control system project with multiple implementations and control strategies:

- **STM32 C++ Implementation**: Real-time control system using STM32F303 microcontroller with line-following capability
- **STM32 Rust Implementation**: Modern async/await Embassy-based implementation with multi-modal control
- **Adorobo Implementation**: MATLAB code generation approach using STM32CubeIDE
- **ESP32 Implementation**: Wireless control interface using ESP32-C3 with BLE
- **MATLAB Simulation**: Multiple control strategies including MPC, LQR, and nonlinear control
- **TDD Implementation**: Test-driven development with CI/CD pipeline for STM32 code

## Build Commands

### STM32 C++ Project (`InvertedPendulum-STM32/`)

```bash
# Build for PC testing (TDD)
cd InvertedPendulum-STM32
./build_pc.sh

# Build using CMake (embedded target)
cmake -G Ninja -S . -B build -DCMAKE_BUILD_TYPE=Release -DCMAKE_TOOLCHAIN_FILE=cmake/gcc-arm-none-eabi.cmake
cmake --build build

# Flash to hardware (requires STM32CubeProgrammer)
# Use VS Code task: "Build + Flash" or manually:
STM32_Programmer_CLI --connect port=swd --download build/InvertedPendulum-STM32.elf -hardRst -rst --start
```

### STM32 Rust Project (`inverted-pendulum-stm32/`)

```bash
# Build using Cargo (embedded target)
cd inverted-pendulum-stm32
cargo build --release

# Flash to hardware using probe-rs
cargo flash --release --chip STM32F303K8Tx

# Run with real-time logging
cargo run --release
```

### Adorobo Project (`Adorobo-Inverted-Pendulum/`)

```bash
# Build using STM32CubeIDE or makefile
cd Adorobo-Inverted-Pendulum
make all  # If makefile is available

# Or use STM32CubeIDE GUI for build and flash
```

### ESP32 Project (`InvertedPendulum-ESP32/`)

```bash
# Build and upload using PlatformIO
cd InvertedPendulum-ESP32
pio run                    # Build only
pio run --target upload    # Build and upload
pio run --target monitor   # Monitor serial output
```

### MATLAB Simulations (`MATLAB/`)

Open MATLAB and navigate to the project directory. Key files:
- `InvertedPendulum.prj` - Main project file
- `control/` - Various control system implementations
- `control_v2/` - Updated control strategies
- `equation_of_state/` - System dynamics and linearization
- `estimate_parameters/` - Parameter identification experiments

## Development Notes

- **Windows Build Note**: When using Windows, use PowerShell commands for building instead of Git Bash, as Git Bash may encounter build issues.

[Remaining content of the file remains the same...]
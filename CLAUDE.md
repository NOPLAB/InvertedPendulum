# CLAUDE.md

This file provides guidance to Claude Code (claude.ai/code) when working with code in this repository.

## Project Overview

This is an inverted pendulum control system project with multiple implementations:

- **STM32 Implementation**: Real-time control system using STM32F303 microcontroller
- **ESP32 Implementation**: Wireless control interface using ESP32-C3 with BLE
- **MATLAB Simulation**: Multiple control strategies including MPC, LQR, and nonlinear control
- **TDD Implementation**: Test-driven development with CI/CD pipeline for STM32 code

## Build Commands

### STM32 Project (`InvertedPendulum-STM32/`)

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
- `equation_of_state/` - System dynamics and linearization
- `estimate_parameters/` - Parameter identification experiments

## Architecture

### STM32 Implementation

**Core Application Structure:**
- `Core/App/` - Main application layer written in C++
- `Core/App/Inc/app.hpp` - Main application class with singleton pattern
- `Core/App/Src/app.cpp` - Application initialization and main loop
- `Core/App/Inc/` - Header files for ADC, motors, interrupts, intervals
- `Core/App/Lib/` - External libraries (QEI encoder library)

**Key Components:**
- **App Class**: Singleton managing system initialization and main loop
- **Interrupt Handling**: Centralized interrupt management for ADC and timers
- **Motor Control**: Dual motor control for cart positioning
- **Encoder Reading**: Quadrature encoder interface for position feedback
- **ADC**: Analog-to-digital conversion for sensor readings
- **PID Controller**: Real-time PID control with anti-windup
- **LPF**: Low-pass filter for sensor signal processing

**STM32 HAL Integration:**
- Uses STM32 HAL library for hardware abstraction
- Generated code from STM32CubeMX in `Core/Src/` and `Core/Inc/`
- Custom application code interfaces with HAL through C++ wrappers

**Test-Driven Development (TDD):**
- `Core/App/Test/` - TDD test suite with custom test framework
- `CMakeLists_PC.txt` - PC build configuration for cross-platform testing
- `build_pc.sh` - Build script for PC testing
- Mock objects for hardware abstraction layer
- CI/CD pipeline with GitHub Actions for automated testing

### ESP32 Implementation

**BLE Communication:**
- Implements UART-over-BLE service for wireless communication
- Service UUID: `ff94ca6f-91a5-4048-b805-755bca71075e`
- Characteristic UUIDs for TX/RX communication
- Callback-based architecture for connection management

### MATLAB Control Systems

**Control Strategies:**
- **State Feedback (LQR)**: Linear quadratic regulator design
- **MPC**: Model predictive control for optimal trajectories
- **Adaptive MPC**: Adaptive model predictive control
- **Nonlinear MPC**: Nonlinear model predictive control

**System Modeling:**
- `equation_of_state/` - Mathematical modeling and linearization
- `equation_of_state_v2/` - Improved dynamics modeling
- Parameter estimation from experimental data

## Development Workflow

### Working with STM32 Code

1. **TDD Development**: Write tests first in `Core/App/Test/`, then implement in `Core/App/`
2. **Run Tests**: Use `./build_pc.sh` for PC testing before embedded build
3. **Modify Application Logic**: Work primarily in `Core/App/` directory
4. **Hardware Configuration**: Use STM32CubeMX to modify `.ioc` file, regenerate code
5. **Build**: Use CMake or VS Code CMake extension
6. **Debug**: Use STM32CubeIDE or VS Code with Cortex-Debug extension

### Working with ESP32 Code

1. **Modify BLE Logic**: Edit `src/main.cpp` 
2. **Configuration**: Modify `platformio.ini` for build settings
3. **Build and Upload**: Use PlatformIO commands

### Working with MATLAB

1. **Open Project**: Use `InvertedPendulum.prj` in MATLAB
2. **Controller Design**: Use live scripts in respective controller directories
3. **Simulation**: Run Simulink models (`.slx` files)
4. **Code Generation**: Use Simulink Coder for embedded targets

## Key Files and Locations

- `InvertedPendulum-STM32/Core/App/` - Main STM32 application code
- `InvertedPendulum-STM32/Core/App/Test/` - TDD test suite
- `InvertedPendulum-STM32/CMakeLists.txt` - Embedded build configuration
- `InvertedPendulum-STM32/CMakeLists_PC.txt` - PC build configuration for TDD
- `InvertedPendulum-STM32/build_pc.sh` - PC build script
- `InvertedPendulum-STM32/.vscode/tasks.json` - Build and flash tasks
- `InvertedPendulum-ESP32/platformio.ini` - ESP32 project configuration
- `InvertedPendulum-ESP32/src/main.cpp` - ESP32 BLE implementation
- `MATLAB/control/` - Control system implementations
- `MATLAB/equation_of_state/` - System dynamics modeling
- `MATLAB/estimate_parameters/` - Parameter identification
- `.github/workflows/tdd-ci.yml` - CI/CD pipeline for TDD

## Hardware Configuration

**STM32F303K8T6 Microcontroller:**
- ARM Cortex-M4 with FPU
- 64KB Flash, 16KB RAM
- Multiple timers, ADC, GPIO for motor control and sensing

**Peripherals:**
- Dual motor drivers for cart control
- Quadrature encoders for position feedback (QEI_L, QEI_R)
- ADC channels for sensor readings
- UART for serial communication

**ESP32-C3:**
- RISC-V single-core processor
- Built-in BLE for wireless communication
- Arduino framework compatibility

## Testing and Validation

**Test-Driven Development (TDD):**
- Custom lightweight test framework in `Core/App/Test/test_framework.hpp`
- Unit tests for PID controller, LPF, and other components
- Cross-platform testing (PC and embedded)
- Automated CI/CD pipeline with GitHub Actions
- Code coverage analysis and quality checks

**System Identification:**
- Experimental data collection in `MATLAB/estimate_parameters/experiments/`
- Parameter estimation using system identification toolbox
- Model validation through simulation vs. experimental data comparison

**Control Validation:**
- MATLAB simulations with different control strategies
- Hardware-in-the-loop testing with STM32 implementation
- Real-time performance analysis and tuning
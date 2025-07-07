cmake_minimum_required(VERSION 3.22)

# PC Build Configuration for TDD Tests
project(InvertedPendulum-PC-Tests)

# Set C++ standard
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# Enable compile command to ease indexing
set(CMAKE_EXPORT_COMPILE_COMMANDS TRUE)

# Define build type
if(NOT CMAKE_BUILD_TYPE)
    set(CMAKE_BUILD_TYPE "Debug")
endif()

# PC-specific compiler flags
set(CMAKE_CXX_FLAGS_DEBUG "-g -O0 -Wall -Wextra -Wpedantic")
set(CMAKE_CXX_FLAGS_RELEASE "-O3 -DNDEBUG")

# Create executable for PC tests
add_executable(${PROJECT_NAME})

# Add test sources only (no STM32 HAL dependencies)
target_sources(${PROJECT_NAME} PRIVATE
    # Core application sources (platform-independent)
    ../Core/App/Src/pid.cpp
    ../Core/App/Src/lpf.cpp
    
    # Test framework and tests
    ../Core/App/Test/test_framework.cpp
    ../Core/App/Test/test_main_pc.cpp
    ../Core/App/Test/mock_hal_pc.cpp
)

# Include directories
target_include_directories(${PROJECT_NAME} PRIVATE
    ../Core/App/Inc
    ../Core/App/Test
)

# PC-specific compile definitions
target_compile_definitions(${PROJECT_NAME} PRIVATE
    ENABLE_TESTING
    PC_BUILD
)

# Link math library for PC
target_link_libraries(${PROJECT_NAME} m)
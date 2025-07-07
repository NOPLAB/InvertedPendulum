#!/bin/bash

# PC Build Script for TDD Tests
echo "🔧 Building InvertedPendulum Tests for PC..."

# Create PC build directory and copy CMakeLists
mkdir -p build_pc
cp CMakeLists_PC.txt build_pc/CMakeLists.txt

cd build_pc

# Configure CMake for PC build
cmake .

# Build the project
make -j$(nproc)

# Check if build was successful
if [ $? -eq 0 ]; then
    echo "✅ Build successful!"
    echo "🚀 Running tests..."
    echo ""
    ./InvertedPendulum-PC-Tests
    exit_code=$?
    echo ""
    if [ $exit_code -eq 0 ]; then
        echo "🎉 All tests passed on PC!"
    else
        echo "❌ Some tests failed on PC!"
    fi
    exit $exit_code
else
    echo "❌ Build failed!"
    exit 1
fi
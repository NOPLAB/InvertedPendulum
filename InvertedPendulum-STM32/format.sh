#!/bin/bash

# Auto-format C++ code in Core/App directory using clang-format

set -e

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
TARGET_DIR="$SCRIPT_DIR/Core/App"

echo "Formatting C++ files in $TARGET_DIR..."

# Check if clang-format is available
if ! command -v clang-format &> /dev/null; then
    echo "Error: clang-format is not installed"
    echo "Install with: sudo apt-get install clang-format"
    exit 1
fi

# Find and format all .cpp and .hpp files
find "$TARGET_DIR" -name "*.cpp" -o -name "*.hpp" | while read -r file; do
    echo "Formatting: $file"
    clang-format -i "$file"
done

echo "Formatting complete!"
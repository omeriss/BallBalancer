#!/bin/bash
# Script to build and run tests on Linux/macOS

echo "============================================="
echo "  Building Inverse Kinematics Tests"
echo "============================================="

# Create build directory
mkdir -p build
cd build

# Use CMake if available
if command -v cmake &> /dev/null; then
    echo "Using CMake..."
    cmake ..
    make
    
    if [ $? -eq 0 ]; then
        echo ""
        echo "============================================="
        echo "  Running Tests"
        echo "============================================="
        ./bin/test_inverse_kinematics
        exit $?
    fi
fi

# Fallback: Direct GCC compilation
if command -v gcc &> /dev/null; then
    echo "Using GCC directly..."
    cd ..
    gcc -o build/test_inverse_kinematics test_inverse_kinematics.c -lm -Wall -Wextra
    
    if [ $? -eq 0 ]; then
        echo ""
        echo "============================================="
        echo "  Running Tests"
        echo "============================================="
        ./build/test_inverse_kinematics
        exit $?
    fi
fi

echo "ERROR: No suitable compiler found!"
echo "Please install GCC or CMake."
exit 1

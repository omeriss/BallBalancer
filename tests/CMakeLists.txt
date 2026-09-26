# tests/CMakeLists.txt
# CMake configuration for running unit tests on the host machine

cmake_minimum_required(VERSION 3.22)

project(inverse_kinematics_tests C)

set(CMAKE_C_STANDARD 11)
set(CMAKE_C_STANDARD_REQUIRED ON)

# Enable testing
enable_testing()

# Add test executable - includes the actual source file
add_executable(test_inverse_kinematics
    src/test_inverse_kinematics.c
    ${CMAKE_CURRENT_SOURCE_DIR}/../User/Src/inverse_kinematics.c
)

# Add include path for the User headers
target_include_directories(test_inverse_kinematics PRIVATE
    ${CMAKE_CURRENT_SOURCE_DIR}/../User/Inc
)

# Link math library on Linux/macOS
if(UNIX)
    target_link_libraries(test_inverse_kinematics m)
endif()

# Add test to CTest
add_test(NAME inverse_kinematics_tests COMMAND test_inverse_kinematics)

# Set output directory
set_target_properties(test_inverse_kinematics PROPERTIES
    RUNTIME_OUTPUT_DIRECTORY ${CMAKE_BINARY_DIR}/bin
)

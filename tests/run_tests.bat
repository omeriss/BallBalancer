@echo off
REM Script to build and run tests on Windows
REM Requires GCC (MinGW), MSVC, or TCC to be installed
REM 
REM Quick Install Options:
REM   1. winget install tcc (Tiny C Compiler - smallest/fastest to install)
REM   2. winget install mingw (MinGW-w64 GCC)
REM   3. Install Visual Studio Build Tools

echo =============================================
echo   Building Inverse Kinematics Tests
echo =============================================

REM Check if build directory exists
if not exist "build" mkdir build

REM Try TCC first (smallest footprint)
where tcc >nul 2>&1
if %ERRORLEVEL% EQU 0 (
    echo Using TCC...
    tcc -o build\test_inverse_kinematics.exe test_inverse_kinematics.c -lm
    if %ERRORLEVEL% EQU 0 (
        echo.
        echo =============================================
        echo   Running Tests
        echo =============================================
        build\test_inverse_kinematics.exe
        exit /b %ERRORLEVEL%
    )
)

REM Try GCC (MinGW)
where gcc >nul 2>&1
if %ERRORLEVEL% EQU 0 (
    echo Using GCC...
    gcc -o build\test_inverse_kinematics.exe test_inverse_kinematics.c -lm -Wall -Wextra
    if %ERRORLEVEL% EQU 0 (
        echo.
        echo =============================================
        echo   Running Tests
        echo =============================================
        build\test_inverse_kinematics.exe
        exit /b %ERRORLEVEL%
    )
)

REM Try MSVC cl.exe
where cl >nul 2>&1
if %ERRORLEVEL% EQU 0 (
    echo Using MSVC...
    cl /Fe:build\test_inverse_kinematics.exe test_inverse_kinematics.c /nologo
    if %ERRORLEVEL% EQU 0 (
        echo.
        echo =============================================
        echo   Running Tests
        echo =============================================
        build\test_inverse_kinematics.exe
        exit /b %ERRORLEVEL%
    )
)

REM Try CMake with available generators
where cmake >nul 2>&1
if %ERRORLEVEL% EQU 0 (
    echo Trying CMake...
    cd build
    cmake .. 2>nul
    if %ERRORLEVEL% EQU 0 (
        cmake --build .
        if %ERRORLEVEL% EQU 0 (
            echo.
            echo =============================================
            echo   Running Tests
            echo =============================================
            REM Try different output paths (MSVC uses Debug/Release subdirs)
            if exist "bin\Debug\test_inverse_kinematics.exe" (
                bin\Debug\test_inverse_kinematics.exe
            ) else if exist "bin\Release\test_inverse_kinematics.exe" (
                bin\Release\test_inverse_kinematics.exe
            ) else if exist "bin\test_inverse_kinematics.exe" (
                bin\test_inverse_kinematics.exe
            ) else if exist "Debug\test_inverse_kinematics.exe" (
                Debug\test_inverse_kinematics.exe
            ) else (
                echo ERROR: Could not find test executable
                cd ..
                exit /b 1
            )
            cd ..
            exit /b %ERRORLEVEL%
        )
    )
    cd ..
)

echo.
echo ERROR: No suitable compiler found!
echo.
echo Please install one of the following:
echo   - TCC:    winget install tcc
echo   - MinGW:  winget install mingw
echo   - MSVC:   Install Visual Studio Build Tools
echo.
exit /b 1

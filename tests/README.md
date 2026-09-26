# Inverse Kinematics Unit Tests

This folder contains unit tests for the `inverse_kinematics.c` functions.

## Quick Start

### Option 1: Install TCC (Tiny C Compiler) - Recommended

TCC is the smallest and fastest option to install:

```powershell
winget install tcc
```

Then run:

```powershell
cd tests
.\run_tests.bat
```

### Option 2: Install MinGW (GCC)

```powershell
winget install mingw
```

Then run:

```powershell
cd tests
.\run_tests.bat
```

### Option 3: Use Visual Studio

If you have Visual Studio installed, open "Developer Command Prompt for VS" and run:

```cmd
cd tests
run_tests.bat
```

### Option 4: Direct Compilation

If you have a C compiler in your PATH:

**With GCC/MinGW:**

```powershell
cd tests
gcc -o test_inverse_kinematics.exe test_inverse_kinematics.c -lm
.\test_inverse_kinematics.exe
```

**With MSVC (from Developer Command Prompt):**

```cmd
cd tests
cl /Fe:test_inverse_kinematics.exe test_inverse_kinematics.c
test_inverse_kinematics.exe
```

## Test Coverage

The tests cover:

1. **`rrs3_options_create`**
   - Basic parameter assignment
   - Zero values
   - Negative values (edge case)

2. **`rrs3_calculate_angles`**
   - Level platform (nx=0, ny=0)
   - X-axis tilt
   - Y-axis tilt
   - Varying heights
   - All three legs (A, B, C)
   - Symmetry verification
   - Extreme tilt values
   - Consistency (same input → same output)

## Expected Output

```
==============================================
  Inverse Kinematics Unit Tests
==============================================

--- Running test_rrs3_options_create_basic ---
  [PASS] buttomLeg should be 50.0 (expected: 50.0000, actual: 50.0000)
  [PASS] topLeg should be 100.0 (expected: 100.0000, actual: 100.0000)
  ...

==============================================
  Test Summary
==============================================
  Total:  XX
  Passed: XX
  Failed: 0
==============================================
```

## Adding New Tests

To add new tests:

1. Create a new test function:

```c
void test_my_new_test(void)
{
    // Setup
    RRS3Options options = rrs3_options_create(50.0f, 80.0f, 60.0f, 40.0f);

    // Execute
    float result = rrs3_calculate_angles(A, &options, 100.0f, 0.0f, 0.0f);

    // Assert
    TEST_ASSERT_FLOAT_VALID(result, "Result should be valid");
    TEST_ASSERT_FLOAT_EQ(expected, result, 0.01f, "Result matches expected");
}
```

2. Add it to `main()`:

```c
RUN_TEST(test_my_new_test);
```

## Notes

- Tests run on the **host machine** (Windows/Linux/macOS), not on the STM32 target
- The test file includes a copy of the functions to avoid cross-compilation issues
- If you modify `inverse_kinematics.c`, update the copy in `test_inverse_kinematics.c`

/**
 * @file test_inverse_kinematics.c
 * @brief Unit tests for inverse_kinematics functions
 */

#include <math.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

/* Include the actual header */
#include "inverse_kinematics.h"

/* Test framework macros */
#define TEST_EPSILON 0.001f

static int tests_run = 0;
static int tests_passed = 0;
static int tests_failed = 0;

#define TEST_ASSERT(condition, message)                                        \
  do {                                                                         \
    tests_run++;                                                               \
    if (condition) {                                                           \
      tests_passed++;                                                          \
      printf("  [PASS] %s\n", message);                                        \
    } else {                                                                   \
      tests_failed++;                                                          \
      printf("  [FAIL] %s\n", message);                                        \
    }                                                                          \
  } while (0)

#define TEST_ASSERT_FLOAT_EQ(expected, actual, epsilon, message)               \
  do {                                                                         \
    tests_run++;                                                               \
    float diff = fabsf((expected) - (actual));                                 \
    if (diff <= (epsilon)) {                                                   \
      tests_passed++;                                                          \
      printf("  [PASS] %s (expected: %.4f, actual: %.4f)\n", message,          \
             (expected), (actual));                                            \
    } else {                                                                   \
      tests_failed++;                                                          \
      printf("  [FAIL] %s (expected: %.4f, actual: %.4f, diff: %.4f)\n",       \
             message, (expected), (actual), diff);                             \
    }                                                                          \
  } while (0)

#define TEST_ASSERT_FLOAT_VALID(value, message)                                \
  do {                                                                         \
    tests_run++;                                                               \
    if (!isnan(value) && !isinf(value)) {                                      \
      tests_passed++;                                                          \
      printf("  [PASS] %s (value: %.4f)\n", message, (value));                 \
    } else {                                                                   \
      tests_failed++;                                                          \
      printf("  [FAIL] %s (value is NaN or Inf)\n", message);                  \
    }                                                                          \
  } while (0)

#define RUN_TEST(test_func)                                                    \
  do {                                                                         \
    printf("\n--- Running %s ---\n", #test_func);                              \
    test_func();                                                               \
  } while (0)

/* ============================================================================
 * TEST CASES
 * ============================================================================
 */

/**
 * Test rrs3_options_create function
 */
void test_rrs3_options_create_basic(void) {
  RRS3Options options = rrs3_options_create(50.0f, 100.0f, 30.0f, 20.0f);

  TEST_ASSERT_FLOAT_EQ(50.0f, options.buttomLeg, TEST_EPSILON,
                       "buttomLeg should be 50.0");
  TEST_ASSERT_FLOAT_EQ(100.0f, options.topLeg, TEST_EPSILON,
                       "topLeg should be 100.0");
  TEST_ASSERT_FLOAT_EQ(30.0f, options.baseR, TEST_EPSILON,
                       "baseR should be 30.0");
  TEST_ASSERT_FLOAT_EQ(20.0f, options.platformR, TEST_EPSILON,
                       "platformR should be 20.0");
}

void test_rrs3_options_create_zero_values(void) {
  RRS3Options options = rrs3_options_create(0.0f, 0.0f, 0.0f, 0.0f);

  TEST_ASSERT_FLOAT_EQ(0.0f, options.buttomLeg, TEST_EPSILON,
                       "buttomLeg should be 0.0");
  TEST_ASSERT_FLOAT_EQ(0.0f, options.topLeg, TEST_EPSILON,
                       "topLeg should be 0.0");
  TEST_ASSERT_FLOAT_EQ(0.0f, options.baseR, TEST_EPSILON,
                       "baseR should be 0.0");
  TEST_ASSERT_FLOAT_EQ(0.0f, options.platformR, TEST_EPSILON,
                       "platformR should be 0.0");
}

void test_rrs3_options_create_negative_values(void) {
  /* Edge case: negative values (may not be physically meaningful but should
   * work) */
  RRS3Options options = rrs3_options_create(-10.0f, -20.0f, -5.0f, -3.0f);

  TEST_ASSERT_FLOAT_EQ(-10.0f, options.buttomLeg, TEST_EPSILON,
                       "buttomLeg should be -10.0");
  TEST_ASSERT_FLOAT_EQ(-20.0f, options.topLeg, TEST_EPSILON,
                       "topLeg should be -20.0");
  TEST_ASSERT_FLOAT_EQ(-5.0f, options.baseR, TEST_EPSILON,
                       "baseR should be -5.0");
  TEST_ASSERT_FLOAT_EQ(-3.0f, options.platformR, TEST_EPSILON,
                       "platformR should be -3.0");
}

/**
 * Test rrs3_calculate_angles with level platform (nx=0, ny=0)
 */
void test_rrs3_calculate_angles_level_platform(void) {
  /* Typical robot dimensions */
  RRS3Options options = rrs3_options_create(50.0f, 80.0f, 60.0f, 40.0f);
  float h = 100.0f; /* Height */
  float nx = 0.0f;  /* No tilt in x */
  float ny = 0.0f;  /* No tilt in y */

  float angle_A = rrs3_calculate_angles(A, &options, h, nx, ny);
  float angle_B = rrs3_calculate_angles(B, &options, h, nx, ny);
  float angle_C = rrs3_calculate_angles(C, &options, h, nx, ny);

  /* All angles should be valid (not NaN or Inf) */
  TEST_ASSERT_FLOAT_VALID(angle_A,
                          "Leg A angle should be valid for level platform");
  TEST_ASSERT_FLOAT_VALID(angle_B,
                          "Leg B angle should be valid for level platform");
  TEST_ASSERT_FLOAT_VALID(angle_C,
                          "Leg C angle should be valid for level platform");

  /* For a symmetric level platform, angles should be equal */
  TEST_ASSERT_FLOAT_EQ(angle_A, angle_B, 0.1f,
                       "Leg A and B angles should be equal for level platform");
  TEST_ASSERT_FLOAT_EQ(angle_B, angle_C, 0.1f,
                       "Leg B and C angles should be equal for level platform");
}

/**
 * Test rrs3_calculate_angles with tilted platform
 */
void test_rrs3_calculate_angles_tilted_platform(void) {
  RRS3Options options = rrs3_options_create(50.0f, 80.0f, 60.0f, 40.0f);
  float h = 100.0f;
  float nx = 0.1f; /* Small tilt in x */
  float ny = 0.0f;

  float angle_A = rrs3_calculate_angles(A, &options, h, nx, ny);
  float angle_B = rrs3_calculate_angles(B, &options, h, nx, ny);
  float angle_C = rrs3_calculate_angles(C, &options, h, nx, ny);

  TEST_ASSERT_FLOAT_VALID(angle_A,
                          "Leg A angle should be valid for tilted platform");
  TEST_ASSERT_FLOAT_VALID(angle_B,
                          "Leg B angle should be valid for tilted platform");
  TEST_ASSERT_FLOAT_VALID(angle_C,
                          "Leg C angle should be valid for tilted platform");

  /* For tilted platform, B and C should be different (asymmetric) */
  /* But this depends on tilt direction, so we just check validity */
}

void test_rrs3_calculate_angles_y_tilt(void) {
  RRS3Options options = rrs3_options_create(50.0f, 80.0f, 60.0f, 40.0f);
  float h = 100.0f;
  float nx = 0.0f;
  float ny = 0.1f; /* Small tilt in y */

  float angle_A = rrs3_calculate_angles(A, &options, h, nx, ny);
  float angle_B = rrs3_calculate_angles(B, &options, h, nx, ny);
  float angle_C = rrs3_calculate_angles(C, &options, h, nx, ny);

  TEST_ASSERT_FLOAT_VALID(angle_A,
                          "Leg A angle should be valid for Y tilted platform");
  TEST_ASSERT_FLOAT_VALID(angle_B,
                          "Leg B angle should be valid for Y tilted platform");
  TEST_ASSERT_FLOAT_VALID(angle_C,
                          "Leg C angle should be valid for Y tilted platform");
}

/**
 * Test with different heights
 */
void test_rrs3_calculate_angles_varying_height(void) {
  RRS3Options options = rrs3_options_create(50.0f, 80.0f, 60.0f, 40.0f);
  float nx = 0.0f;
  float ny = 0.0f;

  /* Low height */
  float angle_low = rrs3_calculate_angles(A, &options, 80.0f, nx, ny);
  /* Medium height */
  float angle_med = rrs3_calculate_angles(A, &options, 100.0f, nx, ny);
  /* High height */
  float angle_high = rrs3_calculate_angles(A, &options, 120.0f, nx, ny);

  TEST_ASSERT_FLOAT_VALID(angle_low, "Angle at low height should be valid");
  TEST_ASSERT_FLOAT_VALID(angle_med, "Angle at medium height should be valid");
  TEST_ASSERT_FLOAT_VALID(angle_high, "Angle at high height should be valid");

  /* Higher platform should result in smaller angle (less bending) - depends on
   * geometry */
  printf("    Info: angle_low=%.2f, angle_med=%.2f, angle_high=%.2f\n",
         angle_low, angle_med, angle_high);
}

/**
 * Test each leg individually
 */
void test_rrs3_calculate_angles_each_leg(void) {
  RRS3Options options = rrs3_options_create(40.0f, 70.0f, 50.0f, 35.0f);
  float h = 90.0f;
  float nx = 0.05f;
  float ny = 0.05f;

  float angle_A = rrs3_calculate_angles(A, &options, h, nx, ny);
  float angle_B = rrs3_calculate_angles(B, &options, h, nx, ny);
  float angle_C = rrs3_calculate_angles(C, &options, h, nx, ny);

  TEST_ASSERT_FLOAT_VALID(angle_A, "Leg A calculation produces valid angle");
  TEST_ASSERT_FLOAT_VALID(angle_B, "Leg B calculation produces valid angle");
  TEST_ASSERT_FLOAT_VALID(angle_C, "Leg C calculation produces valid angle");

  /* Angles should be in reasonable range (0 to 180 degrees) */
  TEST_ASSERT(angle_A >= 0.0f && angle_A <= 180.0f,
              "Leg A angle in valid range [0, 180]");
  TEST_ASSERT(angle_B >= 0.0f && angle_B <= 180.0f,
              "Leg B angle in valid range [0, 180]");
  TEST_ASSERT(angle_C >= 0.0f && angle_C <= 180.0f,
              "Leg C angle in valid range [0, 180]");
}

/**
 * Test symmetry: opposite tilts should produce symmetric results
 */
void test_rrs3_calculate_angles_symmetry(void) {
  RRS3Options options = rrs3_options_create(50.0f, 80.0f, 60.0f, 40.0f);
  float h = 100.0f;

  /* For symmetric robot, B with positive x-tilt should equal C with negative
   * x-tilt */
  float angle_B_pos = rrs3_calculate_angles(B, &options, h, 0.1f, 0.0f);
  float angle_C_neg = rrs3_calculate_angles(C, &options, h, -0.1f, 0.0f);

  TEST_ASSERT_FLOAT_VALID(angle_B_pos, "B angle with positive tilt is valid");
  TEST_ASSERT_FLOAT_VALID(angle_C_neg, "C angle with negative tilt is valid");

  /* They should be approximately equal due to symmetry */
  TEST_ASSERT_FLOAT_EQ(angle_B_pos, angle_C_neg, 0.5f,
                       "B(+tilt) ≈ C(-tilt) due to symmetry");
}

/**
 * Test with extreme tilt values
 */
void test_rrs3_calculate_angles_extreme_tilt(void) {
  RRS3Options options = rrs3_options_create(50.0f, 80.0f, 60.0f, 40.0f);
  float h = 100.0f;

  /* Large tilt values */
  float angle_A = rrs3_calculate_angles(A, &options, h, 0.5f, 0.5f);
  float angle_B = rrs3_calculate_angles(B, &options, h, 0.5f, 0.5f);
  float angle_C = rrs3_calculate_angles(C, &options, h, 0.5f, 0.5f);

  /* May produce NaN if tilt is unreachable - that's expected behavior */
  printf("    Info: Extreme tilt angles: A=%.2f, B=%.2f, C=%.2f\n", angle_A,
         angle_B, angle_C);

  /* Just check that the function doesn't crash */
  TEST_ASSERT(1, "Function handles extreme tilt without crashing");
}

/**
 * Test consistency: same inputs should produce same outputs
 */
void test_rrs3_calculate_angles_consistency(void) {
  RRS3Options options = rrs3_options_create(50.0f, 80.0f, 60.0f, 40.0f);
  float h = 100.0f;
  float nx = 0.1f;
  float ny = 0.1f;

  float angle1 = rrs3_calculate_angles(A, &options, h, nx, ny);
  float angle2 = rrs3_calculate_angles(A, &options, h, nx, ny);
  float angle3 = rrs3_calculate_angles(A, &options, h, nx, ny);

  TEST_ASSERT_FLOAT_EQ(angle1, angle2, 0.0001f,
                       "First and second calls produce same result");
  TEST_ASSERT_FLOAT_EQ(angle2, angle3, 0.0001f,
                       "Second and third calls produce same result");
}

/* ============================================================================
 * MAIN TEST RUNNER
 * ============================================================================
 */

int main(void) {
  printf("==============================================\n");
  printf("  Inverse Kinematics Unit Tests\n");
  printf("==============================================\n");

  /* Run all tests */
  RUN_TEST(test_rrs3_options_create_basic);
  RUN_TEST(test_rrs3_options_create_zero_values);
  RUN_TEST(test_rrs3_options_create_negative_values);
  RUN_TEST(test_rrs3_calculate_angles_level_platform);
  RUN_TEST(test_rrs3_calculate_angles_tilted_platform);
  RUN_TEST(test_rrs3_calculate_angles_y_tilt);
  RUN_TEST(test_rrs3_calculate_angles_varying_height);
  RUN_TEST(test_rrs3_calculate_angles_each_leg);
  RUN_TEST(test_rrs3_calculate_angles_symmetry);
  RUN_TEST(test_rrs3_calculate_angles_extreme_tilt);
  RUN_TEST(test_rrs3_calculate_angles_consistency);

  /* Print summary */
  printf("\n==============================================\n");
  printf("  Test Summary\n");
  printf("==============================================\n");
  printf("  Total:  %d\n", tests_run);
  printf("  Passed: %d\n", tests_passed);
  printf("  Failed: %d\n", tests_failed);
  printf("==============================================\n");

  return (tests_failed > 0) ? 1 : 0;
}

// Anti-windup, limits, step behavior.
// Module: #37   Milestone: M2
// TODO: implement (see robot-arm-todo.md for the 'done when' criteria).

#include "unity.h"
#include "pi_controller.h"
#include <math.h>
#include <string.h>

#define TOLERANCE (1e-3f) //update tolerance in all tests

// Unity calls these around every test. Nothing to set up for pure math.
void setUp(void) {}
void tearDown(void) {}

static const pi_controller_config_t default_pi_controller_config = {
    .Kp = 2.0f,
    .Ki = 100.0f,
    .u_min = -10.0f,
    .u_max = 10.0f,
    .Ts = 0.001f,
};

static const pi_controller_config_t pi_only_controller_config = {
    .Kp = 2.0f,
    .Ki = 0.0f,
    .u_min = -10.0f,
    .u_max = 10.0f,
    .Ts = 0.001f,
};

static void test_pi_only(void)
{
    pi_controller_t controller;
    pi_controller_init(&controller, &pi_only_controller_config);

    float error = 1.0f; // for 5 steps and ensure its 2.0 each step, the output should be 2.0 after 5 steps
    float u;

    for (int i = 0; i < 5; i++) {
        u = pi_controller_update(&controller, error);
    }

    TEST_ASSERT_FLOAT_WITHIN(TOLERANCE, 2.0f, u);
}

static void test_pi_controller_integral_step(void)
{
    float error = 1.0f;
    pi_controller_t controller;
    //step 1: u = 2.05
    //step 10: u = 2.95
    float u_1;
    float u_10;
    pi_controller_init(&controller, &default_pi_controller_config);

    for (int i = 0; i < 10; i++) {
        float u = pi_controller_update(&controller, error);
        if (i == 0) {
            u_1 = u;
        }
        if (i == 9) {
            u_10 = u;
        }
    }

    TEST_ASSERT_FLOAT_WITHIN(TOLERANCE, 2.05f, u_1);
    TEST_ASSERT_FLOAT_WITHIN(TOLERANCE, 2.95f, u_10);
}

static void test_pi_controller_upper_limit(void)
{
    float error = 100.0f;
    //step 1: u = 10.0

    pi_controller_t controller;
    pi_controller_init(&controller, &default_pi_controller_config);

    float u = pi_controller_update(&controller, error);

    TEST_ASSERT_FLOAT_WITHIN(TOLERANCE, 10.0f, u);
}

static void test_pi_controller_lower_limit(void)
{
    float error = -100.0f;
    //step 1: u = -10.0
    pi_controller_t controller;
    pi_controller_init(&controller, &default_pi_controller_config);

    float u = pi_controller_update(&controller, error);
    TEST_ASSERT_FLOAT_WITHIN(TOLERANCE, -10.0f, u);
}

static void test_pi_controller_reaches_limit(void)
{
    float error = 1.0f; 
    //for 200 steps
    //step 80: u = 9.95
    //step 81: u = 10.0
    //check u <= 10.0 for all steps

    pi_controller_t controller;
    pi_controller_init(&controller, &default_pi_controller_config);

    float u_80;
    float u_81;
    float u_200; //make sure never goes above 10.0

    for (int i = 0; i < 200; i++) {
        float u = pi_controller_update(&controller, error);
        TEST_ASSERT_TRUE(u <= 10.0f);
        if (i == 79) {
            u_80 = u;
        }
        if (i == 80) {
            u_81 = u;
        }
        if (i == 199) {
            u_200 = u;
        }
    }

    TEST_ASSERT_FLOAT_WITHIN(TOLERANCE, 9.95f, u_80);
    TEST_ASSERT_FLOAT_WITHIN(TOLERANCE, 10.0f, u_81);
    TEST_ASSERT_FLOAT_WITHIN(TOLERANCE, 10.0f, u_200);
}

static void test_pi_controller_anti_windup(void)
{
    float error = 1.0f;
    //for 10,000 steps then error = -1
    //first step after reversal: u = 6.0
    //u < 0 by step 62 after reversal

    pi_controller_t controller;
    pi_controller_init(&controller, &default_pi_controller_config);

    for (int i = 0; i < 10000; i++) {
        pi_controller_update(&controller, error);
    }
    error = -1.0f;
    float u_first_step_after_reversal;
    float u_step_61;

    for (int i = 0; i < 100; i++) {
        float u = pi_controller_update(&controller, error);
        if (i == 0) {
            u_first_step_after_reversal = u;
        }
        if (i == 60) {
            u_step_61 = u;
        }
        if (i == 61) {
            TEST_ASSERT_TRUE(u < 0.0f);
        }
    }
    TEST_ASSERT_FLOAT_WITHIN(TOLERANCE, 6.0f, u_first_step_after_reversal);
    TEST_ASSERT_FLOAT_WITHIN(TOLERANCE, 0.0f, u_step_61);
}

static void test_pi_controller_limit_shrinks(void)
{
    //saturate as in test_pi_controller_anti_windup, then set_limits(-5, 5), error = 1
    //u = 5 and never abov 5
    //then error = -1, u = 1.0 on the first step
    //check u <= 5.0 after the firt step

    float error = 1.0f;
    pi_controller_t controller;
    pi_controller_init(&controller, &default_pi_controller_config);

    for (int i = 0; i < 10000; i++) {
        pi_controller_update(&controller, error);
    }
    pi_controller_set_limits(&controller, -5.0f, 5.0f);
    float u_after_limit_shrink;
    for (int i = 0; i < 100; i++) {
        float u = pi_controller_update(&controller, error);
        TEST_ASSERT_TRUE(u <= 5.0f);
        if (i == 0) {
            u_after_limit_shrink = u;
        }
    }
    TEST_ASSERT_FLOAT_WITHIN(TOLERANCE, 5.0f, u_after_limit_shrink);
    error = -1.0f;
    float u_first_step_after_reversal;
    for (int i = 0; i < 100; i++) {
        float u = pi_controller_update(&controller, error);
        if (i == 0) {
            u_first_step_after_reversal = u;
        }
    }
    TEST_ASSERT_FLOAT_WITHIN(TOLERANCE, 1.0f, u_first_step_after_reversal);
}

static void test_pi_controller_asymmetric_limits(void)
{
    //limits -1 to 10, error = -100 then error = 100
    //u = -1 then 10

    float error = -100.0f;
    pi_controller_t controller;
    pi_controller_init(&controller, &default_pi_controller_config);
    pi_controller_set_limits(&controller, -1.0f, 10.0f);

    float u_first_step_after_negative_saturation;
    float u_first_step_after_positive_saturation;   

    for (int i = 0; i < 100; i++) {
        float u = pi_controller_update(&controller, error);
        if (i == 0) {
            u_first_step_after_negative_saturation = u;
        }
    }
    error = 100.0f;
    for (int i = 0; i < 100; i++) {
        float u = pi_controller_update(&controller, error);
        if (i == 0) {
            u_first_step_after_positive_saturation = u;
        }
    }
    TEST_ASSERT_FLOAT_WITHIN(TOLERANCE, -1.0f, u_first_step_after_negative_saturation);
    TEST_ASSERT_FLOAT_WITHIN(TOLERANCE, 10.0f, u_first_step_after_positive_saturation);
}

static void test_pi_controller_reset(void)
{
    //saturate, reset() then e = 0, u = 0.0f
    float error = 1.0f;
    pi_controller_t controller;
    pi_controller_init(&controller, &default_pi_controller_config);
    for (int i = 0; i < 10000; i++) {
        pi_controller_update(&controller, error);
    }
    pi_controller_reset(&controller);
    error = 0.0f;
    float u = pi_controller_update(&controller, error);
    TEST_ASSERT_FLOAT_WITHIN(TOLERANCE, 0.0f, u);
}

static void test_pi_init_clears_state(void)
{
    //fill struct with garbage (memset(&c, 0x7F, sizeof c)), init(), then e = 0
    //u = 0.0f
    pi_controller_t controller;
    memset(&controller, 0x7F, sizeof controller);
    pi_controller_init(&controller, &default_pi_controller_config);

    float error = 0.0f;
    float u = pi_controller_update(&controller, error);
    TEST_ASSERT_FLOAT_WITHIN(TOLERANCE, 0.0f, u);
}

int main(void)
{
    UNITY_BEGIN();
    RUN_TEST(test_pi_only);
    RUN_TEST(test_pi_controller_integral_step);
    RUN_TEST(test_pi_controller_upper_limit);
    RUN_TEST(test_pi_controller_lower_limit);
    RUN_TEST(test_pi_controller_reaches_limit);
    RUN_TEST(test_pi_controller_anti_windup);
    RUN_TEST(test_pi_controller_limit_shrinks);
    RUN_TEST(test_pi_controller_asymmetric_limits);
    RUN_TEST(test_pi_controller_reset);
    RUN_TEST(test_pi_init_clears_state);
    return UNITY_END();
}
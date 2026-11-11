// Anti-windup, limits, step behavior.
// Module: #37   Milestone: M2
// TODO: implement (see robot-arm-todo.md for the 'done when' criteria).

#include "unity.h"
#include "pi_controller.h"
#include <math.h>

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

static void test_pi_only(void)
{
    pi_controller_t controller;
    float u;

    pi_controller_init(&controller, 1.0f, 0.1f, 0.01f, -10.0f, 10.0f, 0.1f);
    pi_controller_update(&controller, 1.0f, 0.01f, &u, -10.0f, 10.0f);

    TEST_ASSERT_FLOAT_WITHIN(TOL, 2.0f, u);
}

static void test_pi_controller_integral_step(void)
{
    
}

static void test_pi_controller_upper_limit(void)
{
    
}

static void test_pi_controller_lower_limit(void)
{
    
}

static void test_pi_controller_reaches_limit(void)
{
    
}

static void test_pi_controller_anti_windup(void)
{
    
}

static void test_pi_controller_limit_shrinks(void)
{
    
}

static void test_pi_controller_asymmetric_limits(void)
{
    
}

static void test_pi_controller_reset(void)
{
    
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
    return UNITY_END();
}
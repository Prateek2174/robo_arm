// Transforms and SVPWM against known values.
// Module: #37   Milestone: M2

#include "unity.h"
#include "foc_math.h"
#include <math.h>

#define TOL 1e-4f
#define PI_F 3.14159265f

// Unity calls these around every test. Nothing to set up for pure math.
void setUp(void) {}
void tearDown(void) {}

// Phase a at its peak: the whole vector lies on the alpha axis.
static void test_clarke_phase_a_peak(void)
{
    abc_t in = { .a = 1.0f, .b = -0.5f, .c = -0.5f };
    alpha_beta_t out;

    clarke(&in, &out);

    TEST_ASSERT_FLOAT_WITHIN(TOL, 1.0f, out.alpha);
    TEST_ASSERT_FLOAT_WITHIN(TOL, 0.0f, out.beta);
}

static void test_clarke_phase_b_peak(void)
{
    abc_t in = { .a = -0.5f, .b = 1.0f, .c = -0.5f };
    alpha_beta_t out;

    clarke(&in, &out);

    TEST_ASSERT_FLOAT_WITHIN(TOL, -0.5f, out.alpha);
    TEST_ASSERT_FLOAT_WITHIN(TOL, 0.866f, out.beta);
}

static void test_clarke_phase_c_peak(void)
{
    abc_t in = { .a = -0.5f, .b = -0.5f, .c = 1.0f };
    alpha_beta_t out;

    clarke(&in, &out);

    TEST_ASSERT_FLOAT_WITHIN(TOL, -0.5f, out.alpha);
    TEST_ASSERT_FLOAT_WITHIN(TOL, -0.866f, out.beta);
}

// Balanced phases at any angle: the (alpha, beta) vector stays the length of one phase's peak.
static void test_clarke_balanced_sweep_magnitude(void)
{
    const float peak = 2.0f;

    for (int deg = 0; deg < 360; deg++) {
        float theta = (float)deg * PI_F / 180.0f;

        abc_t in = {
            .a = peak * cosf(theta),
            .b = peak * cosf(theta - 2.0f * PI_F / 3.0f),
            .c = peak * cosf(theta + 2.0f * PI_F / 3.0f),
        };
        alpha_beta_t out;

        clarke(&in, &out);

        float magnitude = sqrtf(out.alpha * out.alpha + out.beta * out.beta);
        TEST_ASSERT_FLOAT_WITHIN(TOL, peak, magnitude);
    }
}

static void test_park_alpha_zero_angle(void)
{
    alpha_beta_t in = { .alpha = 1.0f, .beta = 0.0f };
    float theta = 0.0f;
    sin_cos_t sc;
    theta_to_sin_cos(theta, &sc);
    dq_t out;

    park(&in, &out, &sc);

    TEST_ASSERT_FLOAT_WITHIN(TOL, 1.0f, out.d);
    TEST_ASSERT_FLOAT_WITHIN(TOL, 0.0f, out.q);
}

static void test_park_beta_zero_angle(void)
{
    alpha_beta_t in = { .alpha = 0.0f, .beta = 1.0f };
    float theta = 0.0f;
    sin_cos_t sc;
    theta_to_sin_cos(theta, &sc);
    dq_t out;

    park(&in, &out, &sc);

    TEST_ASSERT_FLOAT_WITHIN(TOL, 0.0f, out.d);
    TEST_ASSERT_FLOAT_WITHIN(TOL, 1.0f, out.q);
}

static void test_park_alpha_90_angle(void)
{
    alpha_beta_t in = { .alpha = 1.0f, .beta = 0.0f };
    float theta = PI_F / 2.0f;
    sin_cos_t sc;
    theta_to_sin_cos(theta, &sc);
    dq_t out;

    park(&in, &out, &sc);

    TEST_ASSERT_FLOAT_WITHIN(TOL, 0.0f, out.d);
    TEST_ASSERT_FLOAT_WITHIN(TOL, -1.0f, out.q);
}



int main(void)
{
    UNITY_BEGIN();
    RUN_TEST(test_clarke_phase_a_peak);
    RUN_TEST(test_clarke_phase_b_peak);
    RUN_TEST(test_clarke_phase_c_peak);
    RUN_TEST(test_clarke_balanced_sweep_magnitude);
    RUN_TEST(test_park_alpha_zero_angle);
    RUN_TEST(test_park_beta_zero_angle);
    RUN_TEST(test_park_alpha_90_angle);
    return UNITY_END();
}

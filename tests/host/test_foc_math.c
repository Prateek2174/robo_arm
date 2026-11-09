// Transforms and SVPWM against known values.
// Module: #37   Milestone: M2

#include "unity.h"
#include "foc_math.h"

#define TOL 1e-4f

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

// TODO(Prateek): 360 degree magnitude sweep.

int main(void)
{
    UNITY_BEGIN();
    RUN_TEST(test_clarke_phase_a_peak);
    RUN_TEST(test_clarke_phase_b_peak);
    RUN_TEST(test_clarke_phase_c_peak);
    return UNITY_END();
}

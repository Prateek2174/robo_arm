#include "unity.h"
#include "calibration.h"
#include <math.h>

#define TOLERANCE  1e-4f
#define TWO_PI_F   6.28318531f
#define PI_T       3.14159265f
#define DEG(x)     ((x) * PI_T / 180.0f)

void setUp(void) {}
void tearDown(void) {}

//========================================================
// Wrap: a negative angle, exactly 2π, and 5π
//========================================================

static void test_wrap_negative_angle(void)
{
    TEST_ASSERT_FLOAT_WITHIN(TOLERANCE, TWO_PI_F - 0.1f, calibration_wrap_2pi(-0.1f));
}

static void test_wrap_exactly_two_pi_is_zero(void)
{
    TEST_ASSERT_FLOAT_WITHIN(TOLERANCE, 0.0f, calibration_wrap_2pi(TWO_PI_F));
}

static void test_wrap_five_pi_is_pi(void)
{
    TEST_ASSERT_FLOAT_WITHIN(TOLERANCE, PI_T, calibration_wrap_2pi(5.0f * PI_T));
}

static void test_wrap_tiny_negative_stays_below_two_pi(void)
{
    // float rounding would give exactly 2π here without the >= 2π guard
    float w = calibration_wrap_2pi(-1e-8f);
    TEST_ASSERT_TRUE(w >= 0.0f);
    TEST_ASSERT_TRUE(w < TWO_PI_F);
}

//========================================================
// Difference across 0 (350° → 10° is +20°)
//========================================================

static void test_angle_diff_across_zero_forward(void)
{
    TEST_ASSERT_FLOAT_WITHIN(TOLERANCE, DEG(20.0f), calibration_angle_diff(DEG(350.0f), DEG(10.0f)));
}

static void test_angle_diff_across_zero_backward(void)
{
    TEST_ASSERT_FLOAT_WITHIN(TOLERANCE, DEG(-20.0f), calibration_angle_diff(DEG(10.0f), DEG(350.0f)));
}

//========================================================
// Summed travel (start angle crosses the wrap)
//========================================================

static void test_travel_first_sample_adds_nothing(void)
{
    calibration_travel_t t;
    calibration_travel_reset(&t);
    calibration_travel_add(&t, 3.0f);
    TEST_ASSERT_FLOAT_WITHIN(TOLERANCE, 0.0f, t.travel_m);
}

static void test_travel_across_wrap(void)
{
    // start 5.90, ten steps of 0.359 -> ends at W(9.49) = 3.21; naive end-start is -2.69
    calibration_travel_t t;
    calibration_travel_reset(&t);
    for (int k = 0; k <= 10; k++)
    {
        calibration_travel_add(&t, calibration_wrap_2pi(5.90f + (float)k * 0.359f));
    }
    TEST_ASSERT_FLOAT_WITHIN(1e-3f, 3.59f, t.travel_m);
}

//========================================================
// Circular mean of 359°, 0° and 2°
//========================================================

static void test_circular_mean_across_zero(void)
{
    calibration_circular_t c;
    float mean, r;
    calibration_circular_reset(&c);
    calibration_circular_add(&c, DEG(359.0f));
    calibration_circular_add(&c, DEG(0.0f));
    calibration_circular_add(&c, DEG(2.0f));

    TEST_ASSERT_TRUE(calibration_circular_mean(&c, &mean, &r));
    TEST_ASSERT_FLOAT_WITHIN(DEG(0.01f), DEG(0.333f), mean);   // plain mean would be 120.3°
    TEST_ASSERT_TRUE(r > 0.999f);
}

static void test_circular_mean_empty_returns_false(void)
{
    calibration_circular_t c;
    float mean, r;
    calibration_circular_reset(&c);
    TEST_ASSERT_FALSE(calibration_circular_mean(&c, &mean, &r));
}

static void test_circular_mean_scattered_has_low_r(void)
{
    calibration_circular_t c;
    float mean, r;
    calibration_circular_reset(&c);
    for (int k = 0; k < 4; k++)
    {
        calibration_circular_add(&c, DEG(90.0f * (float)k));   // 0, 90, 180, 270 cancel out
    }
    TEST_ASSERT_TRUE(calibration_circular_mean(&c, &mean, &r));
    TEST_ASSERT_TRUE(r < 0.01f);
}

//========================================================
// Direction
//========================================================

static void test_direction_positive(void)
{
    int8_t dir = 0;
    TEST_ASSERT_EQUAL_INT(CALIBRATION_OK, calibration_detect_direction(3.59f, 0.1f, &dir));
    TEST_ASSERT_EQUAL_INT(1, dir);
}

static void test_direction_negative(void)
{
    int8_t dir = 0;
    TEST_ASSERT_EQUAL_INT(CALIBRATION_OK, calibration_detect_direction(-3.59f, 0.1f, &dir));
    TEST_ASSERT_EQUAL_INT(-1, dir);
}

static void test_direction_no_motion(void)
{
    int8_t dir = 0;
    TEST_ASSERT_EQUAL_INT(CALIBRATION_ERR_NO_MOTION, calibration_detect_direction(0.001f, 0.1f, &dir));
    TEST_ASSERT_EQUAL_INT(0, dir);   // untouched on failure
}

//========================================================
// Pole pairs 7 and 14 (the quad motor and an 8308)
// Rejecting 7.4 travel ratio, and zero travel (motor stuck)
//========================================================

static void test_pole_pairs_7(void)
{
    uint8_t p = 0;
    TEST_ASSERT_EQUAL_INT(CALIBRATION_OK, calibration_estimate_pole_pairs(4.0f * TWO_PI_F, 3.590f, 0.2f, &p));
    TEST_ASSERT_EQUAL_UINT8(7, p);
}

static void test_pole_pairs_14(void)
{
    uint8_t p = 0;
    TEST_ASSERT_EQUAL_INT(CALIBRATION_OK, calibration_estimate_pole_pairs(4.0f * TWO_PI_F, 1.795f, 0.2f, &p));
    TEST_ASSERT_EQUAL_UINT8(14, p);
}

static void test_pole_pairs_negative_travel_uses_magnitude(void)
{
    uint8_t p = 0;
    TEST_ASSERT_EQUAL_INT(CALIBRATION_OK, calibration_estimate_pole_pairs(4.0f * TWO_PI_F, -3.590f, 0.2f, &p));
    TEST_ASSERT_EQUAL_UINT8(7, p);
}

static void test_pole_pairs_rejects_7_point_4(void)
{
    uint8_t p = 99;
    TEST_ASSERT_EQUAL_INT(CALIBRATION_ERR_POLE_PAIRS, calibration_estimate_pole_pairs(4.0f * TWO_PI_F, 3.396f, 0.2f, &p));
    TEST_ASSERT_EQUAL_UINT8(99, p);   // untouched on failure
}

static void test_pole_pairs_rejects_zero_travel(void)
{
    uint8_t p = 99;
    TEST_ASSERT_EQUAL_INT(CALIBRATION_ERR_NO_MOTION, calibration_estimate_pole_pairs(4.0f * TWO_PI_F, 0.0f, 0.2f, &p));
    TEST_ASSERT_EQUAL_UINT8(99, p);
}

static void test_pole_pairs_rejects_out_of_range(void)
{
    uint8_t p = 99;
    // ratio 60 is a clean integer but above the 1..50 limit
    TEST_ASSERT_EQUAL_INT(CALIBRATION_ERR_POLE_PAIRS, calibration_estimate_pole_pairs(60.0f * TWO_PI_F, TWO_PI_F, 0.2f, &p));
    TEST_ASSERT_EQUAL_UINT8(99, p);
}

//========================================================
// Offset sample, including direction −1
//========================================================

static void test_offset_sample_worked_row(void)
{
    // table row: theta_cmd 0, theta_m 0.1643, p 7 -> 7*0.1643 = 1.15
    TEST_ASSERT_FLOAT_WITHIN(1e-3f, 1.15f, calibration_offset_sample(0.1643f, 0.0f, 7, 1));
}

static void test_offset_sample_dir_negative(void)
{
    // fake motor with s = -1: theta_m = W((theta_cmd + off) / (s*p))
    float off = 1.20f, theta_cmd = 2.0f;
    float theta_m = calibration_wrap_2pi((theta_cmd + off) / -7.0f);
    TEST_ASSERT_FLOAT_WITHIN(1e-3f, off, calibration_offset_sample(theta_m, theta_cmd, 7, -1));
}

//========================================================
// is_valid rejecting pole pairs 0, direction 0, a NaN offset and an offset ≥ 2π
//========================================================

static calibration_data_t good_data(void)
{
    calibration_data_t d = { .pole_pairs = 7, .dir = 1, .theta_offset = 1.2f, .offset_ref = CALIBRATION_REF_ABSOLUTE };
    return d;
}

static void test_is_valid_accepts_good(void)
{
    calibration_data_t d = good_data();
    TEST_ASSERT_TRUE(calibration_is_valid(&d, 7));
    TEST_ASSERT_TRUE(calibration_is_valid(&d, 0));   // 0 = no expected value, accept any 1..50
}

static void test_is_valid_rejects_bad_fields(void)
{
    calibration_data_t d;

    d = good_data(); d.pole_pairs = 0;          TEST_ASSERT_FALSE(calibration_is_valid(&d, 0));
    d = good_data(); d.pole_pairs = 51;         TEST_ASSERT_FALSE(calibration_is_valid(&d, 0));
    d = good_data(); d.dir = 0;                 TEST_ASSERT_FALSE(calibration_is_valid(&d, 7));
    d = good_data(); d.theta_offset = NAN;      TEST_ASSERT_FALSE(calibration_is_valid(&d, 7));
    d = good_data(); d.theta_offset = TWO_PI_F; TEST_ASSERT_FALSE(calibration_is_valid(&d, 7));
    d = good_data(); d.theta_offset = -0.1f;    TEST_ASSERT_FALSE(calibration_is_valid(&d, 7));
    d = good_data();                            TEST_ASSERT_FALSE(calibration_is_valid(&d, 9));
}

//========================================================
// End to end: a fake motor driven by calibration_step()
//========================================================

typedef struct {
    int      p;
    int      s;
    float    off;          // true electrical offset
    float    lag;          // rotor trails the field by this much (rad electrical)
    float    noise_m;      // +- encoder noise during OFFSET phases (rad mechanical)
    bool     stuck;        // rotor never moves
    uint8_t  expected_p;
} fake_motor_t;

static uint32_t lcg_state;
static float lcg_unit(void)   // deterministic -1..1
{
    lcg_state = lcg_state * 1664525u + 1013904223u;
    return (float)(lcg_state >> 8) / 8388608.0f - 1.0f;
}

static calibration_config_t make_cfg(uint8_t p_turns, uint8_t expected)
{
    calibration_config_t cfg = {
        .v_d = 1.0f, .steps_per_ece_turn = 64, .align_steps = 10, .detect_turns = 4,
        .offset_turns = p_turns, .min_travel_m = 0.5f, .pole_pairs_tolerance = 0.2f,
        .r_min = 0.95f, .expected_pole_pairs = expected,
    };
    return cfg;
}

// Runs the sequence to DONE/FAILED. Returns the final result and fills *out.
static calibration_result_t run_fake(const fake_motor_t *m, calibration_t *cal, calibration_data_t *out)
{
    calibration_config_t cfg = make_cfg((uint8_t)m->p, m->expected_p);
    calibration_start(cal, &cfg);

    float theta_m = 2.0f;      // arbitrary power-up angle
    float prev_cmd = 0.0f;
    lcg_state = 12345u;

    for (int calls = 0; calls < 100000; calls++)
    {
        calibration_state_t st = calibration_state(cal);
        if (st == CALIBRATION_STATE_DONE || st == CALIBRATION_STATE_FAILED)
        {
            break;
        }

        calibration_output_t o = calibration_step(cal, theta_m);
        if (o.drive && !m->stuck)
        {
            float d = o.theta_cmd - prev_cmd;
            float sgn = (d > 0.0f) ? 1.0f : ((d < 0.0f) ? -1.0f : 0.0f);
            float theta_e = o.theta_cmd - sgn * m->lag;
            theta_m = calibration_wrap_2pi((theta_e + m->off) / ((float)m->s * (float)m->p));

            calibration_state_t now = calibration_state(cal);
            if (now == CALIBRATION_STATE_OFFSET_BACK || now == CALIBRATION_STATE_OFFSET_FWD)
            {
                theta_m += m->noise_m * lcg_unit();
            }
            prev_cmd = o.theta_cmd;
        }
    }

    return calibration_result(cal, out);
}

static void test_e2e_p7_forward(void)
{
    fake_motor_t m = { .p = 7, .s = 1, .off = 1.20f, .expected_p = 7 };
    calibration_t cal; calibration_data_t d = {0};
    TEST_ASSERT_EQUAL_INT(CALIBRATION_OK, run_fake(&m, &cal, &d));
    TEST_ASSERT_EQUAL_UINT8(7, d.pole_pairs);
    TEST_ASSERT_EQUAL_INT(1, d.dir);
    TEST_ASSERT_FLOAT_WITHIN(1e-3f, 1.20f, d.theta_offset);
    TEST_ASSERT_EQUAL_INT(CALIBRATION_REF_ABSOLUTE, d.offset_ref);
}

static void test_e2e_dir_negative(void)
{
    fake_motor_t m = { .p = 7, .s = -1, .off = 1.20f, .expected_p = 7 };
    calibration_t cal; calibration_data_t d = {0};
    TEST_ASSERT_EQUAL_INT(CALIBRATION_OK, run_fake(&m, &cal, &d));
    TEST_ASSERT_EQUAL_INT(-1, d.dir);
    TEST_ASSERT_FLOAT_WITHIN(1e-3f, 1.20f, d.theta_offset);
}

static void test_e2e_offset_near_seam(void)
{
    fake_motor_t m = { .p = 7, .s = 1, .off = 6.20f, .expected_p = 7 };
    calibration_t cal; calibration_data_t d = {0};
    TEST_ASSERT_EQUAL_INT(CALIBRATION_OK, run_fake(&m, &cal, &d));
    TEST_ASSERT_FLOAT_WITHIN(1e-3f, 6.20f, d.theta_offset);
}

static void test_e2e_p14(void)
{
    fake_motor_t m = { .p = 14, .s = 1, .off = 0.05f, .expected_p = 14 };
    calibration_t cal; calibration_data_t d = {0};
    TEST_ASSERT_EQUAL_INT(CALIBRATION_OK, run_fake(&m, &cal, &d));
    TEST_ASSERT_EQUAL_UINT8(14, d.pole_pairs);
    TEST_ASSERT_FLOAT_WITHIN(1e-3f, 0.05f, d.theta_offset);
}

static void test_e2e_lag_cancels(void)
{
    fake_motor_t m = { .p = 7, .s = 1, .off = 1.20f, .lag = 0.05f, .expected_p = 7 };
    calibration_t cal; calibration_data_t d = {0};
    TEST_ASSERT_EQUAL_INT(CALIBRATION_OK, run_fake(&m, &cal, &d));
    TEST_ASSERT_FLOAT_WITHIN(1e-3f, 1.20f, d.theta_offset);
}

static void test_e2e_wrong_expected_pole_pairs_fails(void)
{
    fake_motor_t m = { .p = 7, .s = 1, .off = 1.20f, .expected_p = 9 };
    calibration_t cal; calibration_data_t d = { .pole_pairs = 99 };
    TEST_ASSERT_EQUAL_INT(CALIBRATION_ERR_POLE_PAIRS, run_fake(&m, &cal, &d));
    TEST_ASSERT_EQUAL_INT(CALIBRATION_STATE_FAILED, calibration_state(&cal));
    TEST_ASSERT_EQUAL_UINT8(99, d.pole_pairs);   // result() must not touch d_out on failure
}

static void test_e2e_stuck_motor_fails_no_motion(void)
{
    fake_motor_t m = { .p = 7, .s = 1, .off = 1.20f, .stuck = true, .expected_p = 7 };
    calibration_t cal; calibration_data_t d = {0};
    TEST_ASSERT_EQUAL_INT(CALIBRATION_ERR_NO_MOTION, run_fake(&m, &cal, &d));
}

static void test_e2e_noisy_offsets_fail_spread(void)
{
    // clean DETECT, then +-0.45 rad mech noise (= +-3.1 rad electrical) during OFFSET
    fake_motor_t m = { .p = 7, .s = 1, .off = 1.20f, .noise_m = 0.45f, .expected_p = 7 };
    calibration_t cal; calibration_data_t d = {0};
    TEST_ASSERT_EQUAL_INT(CALIBRATION_ERR_SPREAD, run_fake(&m, &cal, &d));
}

//========================================================
// Sequence details
//========================================================

static void test_start_rejects_zero_steps(void)
{
    calibration_config_t cfg = make_cfg(7, 7);
    cfg.steps_per_ece_turn = 0;
    calibration_t cal;
    calibration_start(&cal, &cfg);
    TEST_ASSERT_EQUAL_INT(CALIBRATION_STATE_FAILED, calibration_state(&cal));
    TEST_ASSERT_EQUAL_INT(CALIBRATION_ERR_BAD_DATA, cal.error);
}

static void test_result_not_done_while_running(void)
{
    calibration_config_t cfg = make_cfg(7, 7);
    calibration_t cal;
    calibration_data_t d = { .pole_pairs = 99 };
    calibration_start(&cal, &cfg);
    (void)calibration_step(&cal, 0.0f);
    TEST_ASSERT_EQUAL_INT(CALIBRATION_ERR_NOT_DONE, calibration_result(&cal, &d));
    TEST_ASSERT_EQUAL_UINT8(99, d.pole_pairs);
}

static void test_align_holds_zero_then_detect_steps_forward(void)
{
    calibration_config_t cfg = make_cfg(7, 7);
    calibration_t cal;
    calibration_start(&cal, &cfg);

    for (int k = 0; k < cfg.align_steps; k++)
    {
        calibration_output_t o = calibration_step(&cal, 0.0f);
        TEST_ASSERT_TRUE(o.drive);
        TEST_ASSERT_FLOAT_WITHIN(TOLERANCE, 0.0f, o.theta_cmd);
        TEST_ASSERT_FLOAT_WITHIN(TOLERANCE, 1.0f, o.v_d);
    }

    calibration_output_t t = calibration_step(&cal, 0.0f);   // transition call keeps driving
    TEST_ASSERT_TRUE(t.drive);
    TEST_ASSERT_EQUAL_INT(CALIBRATION_STATE_DETECT, calibration_state(&cal));

    float step = TWO_PI_F / 64.0f;
    calibration_output_t a = calibration_step(&cal, 0.0f);
    calibration_output_t b = calibration_step(&cal, 0.0f);
    TEST_ASSERT_TRUE(a.drive && b.drive);
    TEST_ASSERT_FLOAT_WITHIN(TOLERANCE, step, a.theta_cmd);
    TEST_ASSERT_FLOAT_WITHIN(TOLERANCE, 2.0f * step, b.theta_cmd);
}

int main(void)
{
    UNITY_BEGIN();

    RUN_TEST(test_wrap_negative_angle);
    RUN_TEST(test_wrap_exactly_two_pi_is_zero);
    RUN_TEST(test_wrap_five_pi_is_pi);
    RUN_TEST(test_wrap_tiny_negative_stays_below_two_pi);

    RUN_TEST(test_angle_diff_across_zero_forward);
    RUN_TEST(test_angle_diff_across_zero_backward);

    RUN_TEST(test_travel_first_sample_adds_nothing);
    RUN_TEST(test_travel_across_wrap);

    RUN_TEST(test_circular_mean_across_zero);
    RUN_TEST(test_circular_mean_empty_returns_false);
    RUN_TEST(test_circular_mean_scattered_has_low_r);

    RUN_TEST(test_direction_positive);
    RUN_TEST(test_direction_negative);
    RUN_TEST(test_direction_no_motion);

    RUN_TEST(test_pole_pairs_7);
    RUN_TEST(test_pole_pairs_14);
    RUN_TEST(test_pole_pairs_negative_travel_uses_magnitude);
    RUN_TEST(test_pole_pairs_rejects_7_point_4);
    RUN_TEST(test_pole_pairs_rejects_zero_travel);
    RUN_TEST(test_pole_pairs_rejects_out_of_range);

    RUN_TEST(test_offset_sample_worked_row);
    RUN_TEST(test_offset_sample_dir_negative);

    RUN_TEST(test_is_valid_accepts_good);
    RUN_TEST(test_is_valid_rejects_bad_fields);

    RUN_TEST(test_e2e_p7_forward);
    RUN_TEST(test_e2e_dir_negative);
    RUN_TEST(test_e2e_offset_near_seam);
    RUN_TEST(test_e2e_p14);
    RUN_TEST(test_e2e_lag_cancels);
    RUN_TEST(test_e2e_wrong_expected_pole_pairs_fails);
    RUN_TEST(test_e2e_stuck_motor_fails_no_motion);
    RUN_TEST(test_e2e_noisy_offsets_fail_spread);

    RUN_TEST(test_start_rejects_zero_steps);
    RUN_TEST(test_result_not_done_while_running);
    RUN_TEST(test_align_holds_zero_then_detect_steps_forward);

    return UNITY_END();
}

// Hall-sensor decode, direction, speed and interpolation.
// Module: #17   Milestone: M5

#include "unity.h"
#include "hall.h"
#include <math.h>

#define TOLERANCE 1e-4f
#define PI_F 3.14159265f
#define SECTOR_RAD (PI_F / 3.0f)

// Lesson's example order: sector s is entered by code FWD_CODES[s] going forward.
// sector:            0      1      2      3      4      5
static const uint8_t FWD_CODES[6] = { 0b101, 0b100, 0b110, 0b010, 0b011, 0b001 };

// 1 us ticks. 1000 ticks per sector = 1 ms -> omega = (pi/3)/1e-3 ~ 1047 rad/s (interpolates).
#define TICK_S        1e-6f
#define DT_FAST       1000u
#define OMEGA_FAST    (SECTOR_RAD / (DT_FAST * TICK_S))
#define OMEGA_ON      50.0f
#define OMEGA_OFF     20.0f
#define DT_BETWEEN    34907u   // omega ~ 30 rad/s: between OFF and ON
#define DT_SLOW       104720u  // omega ~ 10 rad/s: below OFF

static hall_t hall;

void setUp(void) {}
void tearDown(void) {}

//========================================================
// HELPERS
//========================================================

static hall_config_t make_cfg(void)
{
    hall_config_t cfg = {
        .sector_lut = { -1, 5, 3, 4, 1, 0, 2, -1 }, // index = code, value = sector
        .dir_sign = 1,
        .pole_pairs = 15,
        .tick_s = TICK_S,
        .omega_interpolate_on = OMEGA_ON,
        .omega_interpolate_off = OMEGA_OFF,
        .timeout_s = 0.065f,
        .theta_offset_e = 0.0f,
    };
    return cfg;
}

static uint8_t code_for(int sector)
{
    return FWD_CODES[((sector % 6) + 6) % 6];
}

// Init with the default config and start in sector s.
static void start_in(int sector)
{
    hall_config_t cfg = make_cfg();
    TEST_ASSERT_TRUE(hall_init(&hall, &cfg));
    TEST_ASSERT_EQUAL(HALL_OK, hall_set_initial_sector(&hall, code_for(sector)));
}

// Feed n forward (step = +1) or backward (step = -1) edges, all with the same dt.
// Returns the sector it ends in.
static int spin(int from_sector, int step, int n, uint32_t dt)
{
    int s = from_sector;
    for (int i = 0; i < n; i++)
    {
        s += step;
        TEST_ASSERT_EQUAL(HALL_OK, hall_on_edge(&hall, code_for(s), dt));
    }
    return ((s % 6) + 6) % 6;
}

// Angles near 0 and near 2pi are the same angle.
static void assert_angle(float expected, float actual)
{
    float diff = fabsf(expected - actual);
    float wrapped = fabsf(diff - 2.0f * PI_F);
    TEST_ASSERT_TRUE_MESSAGE(diff < TOLERANCE || wrapped < TOLERANCE, "angle mismatch");
}

static float centre(int sector)
{
    return (float)sector * SECTOR_RAD + SECTOR_RAD / 2.0f;
}

//========================================================
// INIT
//========================================================

static void test_init_valid_config(void)
{
    hall_config_t cfg = make_cfg();

    TEST_ASSERT_TRUE(hall_init(&hall, &cfg));
    TEST_ASSERT_EQUAL_INT8(-1, hall.sector);
    TEST_ASSERT_EQUAL_UINT8(0, hall.period_count);
    TEST_ASSERT_TRUE(hall.ignore_next_dt);
}

static void test_init_rejects_duplicate_sector(void)
{
    hall_config_t cfg = make_cfg();
    cfg.sector_lut[1] = 0; // sector 0 now appears twice, sector 5 never

    TEST_ASSERT_FALSE(hall_init(&hall, &cfg));
}

static void test_init_rejects_out_of_range_sector(void)
{
    hall_config_t cfg = make_cfg();
    cfg.sector_lut[1] = 6;

    TEST_ASSERT_FALSE(hall_init(&hall, &cfg));
}

static void test_init_rejects_bad_thresholds_and_dir(void)
{
    hall_config_t cfg = make_cfg();
    cfg.omega_interpolate_off = cfg.omega_interpolate_on;
    TEST_ASSERT_FALSE(hall_init(&hall, &cfg));

    cfg = make_cfg();
    cfg.dir_sign = 0;
    TEST_ASSERT_FALSE(hall_init(&hall, &cfg));

    cfg = make_cfg();
    cfg.pole_pairs = 0;
    TEST_ASSERT_FALSE(hall_init(&hall, &cfg));

    cfg = make_cfg();
    cfg.tick_s = 0.0f;
    TEST_ASSERT_FALSE(hall_init(&hall, &cfg));
}

// Your init forces codes 000 and 111 to -1 rather than rejecting them.
static void test_init_forces_impossible_codes_invalid(void)
{
    hall_config_t cfg = make_cfg();
    cfg.sector_lut[0] = 3;
    cfg.sector_lut[7] = 4;

    TEST_ASSERT_TRUE(hall_init(&hall, &cfg));
    TEST_ASSERT_EQUAL_INT8(-1, hall.cfg.sector_lut[0]);
    TEST_ASSERT_EQUAL_INT8(-1, hall.cfg.sector_lut[7]);
}

//========================================================
// INITIAL SECTOR
//========================================================

static void test_set_initial_uses_sector_centre(void)
{
    start_in(2);

    TEST_ASSERT_EQUAL_INT8(2, hall.sector);
    TEST_ASSERT_EQUAL_INT8(0, hall.dir);
    assert_angle(centre(2), hall_get_theta_e(&hall, 0));
}

static void test_set_initial_rejects_invalid_code(void)
{
    hall_config_t cfg = make_cfg();
    TEST_ASSERT_TRUE(hall_init(&hall, &cfg));

    TEST_ASSERT_EQUAL(HALL_ERR_INVALID_CODE, hall_set_initial_sector(&hall, 0b000));
    TEST_ASSERT_EQUAL(HALL_ERR_INVALID_CODE, hall_set_initial_sector(&hall, 0b111));
    TEST_ASSERT_EQUAL_INT8(-1, hall.sector);
    TEST_ASSERT_EQUAL_UINT32(2, hall.error_count);
}

static void test_first_edge_without_initial_sets_sector(void)
{
    hall_config_t cfg = make_cfg();
    TEST_ASSERT_TRUE(hall_init(&hall, &cfg));

    TEST_ASSERT_EQUAL(HALL_OK, hall_on_edge(&hall, code_for(4), DT_FAST));
    TEST_ASSERT_EQUAL_INT8(4, hall.sector);
    TEST_ASSERT_EQUAL_INT32(0, hall.edge_count);
    assert_angle(centre(4), hall_get_theta_e(&hall, 0));
}

static void test_theta_unknown_sector_is_zero(void)
{
    hall_config_t cfg = make_cfg();
    TEST_ASSERT_TRUE(hall_init(&hall, &cfg));

    TEST_ASSERT_FLOAT_WITHIN(TOLERANCE, 0.0f, hall_get_theta_e(&hall, 123));
}

//========================================================
// DIRECTION, WRAP, COUNTING
//========================================================

static void test_forward_sequence(void)
{
    start_in(0);
    spin(0, +1, 6, DT_FAST); // a full electrical turn, ending back in sector 0

    TEST_ASSERT_EQUAL_INT8(0, hall.sector);
    TEST_ASSERT_EQUAL_INT8(1, hall.dir);
    TEST_ASSERT_EQUAL_INT32(6, hall.edge_count);
    TEST_ASSERT_EQUAL_INT32(6, hall_get_edge_count(&hall));
    // first edge's dt is ignored (time since boot), the other 5 are measured
    TEST_ASSERT_EQUAL_UINT8(5, hall.period_count);
    TEST_ASSERT_FLOAT_WITHIN(1.0f, OMEGA_FAST, hall_get_omega_e(&hall));
}

static void test_wrap_5_to_0_is_forward(void)
{
    start_in(5);

    TEST_ASSERT_EQUAL(HALL_OK, hall_on_edge(&hall, code_for(0), DT_FAST));
    TEST_ASSERT_EQUAL_INT8(1, hall.dir);
    TEST_ASSERT_EQUAL_INT32(1, hall.edge_count);
}

static void test_wrap_0_to_5_is_backward(void)
{
    start_in(0);

    TEST_ASSERT_EQUAL(HALL_OK, hall_on_edge(&hall, code_for(5), DT_FAST));
    TEST_ASSERT_EQUAL_INT8(-1, hall.dir);
    TEST_ASSERT_EQUAL_INT32(-1, hall.edge_count);
}

static void test_backward_sequence(void)
{
    start_in(3);
    spin(3, -1, 5, DT_FAST);

    TEST_ASSERT_EQUAL_INT8(4, hall.sector); // 3 -> 2 -> 1 -> 0 -> 5 -> 4
    TEST_ASSERT_EQUAL_INT8(-1, hall.dir);
    TEST_ASSERT_EQUAL_INT32(-5, hall.edge_count);
    TEST_ASSERT_FLOAT_WITHIN(1.0f, -OMEGA_FAST, hall_get_omega_e(&hall));
}

static void test_edge_angle_forward_is_sector_start(void)
{
    start_in(0);
    hall_on_edge(&hall, code_for(1), DT_FAST);

    assert_angle(1.0f * SECTOR_RAD, hall.theta_edge);
}

static void test_edge_angle_backward_is_sector_end(void)
{
    start_in(2);
    hall_on_edge(&hall, code_for(1), DT_FAST);

    assert_angle(2.0f * SECTOR_RAD, hall.theta_edge); // end of sector 1
}

static void test_edge_angle_backward_into_5_wraps(void)
{
    start_in(0);
    hall_on_edge(&hall, code_for(5), DT_FAST);

    assert_angle(0.0f, hall.theta_edge); // (5+1)*60 = 360 = 0
    TEST_ASSERT_TRUE(hall.theta_edge >= 0.0f && hall.theta_edge <= 2.0f * PI_F);
}

static void test_dir_sign_flips_outputs(void)
{
    hall_config_t cfg = make_cfg();
    cfg.dir_sign = -1;
    TEST_ASSERT_TRUE(hall_init(&hall, &cfg));
    hall_set_initial_sector(&hall, code_for(0));
    spin(0, +1, 4, DT_FAST);

    TEST_ASSERT_EQUAL_INT32(-4, hall_get_edge_count(&hall));
    TEST_ASSERT_TRUE(hall_get_omega_e(&hall) < 0.0f);
}

//========================================================
// ERRORS
//========================================================

static void test_invalid_code_leaves_state(void)
{
    start_in(0);
    spin(0, +1, 2, DT_FAST);
    float theta_before = hall.theta_edge;

    TEST_ASSERT_EQUAL(HALL_ERR_INVALID_CODE, hall_on_edge(&hall, 0b000, DT_FAST));
    TEST_ASSERT_EQUAL(HALL_ERR_INVALID_CODE, hall_on_edge(&hall, 0b111, DT_FAST));

    TEST_ASSERT_EQUAL_INT8(2, hall.sector);
    TEST_ASSERT_EQUAL_INT32(2, hall.edge_count);
    TEST_ASSERT_EQUAL_UINT32(2, hall.error_count);
    TEST_ASSERT_FLOAT_WITHIN(TOLERANCE, theta_before, hall.theta_edge);
}

static void test_same_code_is_ignored(void)
{
    start_in(0);
    spin(0, +1, 3, DT_FAST);

    TEST_ASSERT_EQUAL(HALL_OK, hall_on_edge(&hall, code_for(3), DT_FAST));
    TEST_ASSERT_EQUAL_INT32(3, hall.edge_count);
    TEST_ASSERT_EQUAL_INT8(3, hall.sector);
}

static void test_skipped_sector_resyncs(void)
{
    start_in(0);
    spin(0, +1, 4, DT_FAST); // now in sector 4, interpolating
    TEST_ASSERT_TRUE(hall.interpolation_active);

    TEST_ASSERT_EQUAL(HALL_ERR_SKIPPED_SECTOR, hall_on_edge(&hall, code_for(0), DT_FAST)); // 4 -> 0 is +2

    TEST_ASSERT_EQUAL_INT8(0, hall.sector);
    TEST_ASSERT_EQUAL_INT8(0, hall.dir);
    TEST_ASSERT_FALSE(hall.interpolation_active);
    TEST_ASSERT_EQUAL_UINT8(0, hall.period_count);
    TEST_ASSERT_EQUAL_UINT32(1, hall.error_count);
    TEST_ASSERT_EQUAL_INT32(4, hall.edge_count); // odometry not changed by a skip
    assert_angle(centre(0), hall.theta_edge);

    // dt of the edge right after a skip is ignored, the one after that is measured
    hall_on_edge(&hall, code_for(1), DT_FAST);
    TEST_ASSERT_EQUAL_UINT8(0, hall.period_count);
    hall_on_edge(&hall, code_for(2), DT_FAST);
    TEST_ASSERT_EQUAL_UINT8(1, hall.period_count);
}

static void test_skip_by_three(void)
{
    start_in(0);
    spin(0, +1, 2, DT_FAST);

    TEST_ASSERT_EQUAL(HALL_ERR_SKIPPED_SECTOR, hall_on_edge(&hall, code_for(5), DT_FAST)); // 2 -> 5 is +3
    TEST_ASSERT_EQUAL_INT8(5, hall.sector);
}

//========================================================
// SPEED
//========================================================

static void test_reversal_clears_speed(void)
{
    start_in(0);
    int s = spin(0, +1, 5, DT_FAST);
    TEST_ASSERT_TRUE(hall.interpolation_active);

    TEST_ASSERT_EQUAL(HALL_OK, hall_on_edge(&hall, code_for(s - 1), DT_FAST));

    TEST_ASSERT_EQUAL_INT8(-1, hall.dir);
    TEST_ASSERT_FLOAT_WITHIN(TOLERANCE, 0.0f, hall_get_omega_e(&hall));
    TEST_ASSERT_FALSE(hall.interpolation_active);
    TEST_ASSERT_EQUAL_UINT8(0, hall.period_count); // partial period never entered the average
    TEST_ASSERT_EQUAL_INT32(4, hall.edge_count);
}

static void test_ring_buffer_caps_at_six(void)
{
    start_in(0);
    spin(0, +1, 20, DT_FAST);

    TEST_ASSERT_EQUAL_UINT8(6, hall.period_count);
    TEST_ASSERT_TRUE(hall.period_index < 6);
    TEST_ASSERT_FLOAT_WITHIN(1.0f, OMEGA_FAST, hall_get_omega_e(&hall));
}

static void test_average_cancels_placement_error(void)
{
    // Sectors alternate 0.8 ms / 1.2 ms (sensor placement error); mean is still 1 ms.
    start_in(0);
    hall_on_edge(&hall, code_for(1), DT_FAST); // ignored dt
    for (int s = 2; s < 8; s++)
    {
        hall_on_edge(&hall, code_for(s), (s % 2) ? 800u : 1200u);
    }

    TEST_ASSERT_EQUAL_UINT8(6, hall.period_count);
    TEST_ASSERT_FLOAT_WITHIN(1.0f, OMEGA_FAST, hall_get_omega_e(&hall));
}

static void test_timeout_zeroes_speed_and_ignores_next_dt(void)
{
    start_in(0);
    int s = spin(0, +1, 6, DT_FAST);

    hall_on_timeout(&hall);

    TEST_ASSERT_FLOAT_WITHIN(TOLERANCE, 0.0f, hall_get_omega_e(&hall));
    TEST_ASSERT_FALSE(hall.interpolation_active);
    TEST_ASSERT_EQUAL_UINT8(0, hall.period_count);
    TEST_ASSERT_EQUAL_INT8((int8_t)s, hall.sector); // still in the same sector

    hall_on_edge(&hall, code_for(s + 1), 65000u); // wrapped CCR1, must be ignored
    TEST_ASSERT_EQUAL_UINT8(0, hall.period_count);
    hall_on_edge(&hall, code_for(s + 2), DT_FAST);
    TEST_ASSERT_EQUAL_UINT8(1, hall.period_count);
}

//========================================================
// INTERPOLATION
//========================================================

static void test_interpolates_forward_mid_sector(void)
{
    start_in(0);
    spin(0, +1, 7, DT_FAST); // ends in sector 1, entered at 60 deg
    TEST_ASSERT_TRUE(hall.interpolation_active);

    float theta_edge = hall.theta_edge;
    assert_angle(theta_edge + SECTOR_RAD / 2.0f, hall_get_theta_e(&hall, DT_FAST / 2));
}

static void test_interpolation_clamps_forward(void)
{
    start_in(0);
    spin(0, +1, 7, DT_FAST);

    // 3x the expected sector time: must stop at the sector boundary, not run on
    assert_angle(hall.theta_edge + SECTOR_RAD, hall_get_theta_e(&hall, 3u * DT_FAST));
}

static void test_interpolates_and_clamps_backward(void)
{
    start_in(0);
    spin(0, -1, 7, DT_FAST); // ends in sector 5, entered at its end (360 = 0 deg)
    TEST_ASSERT_TRUE(hall.interpolation_active);

    float theta_edge = hall.theta_edge;
    assert_angle(theta_edge - SECTOR_RAD / 2.0f, hall_get_theta_e(&hall, DT_FAST / 2));
    assert_angle(theta_edge - SECTOR_RAD, hall_get_theta_e(&hall, 3u * DT_FAST));

    float theta = hall_get_theta_e(&hall, DT_FAST / 2);
    TEST_ASSERT_TRUE(theta >= 0.0f && theta <= 2.0f * PI_F);
}

static void test_slow_uses_sector_centre(void)
{
    start_in(0);
    int s = spin(0, +1, 8, DT_SLOW);

    TEST_ASSERT_FALSE(hall.interpolation_active);
    assert_angle(centre(s), hall_get_theta_e(&hall, DT_SLOW / 2));
}

static void test_hysteresis_band(void)
{
    // Between OFF and ON from a standstill: stays off.
    start_in(0);
    int s = spin(0, +1, 8, DT_BETWEEN);
    TEST_ASSERT_FALSE(hall.interpolation_active);

    // Fast: turns on.
    s = spin(s, +1, 8, DT_FAST);
    TEST_ASSERT_TRUE(hall.interpolation_active);

    // Back into the band: stays on.
    s = spin(s, +1, 8, DT_BETWEEN);
    TEST_ASSERT_TRUE(hall.interpolation_active);

    // Below OFF: turns off.
    spin(s, +1, 8, DT_SLOW);
    TEST_ASSERT_FALSE(hall.interpolation_active);
}

//========================================================
// OFFSET
//========================================================

static void test_offset_shifts_and_wraps(void)
{
    hall_config_t cfg = make_cfg();
    cfg.theta_offset_e = 1.0f;
    TEST_ASSERT_TRUE(hall_init(&hall, &cfg));

    // sector 5 centre is 330 deg + 1 rad, past 2pi: must wrap
    hall_set_initial_sector(&hall, code_for(5));
    float theta = hall_get_theta_e(&hall, 0);
    assert_angle(centre(5) + 1.0f - 2.0f * PI_F, theta);
    TEST_ASSERT_TRUE(theta >= 0.0f && theta < 2.0f * PI_F);

    // forward edge into sector 0 lands on 0 + offset
    hall_on_edge(&hall, code_for(0), DT_FAST);
    assert_angle(1.0f, hall.theta_edge);
}

int main(void)
{
    UNITY_BEGIN();

    RUN_TEST(test_init_valid_config);
    RUN_TEST(test_init_rejects_duplicate_sector);
    RUN_TEST(test_init_rejects_out_of_range_sector);
    RUN_TEST(test_init_rejects_bad_thresholds_and_dir);
    RUN_TEST(test_init_forces_impossible_codes_invalid);

    RUN_TEST(test_set_initial_uses_sector_centre);
    RUN_TEST(test_set_initial_rejects_invalid_code);
    RUN_TEST(test_first_edge_without_initial_sets_sector);
    RUN_TEST(test_theta_unknown_sector_is_zero);

    RUN_TEST(test_forward_sequence);
    RUN_TEST(test_wrap_5_to_0_is_forward);
    RUN_TEST(test_wrap_0_to_5_is_backward);
    RUN_TEST(test_backward_sequence);
    RUN_TEST(test_edge_angle_forward_is_sector_start);
    RUN_TEST(test_edge_angle_backward_is_sector_end);
    RUN_TEST(test_edge_angle_backward_into_5_wraps);
    RUN_TEST(test_dir_sign_flips_outputs);

    RUN_TEST(test_invalid_code_leaves_state);
    RUN_TEST(test_same_code_is_ignored);
    RUN_TEST(test_skipped_sector_resyncs);
    RUN_TEST(test_skip_by_three);

    RUN_TEST(test_reversal_clears_speed);
    RUN_TEST(test_ring_buffer_caps_at_six);
    RUN_TEST(test_average_cancels_placement_error);
    RUN_TEST(test_timeout_zeroes_speed_and_ignores_next_dt);

    RUN_TEST(test_interpolates_forward_mid_sector);
    RUN_TEST(test_interpolation_clamps_forward);
    RUN_TEST(test_interpolates_and_clamps_backward);
    RUN_TEST(test_slow_uses_sector_centre);
    RUN_TEST(test_hysteresis_band);

    RUN_TEST(test_offset_shifts_and_wraps);

    return UNITY_END();
}

// Profile respects limits and reaches the end point.
// Module: #37   Milestone: M2
// TODO: implement (see robot-arm-todo.md for the 'done when' criteria).


#include "unity.h"
#include "trajectory.h"
#include <math.h>

#define TOLERANCE 1e-4f

void setUp(void) {}
void tearDown(void) {}

static void test_trajectory_trapezoid(void)
{
    
}

static void test_trajectory_boundary_no_cruise(void)
{
    
}

static void test_trajectory_triangle(void)
{
    
}

static void test_trajectory_mirrored_trapezoid(void)
{
    
}

static void test_trajectory_no_motion(void)
{
    
}

int main(void)
{
    UNITY_BEGIN();
    RUN_TEST(test_trajectory_trapezoid);
    RUN_TEST(test_trajectory_boundary_no_cruise);
    RUN_TEST(test_trajectory_triangle);
    RUN_TEST(test_trajectory_mirrored_trapezoid);
    RUN_TEST(test_trajectory_no_motion);
    return UNITY_END();
}
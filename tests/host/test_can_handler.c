#include "unity.h"
#include "can_handler.h"
#include <math.h>

#define TOLERANCE 1e-4f

void setUp(void) {}
void tearDown(void) {}

// a dispatch test for each message type, plus one for a frame addressed to another node

static void test_dispatch(void)
{
    
}

// fast and slow telemetry timing

static void test_(void)
{
    
}

// timeout trips, recovers, and stays latched until cleared

static void test_(void)
{
    
}

// no timeout in IDLE

static void test_(void)
{
    
}

// version mismatch

static void test_(void)
{
    
}

// one test where now_ms wraps past 0

static void test_(void)
{
    
}

int main(void)
{
    UNITY_BEGIN();

    return UNITY_END();
}
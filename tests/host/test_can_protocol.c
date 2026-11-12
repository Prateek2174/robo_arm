// Pack/unpack round trips, version mismatch.
// Module: #37   Milestone: M2
// TODO: implement (see robot-arm-todo.md for the 'done when' criteria).

#include "unity.h"
#include "can_protocol.h"
#include <math.h>

#define TOLERANCE 1e-4f

// Unity calls these around every test. Nothing to set up for pure math.
void setUp(void) {}
void tearDown(void) {}

static void test_(void)
{
    
}

int main(void)
{
    UNITY_BEGIN();

    return UNITY_END();
}
// Only legal transitions allowed.
// Module: #37   Milestone: M2
// TODO: implement (see robot-arm-todo.md for the 'done when' criteria).

#include "unity.h"
#include "state_machine.h"
#include <math.h>

#define TOLERANCE 1e-4f

void setUp(void) {}
void tearDown(void) {}

// How to prove every illegal transition is rejected

// With 5 states and N events, there are exactly 5×N combinations. Test all of them with a loop over an expected-result table, not a few hand-picked cases.

// for each state S:
//     for each event E:
//         reach S using only the public API
//         send E
//         check the result and the new state against your table
// Write the expected table by hand in the test. If the test reuses the implementation’s table, a wrong entry passes its own check.
// No test-only “set state” function. Reach each state through real requests, so there’s no backdoor that could end up in firmware.
// On rejection, check that the state didn’t change, not just that an error came back.
// Add these three specific cases: clear with an active cause, a second fault during FAULT, and a fault during CALIBRATING.

static void test_(void)
{
    
}

int main(void)
{
    UNITY_BEGIN();

    return UNITY_END();
}

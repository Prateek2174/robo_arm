#include "unity.h"
#include "hall.h"
#include <math.h>

#define TOLERANCE 1e-4f

void setUp(void) {}
void tearDown(void) {}

// a lookup table with a duplicate sector, or with code 000 mapped to a sector, makes hall_init return false.

static void test_(void)
{
    
}


int main(void)
{
    UNITY_BEGIN();

    return UNITY_END();
}
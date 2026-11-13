// Clarke, Park, inverse Park, SVPWM, sin/cos. NO HAL CALLS (host-testable).
// Module: #6   Milestone: M1-M2
// TODO: implement (see robot-arm-todo.md for the 'done when' criteria).

#include "foc_math.h"
#include <math.h>

#define ONE_OVER_SQRT3 0.57735027f

void theta_to_sin_cos(float theta, sin_cos_t *sc)
{
    sc->s = sinf(theta);
    sc->c = cosf(theta);
}

void clarke(const abc_t *abc, alpha_beta_t *alpha_beta)
{
    alpha_beta->alpha = abc->a;
    alpha_beta->beta = (2*abc->b + abc->a) * ONE_OVER_SQRT3;
}

void park(const alpha_beta_t *alpha_beta, dq_t *dq, const sin_cos_t *sc)
{
    dq->d = alpha_beta->alpha * sc->c + alpha_beta->beta * sc->s;
    dq->q = -alpha_beta->alpha * sc->s + alpha_beta->beta * sc->c;
}
// FOC math API.
// Module: #6   Milestone: M1-M2
// TODO: implement (see robot-arm-todo.md for the 'done when' criteria).

#ifndef FOC_MATH_H
#define FOC_MATH_H

typedef struct {

    float a;
    float b;
    float c;

} abc_t;

typedef struct {

    float alpha;
    float beta;

} alpha_beta_t;

typedef struct {

    float d;
    float q;

} dq_t;

typedef struct {

    float s;
    float c;

} sin_cos_t;

void theta_to_sin_cos(float theta, sin_cos_t *sc);
void clarke(const abc_t *abc, alpha_beta_t *alpha_beta);
void park(const alpha_beta_t *alpha_beta, dq_t *dq, const sin_cos_t *sc);


#endif // FOC_MATH_H

// Trajectory API.
// Module: #8   Milestone: M2
// TODO: implement (see robot-arm-todo.md for the 'done when' criteria).

#ifndef TRAJECTORY_H
#define TRAJECTORY_H

#include <stdbool.h>

//take D as the distance to move
//v as max velocity
//a as max acceleration

//accel to full speed takes: t_accel = v / a and covers v^2 / (2 * a) distance
//shortest move that reaches v_max is D_min = v^2 / a

typedef struct {
    float d; //distance to move  rad
    float v; //max velocity      rad/s
    float a; //max acceleration  rad/s^2
} trajectory_instant_t;

typedef struct {
    float v_max;
    float a_max;
} trajectry_limits_t;

typedef struct {
    float x_i;      // start position
    float x_f;      // end position
    float a;      // signed acceleration
    float v_peak;      // signed cruise or peak velocity
    float t_a;      // accel time
    float t_v;      // cruise time
    float t_f;      // total time
    float y_a;  // position at end of accel phase
} trajectory_plan_t;

//Plan a move for the join that starts/ends at rest.
//Return false (plan unchanged) -> if(limits are <=0 OR NaN OR infinite OR positions arent finite)
bool trajectory_plan(trajectory_plan_t *plan, float d_start, float d_end, const trajectry_limits_t *limit);
//instant at time t since the move started
//t <0 returns the start
//t >= duration returns the exact end with v = a = 0
//Callers compute t from an integer tick count: t = (float)n * dt
trajectory_instant_t trajectory_sample(const trajectory_plan_t *plan, float t);
//Total move time (s).
//Also let caller know the move is done
float trajectory_duration(const trajectory_plan_t *plan);

#endif // TRAJECTORY_H

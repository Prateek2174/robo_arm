// Trapezoidal motion profile. NO HAL CALLS.
// Module: #8   Milestone: M2
// TODO: implement (see robot-arm-todo.md for the 'done when' criteria).

#include "trajectory.h"
#include <math.h>

//Plan a move for the join that starts/ends at rest.
//Return false (plan unchanged) -> if(limits are <=0 OR NaN OR infinite OR positions arent finite)
//v_i = 0 always
bool trajectory_plan(trajectory_plan_t *plan, float d_start, float d_end, const trajectry_limits_t *limit){

    plan->x_f = d_end;
    plan->x_i = d_start;
    float dx = plan->x_f - plan->x_i; //change in d
    float s = copysignf(d_end, d_start);
    
    //calculate v_cruise/v_peak and a
    plan->v_peak = s * limit->v_max;
    plan->a = s * limit->a_max;

    //Time to accel/decel from v_cruise/v_peak
    plan->t_a = plan->v_peak / plan->a;

    //Decide between trapezoidal or triangle movement -> calc the required phase times

    //min displacement required to reach cruising speed which is integral of the velocity slopes:
    float dx_min = 1.0f * plan->t_a * plan->v_peak;

    //check to ensure enough displacement to reach cruising speed:
    if (s*dx < s*dx_min) //Use triangle profile
    {
        plan->v_peak = sqrt(plan->a * abs(dx));
        plan->t_a = max(0.0f, plan->v_peak / plan->a);
        plan->t_v = 0.0f;

    }else
    {              //Use trapezoidal profile
        plan->t_v = (dx - dx_min) / plan->v_peak;

    }
    
    plan->t_f = plan->t_a + plan->t_v;
    plan->y_a = plan->x_i + 0.5f*plan->a*plan->t_a*plan->t_a;

    return true;
}

//instant at time t since the move started
//t <0 returns the start
//t >= duration returns the exact end with v = a = 0
//Callers compute t from an integer tick count: t = (float)n * dt
trajectory_instant_t trajectory_sample(const trajectory_plan_t *plan, float t){

    trajectory_instant_t instant; 

    if (t <= 0) //initial or before start
    { 
        instant.a = plan->a;
        instant.d = 0;
        instant.v = 0;

    }else if (t >= plan->t_f) //done
    { 
        instant.a = 0;
        instant.v = 0;
        instant.d = plan->x_f;

    }else if (t < plan->t_a) //positive accel before cruise
    { 
        instant.d = plan->x_i + 0.5*plan->a*t*t;
        instant.a = plan->a;
        instant.v = plan->a * t;

    }else if (t < plan->t_a + plan->t_v) //cruise time before decel
    {
        instant.d = plan->y_a + plan->v_peak*(t - plan->t_a);
        instant.a = 0;
        instant.v = plan->v_peak;

    }else if (t < plan->t_f) //decel
    {
        instant.d = plan->x_f - 0.5*plan->a*(plan->t_f - t)*(plan->t_f - t);
        instant.a = -1*plan->a;
        instant.v = plan->a * (plan->t_f - t);
    }

    return instant;
}

//Total move time (s).
//Also let caller know the move is done
float trajectory_duration(const trajectory_plan_t *plan){


}
#ifndef FC_PID_H
#define FC_PID_H

/*
 * PID controller.
 */

#include <stdbool.h>
#include <stdint.h>

/* State of a PID controller. */
struct pid {
    uint32_t prev_time_us;
    float integral_err;
    float prev_actual;
    float Kp;
    float Ki;
    float Kd;
    /* Following fields are for debugging: */
    float pterm;
    float iterm;
    float dterm;
    float desired;
    float actual;
};

void pid_setup(struct pid *pid, float kp, float ki, float kd);

/* Update PID controller state and compute control output.
 *  `desired` - setpoint
 *  `actual` - current measurement
 *  `pid` - PID state
 *  `now_us` - current microsecond timestamp
 *  `throttle` - current throttle
 *  `integrate` - whether to update `pid->integral_err`
 *  `iterm_limit` - clamp `pid->iterm` to `-iterm_limit..iterm_limit` */
float pid_update(float desired, float actual, struct pid *pid, uint32_t now_us,
                 bool integrate, float iterm_limit);

#endif /* FC_PID_H */

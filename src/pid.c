#include "pid.h"
#include "util.h"
#include <stdint.h>
#include <string.h>

float pid_update(float desired, float actual, struct pid *pid, uint32_t now_us,
                 bool integrate, float iterm_limit)
{
    if (pid->prev_time_us == 0) {
        /* First iteration -> no PID correction yet. */
        pid->prev_time_us = now_us;
        pid->prev_actual = actual;
        pid->integral_err = 0.0f;
        return 0.0f;
    }

    uint32_t elapsed = now_us - pid->prev_time_us;
    if (elapsed == 0) {
        return 0.0f;
    }
    float dt = elapsed * 1e-6f; /* us -> s */

    float err = desired - actual;
    pid->pterm = pid->Kp * err;

    if (integrate) {
        pid->integral_err = pid->integral_err + err * dt;
    }
    pid->iterm = clamp(pid->Ki * pid->integral_err, -iterm_limit, iterm_limit);

    pid->dterm = pid->Kd * ((pid->prev_actual - actual) / dt);

    pid->prev_time_us = now_us;
    pid->prev_actual = actual;

    pid->desired = desired;
    pid->actual = actual;

    return pid->pterm + pid->iterm + pid->dterm;
}

void pid_setup(struct pid *pid, float kp, float ki, float kd)
{
    memset(pid, 0, sizeof(struct pid));
    pid->Kp = kp;
    pid->Ki = ki;
    pid->Kd = kd;
}

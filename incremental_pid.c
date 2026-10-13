#include "incremental_pid.h"

#include <math.h>
#include <stddef.h>

bool incremental_pid_init(IncrementalPID *pid,
                          double kp,
                          double ki,
                          double kd,
                          double sample_time,
                          double integral_min,
                          double integral_max,
                          double deadband,
                          double initial_output)
{
    if (pid == NULL || !isfinite(kp) || !isfinite(ki) || !isfinite(kd) ||
        !isfinite(sample_time) || sample_time <= 0.0 ||
        !isfinite(integral_min) || !isfinite(integral_max) ||
        integral_min > integral_max || !isfinite(deadband) || deadband < 0.0 ||
        !isfinite(initial_output)) {
        return false;
    }

    pid->kp = kp;
    pid->ki = ki;
    pid->kd = kd;
    pid->sample_time = sample_time;
    pid->integral_min = integral_min;
    pid->integral_max = integral_max;
    pid->deadband = deadband;
    pid->integral = 0.0;
    pid->error_1 = 0.0;
    pid->error_2 = 0.0;
    pid->output = initial_output;
    return true;
}

double incremental_pid_update(IncrementalPID *pid, double setpoint, double measurement)
{
    if (pid == NULL || !isfinite(setpoint) || !isfinite(measurement)) {
        return pid != NULL ? pid->output : 0.0;
    }

    double error = setpoint - measurement;
    if (fabs(error) <= pid->deadband) {
        error = 0.0;
    }

    double previous_integral = pid->integral;
    pid->integral += pid->ki * pid->sample_time * error;
    if (pid->integral > pid->integral_max) {
        pid->integral = pid->integral_max;
    } else if (pid->integral < pid->integral_min) {
        pid->integral = pid->integral_min;
    }

    double delta_output = pid->kp * (error - pid->error_1)
                        + (pid->integral - previous_integral)
                        + (pid->kd / pid->sample_time)
                          * (error - 2.0 * pid->error_1 + pid->error_2);

    pid->output += delta_output;
    pid->error_2 = pid->error_1;
    pid->error_1 = error;
    return pid->output;
}
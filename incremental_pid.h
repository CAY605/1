#ifndef INCREMENTAL_PID_H
#define INCREMENTAL_PID_H

#include <stdbool.h>

typedef struct {
    double kp;
    double ki;
    double kd;
    double sample_time;
    double integral_min;
    double integral_max;
    double deadband;

    double integral;
    double error_1;
    double error_2;
    double output;
} IncrementalPID;

/* Integral limits apply to the integral contribution in output units. */
bool incremental_pid_init(IncrementalPID *pid,
                          double kp,
                          double ki,
                          double kd,
                          double sample_time,
                          double integral_min,
                          double integral_max,
                          double deadband,
                          double initial_output);

double incremental_pid_update(IncrementalPID *pid, double setpoint, double measurement);

#endif
#include <stdio.h>

typedef struct {
    double kp;
    double ki;
    double kd;
    double setpoint;
    double last_error;
    double prev_error;
    double output;
} IncrementalPID;

static void pid_init(IncrementalPID *pid, double kp, double ki, double kd) {
    pid->kp = kp;
    pid->ki = ki;
    pid->kd = kd;
    pid->setpoint = 0.0;
    pid->last_error = 0.0;
    pid->prev_error = 0.0;
    pid->output = 0.0;
}

static double clamp_double(double value, double min, double max) {
    if (value < min) return min;
    if (value > max) return max;
    return value;
}

/* 增量式 PID：
   u(k) = u(k-1) + Kp*(e(k)-e(k-1)) + Ki*e(k) + Kd*(e(k)-2e(k-1)+e(k-2))
*/
static double incremental_pid_step(IncrementalPID *pid, double measurement) {
    double error = pid->setpoint - measurement;

    double delta_u =
        pid->kp * (error - pid->last_error) +
        pid->ki * error +
        pid->kd * (error - 2.0 * pid->last_error + pid->prev_error);

    pid->output += delta_u;
    pid->output = clamp_double(pid->output, 0.0, 100.0);

    pid->prev_error = pid->last_error;
    pid->last_error = error;

    return pid->output;
}

/* 简化的电机动态模型：速度不能瞬间达到目标值，
   需要受惯性、摩擦和PWM影响 */
static double motor_model(double current_rpm, double pwm_percent) {
    const double acceleration_gain = 0.35;
    const double friction_factor = 0.08;

    double desired_speed = pwm_percent * 10.0;  // 0~100 -> 0~1000 RPM 近似量纲
    double next_rpm = current_rpm + (desired_speed - current_rpm * (1.0 + friction_factor)) * acceleration_gain;

    if (next_rpm < 0.0) next_rpm = 0.0;
    return next_rpm;
}

int main(void) {
    const int steps = 30;
    const double final_target_rpm = 1000.0;

    IncrementalPID pid;
    double actual_rpm = 0.0;

    pid_init(&pid, 0.45, 0.08, 0.02);

    printf("Iter  Setpoint  ActualRPM   Error   PWM%%   Output\n");
    printf("----  --------  ---------  ------  -----  ------\n");

    for (int i = 0; i < steps; ++i) {
        /* 让目标转速从 0 平滑升到 1000 RPM */
        double setpoint = (final_target_rpm / (steps - 1)) * i;

        pid.setpoint = setpoint;
        double error = setpoint - actual_rpm;

        /* 控制器输出 PWM 占空比 */
        double pwm_percent = incremental_pid_step(&pid, actual_rpm);

        /* 电机实际转速更新 */
        actual_rpm = motor_model(actual_rpm, pwm_percent);

        printf("%4d %9.1f %10.1f %7.1f %6.2f %7.2f\n",
               i,
               setpoint,
               actual_rpm,
               error,
               pwm_percent,
               pid.output);
    }

    return 0;
}

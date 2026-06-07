#include "pid.h"
#include <math.h>

void pid_init(pid_t *pid, float kp, float ki, float kd,
              float out_min, float out_max, float dt)
{
    pid->kp = kp;
    pid->ki = ki;
    pid->kd = kd;
    pid->setpoint    = 0.0f;
    pid->integral    = 0.0f;
    pid->prev_error  = 0.0f;
    pid->output_min  = out_min;
    pid->output_max  = out_max;
    pid->integral_max = (out_max - out_min) / 2.0f;  // Anti-windup
    pid->dt = dt;
}

float pid_compute(pid_t *pid, float measured)
{
    float error = pid->setpoint - measured;

    // Término proporcional
    float p = pid->kp * error;

    // Término integral con anti-windup (clamping)
    pid->integral += error * pid->dt;
    if (pid->integral >  pid->integral_max) pid->integral =  pid->integral_max;
    if (pid->integral < -pid->integral_max) pid->integral = -pid->integral_max;
    float i = pid->ki * pid->integral;

    // Término derivativo (sobre la medición, no el error → evita derivative kick)
    float d = pid->kd * (measured - pid->prev_error) / pid->dt;
    pid->prev_error = measured;

    float output = p + i - d;

    // Saturación de salida
    if (output > pid->output_max) output = pid->output_max;
    if (output < pid->output_min) output = pid->output_min;

    return output;
}

void pid_reset(pid_t *pid)
{
    pid->integral   = 0.0f;
    pid->prev_error = 0.0f;
}

void pid_set_gains(pid_t *pid, float kp, float ki, float kd)
{
    pid->kp = kp;
    pid->ki = ki;
    pid->kd = kd;
    pid_reset(pid);
}
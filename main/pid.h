#pragma once

typedef struct {
    float kp, ki, kd;
    float setpoint;
    float integral;
    float prev_error;
    float output_min, output_max;
    float integral_max;       // Anti-windup
    float dt;                 // Periodo en segundos
} pid_t;

void pid_init(pid_t *pid, float kp, float ki, float kd,
              float out_min, float out_max, float dt);
float pid_compute(pid_t *pid, float measured);
void pid_reset(pid_t *pid);
void pid_set_gains(pid_t *pid, float kp, float ki, float kd);
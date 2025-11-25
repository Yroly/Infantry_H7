#include "pid.h"

PID_Ctrl PID;

#define LimitMax(input, max)   \
    {                          \
        if (input > max)       \
        {                      \
            input = max;       \
        }                      \
        else if (input < -max) \
        {                      \
            input = -max;      \
        }                      \
    }

void PID_Ctrl::Init(PidTypeDef *pid,PidMode mode,const fp32 PID[3],fp32 max_out,fp32 max_Iout,fp32 I_band)
{
    if (pid == NULL || PID == NULL)
    {
        return;
    }
    pid->mode = mode;
    pid->Kp = PID[0];
    pid->Ki = PID[1];
    pid->Kd = PID[2];
    pid->max_out = max_out;
    pid->max_Iout = max_Iout;
		pid->I_band = I_band;
    pid->Dbuf[0] = pid->Dbuf[1] = pid->Dbuf[2] = 0.0f;
    pid->error[0] = pid->error[1] = pid->error[2] = pid->P_out = pid->I_out = pid->D_out = pid->out = 0.0f;
}

fp32 PID_Ctrl::Calc(PidTypeDef *pid, fp32 measure,fp32 target)
{
    if (pid == NULL)
    {
        return 0.0f;
    }

    pid->error[2] = pid->error[1];
    pid->error[1] = pid->error[0];
    pid->measure = measure ;
    pid->target	 = target;
    pid->error[0] = target - measure;
    pid->P_out = pid->Kp * pid->error[0];
    if (fabs(pid->error[0]) < pid->I_band)
    {
        pid->I_out += pid->Ki * pid->error[0];
    }
    pid->Dbuf[2] = pid->Dbuf[1];
    pid->Dbuf[1] = pid->Dbuf[0];
    pid->Dbuf[0] = (pid->error[0] - pid->error[1]);
    pid->D_out = pid->Kd * pid->Dbuf[0];

    LimitMax(pid->I_out, pid->max_Iout);
    pid->out = pid->P_out + pid->I_out + pid->D_out;
    LimitMax(pid->out, pid->max_out);

    return pid->out;
}

void PID_Ctrl::Clear(PidTypeDef *pid)
{
    if (pid == NULL)
    {
        return;
    }

    pid->error[0] = pid->error[1] = pid->error[2] = 0.0f;
    pid->Dbuf[0] = pid->Dbuf[1] = pid->Dbuf[2] = 0.0f;
    pid->out = pid->P_out = pid->I_out = pid->D_out = 0.0f;
    pid->measure = pid->target = 0.0f;
}

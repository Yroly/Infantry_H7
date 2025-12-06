#ifndef PID_H
#define PID_H
#include "main.h"

#ifdef __cplusplus
extern "C" {
#endif

#ifdef __cplusplus
}
#endif

typedef enum
{
		INIT = 0x00,
    POSITION,
    DELTA
}PidMode;

typedef struct
{
  PidMode mode;
  fp32 Kp;
  fp32 Ki;
  fp32 Kd;

  fp32 max_out;
  fp32 max_Iout;
	fp32 I_band;	
  fp32 measure;
  fp32 target;

  fp32 out;  
	fp32 P_out;
  fp32 I_out;
  fp32 D_out;
  fp32 Dbuf[3];
  fp32 error[3];
} PidTypeDef;

class PID_Ctrl
{
public:
    void Init(PidTypeDef *pid, PidMode mode,fp32 Kp, fp32 Ki, fp32 Kd,fp32 max_out,fp32 max_Iout,fp32 I_band);
    fp32 Calc(PidTypeDef *pid, fp32 measure, fp32 target);
    void Clear(PidTypeDef *pid);
};

extern PID_Ctrl PID;

extern void PID_Init(PidTypeDef *pid, uint8_t mode,const fp32 PID[3],fp32 max_out,fp32 max_Iout);
extern fp32 PID_Calc(PidTypeDef *pid, fp32 measure, fp32 target);
extern void PID_clear(PidTypeDef *pid);
#endif

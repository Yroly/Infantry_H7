#ifndef _KALMAN_FILTER_H
#define _KALMAN_FILTER_H

#include <stdio.h>
#include "stdbool.h"
#include "string.h"
#include "stdint.h"
#include "arm_math.h"

/*************Ò»½×¿¨¶ûÂü**************/
typedef struct
{
	float X_last; //ÉÏÒ»Ê±¿ÌµÄ×îÓÅ½á¹û  X(k-|k-1)
	float X_mid;  //µ±Ç°Ê±¿ÌµÄÔ¤²â½á¹û  X(k|k-1)
	float X_now;  //µ±Ç°Ê±¿ÌµÄ×îÓÅ½á¹û  X(k|k)
	float P_mid;  //µ±Ç°Ê±¿ÌÔ¤²â½á¹ûµÄÐ­·½²î  P(k|k-1)
	float P_now;  //µ±Ç°Ê±¿Ì×îÓÅ½á¹ûµÄÐ­·½²î  P(k|k)
	float P_last; //ÉÏÒ»Ê±¿Ì×îÓÅ½á¹ûµÄÐ­·½²î  P(k-1|k-1)
	float kg;     //kalmanÔöÒæ
	float A;      //ÏµÍ³²ÎÊý
	float B;
	float Q;
	float R;
	float H;
} extKalman_t;

void KalmanCreate(extKalman_t *p, float T_Q, float T_R);
float KalmanFilter(extKalman_t *p, float dat);
/*************Ò»½×¿¨¶ûÂü END**************/

/*************¶þ½×¿¨¶ûÂü**************/
#define mat          arm_matrix_instance_f32 //float
#define mat_64       arm_matrix_instance_f64 //double
#define mat_init     arm_mat_init_f32
#define mat_add      arm_mat_add_f32
#define mat_sub      arm_mat_sub_f32
#define mat_mult     arm_mat_mult_f32
#define mat_trans    arm_mat_trans_f32 //¸¡µã¾ØÕó×ªÖÃ
#define mat_inv      arm_mat_inverse_f32
#define mat_inv_f64  arm_mat_inverse_f64


#define Angle_limit 200         //½Ç¶ÈÐ¡ÓÚ50¿ªÆôÔ¤²â
#define PredictAngle_limit 250 //Ô¤²âÖµÏÞ·ù

#define Kf_Angle 0
#define Kf_Speed 1

typedef struct
{
	float raw_value;
	float filtered_value[2];
	mat xhat, xhatminus, z, A, H, AT, HT, Q, R, P, Pminus, K;
} kalman_filter_t;


class Matrix_kalman
{
	public:
	Matrix_kalman()
	{
		data = new float[2];
	}
	Matrix_kalman(float x11, float x12)
	{
		data = new float[2];
		data[0] = x11;
		data[1] = x12;
	}
	Matrix_kalman(float x11, float x12, float x21, float x22)
	{
		data = new float[4];
		data[0] = x11;
		data[1] = x12;
		data[2] = x21;
		data[3] = x22;
	}
	Matrix_kalman(Matrix_kalman &o)
	{
		data = o.data;
	}

	float *data;

};
struct kalman_filter_init_t_matrix
{
	float raw_value;
	Matrix_kalman filtered_value;
	Matrix_kalman xhat_data, xhatminus_data, z_data, Pminus_data, K_data;
	Matrix_kalman AT_data, HT_data;

	Matrix_kalman P_data;
	Matrix_kalman A_data;
	Matrix_kalman H_data;
	Matrix_kalman Q_data;
	Matrix_kalman R_data;

	kalman_filter_init_t_matrix(Matrix_kalman &P_data,
		Matrix_kalman &A_data,
		Matrix_kalman &H_data,
		Matrix_kalman &Q_data,
		Matrix_kalman &R_data)
		:P_data(P_data), H_data(H_data), Q_data(Q_data), R_data(R_data)
	{

	}
};

typedef struct
{
	float raw_value;
	float filtered_value[2];
	float xhat_data[2], xhatminus_data[2], z_data[2], Pminus_data[4], K_data[4];
	float P_data[4];
	float AT_data[4], HT_data[4];
	float A_data[4];
	float H_data[4];
	float Q_data[4];
	float R_data[4];
} kalman_filter_init_t;

typedef struct
{
	float Vision_Angle; //ÊÓ¾õ--½Ç¶È
	float Vision_Speed; //ÊÓ¾õ--ËÙ¶È
	float *Kf_result;   //¿¨¶ûÂüÊä³öÖµ
	uint16_t Kf_Delay;  //¿¨¶ûÂüÑÓÊ±¼ÆÊ±

	struct
	{
		float Predicted_Factor;   //Ô¤²â±ÈÀýÒò×Ó
		float Predicted_SpeedMin; //Ô¤²âÖµ×îÐ¡ËÙ¶È
		float Predicted_SpeedMax; //Ô¤²âÖµ×î´óËÙ¶È
		float kf_delay_open;      //¿¨¶ûÂüÑÓÊ±¿ªÆôÊ±¼ä
	} Parameter;
} Kalman_Data_t;

void kalman_filter_init(kalman_filter_t *F, kalman_filter_init_t *I);
float *kalman_filter_calc(kalman_filter_t *F, float signal1, float signal2);
void kalman_filter_init_matrix(kalman_filter_t *F, kalman_filter_init_t_matrix *I);
/*************¶þ½×¿¨¶ûÂü END**************/

#endif

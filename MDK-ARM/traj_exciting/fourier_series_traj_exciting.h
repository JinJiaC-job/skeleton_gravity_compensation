#ifndef FOURIER_TRAJ_H
#define FOURIER_TRAJ_H

#include "main.h"

// PARAMETERS
#define pi 3.1415
//�������ʱ���
extern float control_interval_time;
// sampling period
extern float traj_Ts;
// trajectory fundamental frequency = 1/T; T = run time of skeleton = 20s.
extern float traj_f;
// trajectory fundamental frequency in radian
extern float traj_wf;
// number of sampling points
extern uint8_t traj_n;
// order of trajectory generation 
extern uint8_t traj_orde;
// number of revolute joints
extern uint8_t dof;
// 6���ؽڽǶ���Ϣ
extern float q[7];
extern float q_last[7];

void fourier_series_traj(uint8_t time);
void traj_exciting_init(void);
void run_fourier_series_traj(void);


#endif


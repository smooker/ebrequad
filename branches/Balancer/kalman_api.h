
#ifndef _KALDATA_H_
#define _KALDATA_H_


typedef struct _KALDATA {
  float dt;         //delta time for update in timer ticks
  float P[2][2];    //covariance matrix.
  float angle;
  float q_bias;    //gyro estimate
  float rate;
  float Pdot[4];
  float err;
}KALDATA;

void Kalman_InitState(KALDATA* p_kd,float dt);

void Kalman_StateUpdate(
    float       q_m, /* Pitch gyro measurement */
    KALDATA*    p_kd
);

void Kalman_Update(
    float       angle_m, /*measured angle*/
    KALDATA*    p_kd
);


#endif // #define _KALDATA_H_

#ifndef PID_API_H
    #define PID_API_H


/*******************************************************************************
**    Types                                                                   **
*******************************************************************************/

typedef enum
{
    PID_ROLL      =  0,
    PID_PITCH     =  1,
    PID_YAW       =  2,
    PID_CAM_ROLL  =  3,
    PID_CAM_PITCH =  4,
    PID_MAX_NUMER =  5
} PID_channels;

typedef struct
{
  float P, I, D;
  float lastPosition;
  float integratedError;
} PIDdata;


/*******************************************************************************
**    External data                                                           **
*******************************************************************************/

extern PIDdata PID[PID_MAX_NUMER];

/*******************************************************************************
**    Function prototypes                                                     **
*******************************************************************************/
float PID_Update(float targetPosition, float currentPosition, PIDdata *PIDparameters);
void  PID_SetWindupGuard(float value);
float PID_GetWindupGuard(void);
void  PID_ZeroIntegralError(void);

#endif

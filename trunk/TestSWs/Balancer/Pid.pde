/*******************************************************************************
**                                                                            **
**  MODULE    : Pid.c                                                         **
**                                                                            **
**  TARGET    : Arduino                                                       **
**                                                                            **
**  PROJECT   : EbreQuad                                                      **
**                                                                            **
**  AUTHOR    : http://www.arduino.cc/playground/Main/BarebonesPIDForEspresso **
**                                                                            **
**  PURPOSE   : PID controller                                                **
**                                                                            **
**  REMARKS   :                                                               **
**                                                                            **
*******************************************************************************/

/*******************************************************************************
**                      Include Section                                       **
*******************************************************************************/

#include "Pid_api.h"


/*******************************************************************************
**                      Local Data                                            **
*******************************************************************************/

float windupGuard;

PIDdata PID[PID_MAX_NUMER];


/*******************************************************************************
**                      Functions                                             **
*******************************************************************************/

/*******************************************************************************
**                                                                            **
** FUNC-NAME     : PID_Update                                                 **
**                                                                            **
** http://www.arduino.cc/playground/Main/BarebonesPIDForEspresso              **
*******************************************************************************/
float PID_Update(float targetPosition, float currentPosition, PIDdata *PIDparameters)
{
  float error;
  float dTerm;

  error = targetPosition - currentPosition;
  
  PIDparameters->integratedError += error;
  if (PIDparameters->integratedError < -windupGuard)
  {
      PIDparameters->integratedError = -windupGuard;
  }
  else if (PIDparameters->integratedError > windupGuard)
  {
      PIDparameters->integratedError = windupGuard;
  }
  
  dTerm = PIDparameters->D * (currentPosition - PIDparameters->lastPosition);
  
  PIDparameters->lastPosition = currentPosition;

  return (PIDparameters->P * error) + (PIDparameters->I * (PIDparameters->integratedError)) + dTerm;
}

/*******************************************************************************
**                                                                            **
** FUNC-NAME     : PID_SetWindupGuard(float value)                            **
**                                                                            **
*******************************************************************************/
void  PID_SetWindupGuard(float value)
{
    windupGuard = value;
}

/*******************************************************************************
**                                                                            **
** FUNC-NAME     : float PID_GetWindupGuard(void)                             **
**                                                                            **
*******************************************************************************/
float PID_GetWindupGuard(void)
{
    return(windupGuard);
}

/*******************************************************************************
**                                                                            **
** FUNC-NAME     : PID_ZeroIntegralError(void)                                **
**                                                                            **
*******************************************************************************/
void PID_ZeroIntegralError(void)
{
    unsigned char axis;  

    for (axis = PID_ROLL; axis < PID_MAX_NUMER; axis++)
    {
        PID[axis].integratedError = 0;
    }
}

/*******************************************************************************
**                                                                            **
**  MODULE    : Motors.c                                                      **
**                                                                            **
**  TARGET    : Arduino                                                       **
**                                                                            **
**  PROJECT   : EbreQuad                                                      **
**                                                                            **
**  AUTHOR    : Albert Sanz                                                   **
**                                                                            **
**  PURPOSE   :                                                               **
**                                                                            **
**  REMARKS   :                                                               **
**                                                                            **
*******************************************************************************/

/*******************************************************************************
**                      Include Section                                       **
*******************************************************************************/

#include "EbreQuad_cfg.h"
#include "Motors_api.h"
#include "TimerOne.h"
#include "TimerThree.h"
#include "TimerFive.h"

/*******************************************************************************
**                      Local Data                                            **
*******************************************************************************/


/*******************************************************************************
**                      Functions                                             **
*******************************************************************************/

unsigned int Motors_CheckCmd(unsigned int cmd);


/*******************************************************************************
**                                                                            **
** FUNC-NAME     : Motors_Init()                                              **
**                                                                            **
*******************************************************************************/
void Motors_Init(void)
{
    /* Front Motor Timer PWM configuration */
    Timer3.initialize(MOTOR_PWM_FREQ_250HZ);    /* initialize Timer1, and set a 4ms period, 250Hz */
    Timer3.pwm(FRONTMOTOR_PIN, MOTOR_MIN_PWM);  /* Setup pwm 25% duty cycle */

    /* Left/Rigth/Rear Motor Timer PWM configuration */
    Timer5.initialize(MOTOR_PWM_FREQ_250HZ);    /* initialize Timer5, and set a 4ms period, 250Hz */
    Timer5.pwm(RIGHTMOTOR_PIN, MOTOR_MIN_PWM);  /* Setup pwm 25% duty cycle */
    Timer5.pwm(REARMOTOR_PIN, MOTOR_MIN_PWM);   /* Setup pwm 25% duty cycle */
    Timer5.pwm(LEFTMOTOR_PIN, MOTOR_MIN_PWM);   /* Setup pwm 25% duty cycle */
}

/*******************************************************************************
**                                                                            **
** FUNC-NAME     : Motors_Deinit()                                            **
**                                                                            **
*******************************************************************************/
void Motors_Deinit(void)
{
   /* Stop PWM generation */
   Timer3.stop();
   Timer5.stop();
   Timer3.disablePwm(FRONTMOTOR_PIN);
   Timer5.disablePwm(LEFTMOTOR_PIN);
   Timer5.disablePwm(RIGHTMOTOR_PIN);
   Timer5.disablePwm(REARMOTOR_PIN);
}

/*******************************************************************************
**                                                                            **
** FUNC-NAME     : Motors_Init()                                              **
**                                                                            **
*******************************************************************************/
void Motors_Control(unsigned int *motorCommand)
{
    unsigned int tmpCmd;

    tmpCmd = Motors_CheckCmd(motorCommand[MOTOR_CMD_FRONT]);
    Timer3.setPwmDuty(FRONTMOTOR_PIN, (unsigned int)(tmpCmd + MOTOR_CMD_OFFSET));

    tmpCmd = Motors_CheckCmd(motorCommand[MOTOR_CMD_REAR]);
    Timer5.setPwmDuty(REARMOTOR_PIN,  (unsigned int)(tmpCmd + MOTOR_CMD_OFFSET));

    tmpCmd = Motors_CheckCmd(motorCommand[MOTOR_CMD_RIGTH]);
    Timer5.setPwmDuty(RIGHTMOTOR_PIN, (unsigned int)(tmpCmd + MOTOR_CMD_OFFSET));

    tmpCmd = Motors_CheckCmd(motorCommand[MOTOR_CMD_LEFT]);
    Timer5.setPwmDuty(LEFTMOTOR_PIN,  (unsigned int)(tmpCmd + MOTOR_CMD_OFFSET));
}

/*******************************************************************************
**                                                                            **
** FUNC-NAME     : Motors_ControlAll(int motorCommand)                        **
**                                                                            **
*******************************************************************************/
void Motors_ControlAll(unsigned int motorCommand)
{
    unsigned int tmpCmd;

    tmpCmd = Motors_CheckCmd(motorCommand);
    
    /* Write checked command to all motors */
    Timer3.setPwmDuty(FRONTMOTOR_PIN, (unsigned int)(tmpCmd + MOTOR_CMD_OFFSET));
    Timer5.setPwmDuty(REARMOTOR_PIN,  (unsigned int)(tmpCmd + MOTOR_CMD_OFFSET));      
    Timer5.setPwmDuty(RIGHTMOTOR_PIN, (unsigned int)(tmpCmd + MOTOR_CMD_OFFSET));      
    Timer5.setPwmDuty(LEFTMOTOR_PIN,  (unsigned int)(tmpCmd + MOTOR_CMD_OFFSET));
}


/* check if current command is in the valid range */
unsigned int Motors_CheckCmd(unsigned int cmd)
{
    unsigned int motorCmdFilt = MOTOR_MIN_COMMAND;

    if (cmd < MOTOR_MAX_COMMAND)  
    {
        if (cmd > MOTOR_MIN_COMMAND)  
        { 
            motorCmdFilt = cmd;
        }
        else
        {
            motorCmdFilt = MOTOR_MIN_COMMAND;
        }
    }
    else
    {
        motorCmdFilt = MOTOR_MAX_COMMAND;
    }
    return(motorCmdFilt);
}


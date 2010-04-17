#ifndef MOTORS_API_H
    #define MOTORS_API_H


/*******************************************************************************
**    Types                                                                   **
*******************************************************************************/

typedef enum
{
    MOTOR_CMD_FRONT = 0,
    MOTOR_CMD_REAR  = 1,
    MOTOR_CMD_RIGTH = 2,
    MOTOR_CMD_LEFT  = 3
} T_MotorCmd;

/*******************************************************************************
**    Defines                                                                 **
*******************************************************************************/
/* Scale Motor command:
   - INPUT: Motor commands from 0 to 1024 (0% Power to 100% Power)
   - OUTPUT: PWM signal from 1ms pulse (0%) to 2ms pulse (100%)

   TimerX Duty input: 0 (0% duty) 4096 (100% duty)
   Using a 250Hz PWM (T=4ms) ->
    - Duty input for TimerX have to be: 25% (1ms pulse) to 50% (2ms pulse)
   Then, input for TimerX have to be: 1024 (1ms (0% Power)) to 2048 (2ms (100% Power))
*/
#define MOTOR_PWM_FREQ_250HZ 4000  /* T=4000us -> 250Hz   */
#define MOTOR_CMD_OFFSET     1024

#define MOTOR_MIN_COMMAND       0  /*   0% Power to motor */
#define MOTOR_MAX_COMMAND    1024  /* 100% Power to motor */
#define MOTOR_MIN_PWM        1024  /* 25% Duty cycle PWM  */
#define MOTOR_MAX_PWM        2048  /* 50% Duty cycle PWM  */

//#if (((MOTOR_CMD_OFFSET+MOTOR_MAX_COMMAND)>MOTOR_MAX_PWM) || (MOTOR_PWM_FREQ != 4000))
//#error "Invalid PWM/OFFSET configuration, review it!!"
//#endif


/*******************************************************************************
**    External data                                                           **
*******************************************************************************/



/*******************************************************************************
**    Function prototypes                                                     **
*******************************************************************************/

    void Motors_Init(void);
    void Motors_Deinit(void);
    void Motors_Control(unsigned int *motorCommand);
    void Motors_ControlAll(unsigned int motorCommand);


#endif

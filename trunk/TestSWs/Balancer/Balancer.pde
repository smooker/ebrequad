/*******************************************************************************
**                                                                            **
**  MODULE    : EbreQuad.c                                                    **
**                                                                            **
**  TARGET    : Arduino                                                       **
**                                                                            **
**  PROJECT   : EbreQuad 1.0                                                  **
**                                                                            **
**  AUTHOR    : Albert Sanz                                                   **
**                                                                            **
**  PURPOSE   : Basic Scheduler for Task execution                            **
**                                                                            **
**  REMARKS   :                                                               **
**                                                                            **
*******************************************************************************/

/*******************************************************************************
**                      Include Section                                       **
*******************************************************************************/

#include "EbreQuad_cfg.h"
//#include "SerialCom_api.h"
#include "FlightAngle_api.h"
#include "Kalman_api.h"
#include "Pid_api.h"


//#include "Motors.h"
//Motors_PWM motors;

/*******************************************************************************
**                      Type Definitions                                      **
*******************************************************************************/

/* Period Flags */
typedef union{
    unsigned char    tick;
    struct {
        unsigned char  t4ms   :1;
        unsigned char  t8ms   :1;
        unsigned char  t16ms  :1;
        unsigned char  t32ms  :1;
        unsigned char  t64ms  :1;
    }b;
} T_dispatch;

/* CPU load information */
typedef enum
{
    #if defined(TASK_2AMS_ENABLED)
    CPU_LOAD_TASK_2A,
    #endif
    #if defined(TASK_4AMS_ENABLED)
    CPU_LOAD_TASK_4A,
    #endif
    #if defined(TASK_4BMS_ENABLED)
    CPU_LOAD_TASK_4B,
    #endif
    #if defined(TASK_8AMS_ENABLED)
    CPU_LOAD_TASK_8A,
    #endif
    #if defined(TASK_8BMS_ENABLED)
    CPU_LOAD_TASK_8B,
    #endif
    #if defined(TASK_16AMS_ENABLED)
    CPU_LOAD_TASK_16A,
    #endif
    #if defined(TASK_32AMS_ENABLED)
    CPU_LOAD_TASK_32A,
    #endif
    #if defined(TASK_64AMS_ENABLED)
    CPU_LOAD_TASK_64A,
    #endif
    CPU_LOAD_NUMBER_OF_TASK
} enumTaskIds;

typedef struct
{
    unsigned int task_time;
    unsigned int task_avrg;
    unsigned int task_max;
}TypeTaskData;

/*******************************************************************************
**                      Macros                                                **
*******************************************************************************/

#define MICROS_MAX_TIME          0xFFFFFFFF
#define TICK_20_MS                  20000UL                     /* time in us */
#define TICK_2_MS                    2000UL                     /* time in us */
#define MAX_ALLOWED_JITTER           1000UL                     /*      250us */

/*******************************************************************************
**                      Local Data                                            **
*******************************************************************************/

static unsigned long prev_time = 0;

static T_dispatch  dispatch = {0xff};

#if defined(CPU_LOAD_MONITORING)
static TypeTaskData CpuData[CPU_LOAD_NUMBER_OF_TASK];
static unsigned long cpu_slice_start_time = 0;
#endif


/*******************************************************************************
**                      Local Function Prototypes                             **
*******************************************************************************/
void Scheduler(void);

#if defined (CPU_LOAD_MONITORING)
    void Calc_Task_Load_Start(unsigned char);
    void Calc_Task_Load_Stop(unsigned char);
    void Task_Load_Print(void);
    #define Task_Load_Start(a)    Calc_Task_Load_Start(a)
    #define Task_Load_Stop(a)     Calc_Task_Load_Stop(a)
#else
    #define Task_Load_Start(a)    {;}
    #define Task_Load_Stop(a)     {;}
#endif


#if defined (TASK_2AMS_ENABLED)
void Task_20ms_A(void);
#define Task_2A()   {\
                      Task_Load_Start(CPU_LOAD_TASK_2A);\
                      Task_20ms_A();\
                      Task_Load_Stop (CPU_LOAD_TASK_2A);}
#else
#define Task_2A()  {/* Not used */}
#endif

#if defined(TASK_4AMS_ENABLED)
void Task_4ms_A(void);
#define Task_4A()   {\
                      Task_Load_Start(CPU_LOAD_TASK_4A);\
                      Task_40ms_A();\
                      Task_Load_Stop (CPU_LOAD_TASK_4A);}
#else
#define Task_4A()  {/* Not used */}
#endif

#if defined(TASK_4BMS_ENABLED)
void Task_4ms_B(void);
#define Task_4B()   {\
                      Task_Load_Start(CPU_LOAD_TASK_4B);\
                      Task_40ms_B();\
                      Task_Load_Stop (CPU_LOAD_TASK_4B);}
#else
#define Task_4B()  {/* Not used */}
#endif

#if defined(TASK_8AMS_ENABLED)
void Task_8ms_A(void);
#define Task_8A()   {\
                      Task_Load_Start(CPU_LOAD_TASK_8A);\
                      Task_8ms_A();\
                      Task_Load_Stop (CPU_LOAD_TASK_8A);}
#else
#define Task_8A()  {/* Not used */}
#endif

#if defined(TASK_8BMS_ENABLED)
void Task_8ms_B(void);
#define Task_8B()   {\
                      Task_Load_Start(CPU_LOAD_TASK_8B);\
                      Task_8ms_B();\
                      Task_Load_Stop (CPU_LOAD_TASK_8B);}
#else
#define Task_8B()  {/* Not used */}
#endif

#if defined(TASK_16AMS_ENABLED)
void Task_16ms_A(void);
#define Task_16A()   {\
                      Task_Load_Start(CPU_LOAD_TASK_16A);\
                      Task_16ms_A();\
                      Task_Load_Stop (CPU_LOAD_TASK_16A);}
#else
#define Task_16A()  {/* Not used */}
#endif

#if defined(TASK_32AMS_ENABLED)
void Task_32ms_A(void);
#define Task_32A()   {\
                      Task_Load_Start(CPU_LOAD_TASK_32A);\
                      Task_32ms_A();\
                      Task_Load_Stop (CPU_LOAD_TASK_32A);}
#else
#define Task_32A()  {/* Not used */}
#endif

#if defined(TASK_64AMS_ENABLED)
void Task_64ms_A(void);
#define Task_64A()   {\
                      Task_Load_Start(CPU_LOAD_TASK_64A);\
                      Task_64ms_A();\
                      Task_Load_Stop (CPU_LOAD_TASK_64A);}
#else
#define Task_64A()  {/* Not used */}
#endif

#if defined(TASK_JITTER_MONITORING)
void Task_Error_Hook(unsigned long overtime);
#endif

/*******************************************************************************
**                      Main Functions                                        **
*******************************************************************************/

void setup()
{

  /* Initialization */
  Serial.begin(BAUD_RATE);
  pinMode(DEBUG_PIN,OUTPUT);

  // Configure motors
  //motors.initialize();
  //motors.commandAllMotors(MINCOMMAND);

  /* TODO: to be moved to the FSM global state machine */
  FlightAngle_Init();
  FlightAngle_InitAcc();
  FlightAngle_InitGyro();
  
  
  prev_time = micros();
}

void loop()
{
    Scheduler();
}


/*******************************************************************************
**                      Local Functions                                       **
*******************************************************************************/

/*******************************************************************************
**                                                                            **
** FUNC-NAME     : Scheduler                                                  **
**                                                                            **
** DESCRIPTION   : Tasks sequence calling the dispatcher every 2ms:           **
**          Time(ms): Task launched                                           **
**            0  : Task_2A() Task_4A() Task_8A()                              **
**            2  : Task_2A() Task_4B() Task_8B()                              **
**            4  : Task_2A() Task_4A()          Task_16A()                    **
**            6  : Task_2A() Task_4B()                    Task_32A()          **
**            8  : Task_2A() Task_4A() Task_8A()                              **
**           10  : Task_2A() Task_4B() Task_8B()                              **
**           12  : Task_2A() Task_4A()                             Task_64A() **
**           14  : Task_2A() Task_4B()                                        **
**           16  : Task_2A() Task_4A() Task_8A()                              **
**           18  : Task_2A() Task_4B() Task_8B()                              **
**           20  : Task_2A() Task_4A()           Task_16A()                   **
**           22  : Task_2A() Task_4B()                                        **
**                                                                            **
*******************************************************************************/
void Scheduler(void)
{
    unsigned long current_time;
    unsigned long slice_time;
    #if defined(CPU_LOAD_MONITORING)
    unsigned long start_time;
    unsigned long end_time;
    #endif

    /*** Check the current time for having a new tick *************************/
    current_time = micros();
    if (current_time >= prev_time)
    {
        slice_time = current_time - prev_time;
    }
    else /* counter overflow detected (each 70 minutes) */
    {
        slice_time = current_time + (MICROS_MAX_TIME - prev_time);
    }

    /*** Execute the corresponding Tasks **************************************/
    if (slice_time >= TICK_2_MS)
    {   
        #if defined(TASK_JITTER_MONITORING)
        if ((slice_time - TICK_2_MS) > MAX_ALLOWED_JITTER)
        {
            Task_Error_Hook(slice_time - TICK_2_MS);
        }
        #endif
        prev_time = current_time - (slice_time - TICK_2_MS);      /* Capture the value for the next tick */

        Task_2A();
        if(dispatch.b.t4ms){
            Task_4A();
            if(dispatch.b.t8ms){
                Task_8A();
            }else{
                if(dispatch.b.t16ms){
                    Task_16A();
                }else{
                    if(dispatch.b.t32ms){
                        if(dispatch.b.t64ms){
                            Task_64A();
                        }
                    }
                }
            }
        }else{
            Task_4B();
            if(dispatch.b.t8ms){
                Task_8B();
            }else{
                if(dispatch.b.t16ms){
                    if(dispatch.b.t32ms){
                        Task_32A();
                    }
                }
            }
        }
        dispatch.tick--;
    }
}



/*******************************************************************************
**                                                                            **
** FUNC-NAME     : Task_2ms_A                                                 **
**                                                                            **
*******************************************************************************/
#if defined(TASK_2AMS_ENABLED)
void Task_2ms_A(void)
{

}
#endif


/*******************************************************************************
**                                                                            **
** FUNC-NAME     : Task_4ms_A                                                 **
**                                                                            **
*******************************************************************************/
#if defined(TASK_4AMS_ENABLED)
void Task_4ms_A(void)
{
	/* Get Gyro Data from sensor */
    FlightAngle_GetGyroData();
    /* Process data with Kalman filter */
    FlightAngle_Processing_Gyro();
}
#endif

/*******************************************************************************
**                                                                            **
** FUNC-NAME     : Task_4ms_B                                                **
**                                                                            **
*******************************************************************************/
#if defined(TASK_4BMS_ENABLED)
void Task_4ms_B(void)
{
	/* Refresh Motor command to the outputs */
    //motors.write(motorCommand); // Defined in Motors.h
}
#endif

/*******************************************************************************
**                                                                            **
** FUNC-NAME     : Task_8ms_A                                                **
**                                                                            **
*******************************************************************************/
#if defined(TASK_8AMS_ENABLED)
void Task_8ms_A(void)
{

}
#endif

/*******************************************************************************
**                                                                            **
** FUNC-NAME     : Task_8ms_B                                                **
**                                                                            **
*******************************************************************************/
#if defined(TASK_8BMS_ENABLED)
void Task_8ms_B(void)
{
	/* Main EbreQuad State Machine */
	//EbreQuad_Task();
}
#endif


/*******************************************************************************
**                                                                            **
** FUNC-NAME     : Task_16ms_A                                                **
**                                                                            **
*******************************************************************************/
#if defined(TASK_16ADMS_ENABLED)
void Task_16ms_A(void)
{
	/* Get Accel Data from sensor. It's not necessary to do it faster, as the
	   sensor has a 6Hz HW filter. */
    FlightAngle_GetAccData();

}
#endif


/*******************************************************************************
**                                                                            **
** FUNC-NAME     : Task_32ms_A                                                **
**                                                                            **
*******************************************************************************/
#if defined(TASK_32AMS_ENABLED)
void Task_32ms_A(void)
{
    /* Process data with Kalman filter */
    FlightAngle_Processing_Acc();
}
#endif


/*******************************************************************************
**                                                                            **
** FUNC-NAME     : Task_64ms_A                                                **
**                                                                            **
*******************************************************************************/
#if defined(TASK_64AMS_ENABLED)
void Task_64ms_A(void)
{
	/* Telemetry task with configurator */

    /*  current_cmd = COM_GetCommand();
    if (current_cmd != 0)
    {
        COM_ReadCommands(current_cmd);
        COM_SendTelemetry(current_cmd);
    }*/
}
#endif

/*******************************************************************************
**                                                                            **
** FUNC-NAME     : Task_Error_Hook                                            **
**                                                                            **
*******************************************************************************/
#if defined(TASK_JITTER_MONITORING)
void Task_Error_Hook(unsigned long overtime)
{
    Serial.print("Scheduler ERROR: Too much exectution time: ");
    Serial.println(overtime, DEC);
    //do{}
    //while(1);
}
#endif


#if defined(CPU_LOAD_MONITORING)
/*******************************************************************************
**                                                                            **
** FUNC-NAME     : Calc_Task_Load_Start                                       **
**                                                                            **
*******************************************************************************/
void Calc_Task_Load_Start(unsigned char taskID)
{
    cpu_slice_start_time = micros();
}

/*******************************************************************************
**                                                                            **
** FUNC-NAME     : Calc_Task_Load_Stop                                        **
**                                                                            **
*******************************************************************************/
void Calc_Task_Load_Stop(unsigned char taskID)
{
    unsigned long cpu_slice_calc;
    float calc_avrg;

    cpu_slice_calc = micros();
    /* Check counter overflow (each 70 minutes) */
    if (cpu_slice_calc >= cpu_slice_start_time)
    {
        cpu_slice_calc = cpu_slice_calc - cpu_slice_start_time;
    }
    else
    {
        cpu_slice_calc = cpu_slice_calc + (MICROS_MAX_TIME - cpu_slice_start_time);
    }

    CpuData[taskID].task_time = (unsigned int)cpu_slice_calc;
    calc_avrg = ((float)CpuData[taskID].task_avrg * 0.9) + ((float)cpu_slice_calc * 0.1);
    CpuData[taskID].task_avrg = (unsigned int)calc_avrg;
    if (CpuData[taskID].task_max < (unsigned int)cpu_slice_calc)
    {
        CpuData[taskID].task_max = (unsigned int)cpu_slice_calc;
    }
}

/*******************************************************************************
**                                                                            **
** FUNC-NAME     : Task_Load_Print                                            **
**                                                                            **
*******************************************************************************/
void Task_Load_Print(void)
{
    unsigned char loop = 0;

    Serial.println("CPU Load:");
    for(loop = 0 ; loop < CPU_LOAD_NUMBER_OF_TASK ; loop++)
    {
        Serial.print("Task "); Serial.print(loop, DEC);
        Serial.print("-Med: "); Serial.print(CpuData[loop].task_avrg, DEC);
        Serial.print("-Max: "); Serial.println(CpuData[loop].task_max, DEC);
    }
}

#endif

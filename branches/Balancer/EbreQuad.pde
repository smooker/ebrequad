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
#include "FlightCtr_api.h"
#include "SerialCom_api.h"

#include "Motors.h"
Motors_PWM motors;


/*******************************************************************************
**                      Type Definitions                                      **
*******************************************************************************/

/* Period Flags */
typedef union{
    unsigned char    tick;
    struct {
        unsigned char  t40ms   :1;
        unsigned char  t80ms   :1;
        unsigned char  t160ms  :1;
        unsigned char  t320ms  :1;
        unsigned char  t640ms  :1;
    }b;
} T_dispatch;

/* CPU load information */
typedef enum
{
    #if defined(TASK_20AMS_ENABLED)
    CPU_LOAD_TASK_20A,
    #endif
    #if defined(TASK_40AMS_ENABLED)
    CPU_LOAD_TASK_40A,
    #endif
    #if defined(TASK_40BMS_ENABLED)
    CPU_LOAD_TASK_40B,
    #endif
    #if defined(TASK_80AMS_ENABLED)
    CPU_LOAD_TASK_80A,
    #endif
    #if defined(TASK_80BMS_ENABLED)
    CPU_LOAD_TASK_80B,
    #endif
    #if defined(TASK_80CMS_ENABLED)
    CPU_LOAD_TASK_80C,
    #endif
    #if defined(TASK_80DMS_ENABLED)
    CPU_LOAD_TASK_80D,
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


#if defined (TASK_20AMS_ENABLED)
void Task_20ms_A(void);
#define Task_20A()   {\
                      Task_Load_Start(CPU_LOAD_TASK_20A);\
                      Task_20ms_A();\
                      Task_Load_Stop (CPU_LOAD_TASK_20A);}
#else
#define Task_20A()  {/* Not used */}
#endif

#if defined(TASK_40AMS_ENABLED)
void Task_40ms_A(void);
#define Task_40A()   {\
                      Task_Load_Start(CPU_LOAD_TASK_40A);\
                      Task_40ms_A();\
                      Task_Load_Stop (CPU_LOAD_TASK_40A);}
#else
#define Task_40A()  {/* Not used */}
#endif

#if defined(TASK_40BMS_ENABLED)
void Task_40ms_B(void);
#define Task_40B()   {\
                      Task_Load_Start(CPU_LOAD_TASK_40B);\
                      Task_40ms_B();\
                      Task_Load_Stop (CPU_LOAD_TASK_40B);}
#else
#define Task_40B()  {/* Not used */}
#endif

#if defined(TASK_80AMS_ENABLED)
void Task_80ms_A(void);
#define Task_80A()   {\
                      Task_Load_Start(CPU_LOAD_TASK_80A);\
                      Task_80ms_A();\
                      Task_Load_Stop (CPU_LOAD_TASK_80A);}
#else
#define Task_80A()  {/* Not used */}
#endif

#if defined(TASK_80BMS_ENABLED)
void Task_80ms_B(void);
#define Task_80B()   {\
                      Task_Load_Start(CPU_LOAD_TASK_80B);\
                      Task_80ms_B();\
                      Task_Load_Stop (CPU_LOAD_TASK_80B);}
#else
#define Task_80B()  {/* Not used */}
#endif

#if defined(TASK_80CMS_ENABLED)
void Task_80ms_C(void);
#define Task_80C()   {\
                      Task_Load_Start(CPU_LOAD_TASK_80C);\
                      Task_80ms_C();\
                      Task_Load_Stop (CPU_LOAD_TASK_80C);}
#else
#define Task_80C()  {/* Not used */}
#endif

#if defined(TASK_80DMS_ENABLED)
void Task_80ms_D(void);
#define Task_80D()   {\
                      Task_Load_Start(CPU_LOAD_TASK_80D);\
                      Task_80ms_D();\
                      Task_Load_Stop (CPU_LOAD_TASK_80D);}
#else
#define Task_80D()  {/* Not used */}
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
  motors.initialize();
  motors.commandAllMotors(MINCOMMAND);
  
  
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
** DESCRIPTION   : Tasks sequence calling the dispatcher every 10ms:          **
**          Time(ms): Task launched                                           **
**            0   : Task_20A() Task_40A() Task_80A()                          **
**            20  : Task_20A() Task_40B() Task_80B()                          **
**            40  : Task_20A() Task_40A() Task_80C()                          **
**            60  : Task_20A() Task_40B() Task_80D()                          **
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

        Task_20A();
        if(dispatch.b.t40ms){
            Task_40A();
            if(dispatch.b.t80ms){
                Task_80A();
            }else{
                Task_80C();
            }
        }else{
            Task_40B();
            if(dispatch.b.t80ms){
                Task_80B();
            }else{
                Task_80D();
                #if defined(CPU_LOAD_MONITORING)
                start_time = micros();
                if(dispatch.b.t160ms){
                }else{
                    if(dispatch.b.t320ms){
                    }else{
                        if(dispatch.b.t640ms){
                          Task_Load_Print();
                          end_time = micros();
                          prev_time = prev_time - (start_time - end_time);
                        }
                    }
                }
                #endif
            }
        }
        dispatch.tick--;
    }
}



/*******************************************************************************
**                                                                            **
** FUNC-NAME     : Task_20ms_A                                                **
**                                                                            **
*******************************************************************************/
#if defined(TASK_20AMS_ENABLED)
void Task_20ms_A(void)
{

}
#endif


/*******************************************************************************
**                                                                            **
** FUNC-NAME     : Task_40ms_A                                                **
**                                                                            **
*******************************************************************************/
#if defined(TASK_40AMS_ENABLED)
void Task_40ms_A(void)
{
    motors.write(motorCommand); // Defined in Motors.h
}
#endif

/*******************************************************************************
**                                                                            **
** FUNC-NAME     : Task_40ms_B                                                **
**                                                                            **
*******************************************************************************/
#if defined(TASK_40BMS_ENABLED)
void Task_40ms_B(void)
{
}
#endif

/*******************************************************************************
**                                                                            **
** FUNC-NAME     : Task_80ms_A                                                **
**                                                                            **
*******************************************************************************/
#if defined(TASK_80AMS_ENABLED)
void Task_80ms_A(void)
{
    current_cmd = COM_GetCommand();
    if (current_cmd != 0)
    {
        COM_ReadCommands(current_cmd);
        COM_SendTelemetry(current_cmd);
    }
}
#endif

/*******************************************************************************
**                                                                            **
** FUNC-NAME     : Task_80ms_B                                                **
**                                                                            **
*******************************************************************************/
#if defined(TASK_80BMS_ENABLED)
void Task_80ms_B(void)
{

}
#endif

/*******************************************************************************
**                                                                            **
** FUNC-NAME     : Task_80ms_C                                                **
**                                                                            **
*******************************************************************************/
#if defined(TASK_80CMS_ENABLED)
void Task_80ms_C(void)
{

}
#endif

/*******************************************************************************
**                                                                            **
** FUNC-NAME     : Task_80ms_D                                                **
**                                                                            **
*******************************************************************************/
#if defined(TASK_80DMS_ENABLED)
void Task_80ms_D(void)
{

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

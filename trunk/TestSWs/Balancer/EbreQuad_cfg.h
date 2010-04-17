#ifndef EBREQUAD_CFG_H
    #define EBREQUAD_CFG_H


/*******************************************************************************
**    Types                                                                   **
*******************************************************************************/


/*******************************************************************************
**    Configuration                                                           **
*******************************************************************************/    

/******** Scheduler configuration *********************************************/
//#define TASK_2AMS_ENABLED
#define TASK_4AMS_ENABLED
//#define TASK_4BMS_ENABLED
#define TASK_8AMS_ENABLED
#define TASK_8BMS_ENABLED
#define TASK_16AMS_ENABLED
#define TASK_64AMS_ENABLED
//#define TASK_JITTER_MONITORING
//#define CPU_LOAD_MONITORING

/******** General Configuration ***********************************************/
#define BAUD_RATE 115200

/******** Pins Configuration **************************************************/
#define DEBUG_PIN   13

/* Sensor pin assignments */
#define PITCHACCEL_PIN  0
#define ROLLACCEL_PIN   1
#define ZACCEL_PIN      2
#define PITCHRATE_PIN   3
#define ROLLRATE_PIN    4
#define YAWRATE_PIN     5
#define ROLLRATE45_PIN  4  /* TODO: To be wired in the shield! */
#define PITCHRATE45_PIN 5  /* TODO: To be wired in the shield! */
#define AZ_GYRO_PIN    12  /* Pin MUST have a Divider to generate 3.3v instead of 5V! */

/* Motors Pin assignment */


/******** Flight Configuration ************************************************/
//#define SENSOR_USE_45_GYRO



#endif /* EBREQUAD_CFG_H                                                      */

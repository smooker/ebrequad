#ifndef EBREQUAD_CFG_H
    #define EBREQUAD_CFG_H


/*******************************************************************************
**    Types                                                                   **
*******************************************************************************/


/*******************************************************************************
**    Configuration                                                           **
*******************************************************************************/    

/******** Scheduler configuration *********************************************/
#define TASK_20AMS_ENABLED

#define TASK_40AMS_ENABLED
//#define TASK_40BMS_ENABLED

#define TASK_80AMS_ENABLED
#define TASK_80BMS_ENABLED
#define TASK_80CMS_ENABLED
#define TASK_80DMS_ENABLED

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
#define AZ_GYRO_PIN    12  /* Pin MUST have a Divider to generate 3.3v instead of 5V! */

/* Motors Pin assignment */


/******** Flight Configuration ************************************************/
#define LASTAXIS 3



#endif /* EBREQUAD_CFG_H                                                      */

#ifndef FLIGHT_ANGLE_API_H
    #define FLIGHT_ANGLE_API_H


/*******************************************************************************
**    Types                                                                   **
*******************************************************************************/

typedef enum
{
    ANGLE_ROLL  = 0,
    ANGLE_PITCH = 1
} T_AngleChannel;

typedef enum
{
    SENSE_ROLL  = 0,
    SENSE_PITCH = 1,
    SENSE_YAW   = 2,
    SENSE_LAST  = 3
} T_SensorChannel;

/*******************************************************************************
**    Defines                                                                 **
*******************************************************************************/

#define FLIGTH_ANGLE_TASK_PERIOD 8

#define FIND_ZERO_BUFFER_SIZE   50  

#define FILTERING_OFF            0
#define FILTERING_DEFAULT        3
#define FILTERING_MAX            10

/* These A/D values depend on how well the sensors are mounted
   change these values to your configuration */
#define ACC_Z_VALUE_MIN 479
#define ACC_Z_VALUE_MAX 715


/*******************************************************************************
**    External data                                                           **
*******************************************************************************/
/* Gyro Data access for Telemetry */
extern unsigned int  gyroZero[3];       /* Initial Offset value in Raw ADC format   */
extern          int  gyroADC[3];        /* Raw ADC value from sensor without offset */
extern          int  gyroData[3];       /* Filtered ADC value from sensor without offset */
extern unsigned char gyroSmoothFactor;  /* Factor: 0 (no filtering) to 10 (max filtering) */         


/* ACC Data  access for Telemetry */
extern unsigned int  accelZero[3]};     /* Initial Offset value in Raw ADC format   */
extern          int  accelADC[3];       /* Raw ADC value from sensor without offset */
extern          int  accelData[3];      /* Filtered ADC value from sensor without offset */
extern unsigned char accelSmoothFactor; /* Factor: 0 (no filtering) to 10 (max filtering) */ 


/*******************************************************************************
**    Function prototypes                                                     **
*******************************************************************************/

    void FlightAngle_Init(void);
    void FlightAngle_InitAcc(void);
    void FlightAngle_InitGyro(void);
    void FlightAngle_GetAccData(void);
    void FlightAngle_GetGyroData(void);
    void FlightAngle_Processing_Gyro(void);
    void FlightAngle_Processing_Acc(void);
    int  FlightAngle_GetCurrent(T_AngleChannel channel); 

#endif

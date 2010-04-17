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
    SENSE_ROLL      = 0,
    SENSE_PITCH     = 1,
    SENSE_YAW       = 2,
   #ifdef SENSOR_USE_45_GYRO
    SENSE_ROLL_x45  = 3,
    SENSE_PITCH_x45 = 4,
    SENSE_LAST_x45  = 5
   #else
    SENSE_LAST      = 3
   #endif
} T_SensorChannel;

/*******************************************************************************
**    Defines                                                                 **
*******************************************************************************/

#define FLIGTH_ANGLE_TASK_PERIOD 8

#define FIND_ZERO_BUFFER_SIZE   50  

#define FILTERING_OFF            0
#define FILTERING_DEFAULT        2
#define FILTERING_MAX           10

/* These A/D values depend on how well the sensors are mounted
   change these values to your configuration */
#define ACC_Z_VALUE_MIN        479
#define ACC_Z_VALUE_MAX        715

#define GYRO45_CH_OFFSET         3

/* Gyro conversion to real units */
#define ADC_MAX_SCALE           ((int)1023)    /* 2^10 - 1  */
#define ADC_REF               ((float)3.0)     /* 3.000V zener reference */
#define GYRO_SENSITIVITY      ((float)0.0020)  /* V/deg/s */
#define GYRO_SENSITIVITY_x45  ((float)0.0091)  /* V/deg/s */

#define ADC_LSB_MV             (float)(ADC_REF/ADC_MAX_SCALE)
#define GYRO_RATE_CONV         (float)(ADC_LSB_MV/GYRO_SENSITIVITY)      /* deg/s/LSB */
#define GYRO_RATE_x45_CONV     (float)(ADC_LSB_MV/GYRO_SENSITIVITY_x45)  /* deg/s/LSB */

/* Precalculated values for ADC_REF = 3.0V to reduce CPU load */
#define GYRO_RATE_DEG_CONV_FIX     (float)(1.4662756598240469208211143695015)   /*deg/s/LSB */
#define GYRO_RATE_DEG_x45_CONV_FIX (float)(0.32225838677451580677387129000032)  /* deg/s/LSB */
#define GYRO_RATE_RAD_CONV_FIX     (float)(0.025591338005781958605919219479305) /* rad/s/LSB */
#define GYRO_RATE_RAD_x45_CONV_FIX (float)(0.0056244698913806502430591691163308)/* rad/s/LSB */


/* Precalculated values for ADC_REF = 3.3V to reduce CPU load */
//#define GYRO_RATE_CONV_FIX         (float)(1.6129032258064516129032258064516)   /* �/s/LSB */
//#define GYRO_RATE_x45_CONV_FIX     (float)(0.35448422545196738745125841900035)  /* �/s/LSB */
//#define GYRO_RATE_RAD_CONV_FIX     (float)(0.028150471806360154466511141427236) /* rad/s/LSB */
//#define GYRO_RATE_RAD_x45_CONV_FIX (float)(0.0061869168805187152673650860279639)/* rad/s/LSB */




/*******************************************************************************
**    External data                                                           **
*******************************************************************************/
/* Gyro Data access for Telemetry */
extern unsigned int  gyroZero[3];       /* Initial Offset value in Raw ADC format   */
extern          int  gyroADC[3];        /* Raw ADC value from sensor without offset */
extern          int  gyroData[3];       /* Filtered ADC value from sensor without offset */
extern unsigned char gyroSmoothFactor;  /* Factor: 0 (no filtering) to 10 (max filtering) */         


/* ACC Data  access for Telemetry */
extern unsigned int  accelZero[3];     /* Initial Offset value in Raw ADC format   */
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

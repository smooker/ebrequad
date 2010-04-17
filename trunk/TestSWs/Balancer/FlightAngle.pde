/*******************************************************************************
**                                                                            **
**  MODULE    : FligthAngle.c                                                 **
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
#include "FlightAngle_api.h"
#include "Kalman_api.h"
#include "wiring.h"
#include <math.h>

/*******************************************************************************
**                      Local Data                                            **
*******************************************************************************/

/* Gyro Data */
#ifndef SENSOR_USE_45_GYRO  
unsigned int  gyroZero[SENSE_LAST] = {0,0,0}; /* Initial Offset value in Raw ADC format   */
         int  gyroADC[SENSE_LAST]  = {0,0,0}; /* Raw ADC value from sensor without offset */
         int  gyroData[SENSE_LAST] = {0,0,0}; /* Filtered ADC value from sensor without offset */
unsigned char gyroSmoothFactor = FILTERING_DEFAULT;  /* Factor: 0 (no filtering) to 9 (max filtering) */    
static unsigned char gyroChannel[SENSE_LAST] = {ROLLRATE_PIN, PITCHRATE_PIN, YAWRATE_PIN};
#else
unsigned int  gyroZero[SENSE_LAST] = {0,0,0,0,0}; /* Initial Offset value in Raw ADC format   */
         int  gyroADC[SENSE_LAST]  = {0,0,0,0,0}; /* Raw ADC value from sensor without offset */
         int  gyroData[SENSE_LAST] = {0,0,0,0,0}; /* Filtered ADC value from sensor without offset */
unsigned char gyroSmoothFactor = FILTERING_DEFAULT;  /* Factor: 0 (no filtering) to 9 (max filtering) */    
static unsigned char gyroChannel[SENSE_LAST] = {ROLLRATE_PIN, PITCHRATE_PIN, YAWRATE_PIN, ROLLRATE45_PIN, PITCHRATE45_PIN};
#endif


/* ACC Data */
unsigned int  accelZero[3] = {0,0,0}; /* Initial Offset value in Raw ADC format   */
         int  accelADC[3]  = {0,0,0}; /* Raw ADC value from sensor without offset */
         int  accelData[3] = {0,0,0}; /* Filtered ADC value from sensor without offset */
unsigned char accelSmoothFactor = FILTERING_DEFAULT;  /* Factor: 0 (no filtering) to 9 (max filtering) */ 
static unsigned char accelChannel[3] = {ROLLACCEL_PIN, PITCHACCEL_PIN, ZACCEL_PIN};


/* Kalman data, local */
static KALDATA kalman_pitch;
static KALDATA kalman_roll;
static float kalman_dt = FLIGTH_ANGLE_TASK_PERIOD/1000.0;    /*time in seconds*/


/*******************************************************************************
**                      Functions                                             **
*******************************************************************************/

int   FlightAngle_FindMode(unsigned int *data, unsigned char arraySize);
void  FlightAngle_AutoZeroGyros(void);
int   FlightAngle_Filtering(int currentData, int previousData, unsigned char smoothFactor);
float Convert_Gyro2RadPerSec(T_SensorChannel axis);
float Convert_Gyro2DegPerSec(T_SensorChannel axis);
float Convert_Accel2Rad(T_SensorChannel axis);
float Convert_Accel2Deg(T_SensorChannel axis);
float arctan2(float y, float x);




/*******************************************************************************
**                                                                            **
** FUNC-NAME     : FlightAngle_Init()                                         **
**                                                                            **
*******************************************************************************/
void FlightAngle_Init(void)
{
    unsigned char axis;
    
    /* Init pin configuration */
    pinMode (AZ_GYRO_PIN, OUTPUT);
    digitalWrite(AZ_GYRO_PIN, LOW);

    /* Init local variables */
    for (axis = SENSE_ROLL; axis < SENSE_LAST; axis++) 
    {
        gyroZero[axis]  = 0;
        gyroADC[axis]   = 0;
        gyroData[axis]  = 0;
        accelZero[axis] = 0;
        accelADC[axis]  = 0;
        accelData[axis] = 0;
    }
    
    /* Init Kalman filters */
    Kalman_InitState(&kalman_pitch, kalman_dt);
    Kalman_InitState(&kalman_roll,  kalman_dt);

    delay(50);  /* Dalay necessary for Gyro stabilization before to start the AutoZero */
}

/*******************************************************************************
**                                                                            **
** FUNC-NAME     : FlightAngle_InitAcc(void)                                  **
**                                                                            **
*******************************************************************************/
void FlightAngle_InitAcc(void)
{
    unsigned int findZero[FIND_ZERO_BUFFER_SIZE];
    unsigned char axis, i;

    for (axis = SENSE_ROLL; axis < SENSE_YAW; axis++)
    {
        for (i=0; i<FIND_ZERO_BUFFER_SIZE; i++)
        {
            findZero[i] = analogRead(accelChannel[axis]);    /* read zero data */
        }
        accelZero[axis] = FlightAngle_FindMode(&findZero[0], FIND_ZERO_BUFFER_SIZE); /* look for medium */
    }

    /* Z accel (Yaw) can not be Initializated, fix value used */
    accelZero[SENSE_YAW] = ACC_Z_VALUE_MAX - ((ACC_Z_VALUE_MAX - ACC_Z_VALUE_MIN)/2);

    /* Store the zero value in EEPROM */
    /*writeFloat(accelZero[SENSE_ROLL], LEVELROLLCAL_ADR);
    writeFloat(accelZero[SENSE_PITCH], LEVELPITCHCAL_ADR);
    writeFloat(accelZero[SENSE_YAW],   LEVELZCAL_ADR);*/
}

/*******************************************************************************
**                                                                            **
** FUNC-NAME     : FlightAngle_InitGyro(void)                                 **
**                                                                            **
*******************************************************************************/
void FlightAngle_InitGyro(void)
{
    unsigned int findZero[FIND_ZERO_BUFFER_SIZE];
    unsigned char axis, i;

    /* autoInit Zero value of the IXZ500 */
    FlightAngle_AutoZeroGyros();

    for (axis = SENSE_ROLL; axis < SENSE_LAST; axis++) 
    {
        for (i=0; i<FIND_ZERO_BUFFER_SIZE; i++)
        {
            findZero[i] = analogRead(gyroChannel[axis]);    /* read zero data */
        }
        gyroZero[axis] = FlightAngle_FindMode(findZero, FIND_ZERO_BUFFER_SIZE); /* look for medium */
    }

    /* Store the zero value in EEPROM */
  /*writeFloat(gyroZero[SENSE_ROLL],  GYRO_ROLL_ZERO_ADR);
    writeFloat(gyroZero[SENSE_PITCH], GYRO_PITCH_ZERO_ADR);
    writeFloat(gyroZero[SENSE_YAW],   GYRO_YAW_ZERO_ADR);*/
    
}

/*******************************************************************************
**                                                                            **
** FUNC-NAME     : FlightAngle_GetAccData(void)                               **
**                                                                            **
*******************************************************************************/
void FlightAngle_GetAccData(void)
{
    unsigned char axis;
      
    /* Read Sensor data and substract Offset value */
    for (axis = SENSE_ROLL; axis < SENSE_LAST; axis++)
    {
        accelADC[axis] = analogRead(accelChannel[axis]) - accelZero[axis];
    }
    
    /* Low Pass Filtering */
    for (axis = SENSE_ROLL; axis < SENSE_LAST; axis++)
    {
        accelData[axis] = FlightAngle_Filtering(accelADC[axis], accelData[axis], accelSmoothFactor);
    }
}

/*******************************************************************************
**                                                                            **
** FUNC-NAME     : FlightAngle_GetGyroData(void)                              **
**                                                                            **
*******************************************************************************/
void FlightAngle_GetGyroData(void)
{
    unsigned char axis;
  
    /* Read Sensor data and substract Offset value */
    for (axis = SENSE_ROLL; axis < SENSE_LAST; axis++)
    {
        gyroADC[axis] = analogRead(gyroChannel[axis]) - gyroZero[axis];
        
        #ifdef SENSOR_USE_45_GYRO
        /* If value read from Gyro Output is less than 110deg/s, Gyro x4.5 Output 
           can be used to get more resolution. */
        if (gyroADC[axis] < GYRO_MAX_ADC_VALUE_FOR_45)
        {
            gyroADC[axis+GYRO45_CH_OFFSET] = analogRead(gyroChannel[axis+GYRO45_CH_OFFSET]) - gyroZero[axis+GYRO45_CH_OFFSET];
        }
        else /* If Gyro Output is greather than 110deg/s, measurement in x4.5 output
        {       will be out of range, set Gyro x4.5 data to invalid */
            gyroADC[axis+GYRO45_CH_OFFSET] = 0;
        }
        #endif
    }
    /* Adjust Yaw value */
    gyroADC[SENSE_YAW] = -gyroADC[SENSE_YAW];

    
    /* Low Pass Filtering Pitch and Roll normal channels */
    for (axis = SENSE_ROLL; axis < SENSE_LAST; axis++)
    {
        gyroData[axis] = FlightAngle_Filtering(gyroADC[axis], gyroData[axis], gyroSmoothFactor);
    }
    
    #ifdef SENSOR_USE_45_GYRO
    /* Low Pass Filtering Pitch and Roll x4.5 channels */
    for (axis = SENSE_ROLL_x45; axis < SENSE_LAST_x45; axis++)
    {
        /* If current Data is valid, perform filtering, if not, set invalid value */
        if (gyroADC[axis] != 0)
        {
            /* If previous Data was valid, perform filtering, if not, just copy the new valid value */
            if (gyroData[axis] != 0)
            {
                gyroData[axis] = FlightAngle_Filtering(gyroADC[axis], gyroData[axis], gyroSmoothFactor);
            }
            else
            {
                gyroData[axis] = gyroADC[axis];
            }   
        }
        else
        {
            gyroData[axis] = 0;
        }
    }
    #endif
}

/*******************************************************************************
**                                                                            **
** FUNC-NAME     : FlightAngle_Processing_Gyro(void)                          **
**                                                                            **
*******************************************************************************/
void FlightAngle_Processing_Gyro(void)
{
    float rate_roll;
    float rate_pitch;
    
    /* Scale Gyro data to real units [deg/s] */
   #ifdef SENSOR_USE_45_GYRO
    if (gyroData[SENSE_ROLL_x45] != 0)        /* Is there data x45 available? */
    {
        rate_roll  = Convert_Gyro2DegPerSec(SENSE_ROLL_x45);
    }
    else
    {
        rate_roll  = Convert_Gyro2DegPerSec(SENSE_ROLL);
    }
    if (gyroData[SENSE_PITCH_x45] != 0)       /* Is there data x45 available? */
    {
        rate_pitch  = Convert_Gyro2DegPerSec(SENSE_PITCH_x45);
    }
    else
    {
        rate_pitch  = Convert_Gyro2DegPerSec(SENSE_PITCH);
    }
   #else
    rate_roll  = Convert_Gyro2DegPerSec(SENSE_ROLL);
    rate_pitch = Convert_Gyro2DegPerSec(SENSE_PITCH);
   #endif

    /* Process state_update of the Kalman filter */
    Kalman_StateUpdate(rate_roll,  &kalman_roll);
    Kalman_StateUpdate(rate_pitch, &kalman_pitch);
}

/*******************************************************************************
**                                                                            **
** FUNC-NAME     : FlightAngle_Processing_Acc(void)                           **
**                                                                            **
*******************************************************************************/
void FlightAngle_Processing_Acc(void)
{
    float angle_roll;
    float angle_pitch;

    /* Calculate the estimated angle from the Acc data */
    angle_roll  = Convert_Accel2Deg(SENSE_ROLL);
    angle_pitch = Convert_Accel2Deg(SENSE_PITCH);

    /* Process state_update of the Kalman filter */
    Kalman_Update(angle_roll,  &kalman_roll);
    Kalman_Update(angle_pitch, &kalman_pitch);
}

/*******************************************************************************
**                                                                            **
** FUNC-NAME     : int  FlightAngle_GetCurrent(char channel)                  **
**                                                                            **
*******************************************************************************/
int  FlightAngle_GetCurrent(T_AngleChannel channel)
{
    int scaled_angle;
    
    /* Report current angle with +- and multiplied by 100, to work with integer 
       and not use float and to have resolution of 0,01� */
    if (channel == ANGLE_ROLL)
    {
        /* TODO: check if needed out of bounds checking */
        //if (kalman_roll.angle < 180)
        scaled_angle = (int)(100*kalman_roll.angle);
    }
    else /* channel = ANGLE_PITCH */
    {
        scaled_angle = (int)(100*kalman_pitch.angle);
    }
    return(scaled_angle);
}



/*******************************************************************************
**          Local  Functions                                                  **
*******************************************************************************/

int FlightAngle_FindMode(unsigned int *data, unsigned char arraySize)
{
  // The mode of a set of numbers is the value that occurs most frequently
  boolean done = 0;
  byte i;
  int temp, maxData, frequency, maxFrequency;
  
  // Sorts numbers from lowest to highest
  while (done != 1) {        
    done = 1;
    for (i=0; i<(arraySize-1); i++) {
      if (data[i] > data[i+1]) {     // numbers are out of order - swap
        temp = data[i+1];
        data[i+1] = data[i];
        data[i] = temp;
        done = 0;
      }
    }
  }
  
  temp = 0;
  frequency = 0;
  maxFrequency = 0;
  
  // Count number of times a value occurs in sorted array
  for (i=0; i<arraySize; i++) {
    if (data[i] > temp) {
      frequency = 0;
      temp = data[i];
      frequency++;
    } else if (data[i] == temp) frequency++;
    if (frequency > maxFrequency) {
      maxFrequency = frequency;
      maxData = data[i];
    }
  }
  return maxData;
}


/* Force the Gyro AutoZero calibration */
void FlightAngle_AutoZeroGyros(void)
{
  digitalWrite(AZ_GYRO_PIN, HIGH);
  delayMicroseconds(750);          /* IDG500 set time have to be 2us to 1500us */
  digitalWrite(AZ_GYRO_PIN, LOW);
  delay(10);                       /* IDG500 Auto Zero settling time is typically 7ms */
}

/* first order filter funtion (without float variable) */
int FlightAngle_Filtering(int currentData, int previousData, unsigned char smoothFactor)
{
  /* TODO: compare execution time and behavior with float smoothFactor */
  /* TODO: filter using base2, so divide performing a shifting and compare execution time */
  /* See Excel sheet Sensor_filtering.xls for to see filter behavior tests */
  if (smoothFactor < FILTERING_MAX)
  {
    return(((previousData * smoothFactor) + (currentData * (FILTERING_MAX - smoothFactor)))/FILTERING_MAX);
  }
  else /* Invalid smooth factor, disable filter and report current value */
  {
    return (currentData);
  }
}

/* Convert ADC measurement to real units, rad/sec */
float Convert_Gyro2RadPerSec(T_SensorChannel axis)
{
    #ifdef SENSOR_USE_45_GYRO
    if (axis < SENSE_LAST)
    {
    #endif
        return ((float)(gyroData[axis] * GYRO_RATE_RAD_CONV_FIX));
    #ifdef SENSOR_USE_45_GYRO
    }
    else
    {
        return ((float)(gyroData[axis] * GYRO_RATE_RAD_x45_CONV_FIX);
    }
    #endif
}

/* Convert ADC measurement to real units, �/sec */
float Convert_Gyro2DegPerSec(T_SensorChannel axis)
{
    #ifdef SENSOR_USE_45_GYRO
    if (axis < SENSE_LAST)
    {
    #endif
        return ((float)(gyroData[axis] * GYRO_RATE_DEG_CONV_FIX));
    #ifdef SENSOR_USE_45_GYRO
    }
    else
    {
        return ((float)(gyroData[axis] * GYRO_RATE_DEG_x45_CONV_FIX));
    }
    #endif
}

float Convert_Accel2Deg(T_SensorChannel axis)
{
    return(degrees(Convert_Accel2Rad(axis)));
}


float Convert_Accel2Rad(T_SensorChannel axis)
{
  /* Extract from FreescaleSemiconductors AN3461 */
  if (axis == SENSE_PITCH)
  {
      return(arctan2(accelADC[SENSE_PITCH], sqrt((accelADC[SENSE_ROLL] * accelADC[SENSE_ROLL]) + (accelADC[SENSE_YAW] * accelADC[SENSE_YAW]))));
  }
  else if (axis == SENSE_ROLL)
  {
      return(arctan2(accelADC[SENSE_ROLL], sqrt((accelADC[SENSE_PITCH] * accelADC[SENSE_PITCH]) + (accelADC[SENSE_YAW] * accelADC[SENSE_YAW]))));
  }
  else
  {
      return(0);
  }
}

float arctan2(float y, float x)
{
  // Taken from: http://www.dspguru.com/comp.dsp/tricks/alg/fxdatan2.htm
   float coeff_1 = PI/4;
   float coeff_2 = 3*coeff_1;
   float abs_y = abs(y)+1e-10;      // kludge to prevent 0/0 condition
   float r, angle;
   
   if (x >= 0) {
     r = (x - abs_y) / (x + abs_y);
     angle = coeff_1 - coeff_1 * r;
   }
   else {
     r = (x + abs_y) / (abs_y - x);
     angle = coeff_2 - coeff_1 * r;
   }
   if (y < 0)
     return(-angle);     // negate if in quad III or IV
   else
     return(angle);
}




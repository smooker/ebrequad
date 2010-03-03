/*******************************************************************************
**                                                                            **
**  MODULE    : FlightAngle.c                                                 **
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
#include "kalman_api.c"


/*******************************************************************************
**                      Local Data                                            **
*******************************************************************************/

/* Gyro Data */
unsigned int  gyroZero[3] = {0,0,0}; /* Initial Offset value in Raw ADC format   */
         int  gyroADC[3]  = {0,0,0}; /* Raw ADC value from sensor without offset */
         int  gyroData[3] = {0,0,0}; /* Filtered ADC value from sensor without offset */
unsigned char gyroSmoothFactor = FILTERING_DEFAULT;  /* Factor: 0 (no filtering) to 9 (max filtering) */         
static unsigned char gyroChannel[3] = {ROLLRATE_PIN, PITCHRATE_PIN, YAWRATE_PIN};


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

int FlightAngle_FindMode(int *data, int arraySize);
void FlightAngle_AutoZeroGyros(void);
int FlightAngle_Filtering(int currentData, int previousData, unsigned char smoothFactor);


/*******************************************************************************
**                                                                            **
** FUNC-NAME     : FlightAngle_Init()                                         **
**                                                                            **
*******************************************************************************/
void FlightAngle_Init(void)
{
    /* Init pin configuration */
    pinMode (AZ_GYRO_PIN, OUTPUT);
    digitalWrite(AZ_GYRO_PIN, LOW);

    /* Init local variables */
    
    /* Init Kalman filters */
    Kalman_InitState(&kalman_pitch, kalman_dt);
    Kalman_InitState(&kalman_roll, kalman_dt);

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


    for (axis = SENSE_ROLL; axis < SENSE_YAW; axis++)
    {
        for (int i=0; i<FIND_ZERO_BUFFER_SIZE; i++)
        {
            findZero[i] = analogRead(accelChannel[axis]);    /* read zero data */
        }
        accelZero[axis] = FlightAngle_FindMode(findZero, FIND_ZERO_BUFFER_SIZE); /* look for medium */
    }

    /* Z accel (Yaw) can not be Initializated, fix value used */
    accelZero[SENSE_YAW] = ACC_Z_VALUE_MAX - ((ACC_Z_VALUE_MAX - ACC_Z_VALUE_MIN)/2);

    /* Store the zero value in EEPROM */
    /*writeFloat(accelZero[ROLL], LEVELROLLCAL_ADR);
    writeFloat(accelZero[PITCH], LEVELPITCHCAL_ADR);
    writeFloat(accelZero[ZAXIS], LEVELZCAL_ADR);*/
}

/*******************************************************************************
**                                                                            **
** FUNC-NAME     : FlightAngle_InitGyro(void)                                 **
**                                                                            **
*******************************************************************************/
void FlightAngle_InitGyro(void)
{
    unsigned int findZero[FIND_ZERO_BUFFER_SIZE];


    /* autoInit Zero value of the IXZ500 */
    FlightAngle_AutoZeroGyros();

    for (axis = SENSE_ROLL; axis < SENSE_LAST; axis++) 
    {
        for (int i=0; i<FIND_ZERO_BUFFER_SIZE; i++)
        {
            findZero[i] = analogRead(gyroChannel[axis]);    /* read zero data */
        }
        gyroZero[axis] = FlightAngle_FindMode(findZero, FIND_ZERO_BUFFER_SIZE); /* look for medium */
    }

    /* Store the zero value in EEPROM */
  /*writeFloat(gyroZero[ROLL],  GYRO_ROLL_ZERO_ADR);
    writeFloat(gyroZero[PITCH], GYRO_PITCH_ZERO_ADR);
    writeFloat(gyroZero[YAW],   GYRO_YAW_ZERO_ADR);*/
    
}

/*******************************************************************************
**                                                                            **
** FUNC-NAME     : FlightAngle_GetAccData(void)                               **
**                                                                            **
*******************************************************************************/
void FlightAngle_GetAccData(void)
{
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
    /* Read Sensor data and substract Offset value */
    for (axis = SENSE_ROLL; axis < SENSE_LAST; axis++)
    {
        gyroADC[axis] = analogRead(gyroChannel[axis]) - gyroZero[axis];
    }
    gyroADC[SENSE_YAW] = -gyroADC[SENSE_YAW];
    
    /* Low Pass Filtering */
    for (axis = SENSE_ROLL; axis < SENSE_LAST; axis++)
    {
        gyroData[axis] = FlightAngle_Filtering(gyroADC[axis], gyroData[axis], gyroSmoothFactor);
    }
}

/*******************************************************************************
**                                                                            **
** FUNC-NAME     : FlightAngle_Processing_Gyro(void)                          **
**                                                                            **
*******************************************************************************/
void FlightAngle_Processing_Gyro(void)
{
    //Process state_update from kalman
}

/*******************************************************************************
**                                                                            **
** FUNC-NAME     : FlightAngle_Processing_Acc(void)                           **
**                                                                            **
*******************************************************************************/
void FlightAngle_Processing_Acc(void)
{
    //Process kalman_update from kalman
}

/*******************************************************************************
**                                                                            **
** FUNC-NAME     : int  FlightAngle_GetCurrent(char channel)                  **
**                                                                            **
*******************************************************************************/
int  FlightAngle_GetCurrent(char channel)
{
    //report current angle with +- and multiplied by 100, to work with integer and not use float and have resolution of 0,01º
}



/*******************************************************************************
**          Local  Functions                                                  **
*******************************************************************************/

int FlightAngle_FindMode(int *data, int arraySize)
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
  /* TODO: compare execution time with float smoothFactor */
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


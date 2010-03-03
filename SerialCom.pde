/*******************************************************************************
**                                                                            **
**  MODULE    : SerialCom.c                                                   **
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

#include "SerialCom_api.h"
#include "Pid_api.h"


/*******************************************************************************
**                      Local Data                                            **
*******************************************************************************/




/*******************************************************************************
**                      Functions                                             **
*******************************************************************************/

float Com_readFloatSerial(void);
void  Com_SendComma(void);

/*******************************************************************************
**                                                                            **
** FUNC-NAME     :  COM_ReadCommand(void)                                     **
**                                                                            **
*******************************************************************************/
char COM_GetCommand(void)
{
    if (Serial.available())
    {
        return(Serial.read());
    }
    else
    {
        return(0);
    }
}

/*******************************************************************************
**                                                                            **
** FUNC-NAME     :  COM_ReadCommand(void)                                     **
**                                                                            **
*******************************************************************************/
void COM_ReadCommands(char queryType)
{
    switch (queryType) 
    {
        case '0':
            motorCommand[FRONT] = 1000;
            motorCommand[REAR]  = 1000;
          break;
        case '1':
            motorCommand[FRONT] = 1100;
            motorCommand[REAR]  = 1100;
          break;
        case '2':
            motorCommand[FRONT] = 1300;
            motorCommand[REAR]  = 1300;
          break;
        case '3':
            motorCommand[FRONT] = 1500;
            motorCommand[REAR]  = 1500;
          break;
        case 'b': // calibrate gyros
          autoZeroGyros();
          zeroGyros();
          break;
        case 'c': // calibrate accels
          zeroAccelerometers();
          break;
      */
    } /* end switch */
}

/*******************************************************************************
**                                                                            **
** FUNC-NAME     :  COM_SendTelemetry(void)                                   **
**                                                                            **
*******************************************************************************/
void COM_SendTelemetry(void)
{
  update = 0;
  switch (queryType) {
  case 'Q': // Send sensor data
    for (axis = ROLL; axis < LASTAXIS; axis++) {
      Serial.print(gyroADC[axis]);
      Com_SendComma();
    }
    for (axis = ROLL; axis < LASTAXIS; axis++) {
      Serial.print(accelADC[axis]);
      Com_SendComma();
    }
    for (axis = ROLL; axis < YAW; axis++) {
      Serial.print(levelAdjust[axis]);
      Com_SendComma();
    }
    Serial.print(flightAngle[ROLL]);
    Com_SendComma();
    Serial.print(flightAngle[PITCH]);
    Serial.println();
    break;
  case '!': // Send flight software version
    Serial.println("0.1");
    break;
  }
}

/*******************************************************************************
**                                                                            **
** FUNC-NAME     :  Com_SendComma(void)                                       **
**                                                                            **
*******************************************************************************/
void Com_SendComma(void)
{
    Serial.print(',');
}

/*******************************************************************************
**                                                                            **
** FUNC-NAME     :  COM_SendInt(int data)                                     **
**                                                                            **
*******************************************************************************/
void COM_SendInt(int data)
{
    byte msb, lsb;
    
    msb = data >> 8;
    lsb = data & 0xff;
    
    Serial.print(msb, BYTE);
    Serial.print(lsb, BYTE);
}


/*******************************************************************************
**                                                                            **
** FUNC-NAME     :  COM_ReadCommand(void)                                     **
**                                                                            **
*******************************************************************************/
float Com_readFloatSerial(void)
{
  byte index = 0;
  byte timeout = 0;
  char data[128] = "";

  do {
    if (Serial.available() == 0) {
      delay(10);
      timeout++;
    }
    else {
      data[index] = Serial.read();
      timeout = 0;
      index++;
    }
  }  while ((data[limitRange(index-1, 0, 128)] != ';') && (timeout < 5) && (index < 128));
  return atof(data);
}

#ifndef SCP1000_API_H
    #define SCP1000_API_H


/*******************************************************************************
**    Types                                                                   **
*******************************************************************************/

typedef enum
{
    SCP_OP_OK  = 0,
    SCP_OP_NOK = 1
} T_SCP_Result;

typedef enum
{
    SCP_MODE_NO_REQ = 0,
    SCP_MODE_OFF    = 1,
    SCP_MODE_FAST   = 2,
    SCP_MODE_HIRES  = 3
} T_SCP_Mode;

/*******************************************************************************
**    Defines                                                                 **
*******************************************************************************/

#define PRESSURE_INVALID   0UL
#define ALTITUDE_INVALID   0UL

/*******************************************************************************
**    External data                                                           **
*******************************************************************************/



/*******************************************************************************
**    Function prototypes                                                     **
*******************************************************************************/

void SCP_Init(void);
void SCP_Task(void);
T_SCP_Result SCP_ModeChangeRquest(T_SCP_Mode new_mode);
T_SCP_Result SCP_GetAltitude(int *data);
T_SCP_Result SCP_ResetAltitude(unsigned long *data);
T_SCP_Result SCP_GetAbsPressure(void);
T_SCP_Result SCP_GetTemperature(int *data);

#endif

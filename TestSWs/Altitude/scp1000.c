/*******************************************************************************
**                                                                            **
**  MODULE    : SCP1000.c                                                     **
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

/* Altitude calculation based on http://www.engineeringtoolbox.com/air-altitude-pressure-d_462.html */

/*******************************************************************************
**                      Include Section                                       **
*******************************************************************************/

#include "scp1000_api.h"

/*******************************************************************************
**                      Defines                                               **
*******************************************************************************/

#define SCK_PIN   52
#define MISO_PIN  50
#define MOSI_PIN  51
#define SS_PIN    53


/*******************************************************************************
**                      Local Data                                            **
*******************************************************************************/
static unsigned char scp_presReady = 0;
static unsigned char scp_altReady  = 0;
static unsigned char scp_tempReady = 0;
static unsigned char scp_req_mode  = SCP_MODE_NO_REQ;

static unsigned long scp_RawPressure     = PRESSURE_INVALID;
static unsigned long scp_InitRawPressure = 0;
static unsigned int  scp_altitude        = ALTITUDE_INVALID;
static int           scp_temperature     = 0;


/*******************************************************************************
**                      Local Functions                                       **
*******************************************************************************/
void Spi_SetMode(char config);
void Spi_Send(unsigned char data);
unsigned char Spi_Read(void);


/*******************************************************************************
**                                                                            **
** FUNC-NAME     : SCP_Init()                                                 **
**                                                                            **
*******************************************************************************/
void SCP_Init(void)
{
    /* Init SPI peripheral */
    pinMode(SCK_PIN, OUTPUT);
    pinMode(MOSI_PIN, OUTPUT);
    pinMode(MISO_PIN, INPUT);
    pinMode(SS_PIN, OUTPUT);
    SPI_SetMode(_BV(SPIE) | _BV(SPR1)); /* enable SPI ISR, Fosc/16 (250kHz) */

    /* Attach external interrupt to the DRDY pin, SCP1000 data ready. MEGA pin 19 */
    attachInterrupt(EXTERNAL_INT_4, Scp_DataReadyISR, RISING);
}

/*******************************************************************************
**                                                                            **
** FUNC-NAME     : SCP_Task()                                                 **
**                                                                            **
*******************************************************************************/
void SCP_Task(void)
{


}

/*******************************************************************************
**                                                                            **
** FUNC-NAME     : SCP_ModeChangeRequest()                                    **
**                                                                            **
*******************************************************************************/
T_SCP_Result SCP_ModeChangeRequest(T_SCP_Mode new_mode)
{
    if (scp_req_mode == SCP_MODE_NO_REQ)
    { /* Change mode accepted */
        scp_req_mode = new_mode;
        return(SCP_OP_OK);
    }
    else
    { /* Mode change already on-going */
        return(SCP_OP_NOK);
    }
}

/*******************************************************************************
**                                                                            **
** FUNC-NAME     : SCP_GetAltitude()                                          **
**                                                                            **
*******************************************************************************/
T_SCP_Result SCP_GetAltitude(int *data)
{
    if (spc_altReady)
    { /* If sensor is working fine and data is available */
        *data = scp_altitude;
        return(SCP_OP_OK);
    }
    else
    {
        *data = ALTITUDE_INVALID;
        return(SCP_OP_NOK);
    }
}

/*******************************************************************************
**                                                                            **
** FUNC-NAME     : SCP_GetAbsPressure()                                       **
**                                                                            **
*******************************************************************************/
T_SCP_Result SCP_GetAbsPressure(unsigned long *data)
{
    if (spc_presReady)
    { /* If sensor is working fine and data is available */
        *data = scp_RawPressure;
        return(SCP_OP_OK);
    }
    else
    {
        *data = PRESSURE_INVALID;
        return(SCP_OP_NOK);
    }
}

/*******************************************************************************
**                                                                            **
** FUNC-NAME     : SCP_ResetAltitude()                                        **
**                                                                            **
*******************************************************************************/
T_SCP_Result SCP_ResetAltitude(void)
{
    if (scp_altReady)
    { /* If sensor is working fine and data is available */
        scp_InitRawPressure = scp_RawPressure;
        return(SCP_OP_OK);
    }
    else
    {
        return(SCP_OP_NOK);
    }
}

/*******************************************************************************
**                                                                            **
** FUNC-NAME     : SCP_GetTemperature()                                       **
**                                                                            **
*******************************************************************************/
T_SCP_Result SCP_GetTemperature(int *data)
{
    if (scp_tempReady)
    { /* If sensor is working fine and data is available */
        *data = scp_temperature;
        return(SCP_OP_OK);
    }
    else
    {
        *data = PRESSURE_INVALID;
        return(SCP_OP_NOK);
    }
}


/*******************************************************************************
** FUNC-NAME     : Spc_SendCmd(char cmd)                                      **
*******************************************************************************/
char Spc_SendCmd(char cmd)
{
}

/*******************************************************************************
** FUNC-NAME     : SPI_SetMode()                                              **
*******************************************************************************/
void Spi_SetMode(char config)
{
    byte tmp;

    /* enable SPI master with configuration byte specified */
    SPCR = 0;
    SPCR = (config & 0x7F) | (1<<SPE) | (1<<MSTR);
    tmp = SPSR;
    tmp = SPDR;
}

/*******************************************************************************
** FUNC-NAME     : Spi_Send()                                                 **
*******************************************************************************/
void Spi_Send(unsigned char data)
{
    SPDR = data;           /* Start the SPI transmission */
}

/*******************************************************************************
** FUNC-NAME     : Spi_Read()                                                 **
*******************************************************************************/
unsigned char Spi_Read(void)
{
    return(SPDR);           /* Return received SPI data */
}


/*******************************************************************************
**                      Interrputs                                            **
*******************************************************************************/

/*******************************************************************************
**                                                                            **
** FUNC-NAME     : SPI_ISR()                                                  **
**                                                                            **
*******************************************************************************/
ISR(SPI_vect)
{
    data = Spi_Read();
}

/*******************************************************************************
**                                                                            **
** FUNC-NAME     : Scp_DataReadyISR()                                         **
**                                                                            **
*******************************************************************************/
void Scp_DataReadyISR(void)
{
    scp_dataready = 1;
}


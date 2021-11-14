/*!
 * \file      iofHandler.c
 *
 * \brief     IoF LoRa handler implementation
 *
 *
 * \code      IoF Company
 *
 * \endcode
 *
 * \author    Facundo Barrionuevo ( IOF )
 *
 */

#include "iofHandler.h"
#include "LmHandler.h"
#include "secure-element-nvm.h"

SecureElementNvmData_t *SeNvm; //To get DevEui

void printIOF (void)
{
    printf("Imprimiendo desde iofHandler\r\n");
}

// uint8_t iofResetParams(uint8_t *a, uint8_t b)

uint8_t iofJoin (uint8_t *a, uint8_t b)
{
    printf("IoF Join funtion has been called\r\n");
    printf("Params received --- a:%u ---- b:%u\r\n", a[0] , b);
    printf("Proceeding to the Join...\r\n");
    LmHandlerJoin();
    return 0;
}

//uint8_t iofTransmit(uint8_t *a, uint8_t b)

// uint8_t iofGPSTime(uint8_t *a, uint8_t b)

uint8_t iofGetEUI(uint8_t *a, uint8_t b)
{
    printf("IoF get EUI funtion has been called\r\n");
    printf("Params received --- a:%u ---- b:%u\r\n", a[0] , b);
    printf("EUI: ");
    for (uint8_t i = 0 ; i < 8 ; i++)
    {
        printf("%.2x", SeNvm->DevEui[i]);
    }
    printf("\r\n");
    return 0;
}

// uint8_t iofLinkCheck(uint8_t *a, uint8_t b)

// uint8_t iofSleep(uint8_t *a, uint8_t b)

// uint8_t iofConfig(uint8_t *a, uint8_t b)
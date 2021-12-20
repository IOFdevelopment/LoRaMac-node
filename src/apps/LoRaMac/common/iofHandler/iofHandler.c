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
#include "systime.h"
#include "gps.h"
#include <stdio.h>
#include <time.h>

static LmHandlerParams_t *LmHandlerParams; //To transmit function
SecureElementNvmData_t *SeNvm;             //To get DevEui

LmHandlerRxParams_t *RxParams; //To get Rx params
LoRaMacHandlerBeaconParams_t *BeaconParams;
LmHandlerRxParams_t *params;

void printIOF(void)
{
    printf("Imprimiendo desde iofHandler\r\n");
}

// uint8_t iofResetParams(uint8_t *a, uint8_t b)

uint8_t iofJoin(uint8_t *a, uint8_t b)
{
    printf("IoF Join funtion has been called\r\n");
    printf("Params received --- a:%u ---- b:%u\r\n", a[0], b);
    printf("Proceeding to the Join...\r\n");
    LmHandlerJoin();
    return 0;
}

uint8_t iofTransmit(uint8_t *a, uint8_t b)
{
    LmHandlerAppData_t appData =
        {
            .Buffer = a, //Si en a me dan el buffer a enviar y en b el largo
            .BufferSize = b,
            .Port = 0,
        };
    if (LmHandlerSend(&appData, LmHandlerParams->IsTxConfirmed) == LORAMAC_HANDLER_SUCCESS)
    {
        printf("Message sent from iofHandler\r\n");
    }
    else
    {
        printf("Error at send the message from iofHandler\r\n");
    }
    return 0;
}

uint8_t iofGPSTime(uint8_t *a, uint8_t b)
{
    printf("IoF GPS Time funtion has been called\r\n");
    printf("Params received --- a:%u ---- b:%u\r\n", a[0], b);

    SysTime_t timeNow = SysTimeGet();
    printf("UTC seconds: %lu\r\n", timeNow.Seconds);

    // time_t tmi;
    // struct tm* utcTime;
    // time(&tmi);
    // utcTime = gmtime(&tmi);
    // printf("UTC Time: %2d:%02d:%02d Month:%i Year:%i\n", (utcTime->tm_hour) % 24, utcTime->tm_min, utcTime->tm_sec, 1 + utcTime->tm_mon, 1900 + utcTime->tm_year);

    // time_t utc_now;
    // time(&utc_now);
    // printf("UTC time: %s", ctime(&utc_now));
    //a = utc_now;

    return 0;
}

uint8_t iofGetEUI(uint8_t *a, uint8_t b)
{
    printf("IoF get EUI funtion has been called\r\n");
    printf("Params received --- a:%u ---- b:%u\r\n", a[0], b);
    printf("EUI: ");
    for (uint8_t i = 0; i < 8; i++)
    {
        a[i] = SeNvm->DevEui[i]; //retrun DEVEUI at parameter *a
        printf("%.2x", a[i]);
    }
    printf("\r\n");
    return 0;
}


uint8_t iofLinkCheck(uint8_t *a, uint8_t b)
{
    printf("IoF get EUI funtion has been called\r\n");
    printf("Params received --- a:%u ---- b:%u\r\n", a[0], b);

    // printf("RSSI: %i\r\n", RxParams->Rssi);
    // printf("SNR: %i\r\n", RxParams->Snr);
    // printf("Data rate: %i\r\n", RxParams->Datarate);

    printf("DATA RATE   : DR_%d\n", params->Datarate);
    printf("RX RSSI     : %d\n", params->Rssi);
    printf("RX SNR      : %d\n", params->Snr);

    //lleno el buffer a devolver
    a[0] = params->Rssi;
    a[2] = params->Snr;
    a[3] = params->Datarate;
    printf("Vector a total size: %u\n", sizeof(a));
    for(unsigned int i = 0 ; i < sizeof(a) ; i++)
    {
        printf("%u", a[i]);
    }
    printf("\r\n");

    return 0;
}



// uint8_t iofSleep(uint8_t *a, uint8_t b)

// uint8_t iofConfig(uint8_t *a, uint8_t b)
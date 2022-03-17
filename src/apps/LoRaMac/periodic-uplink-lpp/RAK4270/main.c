/*!
 * \file      main.c
 *
 * \brief     Performs a periodic uplink
 *
 * \copyright Revised BSD License, see section \ref LICENSE.
 *
 * \code
 *                ______                              _
 *               / _____)             _              | |
 *              ( (____  _____ ____ _| |_ _____  ____| |__
 *               \____ \| ___ |    (_   _) ___ |/ ___)  _ \
 *               _____) ) ____| | | || |_| ____( (___| | | |
 *              (______/|_____)_|_|_| \__)_____)\____)_| |_|
 *              (C)2013-2018 Semtech
 *
 * \endcode
 *
 * \author    Miguel Luis ( Semtech )
 */

/*! \file periodic-uplink/RAK4270/main.c */

#include <stdio.h>
#include <string.h>
#include "../firmwareVersion.h"
#include "../../common/githubVersion.h"
#include "utilities.h"
#include "board.h"
#include "gpio.h"
#include "uart.h"
#include "delay.h"
#include "RegionCommon.h"

#include "cli.h"
#include "Commissioning.h"
#include "LmHandler.h"
#include "LmhpCompliance.h"
#include "CayenneLpp.h"
#include "LmHandlerMsgDisplay.h"
#include "sx126x.h"

#include "secure-element.h"

#include "iofHandler.h"
#include "iofLoRaWAN.h"

#ifndef ACTIVE_REGION

#warning "No active region defined, LORAMAC_REGION_EU868 will be used as default."

#define ACTIVE_REGION LORAMAC_REGION_EU868

#endif

/*
 * IoF GPIO Pins
 */
iofPins_t iofPins;

extern LmHandlerCallbacks_t LmHandlerCallbacks;

extern LmHandlerParams_t LmHandlerParams;

extern LmhpComplianceParams_t LmhpComplianceParams;

/*!
 * LED GPIO pins objects
 */
extern Gpio_t Led1; // Tx

/*!
 * System time
 */
SysTime_t nowTime =
    {
        .Seconds = 1638141194,
        .SubSeconds = 0,
};

/*
 *LoRa Parameters
 */
extern CommissioningParams_t *CommissioningParams;
extern MibRequestConfirm_t *mibReq;
extern MibParam_t Param;

extern uint32_t TxPeriodicity;

/*!
 * Main application entry point.
 */
int main(void)
{
    boardInit();

    //SysTimeSet( nowTime );

    timersInit();

    //Configure Interrupts GPIO pins
    GpioInit(&iofPins.Test, PA_12, PIN_INPUT, PIN_PUSH_PULL, PIN_NO_PULL, 0);
    GpioSetInterrupt(&iofPins.Test, IRQ_FALLING_EDGE, IRQ_HIGH_PRIORITY, irqUplinkProcess);
    
    iofDefineTxPeriodicity();

    const Version_t appVersion = {.Value = FIRMWARE_VERSION};
    const Version_t gitHubVersion = {.Value = GITHUB_VERSION};
    DisplayAppInfo("periodic-uplink-lpp", &appVersion, &gitHubVersion);

    iofLmHandlerInit();

    //LmHandlerJoin();

    //////////TEST
    uint8_t aTest[] = {'0', '1'}, bTest = 1;
    iofJoin(aTest, bTest);

    iofGetEUI(aTest, bTest);

    iofGPSTime(aTest, bTest);

    iofLinkCheck(aTest, bTest);
    /////////TEST END

    StartTxProcess(LORAMAC_HANDLER_TX_ON_TIMER);

    //DisplayNetworkParametersUpdate(CommissioningParams);

    while (1)
    {
        // Process characters sent over the command line interface
        // CliProcess(&Uart2);

        // Processes the LoRaMac events
        iofLmHandlerProcess();

        GpioToggle(&Led1);

        iofAckRetransmit();

        iofUplinkRequire();

        iofLowPowerMode();

    }
}


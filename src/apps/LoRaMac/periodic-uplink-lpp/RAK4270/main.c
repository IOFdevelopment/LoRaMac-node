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

#ifndef ACTIVE_REGION

#warning "No active region defined, LORAMAC_REGION_EU868 will be used as default."

#define ACTIVE_REGION LORAMAC_REGION_EU868

#endif

/*!
 * LoRaWAN default end-device class
 */
#ifndef LORAWAN_DEFAULT_CLASS
#define LORAWAN_DEFAULT_CLASS CLASS_A
#endif

/*!
 * Defines the application data transmission duty cycle. 5s, value in [ms].
 */
#define APP_TX_DUTYCYCLE 5000

/*!
 * Defines a random delay for application data transmission duty cycle. 1s,
 * value in [ms].
 */
#define APP_TX_DUTYCYCLE_RND 1000

/*!
 * LoRaWAN Adaptive Data Rate
 *
 * \remark Please note that when ADR is enabled the end-device should be static
 */
#define LORAWAN_ADR_STATE LORAMAC_HANDLER_ADR_ON

/*!
 * Default datarate
 *
 * \remark Please note that LORAWAN_DEFAULT_DATARATE is used only when ADR is disabled
 */
#define LORAWAN_DEFAULT_DATARATE DR_0

/*!
 * LoRaWAN confirmed messages
 */
#define LORAWAN_DEFAULT_CONFIRMED_MSG_STATE LORAMAC_HANDLER_UNCONFIRMED_MSG

/*!
 * User application data buffer size
 */
#define LORAWAN_APP_DATA_BUFFER_MAX_SIZE 242

/*!
 * LoRaWAN ETSI duty cycle control enable/disable
 *
 * \remark Please note that ETSI mandates duty cycled transmissions. Use only for test purposes
 */
#define LORAWAN_DUTYCYCLE_ON false

/*!
 * LoRaWAN application port
 * @remark The allowed port range is from 1 up to 223. Other values are reserved.
 */
#define LORAWAN_APP_PORT 2
typedef enum
{
    LORAMAC_HANDLER_TX_ON_TIMER,
    LORAMAC_HANDLER_TX_ON_EVENT,
} LmHandlerTxEvents_t;

/*!
 * User application data
 */
static uint8_t AppDataBuffer[LORAWAN_APP_DATA_BUFFER_MAX_SIZE] = {'0', '1', '2', '3', '4', '5', '6', '7', '8', '9'};
const uint8_t AppDataBufferSize = 10;

/*!
 * User application data structure
 */
static LmHandlerAppData_t AppData =
    {
        .Buffer = AppDataBuffer,
        .BufferSize = AppDataBufferSize,
        .Port = 0,
};

/*!
 * Specifies the state of the application LED
 */
static bool AppLedStateOn = false;

/*!
 * Timer to handle the application data transmission duty cycle
 */
static TimerEvent_t TxTimer;

/*!
 * Timer to handle the state of LED1
 */
static TimerEvent_t Led1Timer;

/*!
 * Timer to handle the state of LED2
 */
static TimerEvent_t Led2Timer;

/*!
 * Timer to handle the state of LED beacon indicator
 */
static TimerEvent_t LedBeaconTimer;

/*
 * IoF GPIO Pins
 */
iofPins_t iofPins;

/*
 * Boolean variable to require uplink procces
 */
bool uplinkRequire = false;

/*
 * Boolean variable to know if the msg OK
 */
extern bool unconfirmedAck;

/*
 * Transmision retries when ACK is not confirmed
 */
uint8_t retriesAck = 1;

/*!
 * System time
 */
SysTime_t nowTime =
    {
        .Seconds = 1638141194,
        .SubSeconds = 0,
};

static void OnMacProcessNotify(void);
static void OnNvmDataChange(LmHandlerNvmContextStates_t state, uint16_t size);
static void OnNetworkParametersChange(CommissioningParams_t *params);
static void OnMacMcpsRequest(LoRaMacStatus_t status, McpsReq_t *mcpsReq, TimerTime_t nextTxIn);
static void OnMacMlmeRequest(LoRaMacStatus_t status, MlmeReq_t *mlmeReq, TimerTime_t nextTxIn);
static void OnJoinRequest(LmHandlerJoinParams_t *params);
static void OnTxData(LmHandlerTxParams_t *params);
static void OnRxData(LmHandlerAppData_t *appData, LmHandlerRxParams_t *params);
static void OnClassChange(DeviceClass_t deviceClass);
static void OnBeaconStatusChange(LoRaMacHandlerBeaconParams_t *params);

static void OnSysTimeUpdate(bool isSynchronized, int32_t timeCorrection);

static void PrepareTxFrame(uint8_t *bufferToSend, uint8_t bufferToSendSize);
static void StartTxProcess(LmHandlerTxEvents_t txEvent);
static void UplinkProcess(uint8_t *bufferToSend, uint8_t bufferToSendSize);

static void OnTxPeriodicityChanged(uint32_t periodicity);
static void OnTxFrameCtrlChanged(LmHandlerMsgTypes_t isTxConfirmed);
static void OnPingSlotPeriodicityChanged(uint8_t pingSlotPeriodicity);

/*!
 * Function executed when Uplink process is require by interrupt
 */
void irqUplinkProcess(void *context);

/*!
 * Function executed on TxTimer event
 */
static void OnTxTimerEvent(void *context);

/*!
 * Function executed on Led 1 Timeout event
 */
static void OnLed1TimerEvent(void *context);

/*!
 * Function executed on Led 2 Timeout event
 */
static void OnLed2TimerEvent(void *context);

/*!
 * \brief Function executed on Beacon timer Timeout event
 */
static void OnLedBeaconTimerEvent(void *context);

/*!
 * Funtion to retransmit the MSG if the ACK is not confirmed
 */
void acksRetransmision(void);

static LmHandlerCallbacks_t LmHandlerCallbacks =
    {
        .GetBatteryLevel = BoardGetBatteryLevel,
        .GetTemperature = NULL,
        .GetRandomSeed = BoardGetRandomSeed,
        .OnMacProcess = OnMacProcessNotify,
        .OnNvmDataChange = OnNvmDataChange,
        .OnNetworkParametersChange = OnNetworkParametersChange,
        .OnMacMcpsRequest = OnMacMcpsRequest,
        .OnMacMlmeRequest = OnMacMlmeRequest,
        .OnJoinRequest = OnJoinRequest,
        .OnTxData = OnTxData,
        .OnRxData = OnRxData,
        .OnClassChange = OnClassChange,
        .OnBeaconStatusChange = OnBeaconStatusChange,
        .OnSysTimeUpdate = OnSysTimeUpdate,
};

static LmHandlerParams_t LmHandlerParams =
    {
        .Region = ACTIVE_REGION,
        .AdrEnable = LORAWAN_ADR_STATE,
        .IsTxConfirmed = LORAWAN_DEFAULT_CONFIRMED_MSG_STATE,
        .TxDatarate = LORAWAN_DEFAULT_DATARATE,
        .PublicNetworkEnable = LORAWAN_PUBLIC_NETWORK,
        .DutyCycleEnabled = LORAWAN_DUTYCYCLE_ON,
        .DataBufferMaxSize = LORAWAN_APP_DATA_BUFFER_MAX_SIZE,
        .DataBuffer = AppDataBuffer,
        .PingSlotPeriodicity = REGION_COMMON_DEFAULT_PING_SLOT_PERIODICITY,
};

static LmhpComplianceParams_t LmhpComplianceParams =
    {
        .FwVersion.Value = FIRMWARE_VERSION,
        .OnTxPeriodicityChanged = OnTxPeriodicityChanged,
        .OnTxFrameCtrlChanged = OnTxFrameCtrlChanged,
        .OnPingSlotPeriodicityChanged = OnPingSlotPeriodicityChanged,
};

/*!
 * Indicates if LoRaMacProcess call is pending.
 *
 * \warning If variable is equal to 0 then the MCU can be set in low power mode
 */
static volatile uint8_t IsMacProcessPending = 0;

static volatile uint8_t IsTxFramePending = 0;

static volatile uint32_t TxPeriodicity = 0;

/*
 *LoRa Parameters
 */
extern CommissioningParams_t *CommissioningParams;
extern MibRequestConfirm_t *mibReq;
extern MibParam_t Param;

/*!
 * LED GPIO pins objects
 */
extern Gpio_t Led1; // Tx
extern Gpio_t Led2; // Rx

/*!
 * UART object used for command line interface handling
 */
extern Uart_t Uart2;

/*
 * Customize buffer to send
 */
uint8_t bufferToSend[] = {'T', 'E', 'S', 'T'};
uint8_t bufferToSendSize = sizeof(bufferToSend);

/*!
 * Main application entry point.
 */
int main(void)
{
    BoardInitMcu();
    BoardInitPeriph();

    //SysTimeSet( nowTime );

    TimerInit(&Led1Timer, OnLed1TimerEvent);
    TimerSetValue(&Led1Timer, 25);

    TimerInit(&Led2Timer, OnLed2TimerEvent);
    TimerSetValue(&Led2Timer, 25);

    TimerInit(&LedBeaconTimer, OnLedBeaconTimerEvent);
    TimerSetValue(&LedBeaconTimer, 5000);

    //Configure Interrupts GPIO pins
    GpioInit(&iofPins.Test, PA_12, PIN_INPUT, PIN_PUSH_PULL, PIN_NO_PULL, 0);
    GpioSetInterrupt(&iofPins.Test, IRQ_FALLING_EDGE, IRQ_HIGH_PRIORITY, irqUplinkProcess);

    // Initialize transmission periodicity variable
    TxPeriodicity = APP_TX_DUTYCYCLE + randr(-APP_TX_DUTYCYCLE_RND, APP_TX_DUTYCYCLE_RND);

    const Version_t appVersion = {.Value = FIRMWARE_VERSION};
    const Version_t gitHubVersion = {.Value = GITHUB_VERSION};
    DisplayAppInfo("periodic-uplink-lpp", &appVersion, &gitHubVersion);

    if (LmHandlerInit(&LmHandlerCallbacks, &LmHandlerParams) != LORAMAC_HANDLER_SUCCESS)
    {
        printf("LoRaMac wasn't properly initialized\n");
        // Fatal error, endless loop.
        while (1)
        {
            GpioToggle(&Led1);
            DelayMs(10);
        }
    }
    else
    {
        printf("LoRaMac properly initialized\r\n");
    }

    // Set system maximum tolerated rx error in milliseconds
    LmHandlerSetSystemMaxRxError(20);

    // The LoRa-Alliance Compliance protocol package should always be
    // initialized and activated.
    LmHandlerPackageRegister(PACKAGE_ID_COMPLIANCE, &LmhpComplianceParams);

    //LmHandlerJoin();

    //////////TEST
    uint8_t aTest[] = {'0', '1'}, bTest = 1;
    iofJoin(aTest, bTest);

    iofGetEUI(aTest, bTest);

    printIOF();

    iofGPSTime(aTest, bTest);

    iofLinkCheck(aTest, bTest);
    /////////TEST END

    StartTxProcess(LORAMAC_HANDLER_TX_ON_TIMER);

    //////////TEST
    // uint8_t aTestTransmit[] = {'H', 'O', 'L', 'A', 'F', 'A', 'C', 'U', 'C', 'O', 'M', 'O', 'V', 'A'},
    //         bTestTransmit;
    // bTestTransmit = sizeof(aTestTransmit);
    // iofTransmit(aTestTransmit, bTestTransmit);
    //////////TEST END

    //DisplayNetworkParametersUpdate(CommissioningParams);

    while (1)
    {
        // Process characters sent over the command line interface
        // CliProcess(&Uart2);

        // Processes the LoRaMac events
        LmHandlerProcess();
        GpioToggle(&Led1);

        if (unconfirmedAck == true)
        {
            if (retriesAck < (MAX_ACK_RETRIES + 1))
            {
                printf("Intento de retransmision #%u por NACK\r\n", retriesAck);
                //uplinkRequire = true;
                printf("Uplink Process require\r\n");
                UplinkProcess(bufferToSend, bufferToSendSize);
                retriesAck++;
            }
            else
            {
                //printf("Se superaron los intentos de retransmision por NACK...\r\nDestruyendo sesion..\r\n");

                //TODO: Destruir sesión
                /////////////////////////////DESTRUCCIÓN DE SESIÓN////////////////////////
                // MibRequestConfirm_t mibReq;

                // mibReq.Type = MIB_NETWORK_ACTIVATION;
                // mibReq.Param.NetworkActivation = ACTIVATION_TYPE_NONE;
                // LoRaMacMibSetRequestConfirm(&mibReq);
                // printf("Sesión destruida\r\n");
                /////////////////////////////DESTRUCCIÓN DE SESIÓN////////////////////////

                uplinkRequire = false;
                unconfirmedAck = false;
                //retriesAck = 1;
            }
        }

        if (uplinkRequire == true)
        {
            // Process application uplinks management
            //iofTransmit(bufferToSend, bufferToSendSize);
            //bufferToSendSize = sizeof(bufferToSend);
            printf("Uplink Process require\r\n");
            UplinkProcess(bufferToSend, bufferToSendSize);
            uplinkRequire = false;
            unconfirmedAck = false;
            retriesAck = 1;
        }

        CRITICAL_SECTION_BEGIN();
        if (IsMacProcessPending == 1)
        {
            // Clear flag and prevent MCU to go into low power modes.
            IsMacProcessPending = 0;
        }
        else
        {
            // The MCU wakes up through events
            BoardLowPowerHandler();
        }
        CRITICAL_SECTION_END();
    }
}

void irqUplinkProcess(void *context)
{
    printf("IRQ Uplink Process called\r\n");
    uplinkRequire = true;
}

static void OnMacProcessNotify(void)
{
    IsMacProcessPending = 1;
}

static void OnNvmDataChange(LmHandlerNvmContextStates_t state, uint16_t size)
{
    DisplayNvmDataChange(state, size);
}

static void OnNetworkParametersChange(CommissioningParams_t *params)
{
    DisplayNetworkParametersUpdate(params);
}

static void OnMacMcpsRequest(LoRaMacStatus_t status, McpsReq_t *mcpsReq, TimerTime_t nextTxIn)
{
    DisplayMacMcpsRequestUpdate(status, mcpsReq, nextTxIn);
}

static void OnMacMlmeRequest(LoRaMacStatus_t status, MlmeReq_t *mlmeReq, TimerTime_t nextTxIn)
{
    DisplayMacMlmeRequestUpdate(status, mlmeReq, nextTxIn);
}

static void OnJoinRequest(LmHandlerJoinParams_t *params)
{
    DisplayJoinRequestUpdate(params);
    if (params->Status == LORAMAC_HANDLER_ERROR)
    {
        LmHandlerJoin();
    }
    else
    {
        LmHandlerRequestClass(LORAWAN_DEFAULT_CLASS);
    }
}

static void OnTxData(LmHandlerTxParams_t *params)
{
    DisplayTxUpdate(params);
}

static void OnRxData(LmHandlerAppData_t *appData, LmHandlerRxParams_t *params)
{
    DisplayRxUpdate(appData, params);

    switch (appData->Port)
    {
    case 1: // The application LED can be controlled on port 1 or 2
    case LORAWAN_APP_PORT:
    {
        AppLedStateOn = appData->Buffer[0] & 0x01;
        break;
    }
    default:
        break;
    }

    // Switch LED 2 ON for each received downlink
    GpioWrite(&Led2, 1);
    TimerStart(&Led2Timer);
}

static void OnClassChange(DeviceClass_t deviceClass)
{
    DisplayClassUpdate(deviceClass);

    // Inform the server as soon as possible that the end-device has switched to ClassB
    LmHandlerAppData_t appData =
        {
            .Buffer = NULL,
            .BufferSize = 0,
            .Port = 0,
        };
    LmHandlerSend(&appData, LORAMAC_HANDLER_UNCONFIRMED_MSG);
}

static void OnBeaconStatusChange(LoRaMacHandlerBeaconParams_t *params)
{
    switch (params->State)
    {
    case LORAMAC_HANDLER_BEACON_RX:
    {
        TimerStart(&LedBeaconTimer);
        break;
    }
    case LORAMAC_HANDLER_BEACON_LOST:
    case LORAMAC_HANDLER_BEACON_NRX:
    {
        TimerStop(&LedBeaconTimer);
        break;
    }
    default:
    {
        break;
    }
    }

    DisplayBeaconUpdate(params);
}

static void OnSysTimeUpdate(bool isSynchronized, int32_t timeCorrection)
{
}

/*!
 * Prepares the payload of the frame and transmits it.
 */
static void PrepareTxFrame(uint8_t *bufferToSend, uint8_t bufferToSendSize)
{
    if (LmHandlerIsBusy() == true)
    {
        return;
    }

    uint8_t channel = 0;

    AppData.Port = LORAWAN_APP_PORT;

    CayenneLppReset();
    CayenneLppAddDigitalInput(channel++, AppLedStateOn);
    CayenneLppAddAnalogInput(channel++, BoardGetBatteryLevel() * 100 / 254);

    CayenneLppCopy(AppData.Buffer);
    AppData.BufferSize = CayenneLppGetSize();

    LmHandlerAppData_t customizeBufferToSend =
        {
            .Buffer = bufferToSend, //Si en a me dan el buffer a enviar y en b el largo
            .BufferSize = bufferToSendSize,
            .Port = 2,
        };

    //if (LmHandlerSend(&AppData, LmHandlerParams.IsTxConfirmed) == LORAMAC_HANDLER_SUCCESS)
    //if (LmHandlerSend(&AppData, LORAMAC_HANDLER_CONFIRMED_MSG) == LORAMAC_HANDLER_SUCCESS)
    if (LmHandlerSend(&customizeBufferToSend, LORAMAC_HANDLER_CONFIRMED_MSG) == LORAMAC_HANDLER_SUCCESS)
    {
        // Switch LED 1 ON
        printf("ENVIADO CON ÉXITO\r\n");
        GpioWrite(&Led1, 1);
        TimerStart(&Led1Timer);
    }
    else
    {
        printf("FALLO EN LmHandlerSend AL ENVIAR\r\n");
        //TODO: Volver a eniar conforme a MAX_ACK_RETRIES?
    }
}

static void StartTxProcess(LmHandlerTxEvents_t txEvent)
{
    switch (txEvent)
    {
    default:
        // Intentional fall through
    case LORAMAC_HANDLER_TX_ON_TIMER:
    {
        // Schedule 1st packet transmission
        TimerInit(&TxTimer, OnTxTimerEvent);
        TimerSetValue(&TxTimer, TxPeriodicity);
        OnTxTimerEvent(NULL);
    }
    break;
    case LORAMAC_HANDLER_TX_ON_EVENT:
    {
    }
    break;
    }
}

static void UplinkProcess(uint8_t *bufferToSend, uint8_t bufferToSendSize)
{
    uint8_t isPending = 0;
    CRITICAL_SECTION_BEGIN();
    isPending = IsTxFramePending;
    IsTxFramePending = 0;
    CRITICAL_SECTION_END();
    if (isPending == 1)
    {
        PrepareTxFrame(bufferToSend, bufferToSendSize);
    }
}

static void OnTxPeriodicityChanged(uint32_t periodicity)
{
    TxPeriodicity = periodicity;

    if (TxPeriodicity == 0)
    { // Revert to application default periodicity
        TxPeriodicity = APP_TX_DUTYCYCLE + randr(-APP_TX_DUTYCYCLE_RND, APP_TX_DUTYCYCLE_RND);
    }

    // Update timer periodicity
    TimerStop(&TxTimer);
    TimerSetValue(&TxTimer, TxPeriodicity);
    TimerStart(&TxTimer);
}

static void OnTxFrameCtrlChanged(LmHandlerMsgTypes_t isTxConfirmed)
{
    LmHandlerParams.IsTxConfirmed = isTxConfirmed;
}

static void OnPingSlotPeriodicityChanged(uint8_t pingSlotPeriodicity)
{
    LmHandlerParams.PingSlotPeriodicity = pingSlotPeriodicity;
}

/*!
 * Function executed on TxTimer event
 */
static void OnTxTimerEvent(void *context)
{
    TimerStop(&TxTimer);

    IsTxFramePending = 1;

    // Schedule next transmission
    TimerSetValue(&TxTimer, TxPeriodicity);
    TimerStart(&TxTimer);
}

/*!
 * Function executed on Led 1 Timeout event
 */
static void OnLed1TimerEvent(void *context)
{
    TimerStop(&Led1Timer);
    // Switch LED 1 OFF
    GpioWrite(&Led1, 0);
}

/*!
 * Function executed on Led 2 Timeout event
 */
static void OnLed2TimerEvent(void *context)
{
    TimerStop(&Led2Timer);
    // Switch LED 2 OFF
    GpioWrite(&Led2, 0);
}

/*!
 * \brief Function executed on Beacon timer Timeout event
 */
static void OnLedBeaconTimerEvent(void *context)
{
    GpioWrite(&Led2, 1);
    TimerStart(&Led2Timer);

    TimerStart(&LedBeaconTimer);
}

/*!
 * \brief Retransmit the msg if the ACK was not confirmed
 */
void acksRetransmision(void)
{
    if (unconfirmedAck)
    { //Si el mensaje no fue confirmado volver a enviarlo la cantidad de veces que diga la variable MAX_ACK_RETRIES
        uint8_t retries = 1;
        while (retries < (MAX_ACK_RETRIES + 1))
        {
            printf("Reenvio #%u por ACK no confirmado\r\n", retries);
            if (unconfirmedAck == true)
            {
                UplinkProcess(bufferToSend, bufferToSendSize);
            }
            else
            {
                return;
            }
            retries++;
        }
        if (retries == 4) //Sginifica que en 3 intentos no se pudo, entonces destruimos la sesión
        {
            //Destruir sesión
            printf("Se llegó a la máxima cantidad de intentos fallidos... Se destruirá la sesión\r\n");
            uplinkRequire = false;
        }
        else
        {
            printf("Reenviado con éxito\r\n");
        }
    }
}

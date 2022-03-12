/*!
 * \file      iofLoRaWAN.c
 *
 * \brief     IoF LoRaWAN handler implementation
 *
 *
 * \code      IoF Company
 *
 * \endcode
 *
 * \author    Facundo Barrionuevo ( IOF )
 *
 */

#include "iofLoRaWAN.h"
#include <stdio.h>
#include <time.h>

/*!
 * User application data
 */
static uint8_t AppDataBuffer[LORAWAN_APP_DATA_BUFFER_MAX_SIZE] = {'0', '1', '2', '3', '4', '5', '6', '7', '8', '9'};
const uint8_t AppDataBufferSize = 10;

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
 * User application data structure
 */
static LmHandlerAppData_t AppData =
    {
        .Buffer = AppDataBuffer,
        .BufferSize = AppDataBufferSize,
        .Port = 0,
};

/*
 * Customize buffer to send
 */
uint8_t bufferToSend[] = {'T', 'E', 'S', 'T'};
uint8_t bufferToSendSize = sizeof(bufferToSend);

/*
 * Transmision retries when ACK is not confirmed
 */
uint8_t retriesAck = 1;

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
 * Boolean variable to require uplink procces
 */
bool uplinkRequire = false;

/* Variable to know if the msg is CONFIRMED - ACK (false) or UNCONFIRMED - NACK (true) */
bool unconfirmedAck = false;

/*!
 * Indicates if LoRaMacProcess call is pending.
 *
 * \warning If variable is equal to 0 then the MCU can be set in low power mode
 */
static volatile uint8_t IsMacProcessPending = 0;

static volatile uint8_t IsTxFramePending = 0;

static volatile uint32_t TxPeriodicity = 0;

/*!
 * LED GPIO pins objects
 */
extern Gpio_t Led1; // Tx
extern Gpio_t Led2; // Rx

/*!
 * UART object used for command line interface handling
 */
extern Uart_t Uart2;


void irqUplinkProcess(void *context)
{
    printf("IRQ Uplink Process called\r\n");
    uplinkRequire = true;
}

void OnMacProcessNotify(void)
{
    IsMacProcessPending = 1;
}

void OnNvmDataChange(LmHandlerNvmContextStates_t state, uint16_t size)
{
    DisplayNvmDataChange(state, size);
}

void OnNetworkParametersChange(CommissioningParams_t *params)
{
    DisplayNetworkParametersUpdate(params);
}

void OnMacMcpsRequest(LoRaMacStatus_t status, McpsReq_t *mcpsReq, TimerTime_t nextTxIn)
{
    DisplayMacMcpsRequestUpdate(status, mcpsReq, nextTxIn);
}

void OnMacMlmeRequest(LoRaMacStatus_t status, MlmeReq_t *mlmeReq, TimerTime_t nextTxIn)
{
    DisplayMacMlmeRequestUpdate(status, mlmeReq, nextTxIn);
}

void OnJoinRequest(LmHandlerJoinParams_t *params)
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

void OnTxData(LmHandlerTxParams_t *params)
{
    DisplayTxUpdate(params);
}

void OnRxData(LmHandlerAppData_t *appData, LmHandlerRxParams_t *params)
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

void OnClassChange(DeviceClass_t deviceClass)
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

void OnBeaconStatusChange(LoRaMacHandlerBeaconParams_t *params)
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

void OnSysTimeUpdate(bool isSynchronized, int32_t timeCorrection)
{
}

/*!
 * Prepares the payload of the frame and transmits it.
 */
void PrepareTxFrame(uint8_t *bufferToSend, uint8_t bufferToSendSize)
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

void StartTxProcess(LmHandlerTxEvents_t txEvent)
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

void UplinkProcess(uint8_t *bufferToSend, uint8_t bufferToSendSize)
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

void OnTxPeriodicityChanged(uint32_t periodicity)
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

void OnTxFrameCtrlChanged(LmHandlerMsgTypes_t isTxConfirmed)
{
    LmHandlerParams.IsTxConfirmed = isTxConfirmed;
}
void OnPingSlotPeriodicityChanged(uint8_t pingSlotPeriodicity)
{
    LmHandlerParams.PingSlotPeriodicity = pingSlotPeriodicity;
}

/*!
 * Function executed on TxTimer event
 */
void OnTxTimerEvent(void *context)
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
void OnLed1TimerEvent(void *context)
{
    TimerStop(&Led1Timer);
    // Switch LED 1 OFF
    GpioWrite(&Led1, 0);
}

/*!
 * Function executed on Led 2 Timeout event
 */
void OnLed2TimerEvent(void *context)
{
    TimerStop(&Led2Timer);
    // Switch LED 2 OFF
    GpioWrite(&Led2, 0);
}

/*!
 * \brief Function executed on Beacon timer Timeout event
 */
void OnLedBeaconTimerEvent(void *context)
{
    GpioWrite(&Led2, 1);
    TimerStart(&Led2Timer);

    TimerStart(&LedBeaconTimer);
}

void boardInit(void)
{
    BoardInitMcu();
    BoardInitPeriph();
}

void timersInit(void)
{
    TimerInit(&Led1Timer, OnLed1TimerEvent);
    TimerSetValue(&Led1Timer, 25);

    TimerInit(&Led2Timer, OnLed2TimerEvent);
    TimerSetValue(&Led2Timer, 25);

    TimerInit(&LedBeaconTimer, OnLedBeaconTimerEvent);
    TimerSetValue(&LedBeaconTimer, 5000);;
}

void iofLmHandlerInit(void)
{
    if (LmHandlerInit(&LmHandlerCallbacks, &LmHandlerParams) != LORAMAC_HANDLER_SUCCESS)
    {
        printf("LoRaMac wasn't properly initialized\n");
        // Fatal error, endless loop.
        while (1)
        {
            GpioToggle(&Led1);
            //DelayMs(10);
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
}

void iofLmHandlerProcess(void)
{
    LmHandlerProcess();
}

void iofAckRetransmit(void)
{
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
}

void iofUplinkRequire(void)
{
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

}

void iofLowPowerMode(void)
{
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

void iofDefineTxPeriodicity(void)
{
    // Initialize transmission periodicity variable
    TxPeriodicity = APP_TX_DUTYCYCLE + randr(-APP_TX_DUTYCYCLE_RND, APP_TX_DUTYCYCLE_RND);
}
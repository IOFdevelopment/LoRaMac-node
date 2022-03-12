/*!
 * \file      iofLoRaWAN.h
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

#ifndef __IOFLORAWAN_H__
#define __IOFLORAWAN_H__

#include <stdio.h>
#include <stdint.h>
#include <string.h>
#include "LmHandler.h"
#include "LmhpCompliance.h"
#include "../firmwareVersion.h"
#include "../../common/githubVersion.h"
#include "secure-element-nvm.h"
#include "systime.h"
#include "gps.h"
#include "cli.h"
#include "Commissioning.h"
#include "CayenneLpp.h"
#include "LmHandlerMsgDisplay.h"
#include "sx126x.h"
#include "board.h"
#include "RegionCommon.h"
#include "secure-element.h"

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
 * \brief Reset all LoRa params to default
 *
 * \param 
 * \retval 
 */
void OnMacProcessNotify(void);

void OnNvmDataChange(LmHandlerNvmContextStates_t state, uint16_t size);
void OnNetworkParametersChange(CommissioningParams_t *params);
void OnMacMcpsRequest(LoRaMacStatus_t status, McpsReq_t *mcpsReq, TimerTime_t nextTxIn);
void OnMacMlmeRequest(LoRaMacStatus_t status, MlmeReq_t *mlmeReq, TimerTime_t nextTxIn);
void OnJoinRequest(LmHandlerJoinParams_t *params);
void OnTxData(LmHandlerTxParams_t *params);
void OnRxData(LmHandlerAppData_t *appData, LmHandlerRxParams_t *params);
void OnClassChange(DeviceClass_t deviceClass);
void OnBeaconStatusChange(LoRaMacHandlerBeaconParams_t *params);

void OnSysTimeUpdate(bool isSynchronized, int32_t timeCorrection);

void PrepareTxFrame(uint8_t *bufferToSend, uint8_t bufferToSendSize);
void StartTxProcess(LmHandlerTxEvents_t txEvent);
void UplinkProcess(uint8_t *bufferToSend, uint8_t bufferToSendSize);

void OnTxPeriodicityChanged(uint32_t periodicity);
void OnTxFrameCtrlChanged(LmHandlerMsgTypes_t isTxConfirmed);
void OnPingSlotPeriodicityChanged(uint8_t pingSlotPeriodicity);

/*!
 * Function executed when Uplink process is require by interrupt
 */
void irqUplinkProcess(void *context);

/*!
 * Function executed on TxTimer event
 */
void OnTxTimerEvent(void *context);

/*!
 * Function executed on Led 1 Timeout event
 */
void OnLed1TimerEvent(void *context);

/*!
 * Function executed on Led 2 Timeout event
 */
void OnLed2TimerEvent(void *context);

/*!
 * \brief Function executed on Beacon timer Timeout event
 */
void OnLedBeaconTimerEvent(void *context);

void boardInit(void);
void timersInit(void);
void iofLmHandlerInit(void);
void iofAckRetransmit(void);
void iofUplinkRequire(void);
void iofLowPowerMode(void);
void iofLmHandlerProcess(void);
void iofDefineTxPeriodicity(void);

#endif // __IOFLORAWAN_H__
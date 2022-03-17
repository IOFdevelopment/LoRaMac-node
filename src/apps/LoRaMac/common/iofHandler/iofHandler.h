/*!
 * \file      iofHandler.h
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

#ifndef __IOFHANDLER_H__
#define __IOFHANDLER_H__

#include <stdio.h>
#include <stdint.h>
#include <string.h>
#include "gpio.h"

#define RAK_RST     PB_14    //Controlled by Master (ST)
#define RAK_IO1     PB_12    //Controlled by Master (ST)
#define RAK_IO2     PB_4     //Controlled by Slave  (RAK)
#define RAK_UART_TX PA_2    //Controlled by Slave  (RAK)
#define RAK_UART_RX PA_3    //Controlled by Master (ST) 

typedef struct iofPins_s
{
    Gpio_t Test;
    Gpio_t rakRst;
    Gpio_t rakIO1;
    Gpio_t rakIO2;
    Gpio_t rakUartTX;
    Gpio_t rakUartRX;
} iofPins_t;

void printIOF(void);

/*!
 * \brief Reset all LoRa params to default
 *
 * \param 
 * \retval 
 */
// uint8_t iofResetParams(uint8_t *a, uint8_t b);

/*!
 * \brief Join to the network
 *
 * \param 
 * \retval 
 */
uint8_t iofJoin(uint8_t *a, uint8_t b);

/*!
 * \brief 
 *
 * \param *a: buffer to transmit
 *         b: buffer size
 * \retval 
 */
uint8_t iofTransmit(uint8_t *a, uint8_t b);

/*!
 * \brief 
 *
 * \param 
 * \retval 
 */
uint8_t iofGPSTime(uint8_t *a, uint8_t b);

/*!
 * \brief Get the DevEui
 *
 * \param 
 * \retval 
 */
uint8_t iofGetEUI(uint8_t *a, uint8_t b);

/*!
 * \brief 
 *
 * \param 
 * \retval 
 */
uint8_t iofLinkCheck(uint8_t *a, uint8_t b);

/*!
 * \brief 
 *
 * \param 
 * \retval 
 */
// uint8_t iofSleep(uint8_t *a, uint8_t b);

/*!
 * \brief 
 *
 * \param 
 * \retval 
 */
// uint8_t iofConfig(uint8_t *a, uint8_t b);

#endif // __IOFHANDLER_H__
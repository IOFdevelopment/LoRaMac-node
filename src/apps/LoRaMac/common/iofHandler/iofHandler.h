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

void printIOF (void);

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
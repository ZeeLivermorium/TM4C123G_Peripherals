/*!
 * SSI2.h
 * ----------
 * Inspired by examples in ValvanoWareTM4C123 by Dr. Jonathan Valvano
 * as well as his book Embedded Systems: Real-Time Interfacing to Arm Cortex-M Microcontrollers
 * You can find ValvanoWareTM4C123 at http://edx-org-utaustinx.s3.amazonaws.com/UT601x/ValvanoWareTM4C123.zip?dl=1
 * You can find his book at https://www.amazon.com/gp/product/1463590156/ref=oh_aui_detailpage_o05_s00?ie=UTF8&psc=1
 * You can find more of his work at http://users.ece.utexas.edu/~valvano/
 * ----------
 * @author Zee Livermorium
 * @date Apr 14, 2018
 */

#ifndef __SSI2_H__
#define __SSI2_H__

#include <stdint.h>

/*
 *  SSI2 B Conncection
 *  ------------------
 *  SCK  --------- PB4
 *  SS   --------- PB5
 *  MISO --------- PB6
 *  MOSI --------- PB7
 */


/****************************************************
 *                                                  *
 *                 Helper Functions                 *
 *                                                  *
 ****************************************************/

/**
 * reverseBitOrder
 * ----------
 * Discription: to output in the order of LSB first, we need to reverse all bits.
 */
static uint8_t reverseBitOrder(uint8_t byte);
/****************************************************
 *                                                  *
 *                   SS Functions                   *
 *                                                  *
 ****************************************************/

static void SSI2_SS_HIGH (void);

static void SSI2_SS_LOW (void);


/****************************************************
 *                                                  *
 *                   Initializer                    *
 *                                                  *
 ****************************************************/

/**
 * SSI2_Init
 * ----------
 * @brief initialize SSI with corresponding setting parameters.
 */
void SSI2_Init();


/****************************************************
 *                                                  *
 *                   I/O Functions                  *
 *                                                  *
 ****************************************************/

/**
 * SSI2_read
 * ----------
 * @return date read from slave device.
 */
static uint16_t SSI2_read (void);

/**
 * SSI2_write
 * ----------
 * @param  data  data to be written.
 */
static void SSI2_write(uint16_t data);

#endif

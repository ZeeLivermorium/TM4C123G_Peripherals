/*!
 * SSI3.h
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

#ifndef __SSI3_H__
#define __SSI3_H__

#include <stdint.h>

/*
 *  SSI3 D Conncection
 *  ------------------
 *  SCK  --------- PD0
 *  SS   --------- PD1
 *  MISO --------- PD2
 *  MOSI --------- PD3
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

static void SSI3_SS_HIGH (void);

static void SSI3_SS_LOW (void);


/****************************************************
 *                                                  *
 *                   Initializer                    *
 *                                                  *
 ****************************************************/

/**
 * SSI3_Init
 * ----------
 * @brief initialize SSI with corresponding setting parameters.
 */
void SSI3_Init();


/****************************************************
 *                                                  *
 *                   I/O Functions                  *
 *                                                  *
 ****************************************************/

/**
 * SSI3_read
 * ----------
 * @return date read from slave device.
 */
static uint16_t SSI3_read (void);

/**
 * SS3I_write
 * ----------
 * @param  data  data to be written.
 */
static void SSI3_write(uint16_t data);

#endif

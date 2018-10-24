/*!
 * SSI1.h
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

#ifndef __SSI1_H__
#define __SSI1_H__

#include <stdint.h>

/*
 *  SSI1 D Conncection | SSI1 F Conncection
 *  ------------------ | ------------------
 *  SCK  --------- PD0 | SCK  --------- PF2
 *  SS   --------- PD1 | SS   --------- PF3
 *  MISO --------- PD2 | MISO --------- PF0
 *  MOSI --------- PD3 | MOSI --------- PF1
 */

#if 1       // set to 1 for SSI1 on Port D, 0 on Port F
    #define SSI1D
#else       // **WARNING** do not use on board LEDs if you want to use Port F SSI, they overlap
    #define SSI1F
#endif

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

static void SS1_SS_HIGH (void);

static void SS1_SS_LOW (void);


/****************************************************
 *                                                  *
 *                   Initializer                    *
 *                                                  *
 ****************************************************/

/**
 * SSI1_Init
 * ----------
 * @brief initialize SSI with corresponding setting parameters.
 */
void SSI1_Init();


/****************************************************
 *                                                  *
 *                   I/O Functions                  *
 *                                                  *
 ****************************************************/

/**
 * SSI1_read
 * ----------
 * @return date read from slave device.
 */
static uint16_t SSI1_read (void);

/**
 * SSI1_write
 * ----------
 * @param  data  data to be written.
 */
static void SSI1_write(uint16_t data);

#endif


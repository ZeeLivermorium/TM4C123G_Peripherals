/*!
 * SSI3.c
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

#include <stdint.h>
#include <string.h>
#include "SSI3.h"
#include "tm4c123gh6pm.h"


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
static uint8_t reverseBitOrder(uint8_t byte) {
    return (((byte & 0x01) << 7) +
            ((byte & 0x02) << 5) +
            ((byte & 0x04) << 3) +
            ((byte & 0x08) << 1) +
            ((byte & 0x10) >> 1) +
            ((byte & 0x20) >> 3) +
            ((byte & 0x40) >> 5) +
            ((byte & 0x80) >> 7));
}

/****************************************************
 *                                                  *
 *                   SS Functions                   *
 *                                                  *
 ****************************************************/

static void SSI3_SS_HIGH (void) {
    GPIO_PORTD_DATA_R |= 0x02;
}

static void SSI3_SS_LOW (void) {
    GPIO_PORTD_DATA_R &= ~0x02;
}


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
void SSI3_Init() {
    /* SSI3 and Port D Activation */
    SYSCTL_RCGCSSI_R |= SYSCTL_RCGCSSI_R3;                 // enable SSI Module 3 clock
    SYSCTL_RCGCGPIO_R |= SYSCTL_RCGCGPIO_R3;               // enable GPIO Port D clock
    while ((SYSCTL_PRGPIO_R & SYSCTL_PRGPIO_R3) == 0) {};  // allow time for activating
    
    /* Port D Set Up */
    GPIO_PORTD_DIR_R |= 0x02;                              // make PD1 output
    GPIO_PORTD_AFSEL_R |= 0x0D;                            // enable alt funct on PD0, 2, 3
    GPIO_PORTD_AFSEL_R &= ~0x02;                           // disable alt funct on PD1
    GPIO_PORTD_PCTL_R &= ((~GPIO_PCTL_PD0_M) &             // clear bit fields for PD0
                          (~GPIO_PCTL_PD1_M) &             // clear bit fields for PD1, PD1 will be used as GPIO
                          (~GPIO_PCTL_PD2_M) &             // clear bit fields for PD2
                          (~GPIO_PCTL_PD3_M));             // clear bit fields for PD3
    GPIO_PORTD_PCTL_R |= (GPIO_PCTL_PD0_SSI3CLK |          // configure PD0 as SSI3CLK
                          GPIO_PCTL_PD2_SSI3RX  |          // configure PD2 as SSI3RX
                          GPIO_PCTL_PD3_SSI3TX);           // configure PD3 as SSI3TX
    GPIO_PORTD_AMSEL_R &= ~0x0F;                           // disable analog functionality on PD0-3
    GPIO_PORTD_DEN_R |= 0x0F;                              // enable digital I/O on PD0-3
    
    /* SSI3 Set Up */
    SSI3_CR1_R &= ~SSI_CR1_SSE;                            // disable SSI operation
    SSI3_CR1_R &= ~SSI_CR1_MS;                             // configure SSI3 as master mode
    SSI3_CPSR_R &= SSI_CPSR_CPSDVSR_M;                     // clear bit fields for SSI Clock Prescale Divisor
    SSI3_CPSR_R += 40;                                     // /40 clock divisor, 2Mhz
    SSI3_CR0_R &= ~SSI_CR0_SCR_M;                          // clear bit fields for SSI3 Serial Clock Rate, SCR = 0
    SSI3_CR0_R &= ~SSI_CR0_SPO;                            // clear bit fields for SSI3 Serial Clock Polarity, SPO = 0
    SSI3_CR0_R &= ~SSI_CR0_SPH;                            // clear bit fields for SSI3 Serial Clock Phase, SPH = 0
    SSI3_CR0_R &= ~SSI_CR0_FRF_M;                          // clear bit fields for SSI3 Frame Format Select
    SSI3_CR0_R |= SSI_CR0_FRF_MOTO;                        // set frame format to Freescale SPI Frame Format
    SSI3_CR0_R &= ~SSI_CR0_DSS_M;                          // clear bit fields for SSI3 Data Size Select
    SSI3_CR0_R |= SSI_CR0_DSS_8;                           // set SSI data size to 8
    SSI3_CR1_R |= SSI_CR1_SSE;                             // enable SSI operation
}


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
static uint16_t SSI3_read (void) {
    while((SSI3_SR_R & SSI_SR_BSY) == SSI_SR_BSY) {};     // wait until SSI3 not busy/transmit FIFO empty
    SSI3_DR_R = 0x00;                                     // data out, garbage, just for synchronization
    while((SSI3_SR_R & SSI_SR_RNE) == 0) {};              // wait until response
    return SSI3_DR_R;                                     // read data
    
}

/**
 * SS3I_write
 * ----------
 * @param  data  data to be written.
 */
static void SSI3_write (uint16_t data){
    while ((SSI3_SR_R & SSI_SR_BSY) == SSI_SR_BSY) {};   // wait until SSI3 not busy/transmit FIFO empty
    SSI3_DR_R = data;                                    // write data
    while ((SSI3_SR_R & SSI_SR_RNE) == 0) {};            // wait until response
    uint16_t sync = SSI3_DR_R;                           // read byte of data, just for synchronization
}


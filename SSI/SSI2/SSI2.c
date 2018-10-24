/*!
 * SSI2.c
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
#include "SSI2.h"
#include "tm4c123gh6pm.h"


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

static void SSI2_SS_HIGH (void) {
    GPIO_PORTB_DATA_R |= 0x20;
}

static void SSI2_SS2_LOW (void) {
    GPIO_PORTB_DATA_R &= ~0x20;
}


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
void SSI2_Init() {
    /* SSI2 and Port B Activation */
    SYSCTL_RCGCSSI_R |= SYSCTL_RCGCSSI_R2;                 // enable SSI Module 2 clock
    SYSCTL_RCGCGPIO_R |= SYSCTL_RCGCGPIO_R1;               // enable GPIO Port B clock
    while ((SYSCTL_PRGPIO_R & SYSCTL_PRGPIO_R1) == 0) {};  // allow time for activating
    
    /* Port B Set Up */
    GPIO_PORTB_DIR_R |= 0x20;                              // make PB5 output
    GPIO_PORTB_AFSEL_R |= 0xD0;                            // enable alt funct on PB4, 6, 7
    GPIO_PORTB_AFSEL_R &= ~0x20;                           // disable alt funct on PB5
    GPIO_PORTB_PCTL_R &= ((~GPIO_PCTL_PB4_M) &             // clear bit fields for PB4
                          (~GPIO_PCTL_PB5_M) &             // clear bit fields for PB5, PB5 will be used as GPIO
                          (~GPIO_PCTL_PB6_M) &             // clear bit fields for PB6
                          (~GPIO_PCTL_PB7_M));             // clear bit fields for PB7
    GPIO_PORTB_PCTL_R |= (GPIO_PCTL_PB4_SSI2CLK |          // configure PB4 as SSI2CLK
                          GPIO_PCTL_PB6_SSI2RX  |          // configure PB6 as SSI2RX
                          GPIO_PCTL_PB7_SSI2TX);           // configure PB7 as SSI2TX
    GPIO_PORTB_AMSEL_R &= ~0xF0;                           // disable analog functionality on PB4-7
    GPIO_PORTB_DEN_R |= 0xF0;                              // enable digital I/O on PB4-7
    
    /* SSI2 Set Up */
    SSI2_CR1_R &= ~SSI_CR1_SSE;                            // disable SSI operation
    SSI2_CR1_R &= ~SSI_CR1_MS;                             // configure SSI2 as master mode
    SSI2_CPSR_R &= SSI_CPSR_CPSDVSR_M;                     // clear bit fields for SSI Clock Prescale Divisor
    SSI2_CPSR_R += 40;                                     // /40 clock divisor, 2Mhz
    SSI2_CR0_R &= ~SSI_CR0_SCR_M;                          // clear bit fields for SSI2 Serial Clock Rate, SCR = 0
    SSI2_CR0_R &= ~SSI_CR0_SPO;                            // clear bit fields for SSI2 Serial Clock Polarity, SPO = 0
    SSI2_CR0_R &= ~SSI_CR0_SPH;                            // clear bit fields for SSI2 Serial Clock Phase, SPH = 0
    SSI2_CR0_R &= ~SSI_CR0_FRF_M;                          // clear bit fields for SSI2 Frame Format Select
    SSI2_CR0_R |= SSI_CR0_FRF_MOTO;                        // set frame format to Freescale SPI Frame Format
    SSI2_CR0_R &= ~SSI_CR0_DSS_M;                          // clear bit fields for SSI2 Data Size Select
    SSI2_CR0_R |= SSI_CR0_DSS_8;                           // set SSI data size to 8
    SSI2_CR1_R |= SSI_CR1_SSE;                             // enable SSI operation
}


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
static uint16_t SSI2_read (void) {
    while((SSI2_SR_R & SSI_SR_BSY) == SSI_SR_BSY) {};     // wait until SSI2 not busy/transmit FIFO empty
    SSI2_DR_R = 0x00;                                     // data out, garbage, just for synchronization
    while((SSI2_SR_R & SSI_SR_RNE) == 0) {};              // wait until response
    return SSI2_DR_R;                                     // read data

}

/**
 * SSI2_write
 * ----------
 * @param  data  data to be written.
 */
static void SSI2_write(uint16_t data){
    while ((SSI2_SR_R & SSI_SR_BSY) == SSI_SR_BSY) {};   // wait until SSI2 not busy/transmit FIFO empty
    SSI2_DR_R = data;                                    // write data
    while ((SSI2_SR_R & SSI_SR_RNE) == 0) {};            // wait until response
    uint16_t sync = SSI2_DR_R;                           // read byte of data, just for synchronization
}


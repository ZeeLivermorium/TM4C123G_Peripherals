/*
 * SSI0.C
 * ----------
 * Inspired by examples in ValvanoWareTM4C123 by Dr. Jonathan Valvano
 * as well as his book Embedded Systems: Real-Time Interfacing to Arm Cortex-M Microcontrollers
 * You can find ValvanoWareTM4C123 at http://edx-org-utaustinx.s3.amazonaws.com/UT601x/ValvanoWareTM4C123.zip?dl=1
 * You can find his book at https://www.amazon.com/gp/product/1463590156/ref=oh_aui_detailpage_o05_s00?ie=UTF8&psc=1
 * You can find more of his work at http://users.ece.utexas.edu/~valvano/
 * ----------
 * Zee Livermorium
 * Apr 14, 2018
 */

#include <stdint.h>
#include "SSI.h"
#include "../inc/tm4c123gh6pm.h"

/*
 *  SSI0 A Conncection | SSI1 D Conncection | SSI1 F Conncection | SSI2 B Conncection | SSI3 D Conncection
 *  ------------------ | ------------------ | ------------------ | ------------------ | ------------------
 *  SCK  --------- PA2 | SCK  --------- PD0 | SCK  --------- PF2 | SCK  --------- PB4 | SCK  --------- PD0
 *  SS   --------- PA3 | SS   --------- PD1 | SS   --------- PF3 | SS   --------- PB5 | SS   --------- PD1
 *  MISO --------- PA4 | MISO --------- PD2 | MISO --------- PF0 | MISO --------- PB6 | MISO --------- PD2
 *  MOSI --------- PA5 | MOSI --------- PD3 | MOSI --------- PF1 | MOSI --------- PB7 | MOSI --------- PD3
 */

/****************************************************
 *                                                  *
 *                    Properties                    *
 *                                                  *
 ****************************************************/

static Bit_Order _bit_order;       // LSB or MSB
static uint8_t _DS;                // data size

/****************************************************
 *                                                  *
 *                   Initializer                    *
 *                                                  *
 ****************************************************/

/**
 * SSI0_A_Init
 * ----------
 * @brief initialize SSI0 on Port A with corresponding setting parameters.
 */
void SSI0_Init (
                  SSI_SS_Mode SSI_SS_mode,                 // SS is trigered regularly or by bit banging (GPIO)
                  SSI_MS_Mode SSI_ms_mode,                 // Master/Slave mode
                  uint8_t CPSDVSR,                         // must be an even number from 2 to 254
                  uint16_t SCR,                            // SSI Serial Clock Rate, a value from 0 to 255
                  uint8_t SSI_clk_mode,                    // SSI Clock Mode, from 0 to 3
                  SSI_Frame_Select SSI_FRF,                // SSI Frame format
                  uint8_t SSI_DS,                          // from 4 bit to 16 bit
                  Bit_Order bit_order                      // LSB or MSB
) {
    /*-- SSI0 and Port A Activation --*/
    SYSCTL_RCGCSSI_R |= SYSCTL_RCGCSSI_R0;                 // enable SSI Module 0 clock
    SYSCTL_RCGCGPIO_R |= SYSCTL_RCGCGPIO_R0;               // enable GPIO Port A clock
    while((SYSCTL_PRGPIO_R & SYSCTL_PRGPIO_R0) == 0){};    // allow time for activating
    
    /*-- Port A Set Up --*/
    if (SSI_SS_mode == Regular) {
        GPIO_PORTA_AFSEL_R |= 0x3C;                        // enable alt funct on PA2-5
        GPIO_PORTA_PUR_R |= 0x3C;                          // enable weak pullup on PA2-5
        GPIO_PORTA_PCTL_R &= ((~GPIO_PCTL_PA2_M) &         // clear bit fields for PA2
                              (~GPIO_PCTL_PA3_M) &         // clear bit fields for PA3, PA3 will be used as GPIO
                              (~GPIO_PCTL_PA4_M) &         // clear bit fields for PA4
                              (~GPIO_PCTL_PA5_M));         // clear bit fields for PA5
        GPIO_PORTA_PCTL_R |= (GPIO_PCTL_PA2_SSI0CLK |      // configure PA2 as SSI0CLK
                              GPIO_PCTL_PA3_SSI0FSS |      // configure PA3 as SSI0FSS
                              GPIO_PCTL_PA4_SSI0RX  |      // configure PA4 as SSI0RX
                              GPIO_PCTL_PA5_SSI0TX);       // configure PA5 as SSI0TX
        GPIO_PORTA_AMSEL_R &= ~0x3C;                       // disable analog functionality on PA2-5
        GPIO_PORTA_DEN_R |= 0x3C;                          // enable digital I/O on PA2-5
    } else { // bit banging
        GPIO_PORTA_DIR_R |= 0x08;                          // make PA3 output
        GPIO_PORTA_AFSEL_R |= 0x34;                        // enable alt funct on PA2, 4, 5
        GPIO_PORTA_AFSEL_R &= ~0x08;                       // disable alt funct on PA3
        GPIO_PORTA_PCTL_R &= ((~GPIO_PCTL_PA2_M) &         // clear bit fields for PA2
                              (~GPIO_PCTL_PA3_M) &         // clear bit fields for PA3, PA3 will be used as GPIO
                              (~GPIO_PCTL_PA4_M) &         // clear bit fields for PA4
                              (~GPIO_PCTL_PA5_M));         // clear bit fields for PA5
        GPIO_PORTA_PCTL_R |= (GPIO_PCTL_PA2_SSI0CLK |      // configure PA2 as SSI0CLK
                              GPIO_PCTL_PA4_SSI0RX  |      // configure PA4 as SSI0RX
                              GPIO_PCTL_PA5_SSI0TX);       // configure PA5 as SSI0TX
        GPIO_PORTA_AMSEL_R &= ~0x3C;                       // disable analog functionality on PA2-5
        GPIO_PORTA_DEN_R |= 0x3C;                          // enable digital I/O on PA2-5
    }
    
    /*-- SSI0 Set Up --*/
    SSI0_CR1_R &= ~SSI_CR1_SSE;                            // disable SSI0 operation
    
    if(SSI_ms_mode == Master) SSI0_CR1_R &= ~SSI_CR1_MS;   // configure SSI0 as master mode
    else SSI0_CR1_R |= SSI_CR1_MS;                         // configure SSI0 as slave mode
    
    SSI0_CC_R &= ~SSI_CC_CS_M;
    SSI0_CC_R |= SSI_CC_CS_SYSPLL;
    
    /* BR = SysClk/(CPSDVSR * (1 + SCR)) */
    SSI0_CPSR_R &= ~SSI_CPSR_CPSDVSR_M;                    // clear bit fields for SSI Clock Prescale Divisor
    SSI0_CPSR_R += CPSDVSR;                                // clock divisor
    SSI0_CR0_R &= ~SSI_CR0_SCR_M;                          // clear bit fields for SSI0 Serial Clock Rate
    SSI0_CR0_R += (SCR << 8);                              // configure serial clock rate
    
    /* clock mode set up */
    switch (SSI_clk_mode) {
        case 0:                                            // mode0, SPO = 0, SPH = 0
            SSI0_CR0_R &= ~SSI_CR0_SPO;                    // clear bit fields for SSI0 Serial Clock Polarity, SPO = 0
            SSI0_CR0_R &= ~SSI_CR0_SPH;                    // clear bit fields for SSI0 Serial Clock Phase, SPH = 0
            break;
        case 1:                                            // mode1, SPO = 0, SPH = 1
            SSI0_CR0_R &= ~SSI_CR0_SPO;                    // clear bit fields for SSI0 Serial Clock Polarity, SPO = 0
            SSI0_CR0_R |= SSI_CR0_SPH;                     // clear bit fields for SSI0 Serial Clock Phase, SPH = 1
            break;
        case 2:                                            // mode2, SPO = 1, SPH = 0
            SSI0_CR0_R |= SSI_CR0_SPO;                     // clear bit fields for SSI0 Serial Clock Polarity, SPO = 1
            SSI0_CR0_R &= ~SSI_CR0_SPH;                    // clear bit fields for SSI0 Serial Clock Phase, SPH = 0
            break;
        case 3:                                            // mode3, SPO = 1, SPH = 1
            SSI0_CR0_R |= SSI_CR0_SPO;                     // clear bit fields for SSI0 Serial Clock Polarity, SPO = 1
            SSI0_CR0_R |= SSI_CR0_SPH;                     // clear bit fields for SSI0 Serial Clock Phase, SPH = 1
            break;
    }
    
    /* set frame format */
    SSI0_CR0_R &= ~SSI_CR0_FRF_M;                          // clear bit fields for SSI0 Frame Format Select
    switch (SSI_FRF) {
        case MOTO:
            SSI0_CR0_R |= SSI_CR0_FRF_MOTO;                // Freescale SPI Frame Format
            break;
        case TI:
            SSI0_CR0_R |= SSI_CR0_FRF_TI;                  // Synchronous Serial Frame Format
            break;
        case NMW:
            SSI0_CR0_R |= SSI_CR0_FRF_NMW;                 // MICROWIRE Frame Format
            break;
    }
    SSI0_CR0_R |= SSI_FRF;                                 // set frame format to Freescale SPI Frame Format
    
    
    SSI0_CR0_R &= ~SSI_CR0_DSS_M;                          // clear bit fields for SSI0 Data Size Select
    SSI0_CR0_R += SSI_DS - 1;                              // set SSI data size
    _DS = SSI_DS;
    _bit_order = bit_order;
    
    SSI0_CR1_R |= SSI_CR1_SSE;                             // enable SSI operation
}


/****************************************************
 *                                                  *
 *                   SS Functions                   *
 *                                                  *
 ****************************************************/

static void SS_HIGH (void) {
#if defined SSI0
    GPIO_PORTA_DATA_R |= 0x08;
#elif defined SSI1D
    GPIO_PORTD_DATA_R |= 0x02;
#elif defined SSI1F
    GPIO_PORTF_DATA_R |= 0x08;
#elif defined SSI2
    GPIO_PORTB_DATA_R |= 0x20;
#elif defined SSI3
    GPIO_PORTD_DATA_R |= 0x02;
#endif
}

static void SS_LOW (void) {
#if defined SSI0
    GPIO_PORTA_DATA_R &= ~0x08;
#elif defined SSI1D
    GPIO_PORTD_DATA_R &= ~0x02;
#elif defined SSI1F
    GPIO_PORTF_DATA_R &= ~0x08;
#elif defined SSI2
    GPIO_PORTB_DATA_R &= ~0x20;
#elif defined SSI3
    GPIO_PORTD_DATA_R &= ~0x02;
#endif
    
}

/****************************************************
 *                                                  *
 *                   I/O Functions                  *
 *                                                  *
 ****************************************************/

/**
 * SSI_Read
 * ----------
 * @return date read from PN532.
 */
static uint8_t SSI_Read (void) {
#if defined SSI0
    while((SSI0_SR_R & SSI_SR_BSY) == SSI_SR_BSY) {};     // wait until SSI0 not busy/transmit FIFO empty
    SSI0_DR_R = 0x00;                                     // data out, garbage, just for synchronization
    while((SSI0_SR_R & SSI_SR_RNE) == 0) {};              // wait until response
    return reverseBitOrder(SSI0_DR_R);                    // LSB, reverse bit order after read
#elif defined SSI1
    while((SSI1_SR_R & SSI_SR_BSY) == SSI_SR_BSY) {};     // wait until SSI1 not busy/transmit FIFO empty
    SSI1_DR_R = 0x00;                                     // data out, garbage, just for synchronization
    while((SSI1_SR_R & SSI_SR_RNE) == 0) {};              // wait until response
    return reverseBitOrder(SSI1_DR_R);                    // LSB, reverse bit order after read
#elif defined SSI2
    while((SSI2_SR_R & SSI_SR_BSY) == SSI_SR_BSY) {};     // wait until SSI2 not busy/transmit FIFO empty
    SSI2_DR_R = 0x00;                                     // data out, garbage, just for synchronization
    while((SSI2_SR_R & SSI_SR_RNE) == 0) {};              // wait until response
    return reverseBitOrder(SSI2_DR_R);                    // LSB, reverse bit order after read
#elif defined SSI3
    while((SSI3_SR_R & SSI_SR_BSY) == SSI_SR_BSY) {};     // wait until SSI3 not busy/transmit FIFO empty
    SSI3_DR_R = 0x00;                                     // data out, garbage, just for synchronization
    while((SSI3_SR_R & SSI_SR_RNE) == 0) {};              // wait until response
    return reverseBitOrder(SSI3_DR_R);                    // LSB, reverse bit order after read
#endif
}

/**
 * SSI_write
 * ----------
 * @param  data  data to be written.
 */
static void SSI_write(uint8_t data){
#if defined SSI0
    while ((SSI0_SR_R & SSI_SR_BSY) == SSI_SR_BSY) {};   // wait until SSI1 not busy/transmit FIFO empty
    SSI0_DR_R = reverseBitOrder(data);                   // LSB, write after reverse bit order
    while ((SSI0_SR_R & SSI_SR_RNE) == 0) {};            // wait until response
    uint16_t sync = SSI0_DR_R;                           // read byte of data, just for synchronization
#elif defined SSI1
    while ((SSI1_SR_R & SSI_SR_BSY) == SSI_SR_BSY) {};   // wait until SSI3 not busy/transmit FIFO empty
    SSI1_DR_R = reverseBitOrder(data);                   // LSB, write after reverse bit order
    while ((SSI1_SR_R & SSI_SR_RNE) == 0) {};            // wait until response
    uint16_t sync = SSI1_DR_R;                           // read byte of data, just for synchronization
#elif defined SSI2
    while ((SSI2_SR_R & SSI_SR_BSY) == SSI_SR_BSY) {};   // wait until SSI2 not busy/transmit FIFO empty
    SSI2_DR_R = reverseBitOrder(data);                   // LSB, write after reverse bit order
    while ((SSI2_SR_R & SSI_SR_RNE) == 0) {};            // wait until response
    uint16_t sync = SSI2_DR_R;                           // read byte of data, just for synchronization
#elif defined SSI3
    while ((SSI3_SR_R & SSI_SR_BSY) == SSI_SR_BSY) {};   // wait until SSI3 not busy/transmit FIFO empty
    SSI3_DR_R = reverseBitOrder(data);                   // LSB, write after reverse bit order
    while ((SSI3_SR_R & SSI_SR_RNE) == 0) {};            // wait until response
    uint16_t sync = SSI3_DR_R;                           // read byte of data, just for synchronization
#endif
}

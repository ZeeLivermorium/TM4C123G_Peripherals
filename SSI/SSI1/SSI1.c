/*!
 * SSI1.c
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
#include "SSI1.h"
#include "tm4c123gh6pm.h"


/*
 *  SSI1 D Conncection | SSI1 F Conncection
 *  ------------------ | ------------------
 *  SCK  --------- PD0 | SCK  --------- PF2
 *  SS   --------- PD1 | SS   --------- PF3
 *  MISO --------- PD2 | MISO --------- PF0
 *  MOSI --------- PD3 | MOSI --------- PF1
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

static void SS1_SS_HIGH (void) {
#if defined SSI1D
    GPIO_PORTD_DATA_R |= 0x02;
#elif defined SSI1F
    GPIO_PORTF_DATA_R |= 0x08;
#endif
}

static void SS1_SS_LOW (void) {
#if defined SSI1D
    GPIO_PORTD_DATA_R &= ~0x02;
#elif defined SSI1F
    GPIO_PORTF_DATA_R &= ~0x08;
#endif
}


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
void SSI1_Init() {
#if defined SSI1
    /* SSI1 Activation */
    SYSCTL_RCGCSSI_R |= SYSCTL_RCGCSSI_R1;                 // enable SSI Module 1 clock
    
#if defined SSI1D
    /* Port D Activation */
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
    GPIO_PORTD_PCTL_R |= (GPIO_PCTL_PD0_SSI1CLK |          // configure PD0 as SSI1CLK
                          GPIO_PCTL_PD2_SSI1RX  |          // configure PD2 as SSI1RX
                          GPIO_PCTL_PD3_SSI1TX);           // configure PD3 as SSI1TX
    GPIO_PORTD_AMSEL_R &= ~0x0F;                           // disable analog functionality on PD0-3
    GPIO_PORTD_DEN_R |= 0x0F;                              // enable digital I/O on PD0-3
    
#elif defined SSI1F
    /* Port F Activation */
    SYSCTL_RCGCGPIO_R |= SYSCTL_RCGCGPIO_R5;               // enable GPIO Port F clock
    while ((SYSCTL_PRGPIO_R & SYSCTL_PRGPIO_R5) == 0) {};  // allow time for activating
    
    /* Port F Set Up */
    GPIO_PORTF_LOCK_R = GPIO_LOCK_KEY;                     // unlock GPIO Port F --- this step is only for Port F
    GPIO_PORTF_CR_R = 0x0F;                                // allow changes to PF0-3 --- this step is only for Port F
    GPIO_PORTF_DIR_R |= 0x08;                              // make PF3 output
    GPIO_PORTF_AFSEL_R |= 0x07;                            // enable alt funct on PF0-2
    GPIO_PORTF_AFSEL_R &= ~0x08;                           // disable alt funct on PF3
    GPIO_PORTF_PCTL_R &= ((~GPIO_PCTL_PF0_M) &             // clear bit fields for PF0
                          (~GPIO_PCTL_PF1_M) &             // clear bit fields for PF1
                          (~GPIO_PCTL_PF2_M) &             // clear bit fields for PF2
                          (~GPIO_PCTL_PF3_M));             // clear bit fields for PF3, PF3 will be used as GPIO
    GPIO_PORTF_PCTL_R |= (GPIO_PCTL_PF0_SSI1RX  |          // configure PF0 as SSI1RX
                          GPIO_PCTL_PF1_SSI1TX  |          // configure PF1 as SSI1TX
                          GPIO_PCTL_PF2_SSI1CLK);          // configure PF2 as SSI1CLK
    GPIO_PORTF_AMSEL_R &= ~0x0F;                           // disable analog functionality on PF0-3
    GPIO_PORTF_DEN_R |= 0x0F;                              // enable digital I/O on PF0-3
    
#endif
    
    /* SSI1 Set Up */
    SSI1_CR1_R &= ~SSI_CR1_SSE;                            // disable SSI operation
    SSI1_CR1_R &= ~SSI_CR1_MS;                             // configure SSI1 as master mode
    SSI1_CPSR_R &= SSI_CPSR_CPSDVSR_M;                     // clear bit fields for SSI Clock Prescale Divisor
    SSI1_CPSR_R += 40;                                     // /40 clock divisor, 2Mhz
    SSI1_CR0_R &= ~SSI_CR0_SCR_M;                          // clear bit fields for SSI1 Serial Clock Rate, SCR = 0
    SSI1_CR0_R &= ~SSI_CR0_SPO;                            // clear bit fields for SSI1 Serial Clock Polarity, SPO = 0
    SSI1_CR0_R &= ~SSI_CR0_SPH;                            // clear bit fields for SSI1 Serial Clock Phase, SPH = 0
    SSI1_CR0_R &= ~SSI_CR0_FRF_M;                          // clear bit fields for SSI1 Frame Format Select
    SSI1_CR0_R |= SSI_CR0_FRF_MOTO;                        // set frame format to Freescale SPI Frame Format
    SSI1_CR0_R &= ~SSI_CR0_DSS_M;                          // clear bit fields for SSI1 Data Size Select
    SSI1_CR0_R |= SSI_CR0_DSS_8;                           // set SSI data size to 8
    SSI1_CR1_R |= SSI_CR1_SSE;                             // enable SSI operation
}


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
static uint16_t SSI1_read (void) {
    while((SSI1_SR_R & SSI_SR_BSY) == SSI_SR_BSY) {};     // wait until SSI1 not busy/transmit FIFO empty
    SSI1_DR_R = 0x00;                                     // data out, garbage, just for synchronization
    while((SSI1_SR_R & SSI_SR_RNE) == 0) {};              // wait until response
    return SSI1_DR_R;                                     // read data
}

/**
 * SSI1_write
 * ----------
 * @param  data  data to be written.
 */
static void SSI1_write(uint16_t data){
    while ((SSI1_SR_R & SSI_SR_BSY) == SSI_SR_BSY) {};   // wait until SSI3 not busy/transmit FIFO empty
    SSI1_DR_R = data;                                    // write data
    while ((SSI1_SR_R & SSI_SR_RNE) == 0) {};            // wait until response
    uint16_t sync = SSI1_DR_R;                           // read byte of data, just for synchronization
}


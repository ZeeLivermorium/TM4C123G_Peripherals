/*
  * SSI0.c
  *
  * ----------
  * Inspired by the ST7735 example in ValvanoWare by Dr. Jonathan Valvano
  * as well as his book Embedded Systems: Real-Time Interfacing to Arm Cortex-M Microcontrollers
  * You can find VavanoWare at http://users.ece.utexas.edu/~valvano/arm/downloadmsp432.html
  * You can find his book at https://www.amazon.com/gp/product/1463590156/ref=oh_aui_detailpage_o05_s00?ie=UTF8&psc=1
  * You can find more of his work at http://users.ece.utexas.edu/~valvano/
  * ----------
  * Zee Livermorium
  * Dec 21, 2017
  */

#include <stdint.h>
#include "tm4c123gh6pm.h"   // put tm4c123gh6pm.h in your project folder or change this line

/*
 * Function: void SSI0_Init(void)
 * Discription: Initialize SSI0(PA2-5).
 */
void SSI0_Init(void) {
    /* SSI and Port A Activation */
    SYSCTL_RCGCSSI_R |= SYSCTL_RCGCSSI_R0;                 // enable SSI Module 0 clock
    SYSCTL_RCGCGPIO_R |= SYSCTL_RCGCGPIO_R0;               // enable GPIO Port A clock
    while((SYSCTL_PRGPIO_R & SYSCTL_PRGPIO_R0) == 0){};    // allow time for activating
    
    /* Port A Set Up */
    GPIO_PORTA_AFSEL_R |= 0x2C;                            // enable alt funct on PA2-5
    GPIO_PORTA_PCTL_R &= ((~GPIO_PCTL_PA2_M) &             // clear bit fields for PA2
                          (~GPIO_PCTL_PA3_M) &             // clear bit fields for PA3
                          (~GPIO_PCTL_PA4_M) &             // clear bit fields for PA4
                          (~GPIO_PCTL_PA5_M));             // clear bit fields for PA5
    GPIO_PORTA_PCTL_R |= (GPIO_PCTL_PA2_SSI0CLK |          // configure PA2 as SSI0CLK
                          GPIO_PCTL_PA3_SSI0FSS |          // configure PA3 as SSI0FSS
                          GPIO_PCTL_PA4_SSI0RX |           // configure PA4 as SSI0RX
                          GPIO_PCTL_PA5_SSI0TX);           // configure PA5 as SSI0TX
    GPIO_PORTA_AMSEL_R &= ~0x2C;                           // disable analog functionality on PA2-5
    GPIO_PORTA_DEN_R |= 0x2C;                              // enable digital I/O on PA2-5
    
    /* SSI0 Set Up */
    SSI0_CR1_R &= ~SSI_CR1_SSE;                            // disable SSI operation
    SSI0_CR1_R &= ~SSI_CR1_MS;                             // configure SSI0 as master mode
    SSI0_CPSR_R &= SSI_CPSR_CPSDVSR_M;                     // clear bit fields for SSI Clock Prescale Divisor
    SSI0_CPSR_R += 2;                                      // /2 clock divisor, must be even number in [0, 254]
    SSI0_CR0_R &= ~SSI_CR0_SCR_M;                          // clear bit fields for SSI0 Serial Clock Rate, SCR = 0
    SSI0_CR0_R &= ~SSI_CR0_SPH;                            // clear bit fields for SSI0 Serial Clock Phase, SPH = 0
    SSI0_CR0_R &= ~SSI_CR0_SPO;                            // clear bit fields for SSI Serial Clock Polarity, SPO = 0
    SSI0_CR0_R &= ~SSI_CR0_FRF_M;                          // clear bit fields for SSI0 Frame Format Select
    SSI0_CR0_R |= SSI_CR0_FRF_MOTO;                        // set frame format to Freescale SPI Frame Format
    SSI0_CR0_R &= ~SSI_CR0_DSS_M;                          // clear bit fields for SSI0 Data Size Select
    SSI0_CR0_R |= SSI_CR0_DSS_16;                          // set SSI data size to 16
    SSI0_CR1_R |= SSI_CR1_SSE;                             // enable SSI operation
}


uint16_t SSI0_Read(void) {
    while((SSI0_SR_R&SSI_SR_TNF) == 0) {};                 // wait until transmit FIFO not full
    return SSI0_DR_R;                                      // read and return
}

void SSI0_Write(uint16_t data) {
    while((SSI0_SR_R&SSI_SR_TNF) == 0) {};                 // wait until transmit FIFO not full
    SSI0_DR_R = data;                                      // write data
}


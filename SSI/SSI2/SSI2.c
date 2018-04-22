/*
  * SSI2.c
  *
  * ----------
  * Inspired by the ST7735 example in ValvanoWare by Dr. Jonathan Valvano
  * as well as his book Embedded Systems: Real-Time Interfacing to Arm Cortex-M Microcontrollers
  * You can find VavanoWare at http://users.ece.utexas.edu/~valvano/arm/downloadmsp432.html
  * You can find his book at https://www.amazon.com/gp/product/1463590156/ref=oh_aui_detailpage_o05_s00?ie=UTF8&psc=1
  * You can find more of his work at http://users.ece.utexas.edu/~valvano/
  * ----------
  * Zee Livermorium
  * Apr 14, 2018
  */

#include <stdint.h>
#include "tm4c123gh6pm.h"   // put tm4c123gh6pm.h in your project folder or change this line

/*
 * Function: void SSI2_Init(void)
 * Discription: Initialize SSI2(PB4-7).
 */
void SSI2_Init (uint8_t bit_banging,
                uint8_t clock_divisor,
                uint8_t SCR,
                uint8_t SPH,
                uint8_t SPO,
                uint8_t frame_formatt,
                uint8_t data_size,
                uint8_t bit_order
                ) {
    /* SSI and Port A Activation */
    SYSCTL_RCGCSSI_R |= SYSCTL_RCGCSSI_R2;                 // enable SSI Module 0 clock
    SYSCTL_RCGCGPIO_R |= SYSCTL_RCGCGPIO_R1;               // enable GPIO Port A clock
    while((SYSCTL_PRGPIO_R & SYSCTL_PRGPIO_R1) == 0){};    // allow time for activating
    
    /* Port A Set Up */
    GPIO_PORTB_AFSEL_R |= 0xF0;                            // enable alt funct on PB2-5
    GPIO_PORTB_PCTL_R &= ((~GPIO_PCTL_PB4_M) &             // clear bit fields for PB2
                          (~GPIO_PCTL_PB5_M) &             // clear bit fields for PB3
                          (~GPIO_PCTL_PB6_M) &             // clear bit fields for PB4
                          (~GPIO_PCTL_PB7_M));             // clear bit fields for PB5
    GPIO_PORTB_PCTL_R |= (GPIO_PCTL_PB4_SSI2CLK |          // configure PB2 as SSI2CLK
                          GPIO_PCTL_PB5_SSI2FSS |          // configure PB3 as SSI2FSS
                          GPIO_PCTL_PB6_SSI2RX |           // configure PB4 as SSI2RX
                          GPIO_PCTL_PB7_SSI2TX);           // configure PB5 as SSI2TX
    GPIO_PORTB_AMSEL_R &= ~0xF0;                           // disable analog functionality on PB2-5
    GPIO_PORTB_DEN_R |= 0xF0;                              // enable digital I/O on PB2-5
    
    /* SSI2 Set Up */
    SSI2_CR1_R &= ~SSI_CR1_SSE;                            // disable SSI operation
    SSI2_CR1_R &= ~SSI_CR1_MS;                             // configure SSI2 as master mode
    SSI2_CPSR_R &= SSI_CPSR_CPSDVSR_M;                     // clear bit fields for SSI Clock Prescale Divisor
    SSI2_CPSR_R += 2;                                      // /2 clock divisor, must be even number in [0, 254]
    SSI2_CR0_R &= ~SSI_CR0_SCR_M;                          // clear bit fields for SSI2 Serial Clock Rate, SCR = 0
    SSI2_CR0_R &= ~SSI_CR0_SPH;                            // clear bit fields for SSI2 Serial Clock Phase, SPH = 0
    SSI2_CR0_R &= ~SSI_CR0_SPO;                            // clear bit fields for SSI Serial Clock Polarity, SPO = 0
    SSI2_CR0_R &= ~SSI_CR0_FRF_M;                          // clear bit fields for SSI2 Frame Format Select
    SSI2_CR0_R |= SSI_CR0_FRF_MOTO;                        // set frame format to Freescale SPI Frame Format
    SSI2_CR0_R &= ~SSI_CR0_DSS_M;                          // clear bit fields for SSI2 Data Size Select
    SSI2_CR0_R |= SSI_CR0_DSS_16;                          // set SSI data size to 16
    SSI2_CR1_R |= SSI_CR1_SSE;                             // enable SSI operation
}


uint16_t SSI2_Read(void) {
    while((SSI2_SR_R&SSI_SR_TNF) == 0) {};                 // wait until transmit FIFO not full
    return SSI2_DR_R;                                      // read and return
}

void SSI2_Write(uint16_t data) {
    while((SSI2_SR_R&SSI_SR_TNF) == 0) {};                 // wait until transmit FIFO not full
    SSI2_DR_R = data;                                      // write data
}


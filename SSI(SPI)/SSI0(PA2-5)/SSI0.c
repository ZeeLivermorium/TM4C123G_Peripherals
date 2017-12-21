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
    /* */
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
    SSI0_CR1_R = 0x0;                                      //
    SSI0_CPSR_R = 0x02;                                    //
    SSI0_CR0_R &= ~(0x0000FFF0);                           //
    SSI0_CR0_R |= 0x0F;                                    //
    SSI0_DR_R = initialData;                               //
    SSI0_CR1_R |= 0x02;                                    //
}



/*
 * M0PWM2.c
 * Use M0PWM2/PB4 to generate pulse-width modulated outputs.
 * ----------
 * Inspired by the PWM example in ValvanoWare by Dr. Jonathan Valvano
 * as well as his book Embedded Systems: Real-Time Interfacing to Arm Cortex-M Microcontrollers
 * You can find VavanoWare at http://users.ece.utexas.edu/~valvano/arm/downloadmsp432.html
 * You can find his book at https://www.amazon.com/gp/product/1463590156/ref=oh_aui_detailpage_o05_s00?ie=UTF8&psc=1
 * You can find more of his work at http://users.ece.utexas.edu/~valvano/
 * ----------
 * Zee Livermorium
 * Dec 20, 2017
 */

#include <stdint.h>
#include "tm4c123gh6pm.h"   // put tm4c123gh6pm.h in your project folder or change this line

/*
 * Function: void M0PWM2_Init(uint16_t period, uint16_t duty)
 * Parameters:
 *   - period: period of the PWM signal
 *   - duty: duty cycle of the PWM signal
 * Discription: Initialize M0PWM2/PB4.
 */
void M0PWM2_Init(uint16_t period, uint16_t duty){
    /* PWM Module and Port Set Up */
    SYSCTL_RCGCPWM_R |= SYSCTL_RCGCPWM_R0;                 // activate PWM Module 0
    SYSCTL_RCGCGPIO_R |= SYSCTL_RCGCGPIO_R1;               // activate Port B
    while((SYSCTL_PRGPIO_R & SYSCTL_RCGCGPIO_R1) == 0){};  // allow time to finish activating
    
    /* Port B Set Up */
    GPIO_PORTB_AFSEL_R |= 0x10;                            // enable alt funct on PB4
    GPIO_PORTB_PCTL_R &= ~GPIO_PCTL_PB4_M;                 // clear bit fields for PB4
    GPIO_PORTB_PCTL_R |= GPIO_PCTL_PB4_M0PWM2;             // configure PB4 as M0PWM2
    GPIO_PORTB_AMSEL_R &= ~0x10;                           // disable analog functionality on PB4
    GPIO_PORTB_DEN_R |= 0x10;                              // enable digital I/O on PB4
    
    /* System Control Run-Mode Clock Configuration (RCC) Set Up */
    SYSCTL_RCC_R |= SYSCTL_RCC_USEPWMDIV;                  // use PWM divider
    SYSCTL_RCC_R &= ~SYSCTL_RCC_PWMDIV_M;                  // clear PWM divider field
    SYSCTL_RCC_R += SYSCTL_RCC_PWMDIV_2;                   // configure for /2 divider
    
    /* M0PWM2 Set Up */
    PWM0_1_CTL_R &= ~PWM_1_CTL_MODE;                       // re-loading down-counting mode
    PWM0_1_GENA_R |= PWM_1_GENA_ACTLOAD_ZERO;              // PB4 goes low on LOAD
    PWM0_1_GENA_R |= PWM_1_GENA_ACTCMPAD_ONE;              // PB4 goes high on CMPA down
    PWM0_1_LOAD_R = period - 1;                            // cycles needed to count down to 0
    PWM0_1_CMPA_R = duty - 1;                              // count value when output rises
    PWM0_1_CTL_R |= PWM_1_CTL_ENABLE;                      // enable M0PWM2
    PWM0_ENABLE_R |= PWM_ENABLE_PWM2EN;                    // enable M0PWM2 output to PB4
}

/*
 * Function: void M0PWM2_Set_Duty(uint16_t duty)
 * Parameters:
 *   - duty: new duty cycle to be set.
 * Discription: Set duty cycle of the PWM signal to a new value.
 */
void M0PWM2_Set_Duty(uint16_t duty){
    PWM0_1_CMPA_R = duty - 1;                              // count value when output rises
}


/*
  * SSI0.h
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

/*
 * Function: void SSI0_Init(void)
 * Discription: Initialize SSI0(PA2-5).
 */
void SSI0_Init(void);

uint16_t SSI0_Read(void);

void SSI0_Write(uint16_t data);

/*
 * SSI0.h
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

/**
 * SSI0_A_Init
 * ----------
 * @brief initialize SSI0 on Port A with corresponding setting parameters.
 */
void SSI0_Init (
                  SSI_SS_Mode SSI_SS_mode,      // SS is trigered regularly or by bit banging (GPIO)
                  SSI_MS_Mode SSI_ms_mode,      // Master/Slave mode
                  uint8_t CPSDVSR,              // must be an even number from 2 to 254
                  uint16_t SCR,                 // SSI Serial Clock Rate, a value from 0 to 255
                  uint8_t SSI_clk_mode,         // SSI Clock Mode, from 0 to 3
                  SSI_Frame_Select SSI_FRF,     // SSI Frame format
                  uint8_t SSI_DS,               // from 4 bit to 16 bit
                  Bit_Order bit_order           // LSB or MSB
);

// for bit banging
void SS_HIGH (void);
void SS_LOW (void);

/**
 * SSI0_Read
 * ----------
 * @return: date read from another device.
 */
uint16_t SSI0_read (void);

/**
 * SSI0_write
 * ----------
 * @param  data  byte of data to be written.
 */
void SSI0_write(uint16_t data);
    

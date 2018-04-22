





/**
 * SSI1_D_Init
 * ----------
 * @brief initialize SSI1 on Port F with corresponding setting parameters.
 */
void SSI1_D_Init (
                  SSI_SS_Mode SSI_SS_mode,      // SS is trigered regularly or by bit banging (GPIO)
                  SSI_MS_Mode SSI_ms_mode,      // Master/Slave mode
                  uint8_t CPSDVSR,              // must be an even number from 2 to 254
                  uint16_t SCR,                 // SSI Serial Clock Rate, a value from 0 to 255
                  uint8_t SSI_clk_mode,         // SSI Clock Mode, from 0 to 3
                  SSI_Frame_Select SSI_FRF,     // SSI Frame format
                  uint8_t SSI_DS,               // from 4 bit to 16 bit
                  Bit_Order bit_order           // LSB or MSB
);

/**
 * SSI1_read
 * ----------
 * @return: date read from another device.
 */
static uint16_t SSI1_read (void);

/**
 * SSI1_write
 * ----------
 * @param  data  byte of data to be written.
 */
static void SSI1_write(uint16_t data);

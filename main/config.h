#ifndef __CONFIG_H__
#define __CONFIG_H__

#define AMP_DAC_TOTAL_NUM            3

// I2C parameters
#define I2C_MS_TO_WAIT              50
#define I2C_MASTER_FREQ_HZ          200000

// I2C addresses
#define AMP_I2C_ADDR_MCP            0x20
#define AMP_I2C_ADDR_RECEIVER       0x40

// IR receiver
#define AMP_PIN_IR_RECEIVER         15

// MCP23017 pins
#define AMP_PIN_MCP_PORTA           4
#define AMP_PIN_MCP_PORTB           5
#define AMP_PIN_MCP_RESET           6

// Leds
#define AMP_PIN_POWER_LED           16
#define AMP_PIN_STANDBY_LED         28
#define AMP_PIN_LED_BACKLIGHT       14

// Buttons
#define AMP_PIN_POWER_BUTTON        17
#define AMP_PIN_INPUT_BUTTON        48
#define AMP_PIN_VOLUME_BUTTON       13

// Encoders
#define AMP_PIN_INPUT_CH1           21
#define AMP_PIN_INPUT_CH2           47

#define AMP_PIN_VOLUME_CH1          11
#define AMP_PIN_VOLUME_CH2          12

// Resets
#define AMP_PIN_HARD_RESET          7

// SPI
#define AMP_PIN_SPI_MOSI            3
#define AMP_PIN_SPI_MISO            8
#define AMP_PIN_SPI_CLK             10
#define AMP_PIN_SPI_CS_EXT          38

// SD Card
#define AMP_PIN_SD_CS               9
#define AMP_PIN_SD_CARD_DETECT      18

// I2S
#define AMP_PIN_I2S_MCLK            41
#define AMP_PIN_I2S_BCLK            42
#define AMP_PIN_I2S_LRCLK           2
#define AMP_PIN_I2S_DATA            1

// UART
#define AMP_PIN_UART_TX             36
#define AMP_PIN_UART_RX             37

#endif 
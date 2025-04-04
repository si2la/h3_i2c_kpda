#ifndef H3_OPI_ONE_H
#define H3_OPI_ONE_H

#include "h3_gpio.h"

#define H3_BOARD_NAME		"Orange Pi One"
#define H3_BOARD_STATUS_LED	H3_PORT_TO_GPIO(H3_GPIO_PORTA, 15)	///< PA15

typedef enum H3_BOARD_OPI_ONE {
    GPIO_EXT_3 = H3_PORT_TO_GPIO(H3_GPIO_PORTA, 12),	///< I2C0 SCA, PA12
    GPIO_EXT_5 = H3_PORT_TO_GPIO(H3_GPIO_PORTA, 11),	///< I2C0 SDL, PA11
    GPIO_EXT_7 = H3_PORT_TO_GPIO(H3_GPIO_PORTA, 6),		///< PA6
    GPIO_EXT_11 = H3_PORT_TO_GPIO(H3_GPIO_PORTA, 1),	///< UART2 RX, PA1
    GPIO_EXT_13 = H3_PORT_TO_GPIO(H3_GPIO_PORTA, 0),	///< UART2 TX, PA0
    GPIO_EXT_15 = H3_PORT_TO_GPIO(H3_GPIO_PORTA, 3),	///< PA3
                                                        ///< 3V3
    GPIO_EXT_19 = H3_PORT_TO_GPIO(H3_GPIO_PORTC, 0),	///< SPI0 MOSI, PC0
    GPIO_EXT_21 = H3_PORT_TO_GPIO(H3_GPIO_PORTC, 1),	///< SPI0 MISO, PC1
    GPIO_EXT_23 = H3_PORT_TO_GPIO(H3_GPIO_PORTC, 2),	///< SPI0 SCLK, PC2
                                                        ///< GND, PIN25
    GPIO_EXT_27 = H3_PORT_TO_GPIO(H3_GPIO_PORTA, 19),	///< I2C1 SCA, PA19
    GPIO_EXT_29 = H3_PORT_TO_GPIO(H3_GPIO_PORTA, 7),	///< PA7
    GPIO_EXT_31 = H3_PORT_TO_GPIO(H3_GPIO_PORTA, 8),	///< PA8
    GPIO_EXT_33 = H3_PORT_TO_GPIO(H3_GPIO_PORTA, 9),	///< PA9
    GPIO_EXT_35 = H3_PORT_TO_GPIO(H3_GPIO_PORTA, 10),	///< PA10
    GPIO_EXT_37 = H3_PORT_TO_GPIO(H3_GPIO_PORTA, 20),	///< PA20
                                                        ///< GND, PIN39
    //
    GPIO_EXT_8 = H3_PORT_TO_GPIO(H3_GPIO_PORTA, 13),	///< UART3 TX, PA13
    GPIO_EXT_10 = H3_PORT_TO_GPIO(H3_GPIO_PORTA, 14),	///< UART3 RX, PA14
    GPIO_EXT_12 = H3_PORT_TO_GPIO(H3_GPIO_PORTD, 14),	///< PD14
                                                        ///< GND, PIN14
    GPIO_EXT_16 = H3_PORT_TO_GPIO(H3_GPIO_PORTC, 4),	///< PC4
    GPIO_EXT_18 = H3_PORT_TO_GPIO(H3_GPIO_PORTC, 7),	///< PC7
    GPIO_EXT_22 = H3_PORT_TO_GPIO(H3_GPIO_PORTA, 2),	///< PA2
    GPIO_EXT_24 = H3_PORT_TO_GPIO(H3_GPIO_PORTC, 3),	///< SPI0 SPICS0, PC3
    GPIO_EXT_26 = H3_PORT_TO_GPIO(H3_GPIO_PORTA, 21),	///< PA21
    //
    GPIO_EXT_28 = H3_PORT_TO_GPIO(H3_GPIO_PORTA, 18),	///< I2C1 SDL, PA18
                                                        ///< GND, PIN30
    GPIO_EXT_32 = H3_PORT_TO_GPIO(H3_GPIO_PORTG, 8),	///< PG8
                                                        ///< GND, PIN34
    GPIO_EXT_36 = H3_PORT_TO_GPIO(H3_GPIO_PORTG, 9),	///< PG9
    GPIO_EXT_38 = H3_PORT_TO_GPIO(H3_GPIO_PORTG, 6),	///< UART1 TX, PG6
    GPIO_EXT_40 = H3_PORT_TO_GPIO(H3_GPIO_PORTG, 7)		///< UART1 RX, PG7
} _gpio_pin;

#endif // H3_OPI_ONE_H

#ifndef SPI_PINS_H_
#define SPI_PINS_H_
/**
 *    How is the ext-flash hardware connected to your board?
 *    Enter pin definitions here.
 */
#include "ioc.h"

// #define BOARD_IOID_FLASH_CS       IOID_14
// #define BOARD_FLASH_CS            (1 << BOARD_IOID_FLASH_CS)

// #define BOARD_IOID_SPI_CLK_FLASH  IOID_10
// #define BOARD_IOID_SPI_MOSI       IOID_9
// #define BOARD_IOID_SPI_MISO       IOID_8



#define BOARD_IOID_FLASH_CS       IOID_4
#define BOARD_FLASH_CS            (1 << BOARD_IOID_FLASH_CS)

#define BOARD_IOID_SPI_CLK_FLASH  IOID_25
#define BOARD_IOID_SPI_MOSI       IOID_24
#define BOARD_IOID_SPI_MISO       IOID_5

#endif


//
// Created by emiel on 20-11-2023.
//

#ifndef ATMELSAMD21_BOARD_DEFINITIONS_H
#define ATMELSAMD21_BOARD_DEFINITIONS_H
#include "sam.h"
#include "hal_gpio.h"
#include "hal_spi_host.h"
#include "hal_i2c_host.h"

/* Main core clock 48 mHz*/
#define MAIN_CLOCK_SPEED 48000000
/* I2C bus speed of 100 KHz */
#define I2C_CLOCK_SPEED 100000
/* Use SERCOM3 as peripheral */
#define I2C_PERIPHERAL I2C_PERIPHERAL_3
/* Use default clock sources, e.g. arduino framework clock system */
#define I2C_CLOCK_SOURCE I2C_CLK_SOURCE_USE_DEFAULT
/* Use the default configuration options */
#define I2C_EXTRA_CONFIG_OPTIONS I2C_EXTRA_OPT_NONE

#define IO_MUX_ADDR 0x20

const gpio_pin_t LedSNPin = GPIO_PIN_PA2;
const gpio_pin_t LedHBPin = GPIO_PIN_PA15;
const gpio_pin_t LedTXPin = GPIO_PIN_PA27;
const gpio_pin_t LedRXPin = GPIO_PIN_PB3;

const gpio_pin_t btn1 = GPIO_PIN_PB31;
const gpio_pin_t btn2 = GPIO_PIN_PB0;
const gpio_pin_t btn3 = GPIO_PIN_PB1;

const gpio_pin_t statNeoPixel  = GPIO_PIN_PA6;
const gpio_pin_t NeoPixelZone1 = GPIO_PIN_PB16;
const gpio_pin_t NeoPixelZone2 = GPIO_PIN_PA19;
const gpio_pin_t NeoPixelZone3 = GPIO_PIN_PB17;
const gpio_pin_t NeoPixelZone4 = GPIO_PIN_PA20;

const gpio_pin_t IO_MUX_SDA = GPIO_PIN_PA16;
const gpio_pin_t IO_MUX_SDO = GPIO_PIN_PA17;

const gpio_pin_t MAIN_SDA  = GPIO_PIN_PA22;
const gpio_pin_t MAIN_SDO  = GPIO_PIN_PA23;
const gpio_pin_t MAIN_INT4 = GPIO_PIN_PA4;
const gpio_pin_t MAIN_INT5 = GPIO_PIN_PA5;

const gpio_pin_t AUX_SDA = GPIO_PIN_PA12;
const gpio_pin_t AUX_SDO = GPIO_PIN_PA13;

const gpio_pin_t LED_SIN   = GPIO_PIN_PB10;
const gpio_pin_t LED_CLK   = GPIO_PIN_PB11;
const gpio_pin_t LED_LAT   = GPIO_PIN_PB12;
const gpio_pin_t LEDG_CLK  = GPIO_PIN_PB13;
const gpio_pin_t LED_BLANK = GPIO_PIN_PB14;

const gpio_pin_t FLASH_CLK  = GPIO_PIN_PA9;
const gpio_pin_t FLASH_MISO = GPIO_PIN_PA14;
const gpio_pin_t FLASH_MOSI = GPIO_PIN_PA8;
const gpio_pin_t FLASH_CS   = GPIO_PIN_PA7;

const gpio_pin_t USB_DP = GPIO_PIN_PA24;
const gpio_pin_t USB_DM = GPIO_PIN_PA25;

const gpio_pin_t GCLK0Pin = GPIO_PIN_PA14;

void init_pins() {
    gpio_set_pin_mode(statNeoPixel, GPIO_MODE_OUTPUT);
    gpio_set_pin_mode(LedHBPin, GPIO_MODE_OUTPUT);

    gpio_set_pin_mode(IO_MUX_SDA, GPIO_MODE_D);
    gpio_set_pin_mode(IO_MUX_SDO, GPIO_MODE_C);

    gpio_set_pin_mode(LedSNPin, GPIO_MODE_OUTPUT);
    gpio_set_pin_mode(LedTXPin, GPIO_MODE_OUTPUT);
    gpio_set_pin_mode(LedRXPin, GPIO_MODE_OUTPUT);

    gpio_set_pin_mode(btn1, GPIO_MODE_INPUT);
    gpio_set_pin_mode(btn2, GPIO_MODE_INPUT);
    gpio_set_pin_mode(btn3, GPIO_MODE_INPUT);

    gpio_set_pin_options(btn1, GPIO_OPT_PULL_UP);
    gpio_set_pin_options(btn2, GPIO_OPT_PULL_UP);
    gpio_set_pin_options(btn3, GPIO_OPT_PULL_UP);

#ifdef I2C_ENABLE
    /* I2C Pins */
    GPIO_SET_PIN_MODE(IO_MUX_SDO, GPIO_MODE_C);
    GPIO_SET_PIN_MODE(IO_MUX_SDA, GPIO_MODE_C);
    GPIO_SET_PIN_MODE(AUX_SDO, GPIO_MODE_D);
    GPIO_SET_PIN_MODE(AUX_SDA, GPIO_MODE_D);
    GPIO_SET_PIN_MODE(MAIN_SDO, GPIO_MODE_C);
    GPIO_SET_PIN_MODE(MAIN_SDA, GPIO_MODE_C);
#endif

#ifdef FRAM_ENABLE
    /* SPI Pins */
    GPIO_SET_PIN_MODE(FLASH_MISO, GPIO_MODE_C);
    GPIO_SET_PIN_MODE(FLASH_MOSI, GPIO_MODE_D);
    GPIO_SET_PIN_MODE(FLASH_CLK, GPIO_MODE_D);
    GPIO_SET_PIN_MODE(FLASH_CS, GPIO_MODE_OUTPUT);
#endif
}
#ifdef FRAM_ENABLE
void init_fram() {
    const spi_bus_opt_t extra_config_opt = static_cast<spi_bus_opt_t>(SPI_BUS_OPT_DOPO_PAD_0 | SPI_BUS_OPT_DIPO_PAD_2);

    SPI_HOST_INIT(SPI_PERIPHERAL_2, SPI_CLK_SOURCE_USE_DEFAULT, MAIN_CLOCK_SPEED, 10e6, extra_config_opt);

    fram_init(fram);
}
#endif

#endif //ATMELSAMD21_BOARD_DEFINITIONS_H

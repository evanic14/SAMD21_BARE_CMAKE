#include "sam.h"
#include <hal_gpio.h>
#include <hal_i2c_host.h>
#include <FreeRTOSConfig.h>
#include <FreeRTOS.h>
#include <task.h>
//#include "lib/NeoPixel_lib/Adafruit_NeoPixel.h"
#include <hal_spi_host.h>

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

//const gpio_pin_t GCLK0Pin = GPIO_PIN_PA14;

// Constants for Clock Generators
#define GENERIC_CLOCK_GENERATOR_0   (0u)
#define GENERIC_CLOCK_GENERATOR_1   (1u)

//Constants for clock identifiers
#define CLOCK_XOSC32K	0x05
#define CLOCK_DFLL48	0x07
#define CLOCK_8MHZ		0x06

#define MAIN_CLOCK_SPEED 48000000
#define STACK_SIZE 200

#define I2C_CLOCK_SPEED 100000 /* I2C bus speed of 100 KHz */

/* Use SERCOM3 as peripheral */
#define I2C_PERIPHERAL I2C_PERIPHERAL_3

/* Use default clock sources, e.g. arduino framework clock system */
#define I2C_CLOCK_SOURCE I2C_CLK_SOURCE_USE_DEFAULT

/* Use the default configuration options */
#define I2C_EXTRA_CONFIG_OPTIONS I2C_EXTRA_OPT_NONE

#define IO_MUX_ADDR 0x20

/* Structure that will hold the TCB of the task being created. */
StaticTask_t xTaskBuffer;

/* Buffer that the task being created will use as its stack.  Note this is
an array of StackType_t variables.  The size of StackType_t is dependent on
the RTOS port. */
StackType_t xStack[ STACK_SIZE ];

//function for setting related to power management system
//See section 16 of the datasheet
void PM_Clock_Bus_Setup(void) {
	//in power management system do not divide system clock down
	PM->CPUSEL.reg  = PM_CPUSEL_CPUDIV_DIV1;
	PM->APBASEL.reg = PM_APBASEL_APBADIV_DIV1_Val;
	PM->APBBSEL.reg = PM_APBBSEL_APBBDIV_DIV1_Val;
	PM->APBCSEL.reg = PM_APBCSEL_APBCDIV_DIV1_Val;
}

void Clock_Init(void) {

    //NVM CTRLB registers and RWS[3:0] bit for setting wait states fro a read operation	//Defaults to 0 and can go as high as 15 (4 bits)
    NVMCTRL->CTRLB.bit.RWS = 1;		// 1 wait state required @ 3.3V & 48MHz

    //the system controller subsystem controls the clocks. The XOSC32K register sets up the External 32.768kHz oscillator
    SYSCTRL->OSC32K.bit.WRTLOCK = 0;		//XOSC32K configuration is not locked
    SYSCTRL->OSC32K.bit.STARTUP = 0x2;		//3 cycle start-up time
    SYSCTRL->OSC32K.bit.ONDEMAND = 0;		//Osc. is always running when enabled
    SYSCTRL->OSC32K.bit.RUNSTDBY = 0;		//Osc. is disabled in standby sleep mode
    SYSCTRL->OSC32K.bit.EN32K = 1;			// 32kHz output is enable
    // Enable the Oscillator - Separate step per data sheet recommendation (sec 17.6.3)
    SYSCTRL->OSC32K.bit.ENABLE = 1; //should this be moved after the sync????
    // Wait for XOSC32K to stabilize
    while(!SYSCTRL->PCLKSR.bit.OSC32KRDY);

    //Generic clock subsystem setting GENDIV register to set the divide factor for Generic clock 1
    GCLK->GENDIV.reg |= GCLK_GENDIV_DIV(1) | GCLK_GENDIV_ID(GENERIC_CLOCK_GENERATOR_1);; //set divide factor for gen clock 1

    // Configure Generic Clock Generator 1 with XOSC32K as source
    GCLK->GENCTRL.bit.RUNSTDBY = 0; // Generic Clock Generator is stopped in stdby
    GCLK->GENCTRL.bit.DIVSEL = 0;   // enable clock divide
    GCLK->GENCTRL.bit.OE = 0;		// Disable generator output to GCLK_IO[1]
    GCLK->GENCTRL.bit.OOV = 0;		// We will not use this signal as an output
    GCLK->GENCTRL.bit.IDC = 1;		// Generator duty cycle is 50/50
    GCLK->GENCTRL.bit.GENEN = 1;	// Enable the generator
    GCLK->GENCTRL.bit.SRC = GCLK_GENCTRL_SRC_OSC32K_Val;	// Generator source: XOSC32K output
    GCLK->GENCTRL.bit.ID = GENERIC_CLOCK_GENERATOR_1;	// This was created in Definitions.h, refers to generic clock 1
    // GENCTRL is Writesrc/Clock_stuff.h-Synchronized...so wait for write to complete
    while(GCLK->STATUS.bit.SYNCBUSY);

    //			Enable the Generic Clock     // Generic Clock Generator 1 is the source			 Generic Clock Multiplexer 0 (DFLL48M Reference)
    GCLK->CLKCTRL.reg |= GCLK_CLKCTRL_CLKEN | GCLK_CLKCTRL_GEN(GENERIC_CLOCK_GENERATOR_1) | GCLK_CLKCTRL_ID_DFLL48;

    // DFLL Configuration in Closed Loop mode, cf product data sheet chapter
    // 17.6.7.1 - Closed-Loop Operation
    // Enable the DFLL48M in open loop mode. Without this step, attempts to go into closed loop mode at 48 MHz will
    // result in Processor Reset (you'll be at the in the Reset_Handler in startup_samd21.c).
    // PCLKSR.DFLLRDY must be one before writing to the DFLL Control register
    // Note that the DFLLRDY bit represents status of register synchronization - NOT clock stability
    // (see Data Sheet 17.6.14 Synchronization for detail)
    while(!SYSCTRL->PCLKSR.bit.DFLLRDY);
    SYSCTRL->DFLLCTRL.reg = (uint16_t)(SYSCTRL_DFLLCTRL_ENABLE);
    while(!SYSCTRL->PCLKSR.bit.DFLLRDY);

    // Set up the Multiplier, Coarse and Fine steps. These values help the clock lock at the set frequency
    //There is not much information in the datasheet on what they do exactly and how to tune them for your specific needs
    //lower values lead to more "overshoot" but faster frequency lock. Higher values lead to less "overshoot" but slower lock time
    //Datasheet says put them at half to get best of both worlds
    SYSCTRL->DFLLMUL.bit.CSTEP = 31; //max value is 2^6 - 1 or 63
    SYSCTRL->DFLLMUL.bit.FSTEP = 511; //max value is 2^10 - 1 or 1023
    SYSCTRL->DFLLMUL.bit.MUL = 1465; //multiplier of ref external clock to get to 48M --> 32768 x 1465 = 48,005,120
    // Wait for synchronization
    while(!SYSCTRL->PCLKSR.bit.DFLLRDY);
    // To reduce lock time, load factory calibrated values into DFLLVAL (cf. Data Sheet 17.6.7.1)
    // Location of value is defined in Data Sheet Table 10-5. NVM Software Calibration Area Mapping

    // Switch DFLL48M to Closed Loop mode and enable WAITLOCK
    SYSCTRL->DFLLCTRL.reg |= (uint16_t) (SYSCTRL_DFLLCTRL_MODE | SYSCTRL_DFLLCTRL_WAITLOCK);

    GCLK->GENDIV.reg = GCLK_GENDIV_DIV(1) | GCLK_GENDIV_ID(GENERIC_CLOCK_GENERATOR_0); //set divide factor for gen clock 0
    // Now that DFLL48M is running, switch CLKGEN0 source to it to run the core at 48 MHz.
    // Enable output of Generic Clock Generator 0 (GCLK_MAIN) to the GCLK_IO[0] GPIO Pin
    GCLK->GENCTRL.bit.RUNSTDBY = 0;		// Generic Clock Generator is stopped in stdby
    GCLK->GENCTRL.bit.DIVSEL = 0;		// Use GENDIV.DIV value to divide the generator
    GCLK->GENCTRL.bit.OE = 0;			// Enable generator output to GCLK_IO[0]
    GCLK->GENCTRL.bit.OOV = 0;			// GCLK_IO[0] output value when generator is off
    GCLK->GENCTRL.bit.IDC = 1;			// Generator duty cycle is 50/50
    GCLK->GENCTRL.bit.GENEN = 1;		// Enable the generator
    //The next two lines are where we set the system clock
    GCLK->GENCTRL.bit.SRC = CLOCK_DFLL48;		// Generator source: DFLL48M output
    GCLK->GENCTRL.bit.ID = GENERIC_CLOCK_GENERATOR_0;	// Generic clock gen 0 is used for system clock
    // GENCTRL is Write-Synchronized...so wait for write to complete
    while(GCLK->STATUS.bit.SYNCBUSY);

    //setup the built-in 8MHz clock
    SYSCTRL->OSC8M.bit.PRESC = 0;		// Prescale by 1 (no divide)
    SYSCTRL->OSC8M.bit.ONDEMAND = 0;	// Oscillator is always on if enabled

    PM_Clock_Bus_Setup(); //setup power management system
}

void IO_MUX_READ()
{
    const uint8_t inBuffer[3] = {0xFF, 0xFF, 0xFF};
    int8_t outBuffer[3];

    I2C_HOST_WRITE_BLOCKING(I2C_PERIPHERAL, IO_MUX_ADDR, inBuffer, 3, 0);
    vTaskDelay(10/portTICK_PERIOD_MS);
    I2C_HOST_READ_BLOCKING(I2C_PERIPHERAL, IO_MUX_ADDR, outBuffer, 3);
    vTaskDelay(10/portTICK_PERIOD_MS);
}

/* Function that implements the task being created. */
void vTaskHeartbeat( void * pvParameters )
{
    gpio_set_pin_mode(statNeoPixel, GPIO_MODE_OUTPUT);
    gpio_set_pin_mode(LedHBPin, GPIO_MODE_OUTPUT);

    //const uint8_t buffer[3] = {0xFF, 0xFF, 0xFF};
    for( ;; )
    {
        gpio_toggle_pin_output(LedHBPin);
        vTaskDelay(500/portTICK_PERIOD_MS);
    }
}

void vTaskIOMUX( void * pvParameters )
{
    gpio_set_pin_mode(IO_MUX_SDA, GPIO_MODE_D);
    gpio_set_pin_mode(IO_MUX_SDO, GPIO_MODE_C);

    I2C_HOST_INIT(I2C_PERIPHERAL, I2C_CLOCK_SOURCE, MAIN_CLOCK_SPEED, I2C_CLOCK_SPEED, I2C_EXTRA_CONFIG_OPTIONS);
    for( ;; )
    {
        IO_MUX_READ();
    }

}


void vTaskButtonRead( void * pvParameters )
{
    gpio_set_pin_mode(LedSNPin, GPIO_MODE_OUTPUT);
    gpio_set_pin_mode(LedTXPin, GPIO_MODE_OUTPUT);
    gpio_set_pin_mode(LedRXPin, GPIO_MODE_OUTPUT);

    gpio_set_pin_mode(btn1, GPIO_MODE_INPUT);
    gpio_set_pin_mode(btn2, GPIO_MODE_INPUT);
    gpio_set_pin_mode(btn3, GPIO_MODE_INPUT);

    gpio_set_pin_options(btn1, GPIO_OPT_PULL_UP);
    gpio_set_pin_options(btn2, GPIO_OPT_PULL_UP);
    gpio_set_pin_options(btn3, GPIO_OPT_PULL_UP);

    for( ;; )
    {
        if (gpio_get_pin_lvl(btn1) == GPIO_LOW)
        {
            gpio_toggle_pin_output(LedSNPin);
        }
        else if (gpio_get_pin_lvl(btn2) == GPIO_LOW)
        {
            gpio_toggle_pin_output(LedTXPin);
        }
        else if (gpio_get_pin_lvl(btn3) == GPIO_LOW)
        {
            gpio_toggle_pin_output(LedRXPin);
        }
        else
        {
            gpio_set_pin_lvl(LedSNPin, GPIO_LOW);
            gpio_set_pin_lvl(LedTXPin, GPIO_LOW);
            gpio_set_pin_lvl(LedRXPin, GPIO_LOW);
        }
    }
}

int main(void)
{
    /*
     * Set the main clock to 48MHz
     */
	Clock_Init();

    /*
     * Call the crossplatform hal to set the pin output dir
     */
//    xTaskCreateStatic(
//            vTaskBlink,       /* Function that implements the task. */
//            "Blink",          /* Text name for the task. */
//            STACK_SIZE,      /* Number of indexes in the xStack array. */
//            ( void * ) 1,    /* Parameter passed into the task. */
//            tskIDLE_PRIORITY,/* Priority at which the task is created. */
//            xStack,          /* Array to use as the task's stack. */
//            &xTaskBuffer      /* Variable to hold the task's data structure. */
//    );

    xTaskCreate(
            vTaskHeartbeat,       /* Function that implements the task. */
            "Heartbeat",          /* Text name for the task. */
            STACK_SIZE,      /* Number of indexes in the xStack array. */
            ( void * ) 1,    /* Parameter passed into the task. */
            tskIDLE_PRIORITY,/* Priority at which the task is created. */
            xStack          /* Array to use as the task's stack. */
    );

    xTaskCreate(
            vTaskIOMUX,       /* Function that implements the task. */
            "Heartbeat",          /* Text name for the task. */
            STACK_SIZE,      /* Number of indexes in the xStack array. */
            ( void * ) 1,    /* Parameter passed into the task. */
            tskIDLE_PRIORITY,/* Priority at which the task is created. */
            xStack          /* Array to use as the task's stack. */
    );

    xTaskCreate(
            vTaskButtonRead,       /* Function that implements the task. */
            "Button Read",          /* Text name for the task. */
            STACK_SIZE,      /* Number of indexes in the xStack array. */
            ( void * ) 1,    /* Parameter passed into the task. */
            tskIDLE_PRIORITY,/* Priority at which the task is created. */
            xStack          /* Array to use as the task's stack. */
    );
    vTaskStartScheduler();

    while(1)
    {

	}
}

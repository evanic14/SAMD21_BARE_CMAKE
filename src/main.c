#include "sam.h"
#include <hal_gpio.h>
#include <FreeRTOSConfig.h>
#include <FreeRTOS.h>
#include <task.h>
#include <hal_spi_host.h>

const gpio_pin_t LedSNPin = GPIO_PIN_PA2;
const gpio_pin_t LedHBPin = GPIO_PIN_PA15;
const gpio_pin_t LedTXPin = GPIO_PIN_PA27;
const gpio_pin_t LedRXPin = GPIO_PIN_PB3;

const gpio_pin_t NeoPixel = GPIO_PIN_PA6;
const gpio_pin_t GCLK0Pin = GPIO_PIN_PA14;

// Constants for Clock Generators
#define GENERIC_CLOCK_GENERATOR_0   (0u)
#define GENERIC_CLOCK_GENERATOR_1   (1u)

//Constants for clock identifiers
#define CLOCK_XOSC32K	0x05
#define CLOCK_DFLL48	0x07
#define CLOCK_8MHZ		0x06

#define MAIN_CLOCK_SPEED 48000000
#define STACK_SIZE 200

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
    GCLK->GENDIV.reg |= GCLK_GENDIV_DIV(1) | GCLK_GENDIV_ID(3); //set divide factor for gen clock 1

    // Configure Generic Clock Generator 1 with XOSC32K as source
    GCLK->GENCTRL.bit.RUNSTDBY = 0; // Generic Clock Generator is stopped in stdby
    GCLK->GENCTRL.bit.DIVSEL = 0;   // enable clock divide
    GCLK->GENCTRL.bit.OE = 0;		// Disable generator output to GCLK_IO[1]
    GCLK->GENCTRL.bit.OOV = 0;		// We will not use this signal as an output
    GCLK->GENCTRL.bit.IDC = 1;		// Generator duty cycle is 50/50
    GCLK->GENCTRL.bit.GENEN = 1;	// Enable the generator
    GCLK->GENCTRL.bit.SRC = GCLK_GENCTRL_SRC_OSC32K_Val;	// Generator source: XOSC32K output
    GCLK->GENCTRL.bit.ID = 3;	// This was created in Definitions.h, refers to generic clock 1
    // GENCTRL is Writesrc/Clock_stuff.h-Synchronized...so wait for write to complete
    while(GCLK->STATUS.bit.SYNCBUSY);

    //			Enable the Generic Clock     // Generic Clock Generator 1 is the source			 Generic Clock Multiplexer 0 (DFLL48M Reference)
    GCLK->CLKCTRL.reg |= GCLK_CLKCTRL_CLKEN | GCLK_CLKCTRL_GEN(3) | GCLK_CLKCTRL_ID_DFLL48;

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

    GCLK->GENDIV.reg = GCLK_GENDIV_DIV(1) | GCLK_GENDIV_ID(0); //set divide factor for gen clock 0
    // Now that DFLL48M is running, switch CLKGEN0 source to it to run the core at 48 MHz.
    // Enable output of Generic Clock Generator 0 (GCLK_MAIN) to the GCLK_IO[0] GPIO Pin
    GCLK->GENCTRL.bit.RUNSTDBY = 0;		// Generic Clock Generator is stopped in stdby
    GCLK->GENCTRL.bit.DIVSEL = 0;		// Use GENDIV.DIV value to divide the generator
    GCLK->GENCTRL.bit.OE = 0;			// Enable generator output to GCLK_IO[0]
    GCLK->GENCTRL.bit.OOV = 0;			// GCLK_IO[0] output value when generator is off
    GCLK->GENCTRL.bit.IDC = 1;			// Generator duty cycle is 50/50
    GCLK->GENCTRL.bit.GENEN = 1;		// Enable the generator
    //The next two lines are where we set the system clock
    GCLK->GENCTRL.bit.SRC = 0x07;		// Generator source: DFLL48M output
    GCLK->GENCTRL.bit.ID = 0;	// Generic clock gen 0 is used for system clock
    // GENCTRL is Write-Synchronized...so wait for write to complete
    while(GCLK->STATUS.bit.SYNCBUSY);

    //setup the built-in 8MHz clock
    SYSCTRL->OSC8M.bit.PRESC = 0;		// Prescale by 1 (no divide)
    SYSCTRL->OSC8M.bit.ONDEMAND = 0;	// Oscillator is always on if enabled

    PM_Clock_Bus_Setup(); //setup power management system
}


/* Function that implements the task being created. */
void vTaskCode( void * pvParameters )
{
    vTaskDelay(100/portTICK_PERIOD_MS);
    //gpio_set_pin_mode(NeoPixel, GPIO_MODE_D);
    //gpio_set_pin_mode(GPIO_PIN_PA7, GPIO_MODE_C);
    //gpio_set_pin_mode(GPIO_PIN_PA4, GPIO_MODE_C);
    spi_host_init(SPI_PERIPHERAL_0, SPI_CLK_SOURCE_USE_DEFAULT, 48e6, 1e6, (SPI_BUS_OPT_DOPO_PAD_1 | SPI_BUS_OPT_DIPO_PAD_0));
    // Set the LED_PIN as an output
    gpio_set_pin_mode(LedHBPin, GPIO_MODE_OUTPUT);
    gpio_set_pin_mode(LedSNPin, GPIO_MODE_OUTPUT);
    gpio_set_pin_mode(LedTXPin, GPIO_MODE_OUTPUT);
    gpio_set_pin_mode(LedRXPin, GPIO_MODE_OUTPUT);
    const uint8_t buffer[3] = {0xFF, 0xFF, 0xFF};
    for( ;; )
    {
        spi_host_write_blocking(SPI_PERIPHERAL_0, buffer, 3);
        // Replace the example data with your own RGB values
        gpio_toggle_pin_output(LedHBPin);
        gpio_toggle_pin_output(LedSNPin);
        gpio_toggle_pin_output(LedTXPin);
        gpio_toggle_pin_output(LedRXPin);
        vTaskDelay(100/portTICK_PERIOD_MS);
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
    gpio_set_pin_mode(LedHBPin, GPIO_MODE_OUTPUT);
    gpio_set_pin_mode(LedSNPin, GPIO_MODE_OUTPUT);
    gpio_set_pin_mode(LedTXPin, GPIO_MODE_OUTPUT);
    gpio_set_pin_mode(LedRXPin, GPIO_MODE_OUTPUT);

    xTaskCreateStatic(
            vTaskCode,       /* Function that implements the task. */
            "CBTASK",          /* Text name for the task. */
            STACK_SIZE,      /* Number of indexes in the xStack array. */
            ( void * ) 1,    /* Parameter passed into the task. */
            tskIDLE_PRIORITY,/* Priority at which the task is created. */
            xStack,          /* Array to use as the task's stack. */
            &xTaskBuffer );  /* Variable to hold the task's data structure. */

    vTaskStartScheduler();

	while(1) {
        //gpio_toggle_pin_output(LedHBPin);
        //gpio_toggle_pin_output(LedSNPin);
        //gpio_toggle_pin_output(LedTXPin);
        //gpio_toggle_pin_output(LedRXPin);
        //for(uint32_t i = 0; i< 48e6/6; i++);
	}
}

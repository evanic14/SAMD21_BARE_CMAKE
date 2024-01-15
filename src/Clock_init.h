//
// Created by emiel on 20-11-2023.
//

#ifndef ATMELSAMD21_CLOCK_INIT_H
#define ATMELSAMD21_CLOCK_INIT_H
#include "sam.h"
#include <hal_gpio.h>

// Constants for Clock Generators
#define GENERIC_CLOCK_GENERATOR_0   (0u)
#define GENERIC_CLOCK_GENERATOR_1   (1u)

//Constants for clock identifiers
#define CLOCK_XOSC32K	0x05
#define CLOCK_DFLL48	0x07
#define CLOCK_8MHZ		0x06

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
    GCLK->GENDIV.reg |= GCLK_GENDIV_DIV(1) | GCLK_GENDIV_ID(GENERIC_CLOCK_GENERATOR_1); //set divide factor for gen clock 1

    // Configure Generic Clock Generator 1 with OSC32K as source
    GCLK->GENCTRL.reg = GCLK_GENCTRL_OE | GCLK_GENCTRL_IDC | GCLK_GENCTRL_SRC_OSC32K | GCLK_GENCTRL_GENEN |
                        GCLK_GENCTRL_ID(1);
    // GENCTRL is Writesrc/Clock_stuff.h-Synchronized...so wait for write to complete
    while(GCLK->STATUS.bit.SYNCBUSY);

    //Enable the Generic Clock     // Generic Clock Generator 1 is the source			 Generic Clock Multiplexer 0 (DFLL48M Reference)
    GCLK->CLKCTRL.reg = GCLK_CLKCTRL_CLKEN | GCLK_CLKCTRL_GEN(GENERIC_CLOCK_GENERATOR_1) | GCLK_CLKCTRL_ID_FDPLL;
    while(GCLK->STATUS.bit.SYNCBUSY);
    // DFLL Configuration in Closed Loop mode, cf product data sheet chapter
    // 17.6.7.1 - Closed-Loop Operation
    // Enable the DFLL48M in open loop mode. Without this step, attempts to go into closed loop mode at 48 MHz will
    // result in Processor Reset (you'll be at the in the Reset_Handler in startup_samd21.c).
    // PCLKSR.DFLLRDY must be one before writing to the DFLL Control register
    // Note that the DFLLRDY bit represents status of register synchronization - NOT clock stability
    // (see Data Sheet 17.6.14 Synchronization for detail)

    PM_Clock_Bus_Setup(); //setup power management system
    // Set up the Multiplier, Coarse and Fine steps. These values help the clock lock at the set frequency
    //There is not much information in the datasheet on what they do exactly and how to tune them for your specific needs
    //lower values lead to more "overshoot" but faster frequency lock. Higher values lead to less "overshoot" but slower lock time
    //Datasheet says put them at half to get best of both worlds
    SYSCTRL->DPLLRATIO.reg = SYSCTRL_DPLLRATIO_LDR(1480) | SYSCTRL_DPLLRATIO_LDRFRAC(3);
    SYSCTRL->DPLLCTRLB.reg = SYSCTRL_DPLLCTRLB_REFCLK_GCLK;
    // Wait for synchronization
    SYSCTRL->DPLLCTRLA.reg = SYSCTRL_DPLLCTRLA_ENABLE;
    while(!SYSCTRL->DPLLSTATUS.bit.CLKRDY)
    // Now that DFLL48M is running, switch CLKGEN0 source to it to run the core at 48 MHz.
    // Enable output of Generic Clock Generator 0 (GCLK_MAIN) to the GCLK_IO[0] GPIO Pin
    GCLK->GENCTRL.reg = GCLK_GENCTRL_ID(0) | GCLK_GENCTRL_SRC_DPLL96M | GCLK_GENCTRL_OE | GCLK_GENCTRL_GENEN;
    GCLK->CLKCTRL.reg = GCLK_CLKCTRL_GEN(0) | GCLK_CLKCTRL_CLKEN | GCLK_CLKCTRL_ID_SERCOM0_CORE;
    // GENCTRL is Write-Synchronized...so wait for write to complete
    while(GCLK->STATUS.bit.SYNCBUSY);

    //setup the built-in 8MHz clock
    SYSCTRL->OSC8M.bit.PRESC = 0;		// Prescale by 1 (no divide)
    SYSCTRL->OSC8M.bit.ONDEMAND = 0;	// Oscillator is always on if enabled
    SYSCTRL->OSC8M.bit.ENABLE = 0;
}
#endif //ATMELSAMD21_CLOCK_INIT_H

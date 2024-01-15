#include "Board_definitions.h"
#include "Clock_init.h"
#include <FreeRTOSConfig.h>
#include <FreeRTOS.h>
#include <task.h>

#define STACK_SIZE 200

/* Structure that will hold the TCB of the task being created. */
StaticTask_t xTaskBuffer;

/* Buffer that the task being created will use as its stack.  Note this is
an array of StackType_t variables.  The size of StackType_t is dependent on
the RTOS port. */
StackType_t xStack[STACK_SIZE];

//function for setting related to power management system
//See section 16 of the datasheet


//void IO_MUX_READ() {
//    const uint8_t inBuffer[3] = {0xFF, 0xFF, 0xFF};
//    int8_t outBuffer[3];
//
//    I2C_HOST_WRITE_BLOCKING(I2C_PERIPHERAL, IO_MUX_ADDR, inBuffer, 3, 0);
//    vTaskDelay(10 / portTICK_PERIOD_MS);
//    I2C_HOST_READ_BLOCKING(I2C_PERIPHERAL, IO_MUX_ADDR, outBuffer, 3);
//    vTaskDelay(10 / portTICK_PERIOD_MS);
//}

/* Function that implements the task being created. */
//void vTaskHeartbeat(void *pvParameters) {
//    for (;;) {
//        gpio_toggle_pin_output(LedHBPin);
//        vTaskDelay(500 / portTICK_PERIOD_MS);
//    }
//}

//void vTaskStatLed(void *pvParameters) {
//    //for (;;) {}
//    gpio_set_pin_mode(GPIO_PIN_PA6, GPIO_MODE_OUTPUT);
//    const uint8_t colorVal[24] = {1,0,0,1,0,1,1,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0};
//    for (;;) {
//        for (uint8_t i = 0; i < sizeof(colorVal); i++) {
//            gpio_set_pin_lvl(GPIO_PIN_PA6, colorVal[i]);
//        }
//    }
//        gpio_set_pin_mode(GPIO_PIN_PA6, GPIO_MODE_OUTPUT);
//        __asm("NOP");
//        gpio_set_pin_lvl(GPIO_PIN_PA6, 1);
//        gpio_set_pin_lvl(GPIO_PIN_PA6, 0);
//        for (uint8_t i = 0; i < 8; i++) {
//            gpio_set_pin_lvl(GPIO_PIN_PA6, 1);
//            __asm("NOP");
//            gpio_set_pin_lvl(GPIO_PIN_PA6, 1);
//            __asm("NOP");
//            gpio_set_pin_lvl(GPIO_PIN_PA6, 0);
//            __asm("NOP");
//        }
//        for (uint8_t i = 0; i < 8; i++) {
//            gpio_set_pin_lvl(GPIO_PIN_PA6, 1);
//            __asm("NOP");
//            gpio_set_pin_lvl(GPIO_PIN_PA6, 1);
//            __asm("NOP");
//            gpio_set_pin_lvl(GPIO_PIN_PA6, 0);
//            __asm("NOP");
//        }
//        for (uint8_t i = 0; i < 8; i++) {
//            gpio_set_pin_lvl(GPIO_PIN_PA6, 0);
//            __asm("NOP");
//            gpio_set_pin_lvl(GPIO_PIN_PA6, 0);
//            __asm("NOP");
//            gpio_set_pin_lvl(GPIO_PIN_PA6, 0);
//            __asm("NOP");
//        }
//        for (uint8_t i = 0; i < 8; i++) {
//            gpio_set_pin_lvl(GPIO_PIN_PA6, 0);
//            __asm("NOP");
//            gpio_set_pin_lvl(GPIO_PIN_PA6, 0);
//            __asm("NOP");
//            gpio_set_pin_lvl(GPIO_PIN_PA6, 0);
//            __asm("NOP");
//        }
//    }
//}

//void vTaskIOMUX(void *pvParameters) {
//    I2C_HOST_INIT(I2C_PERIPHERAL, I2C_CLOCK_SOURCE, MAIN_CLOCK_SPEED, I2C_CLOCK_SPEED, I2C_EXTRA_CONFIG_OPTIONS);
//    for (;;) {
//        IO_MUX_READ();
//    }
//}


//void vTaskButtonRead(void *pvParameters) {
//    for (;;) {
//        if (gpio_get_pin_lvl(btn1) == GPIO_LOW) {
//            gpio_set_pin_lvl(LedSNPin, GPIO_HIGH);
//        } else if (gpio_get_pin_lvl(btn2) == GPIO_LOW) {
//            gpio_set_pin_lvl(LedTXPin, GPIO_HIGH);
//        } else if (gpio_get_pin_lvl(btn3) == GPIO_LOW) {
//            gpio_set_pin_lvl(LedRXPin, GPIO_HIGH);
//        } else {
//            gpio_set_pin_lvl(LedSNPin, GPIO_LOW);
//            gpio_set_pin_lvl(LedTXPin, GPIO_LOW);
//            gpio_set_pin_lvl(LedRXPin, GPIO_LOW);
//        }
//    }
//}

int main(void) {
    /*
     * Set the main clock to 48MHz
     */
    Clock_Init();
    init_pins();

    int toggle = 0;

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

//    xTaskCreate(
//            vTaskHeartbeat,       /* Function that implements the task. */
//            "Heartbeat",          /* Text name for the task. */
//            STACK_SIZE,      /* Number of indexes in the xStack array. */
//            (void *) 1,    /* Parameter passed into the task. */
//            tskIDLE_PRIORITY,/* Priority at which the task is created. */
//            xStack          /* Array to use as the task's stack. */
//    );
//
//    xTaskCreate(
//            vTaskStatLed,       /* Function that implements the task. */
//            "control status led",          /* Text name for the task. */
//            STACK_SIZE,      /* Number of indexes in the xStack array. */
//            (void *) 1,    /* Parameter passed into the task. */
//            tskIDLE_PRIORITY,/* Priority at which the task is created. */
//            xStack          /* Array to use as the task's stack. */
//    );
//
////    xTaskCreate(
////            vTaskIOMUX,       /* Function that implements the task. */
////            "Heartbeat",          /* Text name for the task. */
////            STACK_SIZE,      /* Number of indexes in the xStack array. */
////            (void *) 1,    /* Parameter passed into the task. */
////            tskIDLE_PRIORITY,/* Priority at which the task is created. */
////            xStack          /* Array to use as the task's stack. */
////    );
//
//    xTaskCreate(
//            vTaskButtonRead,       /* Function that implements the task. */
//            "Button Read",          /* Text name for the task. */
//            STACK_SIZE,      /* Number of indexes in the xStack array. */
//            (void *) 1,    /* Parameter passed into the task. */
//            tskIDLE_PRIORITY,/* Priority at which the task is created. */
//            xStack          /* Array to use as the task's stack. */
//    );
//    vTaskStartScheduler();

    while (1) {
//        gpio_set_pin_lvl(statNeoPixel,GPIO_LOW);
//        gpio_set_pin_lvl(statNeoPixel,GPIO_HIGH);
//        gpio_set_pin_lvl(statNeoPixel,GPIO_LOW);
//        gpio_toggle_pin_output(statNeoPixel);
//        PORT->Group[((GPIO_PIN_PA6 >> 8) - 1)].OUTSET.reg = (1 << (GPIO_PIN_PA6 & 0xFF));
//        __asm__("NOP; NOP; NOP");
//        PORT->Group[((GPIO_PIN_PA6 >> 8) - 1)].OUTCLR.reg = (1 << (GPIO_PIN_PA6 & 0xFF));

        //gpio_set_pin_mode(GPIO_PIN_PA6, GPIO_MODE_OUTPUT);
//        const uint8_t colorVal[24] = {1, 0, 0, 1, 0, 1, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0};
//        for (;;) {
//            for (uint8_t i = 0; i < sizeof(colorVal); i++) {
//                //gpio_set_pin_lvl(GPIO_PIN_PA6, colorVal[i]);
//                gpio_set_pin_high(GPIO_PIN_PA6);
//                gpio_set_pin_low(GPIO_PIN_PA6);
//            }
//
//        }
//        gpio_set_pin_high(GPIO_PIN_PA6);
//        gpio_set_pin_low(GPIO_PIN_PA6);



//        uint8_t *ptr, *end, p, bitMask, portNum;
//        uint32_t pinMask;
//
//        portNum = g_APinDescription[pin].ulPort;
//        pinMask = 1ul << g_APinDescription[pin].ulPin;
//        ptr = pixels;
//        end = ptr + numBytes;
//        p = *ptr++;
//        bitMask = 0x80;
//
//        volatile uint32_t *set = &(PORT->Group[portNum].OUTSET.reg),
//                *clr = &(PORT->Group[portNum].OUTCLR.reg);
//
//        for (;;) {
//            *set = pinMask;
//            asm("nop; nop; nop; nop; nop; nop; nop; nop;");
//            if (p & bitMask) {
//                asm("nop; nop; nop; nop; nop; nop; nop; nop;"
//                    "nop; nop; nop; nop; nop; nop; nop; nop;"
//                    "nop; nop; nop; nop;");
//                *clr = pinMask;
//            } else {
//                *clr = pinMask;
//                asm("nop; nop; nop; nop; nop; nop; nop; nop;"
//                    "nop; nop; nop; nop; nop; nop; nop; nop;"
//                    "nop; nop; nop; nop;");
//            }
//            if (bitMask >>= 1) {
//                asm("nop; nop; nop; nop; nop; nop; nop; nop; nop;");
//            } else {
//                if (ptr >= end)
//                    break;
//                p = *ptr++;
//                bitMask = 0x80;
//            }
//        }
        PORT->Group[((GPIO_PIN_PA6 >> 8) - 1)].OUTSET.reg = (1 << (GPIO_PIN_PA6 & 0xFF));
        asm("nop;");
        PORT->Group[((GPIO_PIN_PA6 >> 8) - 1)].OUTCLR.reg = (1 << (GPIO_PIN_PA6 & 0xFF));

        PORT->Group[((GPIO_PIN_PA6 >> 8) - 1)].OUTSET.reg = (1 << (GPIO_PIN_PA6 & 0xFF));
        asm("nop;");
        PORT->Group[((GPIO_PIN_PA6 >> 8) - 1)].OUTCLR.reg = (1 << (GPIO_PIN_PA6 & 0xFF));

        PORT->Group[((GPIO_PIN_PA6 >> 8) - 1)].OUTSET.reg = (1 << (GPIO_PIN_PA6 & 0xFF));
        asm("nop;");
        PORT->Group[((GPIO_PIN_PA6 >> 8) - 1)].OUTCLR.reg = (1 << (GPIO_PIN_PA6 & 0xFF));

        PORT->Group[((GPIO_PIN_PA6 >> 8) - 1)].OUTSET.reg = (1 << (GPIO_PIN_PA6 & 0xFF));
        asm("nop;");
        PORT->Group[((GPIO_PIN_PA6 >> 8) - 1)].OUTCLR.reg = (1 << (GPIO_PIN_PA6 & 0xFF));

        PORT->Group[((GPIO_PIN_PA6 >> 8) - 1)].OUTSET.reg = (1 << (GPIO_PIN_PA6 & 0xFF));
        asm("nop;");
        PORT->Group[((GPIO_PIN_PA6 >> 8) - 1)].OUTCLR.reg = (1 << (GPIO_PIN_PA6 & 0xFF));

        PORT->Group[((GPIO_PIN_PA6 >> 8) - 1)].OUTSET.reg = (1 << (GPIO_PIN_PA6 & 0xFF));
        asm("nop;");
        PORT->Group[((GPIO_PIN_PA6 >> 8) - 1)].OUTCLR.reg = (1 << (GPIO_PIN_PA6 & 0xFF));

        PORT->Group[((GPIO_PIN_PA6 >> 8) - 1)].OUTSET.reg = (1 << (GPIO_PIN_PA6 & 0xFF));
        asm("nop;");
        PORT->Group[((GPIO_PIN_PA6 >> 8) - 1)].OUTCLR.reg = (1 << (GPIO_PIN_PA6 & 0xFF));

        PORT->Group[((GPIO_PIN_PA6 >> 8) - 1)].OUTSET.reg = (1 << (GPIO_PIN_PA6 & 0xFF));
        asm("nop;");
        PORT->Group[((GPIO_PIN_PA6 >> 8) - 1)].OUTCLR.reg = (1 << (GPIO_PIN_PA6 & 0xFF));

        PORT->Group[((GPIO_PIN_PA6 >> 8) - 1)].OUTSET.reg = (1 << (GPIO_PIN_PA6 & 0xFF));
        asm("nop;");
        PORT->Group[((GPIO_PIN_PA6 >> 8) - 1)].OUTCLR.reg = (1 << (GPIO_PIN_PA6 & 0xFF));

        PORT->Group[((GPIO_PIN_PA6 >> 8) - 1)].OUTSET.reg = (1 << (GPIO_PIN_PA6 & 0xFF));
        asm("nop;");
        PORT->Group[((GPIO_PIN_PA6 >> 8) - 1)].OUTCLR.reg = (1 << (GPIO_PIN_PA6 & 0xFF));

        PORT->Group[((GPIO_PIN_PA6 >> 8) - 1)].OUTSET.reg = (1 << (GPIO_PIN_PA6 & 0xFF));
        asm("nop;");
        PORT->Group[((GPIO_PIN_PA6 >> 8) - 1)].OUTCLR.reg = (1 << (GPIO_PIN_PA6 & 0xFF));

        PORT->Group[((GPIO_PIN_PA6 >> 8) - 1)].OUTSET.reg = (1 << (GPIO_PIN_PA6 & 0xFF));
        asm("nop;");
        PORT->Group[((GPIO_PIN_PA6 >> 8) - 1)].OUTCLR.reg = (1 << (GPIO_PIN_PA6 & 0xFF));

        PORT->Group[((GPIO_PIN_PA6 >> 8) - 1)].OUTSET.reg = (1 << (GPIO_PIN_PA6 & 0xFF));
        asm("nop;");
        PORT->Group[((GPIO_PIN_PA6 >> 8) - 1)].OUTCLR.reg = (1 << (GPIO_PIN_PA6 & 0xFF));

        PORT->Group[((GPIO_PIN_PA6 >> 8) - 1)].OUTSET.reg = (1 << (GPIO_PIN_PA6 & 0xFF));
        asm("nop;");
        PORT->Group[((GPIO_PIN_PA6 >> 8) - 1)].OUTCLR.reg = (1 << (GPIO_PIN_PA6 & 0xFF));

        PORT->Group[((GPIO_PIN_PA6 >> 8) - 1)].OUTSET.reg = (1 << (GPIO_PIN_PA6 & 0xFF));
        asm("nop;");
        PORT->Group[((GPIO_PIN_PA6 >> 8) - 1)].OUTCLR.reg = (1 << (GPIO_PIN_PA6 & 0xFF));

        PORT->Group[((GPIO_PIN_PA6 >> 8) - 1)].OUTSET.reg = (1 << (GPIO_PIN_PA6 & 0xFF));
        asm("nop;");
        PORT->Group[((GPIO_PIN_PA6 >> 8) - 1)].OUTCLR.reg = (1 << (GPIO_PIN_PA6 & 0xFF));

        PORT->Group[((GPIO_PIN_PA6 >> 8) - 1)].OUTSET.reg = (1 << (GPIO_PIN_PA6 & 0xFF));
        asm("nop;");
        PORT->Group[((GPIO_PIN_PA6 >> 8) - 1)].OUTCLR.reg = (1 << (GPIO_PIN_PA6 & 0xFF));

        PORT->Group[((GPIO_PIN_PA6 >> 8) - 1)].OUTSET.reg = (1 << (GPIO_PIN_PA6 & 0xFF));
        asm("nop;");
        PORT->Group[((GPIO_PIN_PA6 >> 8) - 1)].OUTCLR.reg = (1 << (GPIO_PIN_PA6 & 0xFF));

        PORT->Group[((GPIO_PIN_PA6 >> 8) - 1)].OUTSET.reg = (1 << (GPIO_PIN_PA6 & 0xFF));
        asm("nop; nop;");
        PORT->Group[((GPIO_PIN_PA6 >> 8) - 1)].OUTCLR.reg = (1 << (GPIO_PIN_PA6 & 0xFF));
        asm("nop;");

        PORT->Group[((GPIO_PIN_PA6 >> 8) - 1)].OUTSET.reg = (1 << (GPIO_PIN_PA6 & 0xFF));
        asm("nop;");
        PORT->Group[((GPIO_PIN_PA6 >> 8) - 1)].OUTCLR.reg = (1 << (GPIO_PIN_PA6 & 0xFF));

        PORT->Group[((GPIO_PIN_PA6 >> 8) - 1)].OUTSET.reg = (1 << (GPIO_PIN_PA6 & 0xFF));
        asm("nop;");
        PORT->Group[((GPIO_PIN_PA6 >> 8) - 1)].OUTCLR.reg = (1 << (GPIO_PIN_PA6 & 0xFF));

        PORT->Group[((GPIO_PIN_PA6 >> 8) - 1)].OUTSET.reg = (1 << (GPIO_PIN_PA6 & 0xFF));
        asm("nop;");
        PORT->Group[((GPIO_PIN_PA6 >> 8) - 1)].OUTCLR.reg = (1 << (GPIO_PIN_PA6 & 0xFF));

        PORT->Group[((GPIO_PIN_PA6 >> 8) - 1)].OUTSET.reg = (1 << (GPIO_PIN_PA6 & 0xFF));
        asm("nop;");
        PORT->Group[((GPIO_PIN_PA6 >> 8) - 1)].OUTCLR.reg = (1 << (GPIO_PIN_PA6 & 0xFF));

        PORT->Group[((GPIO_PIN_PA6 >> 8) - 1)].OUTSET.reg = (1 << (GPIO_PIN_PA6 & 0xFF));
        asm("nop;");
        PORT->Group[((GPIO_PIN_PA6 >> 8) - 1)].OUTCLR.reg = (1 << (GPIO_PIN_PA6 & 0xFF));

//        while(1) {
//            //
//        }

    }
}
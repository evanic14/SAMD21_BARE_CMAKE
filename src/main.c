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
StackType_t xStack[ STACK_SIZE ];

//function for setting related to power management system
//See section 16 of the datasheet


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

    //const uint8_t buffer[3] = {0xFF, 0xFF, 0xFF};
    for( ;; )
    {
        gpio_toggle_pin_output(LedHBPin);
        vTaskDelay(500/portTICK_PERIOD_MS);
    }
}

void vTaskStatLed( void * pvParameters)
{
    gpio_set_pin_mode(GPIO_PIN_PA6, GPIO_MODE_OUTPUT);
    __asm("NOP");
    gpio_set_pin_lvl(GPIO_PIN_PA6, 1);
    gpio_set_pin_lvl(GPIO_PIN_PA6, 0);
    for(uint8_t i =0; i < 8; i++) {
        gpio_set_pin_lvl(GPIO_PIN_PA6, 1);
        __asm("NOP");
        gpio_set_pin_lvl(GPIO_PIN_PA6, 1);
        __asm("NOP");
        gpio_set_pin_lvl(GPIO_PIN_PA6, 0);
        __asm("NOP");
    }
    for(uint8_t i =0; i < 8; i++) {
        gpio_set_pin_lvl(GPIO_PIN_PA6, 1);
        __asm("NOP");
        gpio_set_pin_lvl(GPIO_PIN_PA6, 1);
        __asm("NOP");
        gpio_set_pin_lvl(GPIO_PIN_PA6, 0);
        __asm("NOP");
    }
    for(uint8_t i =0; i < 8; i++) {
        gpio_set_pin_lvl(GPIO_PIN_PA6, 0);
        __asm("NOP");
        gpio_set_pin_lvl(GPIO_PIN_PA6, 0);
        __asm("NOP");
        gpio_set_pin_lvl(GPIO_PIN_PA6, 0);
        __asm("NOP");
    }
    for(uint8_t i =0; i < 8; i++) {
        gpio_set_pin_lvl(GPIO_PIN_PA6, 0);
        __asm("NOP");
        gpio_set_pin_lvl(GPIO_PIN_PA6, 0);
        __asm("NOP");
        gpio_set_pin_lvl(GPIO_PIN_PA6, 0);
        __asm("NOP");
    }
}

void vTaskIOMUX( void * pvParameters )
{
    I2C_HOST_INIT(I2C_PERIPHERAL, I2C_CLOCK_SOURCE, MAIN_CLOCK_SPEED, I2C_CLOCK_SPEED, I2C_EXTRA_CONFIG_OPTIONS);
    for( ;; )
    {
        IO_MUX_READ();
    }
}


void vTaskButtonRead( void * pvParameters )
{
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
    init_pins();

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
            vTaskStatLed,       /* Function that implements the task. */
            "control status led",          /* Text name for the task. */
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


#define ESP_INTR_FLAG_DEFAULT 0
#define BUTTON_PIN GPIO_NUM_0 //heltec prg button
//#include "freertos/FreeRTOS.h"
//#include "freertos/task.h"
//#include "freertos/semphr.h"
#include "Arduino.h"

uint8_t OLEDoff;

void IRAM_ATTR btn_ISR()
{
    noInterrupts();
    Serial.print("Oh, button pushed!: ");
    OLEDoff ^= 1;
    Serial.println(OLEDoff, HEX);
   // OLED.setPowerSave(OLEDon); //ToDo this is uggly as it depends from the include order

    interrupts();


} // end of isr

void button_init()
{
   attachInterrupt(digitalPinToInterrupt(0), btn_ISR, FALLING);
}

/*
SemaphoreHandle_t semaphore = nullptr;

static unsigned char OLEDon = 0;

void IRAM_ATTR handler(void *arg)
{
    xSemaphoreGiveFromISR(semaphore, NULL);
}

void button_task(void *arg)
{
    for (;;)
    {
        if (xSemaphoreTake(semaphore, portMAX_DELAY) == pdTRUE)
        {
            Serial.print("Oh, button pushed!: ");
            OLEDon ^= 1;
            Serial.println(OLEDon, HEX);
            OLED.setPowerSave(OLEDon); //ToDo this is uggly as it depends from the include order
            
        }
    }
}

void button_init()
{
    semaphore = xSemaphoreCreateBinary();
    // Setup the button GPIO pin
    gpio_pad_select_gpio(BUTTON_PIN);
    // Quite obvious, a button is a input
    gpio_set_direction(BUTTON_PIN, GPIO_MODE_INPUT);

    // Trigger the interrupt when going from HIGH -> LOW ( == pushing button)
    gpio_set_intr_type(BUTTON_PIN, GPIO_INTR_NEGEDGE);

    // Associate button_task method as a callback
    xTaskCreate(button_task, "button_task", 4096, NULL, 10, NULL);

    // Install default ISR service
    gpio_install_isr_service(ESP_INTR_FLAG_DEFAULT);

    // Add our custom button handler to ISR
    gpio_isr_handler_add(BUTTON_PIN, handler, NULL);                

} */
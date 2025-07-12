#include <stdio.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "sx1262_interface.hpp"

extern "C" 
{
    void app_main(void);
}

void app_main(void)
{
    sx1262_init();

    while (1) 
    {
        printf("Hello from wave_rover_LoRa! \n");
        vTaskDelay(pdMS_TO_TICKS(1000));
    }
}

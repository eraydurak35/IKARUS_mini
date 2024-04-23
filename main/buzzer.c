#include <stdint.h>
#include "buzzer.h"
#include "gpio.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

void play_notification(uint8_t sound_number)
{

    switch (sound_number)
    {
    case 1:
        start_beep(440);
        vTaskDelay(50 / portTICK_PERIOD_MS);
        stop_beep();
        vTaskDelay(50 / portTICK_PERIOD_MS);
        start_beep(880);
        vTaskDelay(50 / portTICK_PERIOD_MS);
        stop_beep();
        vTaskDelay(50 / portTICK_PERIOD_MS);
        start_beep(1760);
        vTaskDelay(50 / portTICK_PERIOD_MS);
        stop_beep();
        vTaskDelay(50 / portTICK_PERIOD_MS);
        break;
    

    case 2:
        start_beep(880);
        vTaskDelay(150 / portTICK_PERIOD_MS);
        stop_beep();
        vTaskDelay(50 / portTICK_PERIOD_MS);
        start_beep(880);
        vTaskDelay(50 / portTICK_PERIOD_MS);
        stop_beep();
        vTaskDelay(50 / portTICK_PERIOD_MS);
        start_beep(1760);
        vTaskDelay(50 / portTICK_PERIOD_MS);
        stop_beep();
        vTaskDelay(50 / portTICK_PERIOD_MS);
        break;

    case 3:
        start_beep(1760);
        vTaskDelay(50 / portTICK_PERIOD_MS);
        stop_beep();
        vTaskDelay(50 / portTICK_PERIOD_MS);
        start_beep(880);
        vTaskDelay(50 / portTICK_PERIOD_MS);
        stop_beep();
        vTaskDelay(50 / portTICK_PERIOD_MS);
        start_beep(440);
        vTaskDelay(50 / portTICK_PERIOD_MS);
        stop_beep();
        vTaskDelay(50 / portTICK_PERIOD_MS);
        break;

    case 4:
        start_beep(1760);
        vTaskDelay(150 / portTICK_PERIOD_MS);
        stop_beep();
        vTaskDelay(50 / portTICK_PERIOD_MS);
        start_beep(880);
        vTaskDelay(50 / portTICK_PERIOD_MS);
        stop_beep();
        vTaskDelay(50 / portTICK_PERIOD_MS);
        start_beep(880);
        vTaskDelay(50 / portTICK_PERIOD_MS);
        stop_beep();
        vTaskDelay(50 / portTICK_PERIOD_MS);
        break;

    case 5:
        start_beep(1760);
        vTaskDelay(150 / portTICK_PERIOD_MS);
        stop_beep();
        vTaskDelay(50 / portTICK_PERIOD_MS);
        start_beep(880);
        vTaskDelay(50 / portTICK_PERIOD_MS);
        stop_beep();
        vTaskDelay(50 / portTICK_PERIOD_MS);
        start_beep(1760);
        vTaskDelay(50 / portTICK_PERIOD_MS);
        stop_beep();
        vTaskDelay(50 / portTICK_PERIOD_MS);
        break;

    case 6:
        start_beep(880);
        vTaskDelay(150 / portTICK_PERIOD_MS);
        stop_beep();
        vTaskDelay(50 / portTICK_PERIOD_MS);
        start_beep(1760);
        vTaskDelay(50 / portTICK_PERIOD_MS);
        stop_beep();
        vTaskDelay(50 / portTICK_PERIOD_MS);
        start_beep(880);
        vTaskDelay(50 / portTICK_PERIOD_MS);
        stop_beep();
        vTaskDelay(50 / portTICK_PERIOD_MS);
        break;
    
    default:
        break;
    }
}
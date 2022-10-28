#include <stdio.h>
#include "DendoStepper.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_log.h"

DendoStepper step;

extern "C" void app_main(void)
{
    DendoStepper_config_t step_cfg = {
        .stepPin = 12,
        .dirPin = 13,
        .enPin = 14,
        .timer_group = TIMER_GROUP_0,
        .timer_idx = TIMER_0,
        .miStep = MICROSTEP_32,
        .stepsPerRot = 1.8
    };

    step.config(&step_cfg);

    step.init();

    step.runPos(1000);

    while(1){

        vTaskDelay(100);
    }

}
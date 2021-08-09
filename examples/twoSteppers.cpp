#include "DendoStepper.h"

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_log.h"

DendoStepper_config_t conf={
    .step_p=14,
    .dir_p=27,
    .en_p=26,
    .timer_group=TIMER_GROUP_0,
    .timer_idx=TIMER_0,
};

DendoStepper_config_t conf1={
    .step_p=13,
    .dir_p=12,
    .en_p=22,
    .timer_group=TIMER_GROUP_0,
    .timer_idx=TIMER_1,
};

DendoStepper step(&conf);
DendoStepper step1(&conf1);

void task(void*p){
    while(1){
        if(step.getState()<=IDLE){
            step.runAbsolute(esp_random()>>18);
        }
        if(step1.getState()<=IDLE){
            step1.runAbsolute(esp_random()>>18);
        }
        vTaskDelay(1000);
    }
}

extern "C" void app_main() {
    step.init();
    step.setSpeed(1000,1000);
    step1.init();
    step1.setSpeed(1000,1000);
    /*step.runPos(-1000);
    step.runAbsolute(2500);*/
    xTaskCreate(task,"stepper",4096,NULL,10,NULL);
}
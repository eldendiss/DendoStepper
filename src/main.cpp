#include "DendoStepper.h"

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

DendoStepper_config_t conf={
    .step_p=14,
    .dir_p=27,
    .en_p=26,
    .endSw_p=25,
    .timer_group=TIMER_GROUP_0,
    .timer_idx=TIMER_0,
};

DendoStepper step(&conf);

void task(void*p){
    while(1){
        if(step.getState()<=IDLE){
            uint32_t pos=esp_random()>>16;
            ESP_LOGI("runTo","%d",pos);
            if(step.runAbsolute(pos))
                ESP_LOGI("runAbs","illegal");
        }
    vTaskDelay(100);
    }
}

extern "C" void app_main() {
    step.init();
    step.setSpeed(3200,500);
    //step.runPos(-1000);
    xTaskCreate(task,"stepper",4096,NULL,10,NULL);
}
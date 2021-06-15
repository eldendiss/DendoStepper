#include "DendoStepper.h"

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_log.h"

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
        vTaskDelay(1000);
        if(step.getState()<=IDLE){
            if(step.home(1000,1000,CCW)){
            ESP_LOGI("homing","now");
            vTaskDelete(NULL);
        }
        }
    }
}

extern "C" void app_main() {
    step.init();
    step.setSpeed(1000,1000);
    //step.runPos(-1000);
    step.runAbsolute(2500);
    xTaskCreate(task,"stepper",4096,NULL,10,NULL);
}
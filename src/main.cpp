#include "DendoStepper.h"

DendoStepper_config_t conf={
    .step_p=14,
    .dir_p=27,
    .en_p=26,
    .timer_group=TIMER_GROUP_0,
    .timer_idx=TIMER_0,
};

DendoStepper step(&conf);

extern "C" void app_main() {
    step.init();
    step.setSpeed(300,5000);
    step.runPos(100000);
}
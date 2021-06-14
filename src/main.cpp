/*#include "XYZ.h"

XYZ_config_t xyz_conf{
    .X = {
        .step_pin=14,
        .dir_pin=27,
    },
    .Y = {
        .step_pin=25,
        .dir_pin=26,
    },
};

XYZ axis(&xyz_conf);*/

#include "DendoStepper.h"

DendoStepper_config_t conf={
    .X={
        .step_p=14,
        .dir_p=27,
    },
    .Y={
        .step_p=27,
        .dir_p=26,
    },
};
TaskHandle_t *runHandle=NULL;

DendoStepper step(&conf);

extern "C" void app_main() {
    step.init(runHandle);
    step.setSpeed(200,50);
    step.runPos(200);
}
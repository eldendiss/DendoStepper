#pragma once

#ifndef DENDOSTEPPER_H
#define DENDOSTEPPER_H

#include "stdint.h"
#include "driver/timer.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/gpio.h"
#include "esp_log.h"
#include "math.h"

/* HW configuration library */
typedef struct{
    uint8_t step_p;
    uint8_t dir_p;
}X_config_t;

typedef struct{
    uint8_t step_p;
    uint8_t dir_p;
}Y_config_t;

typedef struct
{
    X_config_t X;
    Y_config_t Y;
} DendoStepper_config_t;

typedef struct{
    uint32_t    stepInterval=2000;  //step interval in us
    uint16_t    accStepInc=100;     //step interval increase during acc/dec phase
    uint32_t    stepCnt=0;          //step counter
    uint32_t    accEnd;             //when to end acc and start coast
    uint32_t    coastEnd;           //when to end coast and start decel
    uint32_t    stepsToGo=0;        //steps we need to take
    uint16_t    speed=100;          //speed in steps*second^-1
    uint16_t    acc=100;            //acceleration in steps*second^-2
}ctrl_var_t;

class DendoStepper
{
private:
    DendoStepper_config_t *conf;
    ctrl_var_t ctrl;
    void calc(uint16_t, uint16_t,uint32_t);
    static bool xISRwrap(void*);
    bool xISR();
    static void yISR(void*);
public:
    /** @brief Costructor - prepares conf variables
     *  @param config DendoStepper_config_t pointer
     */
    DendoStepper(DendoStepper_config_t*);
    /** @brief initialize peripherals
     */
    void init(TaskHandle_t*);
    /** @brief freeRTOS fx wrapper for run() task
     *  @param _this this pointer DendoStepper*
     */
    static void runTask(void *);

    void run();
    /** @brief runs motor to relative position in steps
     *  @param relative number of steps to run, negative is reverse 
     */
    void runPos(int32_t);

    /** @brief sets motor speed
     *  @param speed speed in steps per second
     */
    void setSpeed(uint16_t,uint16_t);
};

#endif
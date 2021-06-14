#pragma once

#ifndef DENDOSTEPPER_H
#define DENDOSTEPPER_H

#include "stdint.h"
#include "driver/timer.h"
#include "driver/gpio.h"
#include "esp_log.h"
#include "math.h"

/* HW configuration struct */
typedef struct
{
    uint8_t         step_p;         //step signal gpio
    uint8_t         dir_p;          //dir signal gpio
    uint8_t         en_p;           //enable signal gpio
    timer_group_t   timer_group;    //timer group, useful if we are controlling more than 2 steppers
    timer_idx_t     timer_idx;      //timer index, useful if we are controlling 2steppers
} DendoStepper_config_t;

enum motor_status{
    DISABLED,
    IDLE,
    ACC,
    COAST,
    DEC,
};

typedef struct{
    uint32_t    stepInterval=2000;  //step interval in us
    uint16_t    accStepInc=100;     //step interval increase during acc/dec phase
    uint32_t    stepCnt=0;          //step counter
    uint32_t    accEnd;             //when to end acc and start coast
    uint32_t    coastEnd;           //when to end coast and start decel
    uint32_t    stepsToGo=0;        //steps we need to take
    uint16_t    speed=100;          //speed in steps*second^-1
    uint16_t    acc=100;            //acceleration in steps*second^-2
    uint8_t     status=DISABLED;
}ctrl_var_t;

class DendoStepper
{
private:
    DendoStepper_config_t *conf;
    ctrl_var_t ctrl;
    /** @brief PRIVATE: Step interval calculation
     *  @param speed maximum movement speed
     *  @param accTimeMs acceleration time in ms
     *  @param target target position
     */
    void calc(uint16_t, uint16_t,uint32_t);

    /** @brief sets En GPIO
     *  @param state 0-LOW,1-HIGH
     *  @return void
     */
    void setEn(bool);

    /** @brief sets Dir GPIO
     *  @param state 0-CW 1-CCW
     */
    void setDir(bool);

    /** @brief static wrapper for ISR function
     *  @param _this DendoStepper* this pointer
     *  @return bool
     */
    static bool xISRwrap(void* _this){
        return static_cast<DendoStepper *>(_this)->xISR();
    }

    bool xISR();


public:
    /** @brief Costructor - prepares conf variables
     *  @param config DendoStepper_config_t pointer
     */
    DendoStepper(DendoStepper_config_t*);
    
    /** @brief initialize GPIO and Timer peripherals
     */
    void init();
    
    /** @brief runs motor to relative position in steps
     *  @param relative number of steps to run, negative is reverse 
     */
    void runPos(int32_t);

    /** @brief sets motor speed
     *  @param speed speed in steps per second
     *  @param accTimeMs acceleration time in ms
     */
    void setSpeed(uint16_t,uint16_t);

    /** @brief set EN pin 1, stops movement
    */
    void disableMotor();

    /** @brief set EN pin to 0, enables movement
     */
    void enableMotor();

    /** @brief returns current state
     *  @return motor_status enum
     */
    uint8_t getState();
};

#endif
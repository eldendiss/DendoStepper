#pragma once

#ifndef DENDOSTEPPER_H
#define DENDOSTEPPER_H

#include "stdint.h"
#include "driver/timer.h"
#include "driver/gpio.h"
#include "math.h"


#define HOME_ISR_DEBOUNCE 0
#define ACCTIME_MAX (uint64_t)(accTime*25000000ULL)
#define TICK_PER_S 25000000ULL
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
    INF,
};

enum dir{
    CW,
    CCW
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
    uint8_t     recalcInt=1;        //how much steps to take until recalculation (high speeds)
    uint8_t     status=DISABLED;
    bool        dir=CW;
    bool        homed=false;
    bool        runInfinite=true;
}ctrl_var_t;

class DendoStepper
{
private:
    const DendoStepper_config_t *conf;
    ctrl_var_t ctrl;
    uint64_t currentPos=0;  //absolute position
    /** @brief PRIVATE: Step interval calculation
     *  @param speed maximum movement speed
     *  @param accTimeMs acceleration time in ms
     *  @param target target position
     */
    void calc(uint16_t, uint16_t,uint32_t);

    /** @brief PRIVATE: Step interval calculation for infinite movement
     *  @param speed maximum movement speed
     *  @param accTimeMs acceleration time in ms
     */
    void calc(uint16_t, uint16_t);

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
    void decel();

public:
    /** @brief Costructor - prepares conf variables
     *  @param config DendoStepper_config_t pointer
     */
    DendoStepper(const DendoStepper_config_t* config);

    /** @brief Costructor - conf variables to be passed later
     */
    DendoStepper();

    /** @brief Configuration of library, used with constructor w/o params
     *  @param config DendoStepper_config_t pointer
     */
    void config(const DendoStepper_config_t* config);
    
    /** @brief initialize GPIO and Timer peripherals
     */
    void init();
    
    /** @brief runs motor to relative position in steps
     *  @param relative number of steps to run, negative is reverse 
     */
    void runPos(int32_t relative);

    /** @brief sets motor speed
     *  @param speed speed in steps per second
     *  @param accTimeMs acceleration time in ms
     */
    void setSpeed(uint16_t speed,uint16_t accTimeMs);

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

    /** @brief run motor to position in absolute coordinates (steps)
     *  @param postition absolute position in steps from homing position (must be positive);
     *  @return true if motor can run immediately, false if it is currently moving
     */
    bool runAbsolute(uint32_t position);

    /** @brief homes motor, stops when stopswitch is hit and sets absolute position as 0
     *  @param speed speed which will be used for homing
     *  @param accTimeMs acceleration time in ms
     *  @param dir
     *  @return false if motor cant be moved rn or homing is in progress, true if we are homed
     */
    bool home(uint16_t speed,uint16_t accTimeMs,bool dir);

    /** @brief returns current absolute position
     *  @return current absolute postion in steps
     */
    uint64_t getPosition();

    /** @brief resets absolute pos to 0
     */
    void resetAbsolute();

    /** @brief returns current speed
     */
    uint16_t getSpeed();

    /** @brief returns current acceleration time in ms
     */
    uint16_t getAcc();

    /** @brief stops the motor dead, but stays enabled
     */
    void stop();
};

#endif
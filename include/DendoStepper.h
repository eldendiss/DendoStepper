#pragma once

#ifndef DENDOSTEPPER_H
#define DENDOSTEPPER_H

#include "stdint.h"
#include "driver/timer.h"
#include "driver/gpio.h"
#include "math.h"

//#define STEP_DEBUG


#define NS_TO_T_TICKS(x) (x/25)
#define TIMER_F 20000000ULL
#define TICK_PER_S 40000000ULL

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
    uint32_t    stepInterval=40000;  //step interval in ns/25
    uint32_t    constInterval=10;
    int32_t     accelC;
    uint32_t    rest=0;     //step interval increase during acc/dec phase
    uint32_t    stepCnt=0;          //step counter
    uint32_t    accEnd;             //when to end acc and start coast
    uint32_t    accLim;
    uint32_t    coastEnd;           //when to end coast and start decel
    uint32_t    stepsToGo=0;        //steps we need to take
    float       speed=100;          //speed in steps*second^-1
    float       acc=100;            //acceleration in steps*second^-2
    uint8_t     status=DISABLED;
    bool        dir=CW;
    bool        homed=false;
    bool        runInfinite=true;
}ctrl_var_t;

class DendoStepper
{
private:
    DendoStepper_config_t conf;
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

    /** @brief Costructor - conf variables to be passed later
     */
    DendoStepper();

    /** @brief Configuration of library, used with constructor w/o params
     *  @param config DendoStepper_config_t pointer
     */
    void config(DendoStepper_config_t config);
    
    /** @brief initialize GPIO and Timer peripherals
     */
    void init(uint8_t,uint8_t,uint8_t,timer_group_t,timer_idx_t);
    
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
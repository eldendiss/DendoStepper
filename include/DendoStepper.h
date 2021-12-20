#pragma once

#ifndef DENDOSTEPPER_H
#define DENDOSTEPPER_H

#include "stdint.h"
#include "driver/timer.h"
#include "driver/gpio.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_timer.h"
#include "math.h"

//#define STEP_DEBUG


#define NS_TO_T_TICKS(x) (x/25)
#define TIMER_F 20000000ULL
#define TICK_PER_S 40000000ULL

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

enum microStepping_t {
    MICROSTEP_1=0x1,
    MICROSTEP_2,
    MICROSTEP_4=0x4,
    MICROSTEP_8=0x8,
    MICROSTEP_16=0x10,
    MICROSTEP_32=0x20,
    MICROSTEP_64=0x40,
    MICROSTEP_128=0x80,
    MICROSTEP_256=0x100,
};

/* HW configuration struct */
typedef struct
{
    uint8_t         step_p;         //step signal gpio
    uint8_t         dir_p;          //dir signal gpio
    uint8_t         en_p;           //enable signal gpio
    timer_group_t   timer_group;    //timer group, useful if we are controlling more than 2 steppers
    timer_idx_t     timer_idx;      //timer index, useful if we are controlling 2steppers
    microStepping_t miStep=MICROSTEP_1;
} DendoStepper_config_t;

typedef struct{
    uint32_t    stepInterval=40000;  //step interval in ns/25
    int32_t     accelC;
    uint32_t    rest=0;     //step interval increase during acc/dec phase
    uint32_t    stepCnt=0;          //step counter
    uint32_t    accEnd;             //when to end acc and start coast
    uint32_t    accLim;
    uint32_t    coastEnd;           //when to end coast and start decel
    uint32_t    stepsToGo=0;        //steps we need to take
    float       speed=100;          //speed in steps*second^-1
    float       acc=100;            //acceleration in steps*second^-2
    uint64_t    enOffTime=10000L;
    volatile uint8_t     status=DISABLED;
    bool        dir=CW;
    bool        homed=false;
    bool        runInfinite=false;
}ctrl_var_t;

class DendoStepper
{
private:
    DendoStepper_config_t conf;
    ctrl_var_t ctrl;
    esp_timer_handle_t dyingTimer;
    TaskHandle_t enTask;
    uint64_t currentPos=0;  //absolute position
    bool timerStarted=0;

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

    /** @brief static wrapper for En timer
     *  @param _this DendoStepper* this pointer
     *  @return bool
     */
    static void enTimerWrap(void* _this){
        static_cast<DendoStepper *>(_this)->enTimer();
    }

    /** @brief enableMotor wrapper
     */
    static void _disableMotor(void* _this){
        static_cast<DendoStepper *>(_this)->disableMotor();
    }

    bool xISR();

    void enTimer();


public:

    /** @brief Costructor - conf variables to be passed later
     */
    DendoStepper();

    /** @brief Configuration of library, used with constructor w/o params
     *  @param config DendoStepper_config_t pointer
     */
    void config(DendoStepper_config_t config);
    
    /** @brief initialize GPIO and Timer peripherals
     *  @param stepP step pulse pin
     *  @param dirP direction signal pin
     *  @param enP enable signal Pin
     *  @param group timer group to use (0 or 1)
     *  @param index which timer to use (0 or 1)
     *  @param microstepping microstepping performed by the driver, used for more accuracy
     *  @param stepsPerRot how many steps it takes for the motor to move 2Pi rads. this can be also used instead of microstepping parameter
     */
    void init(uint8_t,uint8_t,uint8_t,timer_group_t,timer_idx_t,microStepping_t microstep,uint16_t stepsPerRot);
    
    /** @brief runs motor to relative position in steps
     *  @param relative number of steps to run, negative is reverse 
     */
    esp_err_t runPos(int32_t relative);

    /** @brief sets motor speed
     *  @param speed speed in steps per second
     *  @param accTimeMs acceleration time in ms
     */
    void setSpeed(uint32_t speed,uint16_t accTimeMs);

    /** @brief sets motor speed and accel in radians
     *  @param speed speed rad*s^-1
     *  @param accTimeMs acceleration in rad*s^-2
     */
    void setSpeedRad(float speed, float accTimeMs);

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

    /** @brief sets the timeout after which motor is disabled
     */
    void setEnTimeout(uint64_t timeout);
};

#endif
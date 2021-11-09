#include "DendoStepper.h"
#include "esp_log.h"

#define STEP_DEBUG

#ifdef STEP_DEBUG
#define STEP_LOGI(...) ESP_LOGI(__VA_ARGS__)
#define STEP_LOGW(...) ESP_LOGW(__VA_ARGS__)
#define STEP_LOGE(...) ESP_LOGE(__VA_ARGS__)
#else
#define STEP_LOGI(...) while (0)
#define STEP_LOGW(...) while (0)
#define STEP_LOGE(...) ESP_LOGE(__VA_ARGS__)
#endif

bool state=0;

DendoStepper::DendoStepper()
{

}

void DendoStepper::config(DendoStepper_config_t config)
{
    conf = config;
}

void DendoStepper::setEn(bool state)
{
    ESP_ERROR_CHECK(gpio_set_level((gpio_num_t)conf.en_p, state));
}

void DendoStepper::setDir(bool state)
{
    ctrl.dir = state;
    ESP_ERROR_CHECK(gpio_set_level((gpio_num_t)conf.dir_p, state));
}

void DendoStepper::disableMotor()
{
    setEn(true);
    ctrl.status = DISABLED;
}

void DendoStepper::enableMotor()
{
    setEn(false);
    ctrl.status = IDLE;
}

bool DendoStepper::xISR()
{
    gpio_set_level((gpio_num_t)14, (state=!state)); //step pulse
    //add and substract one step
    if(state==0)
        return 0; //just turn off the pin
    
    ctrl.stepCnt++;
    ctrl.stepsToGo--;
    //absolute coord handling
    if (ctrl.dir == CW)
        currentPos++;
    else if (currentPos > 0)
        currentPos--; //we cant go below 0, or var will overflow
    //if we have nothing more to do, stop timer and end
    if (ctrl.stepsToGo == 0)
    {
        timer_pause(conf.timer_group, conf.timer_idx);
        ctrl.status = IDLE;
        ctrl.stepCnt = 0;
        gpio_set_level((gpio_num_t)conf.step_p, 0); //this should be enough for driver to register pulse
        return 0;
    }
    //ESP_LOGI("kk","k");

    if(ctrl.accelC >0 && ctrl.accelC<ctrl.accEnd){   //we are accelerating
        uint32_t oldInt=ctrl.stepInterval;
        ctrl.stepInterval=oldInt-(2*oldInt+ctrl.rest)/(4*ctrl.accelC+1);
        ctrl.rest=(2*oldInt+ctrl.rest)%(4*ctrl.accelC+1);
        ctrl.accelC++;
    } else if(ctrl.stepCnt>ctrl.coastEnd){  //we must be deccelerating then
        uint32_t oldInt=ctrl.stepInterval;
        ctrl.stepInterval=oldInt+((2*oldInt+ctrl.rest)/(4*ctrl.accelC+1));
        ctrl.rest=(2*oldInt+ctrl.rest)%(4*ctrl.accelC+1);
        ctrl.accelC++;
    } else { //we are coasting
        //ctrl.stepInterval=ctrl.constInterval;
        ctrl.accelC=ctrl.accEnd;
    }
    
    //set alarm to calculated interval
    timer_set_alarm_value(conf.timer_group, conf.timer_idx, ctrl.stepInterval/2);
    //gpio_set_level((gpio_num_t)conf.step_p, 0);
    return 1;
}

void DendoStepper::init(uint8_t stepP,uint8_t dirP,uint8_t enP,timer_group_t group,timer_idx_t index)
{
    conf.step_p=stepP;
    conf.dir_p=dirP;
    conf.en_p=enP;
    conf.timer_group=group;
    conf.timer_idx=index;

    uint64_t mask = (1ULL << stepP) | (1ULL << dirP) | (1ULL << enP);   //put gpio pins in bitmask
    gpio_config_t gpio_conf = { //config gpios
        .pin_bit_mask = mask,
        .mode = GPIO_MODE_OUTPUT,
        .pull_up_en = GPIO_PULLUP_DISABLE,
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
        .intr_type = GPIO_INTR_DISABLE,
    };
    //set the gpios as per gpio_conf
    ESP_ERROR_CHECK(gpio_config(&gpio_conf));
    timer_config_t timer_conf = {
        .alarm_en = TIMER_ALARM_EN,         //we need alarm
        .counter_en = TIMER_PAUSE,          //dont start now lol
        .intr_type = TIMER_INTR_LEVEL,      //interrupt
        .counter_dir = TIMER_COUNT_UP,      //count up duh
        .auto_reload = TIMER_AUTORELOAD_EN, //reload pls
        .divider = 2,                       //25ns resolution
    };
    
    ESP_ERROR_CHECK(timer_init(conf.timer_group, conf.timer_idx, &timer_conf));   //init the timer
    ESP_ERROR_CHECK(timer_set_counter_value(conf.timer_group, conf.timer_idx, 0)); //set it to 0
    ESP_ERROR_CHECK(timer_isr_callback_add(conf.timer_group, conf.timer_idx, xISRwrap, this, 0)); //add callback fn to run when alarm is triggrd
}

void DendoStepper::runPos(int32_t relative)
{
    if (!relative) //why would u call it with 0 wtf
        return;
    if (ctrl.status > IDLE)
    { //we are running, we need to adjust steps accordingly
        if (relative < ctrl.stepsToGo)
        {
            return; //nothing
        }
        //if we are moving in the right direction, just add or sub steps to go
        if (ctrl.dir == (relative < 0))
        {
            if (relative > 0)
                relative += ctrl.stepsToGo;
            else
                relative -= ctrl.stepsToGo;
        }
        else
        {
            return; //idk what to do here yet, we need to move in the opposite direction
        }
        stop();
    }
    ctrl.homed = false;          //we are not longer homed
    if (ctrl.status == DISABLED) //if motor is disabled, enable it
        enableMotor();              
    setDir(relative < 0); //set CCW if <0, else set CW
    calc(ctrl.speed, ctrl.acc, abs(relative));  //calculate velocity profile
    ESP_ERROR_CHECK(timer_set_alarm_value(conf.timer_group, conf.timer_idx, ctrl.stepInterval));  //set HW timer alarm to stepinterval
    ESP_ERROR_CHECK(timer_start(conf.timer_group, conf.timer_idx));   //start the timer
}

void DendoStepper::setSpeed(uint16_t speed, uint16_t accT)
{
    ctrl.speed = speed/(200.0*32.0);
    ctrl.acc = ctrl.speed/(accT/1000.0);
    STEP_LOGI("DendoStepper","Speed set: %f %f",ctrl.speed,ctrl.acc);
}

void DendoStepper::calc(uint16_t speed, uint16_t accTimeMs, uint32_t target)
{

    ctrl.accEnd=(ctrl.speed*ctrl.speed)/(2.0*0.0005*ctrl.acc);
    ctrl.accLim= (target* ctrl.acc)/(ctrl.acc*2);
    if(ctrl.accEnd<ctrl.accLim){   //we will be coasting
        ctrl.coastEnd = target-ctrl.accEnd;
    } else {
        ctrl.accEnd=target-ctrl.accLim;
        ctrl.coastEnd=ctrl.accEnd;
    }
    //init vars
    ctrl.accelC=1;
    ctrl.rest=0;
    //calculate initial stepinterval = timer_freq[Hz]*sqrt(((2*2*PI)/step_per_rot)/acceleration[rad*s^-2])
    ctrl.stepInterval = (float)TIMER_F*sqrt((((4*3.14)/(200.0*32.0))/ctrl.acc));
    //calculate interval for coasting = 1[s]/(speed[rad/s]*steps_per_rot)
    //ctrl.constInterval= (float)TICK_PER_S/(ctrl.speed*200*32);
    //set steps we will take 
    ctrl.stepsToGo=target;
    STEP_LOGI("calc", "acc end:%u coastend:%u acclim:%u stepstogo:%u speed:%f acc:%f int: %u", ctrl.accEnd, ctrl.coastEnd, ctrl.accLim, ctrl.stepsToGo, ctrl.speed, ctrl.acc,ctrl.stepInterval);
    //init old interval
    uint32_t oldInt=ctrl.stepInterval;
    /*ctrl.stepInterval=oldInt-(2*oldInt+ctrl.rest)/(4*ctrl.accelC+1);
    ctrl.rest=(2*oldInt+ctrl.rest)%(4*ctrl.accelC+1);*/
    STEP_LOGI("calc","int: %u rest %u",ctrl.stepInterval,ctrl.rest);
}

uint8_t DendoStepper::getState()
{
    return ctrl.status;
}

bool DendoStepper::runAbsolute(uint32_t position)
{
    if (getState() > IDLE)  //we are already moving, so stop it
        stop();
    while (getState() > IDLE)
    {
        //waiting for idle, watchdog should take care of inf loop if it occurs
    }                              //shouldnt take long tho
    runPos(position - currentPos); //run to new position
    return 1;
}

uint64_t DendoStepper::getPosition()
{
    return currentPos;
}

void DendoStepper::resetAbsolute()
{
    currentPos = 0;
}

uint16_t DendoStepper::getSpeed()
{
    return ctrl.speed;
}

uint16_t DendoStepper::getAcc()
{
    return ctrl.acc;
}

void DendoStepper::stop()
{
    ctrl.stepsToGo = 1; //no more steps needed, xISR should take care of the rest
    //todo: deccelerate
}

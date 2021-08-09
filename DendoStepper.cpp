#include "DendoStepper.h"
#include "esp_log.h"

#ifdef STEP_DEBUG
#define STEP_LOGI(...) ESP_LOGI(__VA_ARGS__)
#define STEP_LOGW(...) ESP_LOGW(__VA_ARGS__)
#define STEP_LOGE(...) ESP_LOGE(__VA_ARGS__)
#else
#define STEP_LOGI(...) while (0)
#define STEP_LOGW(...) while (0)
#define STEP_LOGE(...) ESP_LOGE(__VA_ARGS__)
#endif

DendoStepper::DendoStepper(const DendoStepper_config_t *config)
{
    conf = config;
}

DendoStepper::DendoStepper() {}

void DendoStepper::config(const DendoStepper_config_t *config)
{
    conf = config;
}

void DendoStepper::setEn(bool state)
{
    ESP_ERROR_CHECK(gpio_set_level((gpio_num_t)conf->en_p, state));
}

void DendoStepper::setDir(bool state)
{
    ctrl.dir = state;
    ESP_ERROR_CHECK(gpio_set_level((gpio_num_t)conf->dir_p, state));
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
    gpio_set_level((gpio_num_t)conf->step_p, 1); //step pulse
    //add and substract one step
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
        timer_pause(conf->timer_group, conf->timer_idx);
        ctrl.status = IDLE;
        ctrl.stepCnt = 0;
        return 0;
    }

    if (ctrl.stepCnt % (ctrl.recalcInt+1) == 0)
    {
        if (ctrl.stepCnt <= ctrl.accEnd)
        { //we are currently accelerating
            ctrl.stepInterval -= ctrl.accStepInc;
            ctrl.status = ACC;
        }
        else if (ctrl.stepCnt >= ctrl.coastEnd)
        { //if we are not running infinitely we are done coasting, now decelerating
            ctrl.stepInterval += ctrl.accStepInc;
            ctrl.status = DEC;
        }
        else
        {
            ctrl.status = COAST; //we must be coasting then
        }

        if (ctrl.stepInterval <= 0) //just to be safe, needs to be fixed later
            ctrl.stepInterval = 1000000ULL / ctrl.speed;
    }
    //set alarm to calculated interval
    timer_set_alarm_value(conf->timer_group, conf->timer_idx, ctrl.stepInterval);
    gpio_set_level((gpio_num_t)conf->step_p, 0); //this should be enough for driver to register pulse
    return 0;
}

void DendoStepper::init()
{
    uint64_t mask = (1LL << conf->step_p) | (1LL << conf->dir_p) | (1 << conf->en_p);   //put gpio pins in bitmask
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
        .divider = 2,                       //25 ns resolution
    };
    ESP_ERROR_CHECK(timer_init(conf->timer_group, conf->timer_idx, &timer_conf));   //init the timer
    ESP_ERROR_CHECK(timer_set_counter_value(conf->timer_group, conf->timer_idx, 0)); //set it to 0
    ESP_ERROR_CHECK(timer_isr_callback_add(conf->timer_group, conf->timer_idx, xISRwrap, this, 0)); //add callback fn to run when alarm is triggrd
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
    ESP_ERROR_CHECK(timer_set_alarm_value(conf->timer_group, conf->timer_idx, ctrl.stepInterval));  //set HW timer alarm to stepinterval
    ESP_ERROR_CHECK(timer_start(conf->timer_group, conf->timer_idx));   //start the timer
}

void DendoStepper::setSpeed(uint16_t speed, uint16_t acc)
{
    ctrl.speed = speed;
    ctrl.acc = acc;
}

void DendoStepper::calc(uint16_t speed, uint16_t accTimeMs, uint32_t target)
{
    float acc = 0; //acceleration time
    uint32_t stepsLeft = 0; //steps we need to take
    uint32_t tdS = 0;       //steps displacement during acc/decc
    float accTime = accTimeMs / 1000.0; //ms to s
    
    while (1)
    {
        acc = speed / accTime;          //calculate acceleration
        tdS = acc * accTime * accTime;  //calculate acc and dec steps displacement - number of steps needed for acc/dec

        if (target > tdS) //we will be coasting during movement
        {
            stepsLeft = target - tdS; //calculate how many steps we will be coasting
            break;  //we are done, break out
        }
        else if (target < tdS) //not enough steps, calc triangular params recursively
        {
            speed = sqrt(target * acc);
        }
        else
        {
            //nothing to do e.g recursive triangular param calc is done, break out
            break;
        }
    }
    //save to ctrlVar
    uint32_t dS = tdS * 0.5;    //acc or decc displacement
    ctrl.accEnd = dS;           //in how many steps acceleration phase ends
    ctrl.coastEnd = dS + stepsLeft; //in how many steps coast phase ends
    ctrl.stepsToGo = target;    //how many steps we will be taking
    uint64_t dspow2 = dS * dS;  //used to calculate step interval
    if (dspow2 > ACCTIME_MAX){  //we cant use that acceleration bcs of timer resolution, use smallest possible recalc interval instead (step interval is calculated every x steps)
        ctrl.recalcInt = dspow2 / ACCTIME_MAX;
        ctrl.accStepInc = ACCTIME_MAX / ((dspow2 != (ctrl.recalcInt*ACCTIME_MAX))?(dspow2 % (ctrl.recalcInt*ACCTIME_MAX)):ACCTIME_MAX);
        ctrl.accStepInc=ctrl.accStepInc*ctrl.recalcInt;
    }
    else{   //recalc every step
        ctrl.recalcInt=1;
        ctrl.accStepInc = ACCTIME_MAX / dspow2; //calculate stepinterval differential during acc/dec phase
    }
    ctrl.stepInterval = (TICK_PER_S / ((float)speed)) + (dS * (ctrl.accStepInc/ctrl.recalcInt));    //calculate stepinterval during coast phase
    STEP_LOGI("calc", "tds:%d accend:%d coastend:%d stepstogo:%d dspow:%llu stepinc:%d stepinterval:%d recalcInt:%d", tdS, ctrl.accEnd, ctrl.coastEnd, ctrl.stepsToGo, dspow2, ctrl.accStepInc, ctrl.stepInterval,ctrl.recalcInt);
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

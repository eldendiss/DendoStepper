#include "DendoStepper.h"

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
    if (ctrl.runInfinite)   //do nothing here if we are running infinitely
    {
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
    }
    if (ctrl.stepCnt <= ctrl.accEnd)
    { //we are currently accelerating
        ctrl.stepInterval -= ctrl.accStepInc;
        ctrl.status = ACC;
    }
    else if (ctrl.stepCnt >= ctrl.coastEnd && !ctrl.runInfinite)
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

    //set alarm to calculated interval
    timer_set_alarm_value(conf->timer_group, conf->timer_idx, ctrl.stepInterval);
    gpio_set_level((gpio_num_t)conf->step_p, 0); //this should be enough for driver to register pulse
    return 0;
}

void DendoStepper::init()
{
    uint64_t mask = (1LL << conf->step_p) | (1LL << conf->dir_p) | (1 << conf->en_p);
    gpio_config_t gpio_conf = {
        .pin_bit_mask = mask,
        .mode = GPIO_MODE_OUTPUT,
        .pull_up_en = GPIO_PULLUP_DISABLE,
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
        .intr_type = GPIO_INTR_DISABLE,
    };
    //set outputs
    ESP_ERROR_CHECK(gpio_config(&gpio_conf));

    if (conf->endSw_p != ENDSW_DISABLED)
    {
        mask = (1LL << conf->endSw_p);
        gpio_conf = {
            .pin_bit_mask = mask,
            .mode = GPIO_MODE_INPUT,
            .pull_up_en = GPIO_PULLUP_ENABLE,
            .pull_down_en = GPIO_PULLDOWN_DISABLE,
            .intr_type = GPIO_INTR_ANYEDGE, //we need to fire when we are low
        };
        //set inputs
        ESP_ERROR_CHECK(gpio_config(&gpio_conf));
        gpio_install_isr_service(0); //install gpio interrputs
    }
    timer_config_t timer_conf = {
        .alarm_en = TIMER_ALARM_EN,         //we need alarm
        .counter_en = TIMER_PAUSE,          //dont start now lol
        .intr_type = TIMER_INTR_LEVEL,      //interrupt
        .counter_dir = TIMER_COUNT_UP,      //count up duh
        .auto_reload = TIMER_AUTORELOAD_EN, //reload pls
        .divider = 80,                      //1us resolution
    };
    ESP_ERROR_CHECK(timer_init(conf->timer_group, conf->timer_idx, &timer_conf));
    ESP_ERROR_CHECK(timer_set_counter_value(conf->timer_group, conf->timer_idx, 0)); //set it to 0
    ESP_ERROR_CHECK(timer_isr_callback_add(conf->timer_group, conf->timer_idx, xISRwrap, this, 0));
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
        //if we are moving in the right direction, just add steps to go
        if (ctrl.dir == (relative < 0))
        {
            if (relative > 0)
                relative += ctrl.stepsToGo;
            else
                relative -= ctrl.stepsToGo;
        }
        else
        {
            return; //idk what to do here yet
        }
        stop();
    }
    ctrl.homed = false;          //we are not longer homed
    if (ctrl.status == DISABLED) //if motor is disabled, enable it
        enableMotor();
    setDir(relative < 0); //set CCW if <0, else set CW
    calc(ctrl.speed, ctrl.acc, abs(relative));
    ESP_ERROR_CHECK(timer_set_alarm_value(conf->timer_group, conf->timer_idx, ctrl.stepInterval));
    ESP_ERROR_CHECK(timer_start(conf->timer_group, conf->timer_idx));
}

void DendoStepper::runInf()
{
    if (ctrl.status > IDLE)
        stop();
    ctrl.homed = false;
    ctrl.runInfinite = 1;
    calc(ctrl.speed, ctrl.acc);
    ESP_ERROR_CHECK(timer_set_alarm_value(conf->timer_group, conf->timer_idx, ctrl.stepInterval));
    ESP_ERROR_CHECK(timer_start(conf->timer_group, conf->timer_idx));
    ctrl.status = INF;
}

void DendoStepper::setSpeed(uint16_t speed, uint16_t acc)
{
    ctrl.speed = speed;
    ctrl.acc = acc;
}

void DendoStepper::calc(uint16_t speed, uint16_t accTimeMs, uint32_t target)
{
    float acc = 0; //acceleration time
    uint32_t stepsLeft = 0;
    uint32_t tdS = 0;
    float accTime = accTimeMs / 1000.0; //ms to s
    //uint32_t coasttime=0;
    while (1)
    {
        acc = speed / accTime;
        tdS = acc * accTime * accTime; //acc and dec steps displacement - number of steps needed for acc/dec

        if (target > tdS) //we will be coasting
        {
            stepsLeft = target - tdS; //how many steps we will be coasting
            //coasttime=(stepsLeft/speed);
            break;
        }
        else if (target < tdS) //not enough steps, calc triangular params recursively
        {
            speed = sqrt(target * acc);
        }
        else
        {
            break;
        }
    }
    //save to ctrlVar
    uint32_t dS = tdS * 0.5;
    ctrl.accEnd = dS;
    ctrl.coastEnd = dS + stepsLeft;
    ctrl.stepsToGo = target;
    uint64_t dspow2 = dS * dS;
    if (dspow2 > (accTime * 1000000ULL)) //we cant use that acceleration bcs of timer resolution, use 1
        ctrl.accStepInc = 1;             //TODO: update int every x steps
    else
        ctrl.accStepInc = (accTime * 1000000L) / dspow2;
    ctrl.stepInterval = (1000000ULL / ((float)speed)) + (dS * ctrl.accStepInc);
}

void DendoStepper::calc(uint16_t speed, uint16_t accTimeMs)
{
    float acc = 0; //acceleration time
    uint32_t stepsLeft = 0;
    uint32_t tdS = 0;
    float accTime = accTimeMs / 1000.0; //ms to s

    acc = speed / accTime;
    tdS = acc * accTime; //acc steps displacement - number of steps needed for acc

    //save to ctrlVar
    ctrl.accEnd = tdS;
    uint64_t dspow2 = tdS * tdS;
    if (dspow2 > (accTime * 1000000ULL)) //we cant use that acceleration bcs of timer resolution, use 1
        ctrl.accStepInc = 1;             //TODO: update int every x steps
    else
        ctrl.accStepInc = (accTime * 1000000L) / dspow2;
    ctrl.stepInterval = (1000000ULL / ((float)speed)) + (tdS * ctrl.accStepInc);
}

uint8_t DendoStepper::getState()
{
    return ctrl.status;
}

bool DendoStepper::runAbsolute(uint32_t position)
{
    if (getState() > IDLE)
        stop();
    while (getState() > IDLE)
    {
        //waiting for idle, watchdog should take care of infloop
    }                              //shouldnt take long
    runPos(position - currentPos); //run to new position
    return 1;
}

bool DendoStepper::home(uint16_t speed, uint16_t accTimeMs, bool dir)
{
    if (getState() > IDLE || conf->endSw_p == ENDSW_DISABLED)
        return false; //we cant do this rn, try again later
    if (ctrl.homed)   //we are homed already
        return true;
    setDir(dir);
    gpio_isr_handler_add((gpio_num_t)conf->endSw_p, homeISRwrap, this);
    setSpeed(speed, accTimeMs);
    runInf();
    return true;
}

void DendoStepper::homeISR()
{
    ctrl.stepsToGo = 0;
    timer_pause(conf->timer_group, conf->timer_idx); //stop movement
    currentPos = 0;                                  //we are homed
    ctrl.status = IDLE;
    ctrl.homed = true;                                  //mark that we are homed
    ctrl.runInfinite=0;
    gpio_isr_handler_remove((gpio_num_t)conf->endSw_p); //remove handler so we wont trip it unintentionally more than once
}

uint64_t DendoStepper::getPosition()
{
    return currentPos;
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
    ctrl.stepsToGo = 0; //no need more steps needed, xISR should take care of the rest
    timer_pause(conf->timer_group, conf->timer_idx);
    ctrl.status = IDLE;
    ctrl.stepCnt = 0;
    //ctrl.status=IDLE;
}

bool DendoStepper::isHomed()
{
    return ctrl.homed;
}
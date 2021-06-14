#include "DendoStepper.h"


DendoStepper::DendoStepper(DendoStepper_config_t *config)
{
    conf = config;
}

void DendoStepper::runTask(void *_this)
{
    static_cast<DendoStepper *>(_this)->run();
}

void DendoStepper::run()
{
    while (1)
    {
        /*if(stepTaken){
            ctrl.stepCnt++;
            ctrl.stepsToGo--;
            stepTaken=0;
        }
        if(ctrl.stepsToGo>0){
            //gpio_set_level(GPIO_NUM_14,0);
        }
        else{
            timer_pause(conf->timer_group,conf->timer_idx);
            ESP_LOGI("timer","disabled");
        }*/
        //ESP_LOGI("stepInt","%d",ctrl.stepInterval);
        if (ctrl.stepsToGo == 0)
        {

            ESP_LOGI("step", "%d", ctrl.stepCnt);
        }
        vTaskDelay(20);
    }
}

bool DendoStepper::xISRwrap(void *_this)
{
    return static_cast<DendoStepper *>(_this)->xISR();
}

bool DendoStepper::xISR()
{
    gpio_set_level(GPIO_NUM_14, 1);
    if (ctrl.stepsToGo == 0)
    {
        timer_pause(conf->timer_group, conf->timer_idx);
        return 0;
    }
    if (ctrl.stepCnt <= ctrl.accEnd)
        ctrl.stepInterval -= ctrl.accStepInc;
    else if (ctrl.stepCnt >= ctrl.coastEnd)
        ctrl.stepInterval += ctrl.accStepInc;
    if (ctrl.stepInterval <= 0)
    { //safeguard
        ctrl.stepInterval = ctrl.accStepInc;
    }

    timer_set_alarm_value(conf->timer_group, conf->timer_idx, ctrl.stepInterval);
    ctrl.stepCnt++;
    ctrl.stepsToGo--;
    gpio_set_level(GPIO_NUM_14, 0); //this should be enough for driver to register pulse
    return 0;
}

void DendoStepper::init(TaskHandle_t *h)
{
    uint64_t mask = (1 << conf->step_p) | (1 << conf->dir_p);
    gpio_config_t gpio_conf = {
        .pin_bit_mask = mask,
        .mode = GPIO_MODE_OUTPUT,
        .pull_up_en = GPIO_PULLUP_DISABLE,
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
        .intr_type = GPIO_INTR_DISABLE,
    };
    gpio_config(&gpio_conf);
    ESP_LOGI("gpio", "configured");

    timer_config_t timer_conf = {
        .alarm_en = TIMER_ALARM_EN,         //we do not need alarm
        .counter_en = TIMER_PAUSE,          //start right away YOLO
        .intr_type = TIMER_INTR_LEVEL,      //interrupt is overrated
        .counter_dir = TIMER_COUNT_UP,      //count up duh
        .auto_reload = TIMER_AUTORELOAD_EN, //whatever
        .divider = 80,                      //1us resolution
    };
    ESP_ERROR_CHECK(timer_init(conf->timer_group, conf->timer_idx, &timer_conf));
    timer_set_counter_value(conf->timer_group, conf->timer_idx, 0); //set it to 0
    timer_isr_callback_add(conf->timer_group, conf->timer_idx, xISRwrap, this, 0);
    ESP_LOGI("timer", "configured");
}

void DendoStepper::runPos(int32_t relative)
{
    calc(ctrl.speed, ctrl.acc, relative);
    timer_set_alarm_value(conf->timer_group, conf->timer_idx, ctrl.stepInterval);
    timer_start(conf->timer_group, conf->timer_idx);
    ESP_LOGI("alarm", "set %d", ctrl.stepInterval);
    xTaskCreate(runTask, "DendoStepper", 4096, this, 5, NULL);
}

void DendoStepper::setSpeed(uint16_t speed, uint16_t acc)
{
    ctrl.speed = speed;
    ctrl.acc = acc;
    ESP_LOGI("speed", "set");
}

void DendoStepper::calc(uint16_t speed, uint16_t accTimeMs, uint32_t target)
{
    float acc = 0; //acceleration time
    uint32_t stepsLeft = 0;
    uint32_t tdS = 0;
    float accTime = accTimeMs / 1000.0; //ms to s
    while (1)
    {
        acc = speed / accTime;
        tdS = acc * accTime * accTime; //acc and dec steps displacement - number of steps needed for acc/dec

        if (target > tdS) //we will be coasting
        {
            stepsLeft = target - tdS; //how many steps we will be coasting
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
    ctrl.accStepInc = (accTime*1000ULL) / dS;
    ctrl.stepInterval = ((stepsLeft) / (float)speed) + (dS * ctrl.accStepInc);
    ESP_LOGI("calcprint", "accTime=%d tds=%u accend=%d coastend=%d stepstogo=%d accstepinc=%d stepint=%d stepsleft=%d", accTimeMs, tdS, ctrl.accEnd, ctrl.coastEnd, ctrl.stepsToGo, ctrl.accStepInc, ctrl.stepInterval, stepsLeft);
}
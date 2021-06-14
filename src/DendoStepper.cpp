#include "DendoStepper.h"

static bool stepTaken = 0;
static uint32_t cnt = 0;

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
            timer_pause(TIMER_GROUP_0,TIMER_0);
            ESP_LOGI("timer","disabled");
        }*/
        //ESP_LOGI("stepInt","%d",ctrl.stepInterval);
        if (ctrl.stepsToGo == 0)
        {

            ESP_LOGI("step", "done");
            stepTaken = 0;
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
        timer_pause(TIMER_GROUP_0, TIMER_0);
        return 0;
    }
    if(ctrl.stepCnt<=ctrl.accEnd){
        ctrl.stepInterval-=ctrl.accStepInc;
        timer_set_alarm_value(TIMER_GROUP_0,TIMER_0,ctrl.stepInterval);
    } else if(ctrl.stepCnt>=ctrl.coastEnd){
        ctrl.stepInterval+=ctrl.accStepInc;
        timer_set_alarm_value(TIMER_GROUP_0,TIMER_0,ctrl.stepInterval);
    }
    ctrl.stepCnt++;
    ctrl.stepsToGo--;
    gpio_set_level(GPIO_NUM_14, 0); //this should be enough for driver to register pulse
    return 0;
}

void DendoStepper::yISR(void *p)
{
}

void DendoStepper::init(TaskHandle_t *h)
{
    uint64_t mask = (1 << conf->X.step_p);
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
        .divider = 80,                      //1ms resolution
    };
    ESP_ERROR_CHECK(timer_init(TIMER_GROUP_0, TIMER_0, &timer_conf));
    timer_set_counter_value(TIMER_GROUP_0, TIMER_0, 0); //set it to 0
    timer_isr_callback_add(TIMER_GROUP_0, TIMER_0, xISRwrap, this, 0);
    ESP_LOGI("timer", "configured");
}

void DendoStepper::runPos(int32_t relative)
{
    calc(ctrl.speed, ctrl.acc, relative);
    timer_set_alarm_value(TIMER_GROUP_0, TIMER_0, ctrl.stepInterval);
    timer_start(TIMER_GROUP_0, TIMER_0);
    ESP_LOGI("alarm", "set %d", ctrl.stepInterval);
    xTaskCreate(runTask, "DendoStepper", 4096, this, 5, NULL);
}

void DendoStepper::setSpeed(uint16_t speed,uint16_t acc)
{
    ctrl.speed=speed;
    ctrl.acc=acc;
    ESP_LOGI("speed", "set");
}

void DendoStepper::calc(uint16_t speed, uint16_t acc, uint32_t target)
{
    float accTime =0;                       //acceleration time
    float coastTime=0;
    uint32_t stepsLeft=0;
    uint32_t tdS=0;
    while (1)
    {
        accTime=speed / acc;
        tdS = acc * (accTime * accTime); //acc and dec steps displacement

        if (target > tdS)   //we will be coasting
        {
            stepsLeft = target - tdS;       //how many steps we will be coasting
            coastTime = stepsLeft/speed;    //how long we will be coasting
            break;  //we are done break out of loop
        }
        else if(target==tdS){   //triangular move
            break;  //we are done, end loop
        }
        else if (target < tdS)  //not enough steps, recalculate acctime
        {
            speed=acc*2;
            /*speed = sqrt(tdS * acc);
            acc=*/
            ESP_LOGI("calc","recalculation");
            //go back and recalculate with new acctime
        }
    }
    //save to ctrlVar
    uint32_t dS=tdS*0.5;
    ctrl.accEnd=dS;
    ctrl.coastEnd=dS+stepsLeft;
    ctrl.stepsToGo=target;
    ctrl.accStepInc=(accTime*1000ULL)/dS;
    ctrl.stepInterval=((stepsLeft/speed)*1000ULL)+(dS*ctrl.accStepInc);
}
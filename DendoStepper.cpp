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

void DendoStepper::setEn(bool state)
{
    ESP_ERROR_CHECK(gpio_set_level((gpio_num_t)conf.en_p, state));
}

void DendoStepper::setDir(bool state)
{
    //ctrl.dir = state;
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
    timerStarted=0;
}

/* Timer callback, used for generating pulses and calculating speed profile in real time */
bool DendoStepper::xISR()
{
    gpio_set_level((gpio_num_t)conf.step_p, (state=!state)); //step pulse
    //add and substract one step
    if(state==0)
        return 0; //just turn off the pin in this iteration
    
    ctrl.stepCnt++;
    ctrl.stepsToGo--;
    //absolute coord handling
    if (ctrl.dir == CW)
        currentPos++;
    else if (currentPos > 0)
        currentPos--; //we cant go below 0, or var will overflow
    
    //we are done
    if (ctrl.stepsToGo == 0)
    {
        timer_pause(conf.timer_group, conf.timer_idx);  //stop the timer
        ctrl.status = IDLE;
        ctrl.stepCnt = 0;
        gpio_set_level((gpio_num_t)conf.step_p, 0); //this should be enough for driver to register pulse
        return 0;
    }
    
    if(ctrl.accelC >0 && ctrl.accelC<ctrl.accEnd){   //we are accelerating
        uint32_t oldInt=ctrl.stepInterval;
        ctrl.stepInterval=oldInt-(2*oldInt+ctrl.rest)/(4*ctrl.accelC+1);
        ctrl.rest=(2*oldInt+ctrl.rest)%(4*ctrl.accelC+1);
        ctrl.accelC++;
        ctrl.status=ACC;    //we are accelerating, note that

    } else if(ctrl.stepCnt>ctrl.coastEnd){  //we must be deccelerating then
        uint32_t oldInt=ctrl.stepInterval;
        ctrl.stepInterval=(int32_t)oldInt-((2*(int32_t)oldInt+(int32_t)ctrl.rest)/(4*ctrl.accelC+1));
        ctrl.rest=(2*(int32_t)oldInt+(int32_t)ctrl.rest)%(4*ctrl.accelC+1);
        ctrl.accelC++;
        ctrl.status=DEC;   //we are deccelerating

    } else { //we are coasting
        ctrl.status=COAST;  //we are coasting
        ctrl.accelC=(int32_t)ctrl.coastEnd*-1;

    }
    
    //set alarm to calculated interval
    timer_set_alarm_value(conf.timer_group, conf.timer_idx, ctrl.stepInterval/2);
    return 1;
}

void DendoStepper::enTimer(){
    while(1){
        if(!timerStarted && ctrl.status==IDLE){
            //create the en timer
            esp_timer_create_args_t t_arg={
                .callback=_disableMotor,
                .arg=this,
                .dispatch_method=ESP_TIMER_TASK,
                .name="En timer",
                .skip_unhandled_events=1,
            };
            esp_timer_create(&t_arg,&dyingTimer);
            esp_timer_start_once(dyingTimer,ctrl.enOffTime);
            timerStarted=1;
        } else if(ctrl.status>IDLE){
            esp_timer_delete(dyingTimer);
        }
        vTaskDelay(100);
    }
}

void DendoStepper::init(uint8_t stepP,uint8_t dirP,uint8_t enP,timer_group_t group,timer_idx_t index,microStepping_t microstepping=MICROSTEP_1,uint16_t stepsPerRot=200)
{
    conf.step_p=stepP;
    conf.dir_p=dirP;
    conf.en_p=enP;
    conf.timer_group=group;
    conf.timer_idx=index;
    conf.miStep=microstepping;
    ctrl.status=0;
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

esp_err_t DendoStepper::runPos(int32_t relative)
{
    if (!relative) //why would u call it with 0 wtf
        return ESP_ERR_NOT_SUPPORTED;
    if (ctrl.status > IDLE)
    { //we are running, we need to adjust steps accordingly, for now just stop the movement
        STEP_LOGW("DendoStepper","Finising previous move, this command will be ignored");
        return ESP_ERR_NOT_SUPPORTED;
    }
    ctrl.homed = false;          //we are not longer homed
    if (ctrl.status == DISABLED) //if motor is disabled, enable it
        enableMotor();              
    setDir(relative < 0); //set CCW if <0, else set CW
    calc(ctrl.speed, ctrl.acc, abs(relative));  //calculate velocity profile
    ESP_ERROR_CHECK(timer_set_alarm_value(conf.timer_group, conf.timer_idx, ctrl.stepInterval));  //set HW timer alarm to stepinterval
    ESP_ERROR_CHECK(timer_start(conf.timer_group, conf.timer_idx));   //start the timer
    return ESP_OK;
}

void DendoStepper::setSpeed(uint32_t speed, uint16_t accT)
{
    ctrl.speed = speed/(200.0*(float)conf.miStep);
    ctrl.acc = ctrl.speed/(accT/1000.0);
    STEP_LOGI("DendoStepper","Speed set: %f %f",ctrl.speed,ctrl.acc);
}

void DendoStepper::setSpeedRad(float speed, float acc){
    ctrl.speed = speed;
    ctrl.acc=acc;
    STEP_LOGI("DendoStepper","Speed set: %f %f",ctrl.speed,ctrl.acc);
}

void DendoStepper::calc(uint16_t speed, uint16_t accTimeMs, uint32_t target)
{

    //calculate number of steps needed for acceleration
    ctrl.accEnd=(ctrl.speed*ctrl.speed)/(2.0*0.0005*ctrl.acc);
    //calculate the limit value of steps needed for acceleration
    //(used if we dont have enough steps to perform full acc to max speed)
    ctrl.accLim= (target* ctrl.acc)/(ctrl.acc*2);

    if(ctrl.accEnd<ctrl.accLim){   //acceleration is limited by max speed
        ctrl.coastEnd = target-ctrl.accEnd; //calculate when to start deccelerating in steps 
    } else {                        //acceleration is limited by start of the deceleration (triangular profile)
        ctrl.coastEnd=ctrl.accEnd=target-ctrl.accLim; //no coast phase, we are limited by number of steps
    }
    //init vars
    ctrl.accelC=1;
    ctrl.rest=0;
    
    //calculate initial interval, also known as c0
    ctrl.stepInterval = (float)TIMER_F*sqrt((((4*3.14)/(200.0*(float)conf.miStep))/ctrl.acc));
    
    //set steps we will take 
    ctrl.stepsToGo=target;
    //debug
    STEP_LOGI("calc", "acc end:%u coastend:%u acclim:%u stepstogo:%u speed:%f acc:%f int: %u", ctrl.accEnd, ctrl.coastEnd, ctrl.accLim, ctrl.stepsToGo, ctrl.speed, ctrl.acc,ctrl.stepInterval);
    STEP_LOGI("calc","int: %u rest %u",ctrl.stepInterval,ctrl.rest);
    //init old interval
    uint32_t oldInt=ctrl.stepInterval;
    
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
    ctrl.stepsToGo = 0; //no more steps needed, xISR should take care of the rest
    //todo: deccelerate
}

void DendoStepper::setEnTimeout(uint64_t timeout){
    if(timeout==0){
        vTaskDelete(enTask);
    }
    ctrl.enOffTime=timeout;
    xTaskCreate(enTimerWrap,"En timer task",2048,this,7,&enTask);
}


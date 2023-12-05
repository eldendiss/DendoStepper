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

bool state = 0;

// PUBLIC definitions

DendoStepper::DendoStepper()
{
}

void DendoStepper::config(DendoStepper_config_t *config)
{
    memcpy(&conf, config, sizeof(conf));
}

void DendoStepper::disableMotor()
{
    setEn(true);
    STEP_LOGI("DendoStepper", "Disabled");
    ctrl.status = DISABLED;
}

void DendoStepper::enableMotor()
{
    setEn(false);
    ctrl.status = IDLE;
    STEP_LOGI("DendoStepper", "Enabled");
    timerStarted = 0;
}

void DendoStepper::init(uint8_t stepP, uint8_t dirP, uint8_t enP, microStepping_t microstepping = MICROSTEP_1, uint16_t stepsPerRot = 200)
{
    conf.stepPin = stepP;
    conf.dirPin = dirP;
    conf.enPin = enP;
    conf.miStep = microstepping;
    ctrl.status = 0;
    init();
}

void DendoStepper::init()
{
    uint64_t mask = (1ULL << conf.stepPin) | (1ULL << conf.dirPin) | (1ULL << conf.enPin); // put output gpio pins in bitmask
    gpio_config_t gpio_conf = {
        // config gpios
        .pin_bit_mask = mask,
        .mode = GPIO_MODE_OUTPUT,
        .pull_up_en = GPIO_PULLUP_DISABLE,
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
        .intr_type = GPIO_INTR_DISABLE,
    };
    // set the gpios as per gpio_conf
    ESP_ERROR_CHECK(gpio_config(&gpio_conf));

    gptimer_config_t timer_conf = {
        .clk_src = GPTIMER_CLK_SRC_DEFAULT,
        .direction = GPTIMER_COUNT_UP,
        .resolution_hz = TIMER_F,
    };

    timer_conf.flags.intr_shared = false;

    // calculate stepsPerRot
    ctrl.stepsPerRot = (360.0 / conf.stepAngle) * conf.miStep;

    STEP_LOGI("DendoStepper", "Steps per one rotation:%d", ctrl.stepsPerRot);

    ESP_ERROR_CHECK(gptimer_new_timer(&timer_conf, &timer_handle));

    gptimer_event_callbacks_t cb_group;
    cb_group.on_alarm = xISRwrap;
    alarm_cfg.flags.auto_reload_on_alarm = 1;
    gptimer_register_event_callbacks(timer_handle, &cb_group, this);
}

esp_err_t DendoStepper::runPos(int32_t relative)
{
    if (!relative) // why would u call it with 0 wtf
        return ESP_ERR_NOT_SUPPORTED;
    if (ctrl.status > IDLE)
    { // we are running, we need to adjust steps accordingly, for now just stop the movement
        STEP_LOGW("DendoStepper", "Finising previous move, this command will be ignored");
        return ESP_ERR_NOT_SUPPORTED;
    }

    if (ctrl.status == DISABLED) // if motor is disabled, enable it
        enableMotor();
    ctrl.status = ACC;
    setDir(relative < 0); // set CCW if <0, else set CW
    currentPos += relative;
    calc(abs(relative)); // calculate velocity profile

    alarm_cfg.alarm_count = ctrl.stepInterval;
    gptimer_set_alarm_action(timer_handle, &alarm_cfg);
    gptimer_enable(timer_handle);
    return gptimer_start(timer_handle);
}

esp_err_t DendoStepper::runPosMm(int32_t relative)
{
    if (ctrl.stepsPerMm == 0)
    {
        STEP_LOGE("DendoStepper", "Steps per millimeter not set, cannot move!");
    }
    return runPos(relative * ctrl.stepsPerMm);
}

esp_err_t DendoStepper::runAbs(uint32_t position)
{
    if (getState() > IDLE) // we are already moving, so stop it
        stop();
    while (getState() > IDLE)
    {
        // waiting for idle, watchdog should take care of inf loop if it occurs
        vTaskDelay(1);
    } // shouldnt take long tho

    if (position == currentPos) // we cant go anywhere
        return 0;

    int32_t relativeSteps = 0;
    relativeSteps = (int32_t)position - currentPos;

    ESP_LOGI("DendoStepper", "Cur: %llu move %lu", currentPos, relativeSteps);
    return runPos(relativeSteps); // run to new position
}

esp_err_t DendoStepper::runAbsMm(uint32_t position)
{
    if (ctrl.stepsPerMm == 0)
    {
        STEP_LOGE("DendoStepper", "Steps per millimeter not set, cannot move!");
    }
    return runAbs(position * ctrl.stepsPerMm);
}

void DendoStepper::setSpeed(uint32_t speed, uint16_t accT, uint16_t decT)
{
    ctrl.speed = speed;
    ctrl.acc = ctrl.speed / (accT / 4000.0);
    ctrl.dec = ctrl.speed / (decT / 4000.0);
    STEP_LOGI("DendoStepper", "Speed set: %lu steps/s t+=%d s t-=%d s", speed, accT, decT);
}

void DendoStepper::setSpeedMm(uint32_t speed, uint16_t accT, uint16_t decT)
{
    if (ctrl.stepsPerMm == 0)
    {
        STEP_LOGE("DendoStepper", "Steps per millimeter not set, cannot set the speed!");
    }
    ctrl.speed = speed * ctrl.stepsPerMm;

    ctrl.acc = ctrl.speed / (accT / 4000.0);
    ctrl.dec = ctrl.speed / (decT / 4000.0);
    STEP_LOGI("DendoStepper", "Speed set: v=%lu mm/s t+=%d s t-=%d s", speed, accT, decT);
}

void DendoStepper::setStepsPerMm(uint16_t steps)
{
    ctrl.stepsPerMm = steps;
}

uint16_t DendoStepper::getStepsPerMm()
{
    return ctrl.stepsPerMm;
}

uint8_t DendoStepper::getState()
{
    return ctrl.status;
}

uint64_t DendoStepper::getPosition()
{
    return currentPos;
}

uint64_t DendoStepper::getPositionMm()
{
    return getPosition() / ctrl.stepsPerMm;
}

void DendoStepper::resetAbsolute()
{
    currentPos = 0;
}

void DendoStepper::runInf(bool direction)
{
    if (ctrl.status > IDLE)
    {
        STEP_LOGE("DendoStepper", "Motor is moving, this command will be ignored");
        return;
    }
    if (ctrl.status == DISABLED)
    {
        enableMotor();
    }
    ctrl.runInfinite = true;
    setDir(direction);
    calc(UINT32_MAX);
    alarm_cfg.alarm_count = ctrl.stepInterval;
    gptimer_set_alarm_action(timer_handle, &alarm_cfg);
    gptimer_enable(timer_handle);
    gptimer_start(timer_handle);
}

uint16_t DendoStepper::getSpeed()
{
    return ctrl.speed;
}

uint16_t DendoStepper::getTargetSpeed()
{
    return ctrl.targetSpeed;
}

float DendoStepper::getAcc()
{
    return ctrl.acc;
}

float DendoStepper::getDec()
{
    return ctrl.dec;
}

void DendoStepper::stop()
{
    if (ctrl.status <= IDLE)
    {
        return;
    }
    ctrl.runInfinite = false;
    gptimer_stop(timer_handle); // stop the timer
    ctrl.status = IDLE;
    ctrl.stepCnt = 0;
    gptimer_disable(timer_handle);
    gpio_set_level((gpio_num_t)conf.stepPin, 0);
}

// PRIVATE definitions

void DendoStepper::setEn(bool state)
{
    ESP_ERROR_CHECK(gpio_set_level((gpio_num_t)conf.enPin, state));
}

void DendoStepper::setDir(bool state)
{
    ctrl.dir = state;
    ESP_ERROR_CHECK(gpio_set_level((gpio_num_t)conf.dirPin, state));
}

/* Timer callback, used for generating pulses and calculating speed profile in real time */
bool DendoStepper::xISR(gptimer_t *timer, const gptimer_alarm_event_data_t *data)
{
    GPIO.out_w1ts = (1ULL << conf.stepPin);
    // add and substract one step

    ctrl.stepCnt++;

    // we are done
    if (ctrl.stepsToGo == ctrl.stepCnt && !ctrl.runInfinite)
    {
        gptimer_stop(timer_handle); // stop the timer
        ctrl.status = IDLE;
        ctrl.stepCnt = 0;
        gptimer_disable(timer_handle);
        GPIO.out_w1tc = (1ULL << conf.stepPin);
        return 0;
    }

    if (ctrl.stepCnt > 0 && ctrl.stepCnt < ctrl.accEnd)
    { // we are accelerating
        ctrl.currentSpeed += ctrl.accInc;
        ctrl.status = ACC; // we are accelerating, note that
    }
    else if (ctrl.stepCnt > ctrl.coastEnd && !ctrl.runInfinite)
    { // we must be deccelerating then
        ctrl.currentSpeed -= ctrl.decInc;
        ctrl.status = DEC; // we are deccelerating
    }
    else
    {
        ctrl.currentSpeed = ctrl.targetSpeed;
        ctrl.status = COAST; // we are coasting
    }

    ctrl.stepInterval = TIMER_F / ctrl.currentSpeed;
    // set alarm to calculated interval and disable pin
    GPIO.out_w1tc = (1ULL << conf.stepPin);
    alarm_cfg.alarm_count = ctrl.stepInterval;
    gptimer_set_alarm_action(timer_handle, &alarm_cfg);
    return 1;
}

void DendoStepper::calc(uint32_t targetSteps)
{

    ctrl.accSteps = 0.5 * ctrl.acc * (ctrl.speed / ctrl.acc) * (ctrl.speed / ctrl.acc);

    ctrl.decSteps = 0.5 * ctrl.dec * (ctrl.speed / ctrl.dec) * (ctrl.speed / ctrl.dec);

    if (targetSteps < (ctrl.decSteps + ctrl.accSteps))
    {
        ESP_LOGI("Dendostepper", "Computing new speed");

        ctrl.speed = sqrt(2 * targetSteps * ((ctrl.dec * ctrl.acc) / (ctrl.dec + ctrl.acc)));
        ctrl.accSteps = 0.5 * ctrl.acc * (ctrl.speed / ctrl.acc) * (ctrl.speed / ctrl.acc);
        ctrl.decSteps = 0.5 * ctrl.dec * (ctrl.speed / ctrl.dec) * (ctrl.speed / ctrl.dec);
    }

    ctrl.accEnd = ctrl.accSteps;
    ctrl.coastEnd = targetSteps - ctrl.decSteps;
    ctrl.targetSpeed = ctrl.speed;

    ctrl.accInc = ctrl.targetSpeed / (double)ctrl.accSteps;
    ctrl.decInc = ctrl.targetSpeed / (double)ctrl.decSteps;

    ctrl.currentSpeed = ctrl.accInc;

    ctrl.stepInterval = TIMER_F / ctrl.currentSpeed;
    ctrl.stepsToGo = targetSteps;

    STEP_LOGI("calc", "acc end:%lu coastend:%lu stepstogo:%lu speed:%f acc:%f int: %lu", ctrl.accEnd, ctrl.coastEnd, ctrl.stepsToGo, ctrl.speed, ctrl.acc, ctrl.stepInterval);
}

# DendoStepper

#### This docs are outdated, new ones will be ready soon

Work in progress, maybe unstable.  
This library takes care of pulse generating for stepper motor drivers with STEP/DIR interface. Pulse generating utilizes general purpose timers to achieve some usable accuracy and smoothness.  
Currently supports only linear acceleration and deceleration, exponential profile is TBD.

### Known limitations
- maximum number of controlled stepper motors is 4, this is limited by number of general purpose timers

## Usage

```c++
typedef struct
{
    uint8_t         step_p;         //step signal gpio
    uint8_t         dir_p;          //dir signal gpio
    uint8_t         en_p;           //enable signal gpio
    timer_group_t   timer_group;    //timer group, useful if we are controlling more than 2 steppers
    timer_idx_t     timer_idx;      //timer index, useful if we are controlling 2steppers
} DendoStepper_config_t;

DendoStepper(DendoStepper_config_t*);
```  
Constructor - accepts DendoStepper_config_t struct as parameter. This struct needs to be initialized before calling constructor.  
timer_group and timer_idx are used to assign different timers to different instances.  

```c++
void init();
```  
Initializes GPIO and Timer peripherals, registers ISR. Expects populated config struct is alreay passed to the object  

```c++
void config(const DendoStepper_config_t* config);
```
Same as init, but you can pass populated config struct if you didn't pass it in constructor.  

```c++
void setSpeed(uint16_t speed,uint16_t accTimeMs);
```
Sets maximum speed in steps per second and acceleration time in milliseconds.  

```c++
void runPos(int32_t relative);
```
Runs motor to position relative from current position, respecting constraints set with setSpeed()

```c++
void disableMotor();
void enableMotor();
```
Disables and enables motor via EN pin

```c++
uint8_t getState();

enum motor_status{
    DISABLED,
    IDLE,
    ACC,
    COAST,
    DEC,
};
```
Returns current state of motor, return type is enum motor_status  

```c++
 bool runAbsolute(uint32_t position);
```
Runs motor in absolute coordinate plane(should be constrained with home switch)  

```c++
void resetAbsolute();
```
Resets absolute position to 0. Called for ex. when endswitch is hit.

```c++
uint16_t getSpeed();
```
Returns currently set motor speed

```c++
uint16_t getAcc();
```
Returns currently set acceleration time in ms

```c++
void stop();
```
Stops the motor dead on the spot. Eg. e-stop.



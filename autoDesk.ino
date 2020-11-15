#include "TeensyStep.h"

#define PR_DEBUG 0
/*
 * #define PR_DEBUG 1
 * Sketch uses 24760 bytes (9%) of program storage space. Maximum is 262144 bytes.
 * Global variables use 4576 bytes (6%) of dynamic memory, leaving 60960 bytes for local variables. Maximum is 65536 bytes.
 * 
 * #define PR_DEBUG 0
 * Sketch uses 22008 bytes (8%) of program storage space. Maximum is 262144 bytes.
 * Global variables use 4552 bytes (6%) of dynamic memory, leaving 60984 bytes for local variables. Maximum is 65536 bytes.
 * 
 */
#if PR_DEBUG == 1
#define log(x) Serial.print(x)
#define logl(x) Serial.println(x)
#else
#define log(x)
#define logl(x)
#endif

#define up true // this is just to make the code slighly more verbose in one part
#define down false

// stepper  and controller=
constexpr byte stpPin = 2, dirPin = 3;
Stepper desk(stpPin, dirPin);
StepControl controller;

// position defaults
long defaultMaxHeight = 2147483647L;
long maxHeight = defaultMaxHeight; // might need to add promis or hard code a start.
long maxSafePos; // change to heighten top height
int lowestSafePos = 2500; // change to lowestSafePos
int startPos = 5000;
unsigned short safetyReversePos = 10000; 
long upPos;
long downPos = startPos;
bool pos = down;
long currentPos;


long calibrateDestination = -defaultMaxHeight;
bool calibrating = false;
bool waitingForButtonPress = true;
bool calibrationStartUp = true;

// motor settings
constexpr int normalSpeed = 19000;
constexpr int calibrationSpeed = 10000;
constexpr int destinationSpeed = 22500;
constexpr int acceleration = 30000;

// limit switches
constexpr byte limitSwitch = 5;

// buttons
constexpr byte numOfButtons = 3;
constexpr byte buttonPins[numOfButtons] = {7,8,9};
byte buttonState[numOfButtons];
byte lastButtonState[numOfButtons];
   
// millis() variables
elapsedMillis currentMillis;
unsigned long lastMillis[numOfButtons];

// button hold and toggle
unsigned long buttonHold;
bool buttonHolding = false;

void handleButtons();
void checkLimits();
void calibrate();
void motorControl();

void setup() {     
    // serial setup
    #if PR_DEBUG == 1
    Serial.begin(9600);
    while(!Serial);
    #endif
    logl("program has started");
    
    // limit switch and button setup
    pinMode(limitSwitch, INPUT_PULLUP);
    for(int i = 0; i < numOfButtons; i++) {
        pinMode(buttonPins[i], INPUT_PULLUP);
        buttonState[i] = LOW;
        lastButtonState[i] = LOW;
        lastMillis[i] = currentMillis;
    }
    
    // motor parameters
    desk.setMaxSpeed(normalSpeed);
    desk.setAcceleration(acceleration);
    // wait for initial button press to calibrate
    while(waitingForButtonPress) {
        handleButtons();
    }
    calibrate();
}

void loop() {
    handleButtons();
}

void handleButtons() {
    // handles button press and debounce
    for(int i = 0; i < numOfButtons; i++) {
        checkLimits();
        // check if button is pressed
        byte reading = digitalReadFast(buttonPins[i]);
        // button is pressed
        if(reading == LOW && currentMillis - 500L > lastMillis[i]) { // reduce the number ending in L to increase sensativity but MUST balance debounce
            lastMillis[i] = currentMillis;
            buttonState[i] = HIGH;
            if(waitingForButtonPress) {
                waitingForButtonPress = false;
                break;
            }
            // button 2 toggles a state and has a hold
            if(i == 2) { // this is the high state
                if(currentMillis - 2000L > buttonHold && buttonHolding == true && currentMillis < buttonHold + 2500L) { 
                    log("buttonHold: ");
                    logl(buttonHold);
                    motorControl(i);
                }
            }
            logl("in button press");
        } else if(reading == HIGH) {
            buttonState[i] = LOW;
        }
    }
    // handles button state
    for(int i = 0; i < numOfButtons; i++) {
        if(buttonState[i] != lastButtonState[i]) {
            if(buttonState[i] == HIGH) { // this is when button is pressed
                if(i == 2) { // this is the toggle of button two
                    buttonHold = currentMillis; 
                    buttonHolding = true;
                } else { // all other buttons
                    motorControl(i);
                    log("Button: ");
                    log(i);
                    logl(" pressed");
               }
            } else if(buttonState[i] == LOW) { // button released
                if(i == 2){
                    buttonHolding = false;
                    if(currentMillis - 2000L > buttonHold) { // when held button is released stores new up/down position
                        if(pos == up && currentPos > downPos) {
                            upPos = desk.getPosition();
                            log("new upPos: ");
                            logl(upPos);
                        } else if(pos == up && currentPos < downPos) {
                            downPos = desk.getPosition();
                            log("new downPos: ");
                            logl(downPos);
                            pos = down;
                        }else if(pos == down && currentPos < upPos) {
                            downPos = desk.getPosition();
                            log("new downPos: ");
                            logl(downPos);
                        } else if(pos == down && currentPos > upPos) {
                            upPos = desk.getPosition();
                            log("new upPos: ");
                            logl(upPos);
                            pos = up;
                        }
                    } else { // button just pressed
                        if(pos == up) {
                            motorControl('d');
                            pos = down;
                        } else if (pos == down) {
                            motorControl('u');
                            pos = up;
                        }
                        log("Button: ");
                        log(i);
                        logl(" released");
                    }
                } else { // other buttons release state
                    motorControl('s');
                    log("Button: ");
                    log(i);
                    logl(" released");
                }
            }
            lastButtonState[i] = buttonState[i];
        }
    }
}

void checkLimits() {
    if(controller.isRunning()) {
        byte reading = digitalReadFast(limitSwitch);
        long deskHeight = desk.getPosition();
        if(reading == HIGH || deskHeight >= maxHeight) {
            controller.emergencyStop();
            while(controller.isRunning()) {
                logl("Waiting for the motor to stop.");
                delay(1);
            }
            if(calibrating) {
                logl("limit switch reached while calibrating");
            } else {
                if(reading == HIGH) {
                    logl("limit switch triggered. Emergency stop imminent!");
                    logl("Recalibration initiated!");
                    if(!controller.isRunning()) { // safety reverse
                        desk.setTargetRel(safetyReversePos);
                        controller.moveAsync(desk);
                        while(controller.isRunning()) {
                            logl("Waiting for the motor to stop.");
                            delay(1);
                        }
                    } else {
                        logl("Safety reverse and reset start position didn't happen!");
                    }
                } else if(deskHeight >= maxHeight) {
                    logl("Desk is too High. Emergency stop imminent!");
                    logl("Recalibration initiated!");
                    if(!controller.isRunning()) { // safety reverse
                        desk.setTargetRel(-safetyReversePos);
                        controller.moveAsync(desk);
                        while(controller.isRunning()) {
                            logl("Waiting for the motor to stop.");
                            delay(1);
                        }
                    } else {
                        logl("Safety reverse and reset start position didn't happen!");
                    }
                }
                calibrate();
            }
        }
    }
}

void calibrate() {
    calibrating = true;
    desk.setMaxSpeed(calibrationSpeed);
    desk.setPosition(0);
    logl("calibrating");
    desk.setTargetRel(calibrateDestination);
    controller.moveAsync(desk);
    while(controller.isRunning()) {
        checkLimits();
    }
    if(calibrationStartUp) { // initialize setup variables here
        maxHeight = abs(desk.getPosition()); 
        maxSafePos = maxHeight - 5000L;
        upPos = maxSafePos;
        log("maxHeight: ");
        logl(maxHeight);
        calibrationStartUp = false;
    }
    desk.setTargetRel(startPos);
    controller.moveAsync(desk);
    while(controller.isRunning()) {
        logl("Waiting for the motor to stop.");
        delay(1);
    }
    desk.setPosition(startPos);
    calibrating = false;
}

void motorControl(char control) {
    switch(control) {
        case 0: // up
            if(!controller.isRunning()) {
                desk.setMaxSpeed(normalSpeed);
                desk.setTargetAbs(maxSafePos);
                controller.moveAsync(desk);
                logl("desk is going up");
                log("desk position: ");
                logl(desk.getPosition());
            } else {
                logl("controller is running");
            }
            break;
        case 1: // down
            if(!controller.isRunning()) {
                desk.setMaxSpeed(normalSpeed);
                desk.setTargetAbs(lowestSafePos);
                controller.moveAsync(desk);
                logl("desk is going down");
                log("desk position: ");
                logl(desk.getPosition());
            } else {
                logl("controller is running");
            }
            break;
        case 2:
            currentPos = desk.getPosition();
            desk.setMaxSpeed(normalSpeed);
            if(currentPos >= maxSafePos / 2) {
                if(!controller.isRunning()) {
                    desk.setTargetRel(-50);
                    controller.moveAsync(desk);
                    while(controller.isRunning()) {
                        delay(1);
                    }
                    desk.setTargetRel(50);
                    controller.moveAsync(desk);
                    while(controller.isRunning()) {
                        delay(1);
                    }
                } else {
                    logl("tactile response didn't work!");
                }
            } else {
                if(!controller.isRunning()) {
                    desk.setTargetRel(50);
                    controller.moveAsync(desk);
                    while(controller.isRunning()) {
                        delay(1);
                    }
                    desk.setTargetRel(-50);
                    controller.moveAsync(desk);
                    while(controller.isRunning()) {
                        delay(1);
                    }
                } else {
                    logl("tactile response didn't work!");
                }
            }
            break;
        case 's':
            controller.stopAsync();
            while(controller.isRunning()){
                logl("Waiting for the motor to stop.");
                delay(1);
            }
            log("stopped at pos: ");
            logl(desk.getPosition());
            break;
        case 'u':
            if(!controller.isRunning()) {
                log("upPos: ");
                logl(upPos);
                desk.setMaxSpeed(destinationSpeed);
                desk.setTargetAbs(upPos);
                controller.moveAsync(desk);
                logl("desk is going to stored up position");
            }
            break;
        case 'd':
            if(!controller.isRunning()) {
                desk.setMaxSpeed(destinationSpeed);
                desk.setTargetAbs(downPos);
                controller.moveAsync(desk);
                logl("desk is going to down position");
                
            }
            break;
    }
}
    
    

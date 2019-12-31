/*
    Name:       automatico.ino
    Created:	30/12/2019 14.43.34
    Author:     HPZBOOK\vito4
*/



/*
use gps to get car-speed and automatically roll up and down window(s) depending on preset vehicle speed.
use time to set end-stops.
open window, activate and close window when speed is reached. when speed drops below window opens to previous position
activated window will open and close between set positions.
haptic feedback on actions
window action will be at speed measured when AW is activated

TODO
-hysteresis to avoid jittering when cruising around set speed OK
*/

#include <TinyGPS++.h>
#include <SoftwareSerial.h>
#include "automatico.h"

/*define hardware constants*/

const bool relayOn = 1;
const bool relayOff = 0;

static const uint32_t GPSBaud = 9600;  // internal GPS communication

TinyGPSPlus gps;            // The TinyGPS++ object

static const int RXPin = 3, TXPin = 4;
SoftwareSerial ss(RXPin, TXPin);    // The Serial connection to the GPS device

unsigned long last = 0UL;       // For stats that happen every 5 seconds

/*define pins*/
int buttonOnOff = 5;          // switch on-off window control
int buttonOpen = 6;           // open window, will open window and start timer to get window position
int buttonClose = 7;          // close window, will close window and reset timer defined by open window
int gpsLock = 8;            // has satelite-lock? output to led
int haptic = 9;             // haptic feedback on actions
int relayUp = 10;
int relayDown = 11;
int inputUp = 12;           // input signal when windows is operated manuallly from generic button
int inputDown = 12;           // input signal when windows is operated manuallly from generic button

/* define variables*/
boolean pos;
int state;                //open and close state for button   
unsigned long btnPressElapsed;
unsigned long btnBeginPressTimer;
unsigned long buttonBounceElapsed;
int currentSpeed;
int setSpeedOpen;
int setSpeedClose;
int hasLocation;
int ledState = LOW;
bool buttonState;
bool setButton;


/*define software constants*/
unsigned long lastBounceTime = 0;       //last time button pin was toggled
unsigned long bounceDelay = 50;         //bounce time to filter out noise
unsigned long buttonPressLong_len = 2000;     //no of cycles to register as long button press
unsigned long buttonPressShort_len = 100;     //no of cycles to register as short button press
unsigned long previousMillis = 0;
int DELAY = 20;
int buttonPressedCounter = 0;
int lastButtonState = LOW;
bool btnCurrentState = false;

void setup()
{

    Serial.begin(9600);
    ss.begin(GPSBaud);
    pinMode(buttonOpen, INPUT_PULLUP);
    pinMode(buttonClose, INPUT_PULLUP);
    pinMode(buttonOnOff, INPUT_PULLUP);
    pinMode(relayUp, OUTPUT);
    pinMode(relayDown, OUTPUT);
    pinMode(gpsLock, OUTPUT);
    pinMode(haptic, OUTPUT);


    buttonPressedCounter = 0;   //count no of cycles to determine if button is pressed

}

void loop()
{

    while (ss.available() > 1)
        if (gps.encode(ss.read()))
            Serial.println("GPS module found");
    arm();

    if (millis() > 5000 && gps.charsProcessed() < 10) {
        Serial.println("no GPS module");
        while (true);
    }
}



void arm() {

    isLocationValid();

    if (hasLocation == HIGH) {
        isSpeedUpdated();
        actionButton();
        Serial.println("GPS Lock - armed");
    }

    else {
        alarm();
        Serial.println("alarm - no location fix");
    }

}


void actionButton() {                         //enable open close if mode on

    if (classButton(buttonOnOff) == true) {
        Serial.println("automatic ON");
    }

    else if (classButton(buttonOnOff) == false) {
        Serial.println("automatic OFF");
    }



}

bool actionButtonClose() {                                //manual close window

    bool clsMode;
    bool btnState = classButton(buttonClose);

    if (btnState == true) {
        clsMode = true;
        setSpeedClose = isSpeedUpdated();
        hapticFeedback(10, 1, 10);
        Serial.print("hastighed lukke");
        Serial.println(setSpeedClose);
    }

    else
        clsMode = false;

    return clsMode;

}

bool actionButtonOpen() {                               //manual open window

    bool opnMode;
    bool btnState = classButton(buttonOpen);

    if (btnState == true) {
        opnMode = true;
        setSpeedOpen = isSpeedUpdated();
        hapticFeedback(10, 1, 10);
        Serial.print("hastighed åbne");
        Serial.println(setSpeedClose);
    }

    else
        opnMode = false;

    return opnMode;
}

bool classButton(int pin) {

    int pinValue = digitalRead(pin);

    if (pinValue != lastButtonState) {
        lastBounceTime = millis();
    }

        if ((millis() - lastBounceTime) > bounceDelay) {

            if (pinValue != btnCurrentState) {

                btnCurrentState = pinValue;

                if (btnCurrentState == LOW) {
                    buttonTimer(true);
                }

               /* else if (btnCurrentState == HIGH && state == 1) {
                    buttonTimer(false);
                }*/
            }
            
        }
Else if (pinValue != lastButtonState && state == 1) {
buttonTimer(false);
}


       lastButtonState = pinValue;
       return btnCurrentState;
}

unsigned long buttonTimer(bool setButton) {

    if (setButton == true) {
        btnBeginPressTimer = millis();
        state = !state;
    }

    else if (setButton == false) {
        btnPressElapsed = (millis() - btnBeginPressTimer);
       
    }

    return btnPressElapsed;
}

void automaticWindow() {

    unsigned long elapsed = getElapsed();
    unsigned long startTimer = millis();

    double bandwith = 0.1;        //bandwidth +-10% of PV

    int bandwithValueOpen = (int)setSpeedOpen * (int)bandwith;
    int bandwithValueClose = (int)setSpeedClose * (int)bandwith;

    int lowSpeedLimitValue = currentSpeed - bandwithValueOpen;
    int highSpeedLimitValue = currentSpeed + bandwithValueClose;

    if (setSpeedOpen == lowSpeedLimitValue) {
        digitalWrite(relayUp, LOW);
        delay(DELAY);
        while (startTimer - millis() < elapsed) {
            digitalWrite(relayDown, HIGH);
        }

    }

    if (setSpeedClose == highSpeedLimitValue) {
        digitalWrite(relayDown, LOW);
        delay(DELAY);
        while (startTimer - millis() < elapsed) {
            digitalWrite(relayUp, HIGH);
        }
    }

}

unsigned long getElapsed() {

    unsigned long elapsedRunTime;
    bool actionButton = actionButtonOpen(), actionButtonClose();

    if (actionButton == true) {
        elapsedRunTime = getTimeToWindowOpen(actionButton);
    }

    return elapsedRunTime;

}


unsigned long getTimeToWindowOpen(bool start) {

    unsigned long elapsed;
    unsigned long timeStart;        //variable to hold start time for window
    unsigned long timeStop;         //variable to hold stop time for window
    bool setVal;

    if (start && setVal == false) {

        timeStart = getTimeSecond();
        setVal = true;
    }

    else if (!start && setVal == true) {
        timeStop = getTimeSecond();
        setVal = false;
    }

    elapsed = calculateTime(timeStart, timeStop);
    Serial.println(elapsed);
    return elapsed;
}

void alarm() {
    hapticFeedback(500, 5, 500);
    ledFeedback(100);
}



/*helpers*/

uint8_t calculateTime(uint8_t timeStart, uint8_t timeStop) {

    uint8_t timeElapsed;

    timeElapsed = timeStop - timeStart;

    return timeElapsed;
}

void hapticFeedback(int activeLength, int repeats, int silentLength) {          //haptic feedback on button actions milliseconds

    int repeatCount, hapticCount, hapticSilent = 0;

    while (repeatCount <= repeats) {

        while (hapticCount < activeLength) {

            digitalWrite(haptic, HIGH);

            ++hapticCount;

            if (hapticCount == activeLength) {
                while (hapticSilent < silentLength) {
                    digitalWrite(haptic, LOW);

                    ++hapticSilent;
                }
            }
        }

        ++repeatCount;
    }
}

void ledFeedback(unsigned long interval) {

    unsigned long currentMillis = millis();


    if (currentMillis - previousMillis >= interval) {

        previousMillis = currentMillis;


        if (ledState == LOW) {
            ledState = HIGH;
        }
        else {
            ledState = LOW;
        }

        digitalWrite(gpsLock, ledState);
    }
}

void isLocationValid() {

    if (gps.location.isValid()) {
        Serial.print("Latitude: ");
        Serial.println(gps.location.lat(), 6);
        Serial.print("Longitude: ");
        Serial.println(gps.location.lng(), 6);
        hasLocation = HIGH;
        digitalWrite(gpsLock, HIGH);
    }

    else {
        ledFeedback(200);
    }

}

int isSpeedUpdated() {
    if (gps.speed.isUpdated()) {
        currentSpeed = gps.speed.kmph();
        Serial.print(currentSpeed);
        Serial.println(" km/t");
    }

    return currentSpeed;
}

uint8_t getTimeSecond() {
    uint8_t timeSecond;

    if (gps.time.isUpdated()) {
        timeSecond = gps.time.second();
        Serial.println(timeSecond);
    }

    return timeSecond;
}

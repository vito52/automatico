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
bool pos;
bool automaticoEnabledState;                //state for autorun  
int btnPressElapsed;
unsigned long btnBeginPressTimer;
unsigned long buttonBounceElapsed;
int openingTimer;
int closingTimer;
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
int buttonPressedCounter = 0;
int DELAY = 20;
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

    buttonPressedCounter = 0;        //reset button press
}



void arm() {

    isLocationValid();

    if (hasLocation == HIGH) {
        isSpeedUpdated();
        automaticoEnable();
        Serial.println("GPS Lock - armed");
    }

    else {
        alarm();
        Serial.println("alarm - no location fix");
    }

    if (automaticoEnabledState == true) {
        actionButtons();
        automaticWindow();
    }

}


int classButton(int pin, int outputPin) {


          while (digitalRead(pin) == LOW) {
            delay(100);
            buttonPressedCounter = buttonPressedCounter + 100;
            digitalWrite(outputPin, HIGH);

            if (buttonPressedCounter == buttonPressShort_len) {
                hapticFeedback(500, 3, 500);
            }

            else if (buttonPressedCounter == buttonPressLong_len) {
                hapticFeedback(2000, 2, 2000);
            }

            
         }

        btnPressElapsed = buttonPressedCounter;
        digitalWrite(outputPin, LOW);

        return btnPressElapsed;
}

void automaticoEnable() {                         //enable open close if mode on
    if (classButton(buttonOnOff, 0) >= buttonPressShort_len && classButton(buttonOnOff, 0) < buttonPressLong_len && automaticoEnabledState == false) {
        automaticoEnabledState = true;
        hapticFeedback(1000, 500, 1000);
        
    }

    else if (classButton(buttonOnOff, 0) >=  buttonPressLong_len && automaticoEnabledState == true) {
        automaticoEnabledState = false;
        
    }
}


void actionButtons() { 

        if (digitalRead(buttonClose) == LOW) {
                setSpeedClose = isSpeedUpdated();
                closingTimer = classButton(buttonClose, relayUp);
                hapticFeedback(1000, 1000, 1000);
        }

        if (digitalRead(buttonOpen) == LOW) {
                setSpeedOpen = isSpeedUpdated();
                openingTimer = classButton(buttonOpen, relayDown);
                hapticFeedback(2000, 1000, 2000);
        }
}

void automaticWindow() {

    unsigned long startTimer = millis();

    double bandwith = 0.1;        //bandwidth +-10% of PV

    int bandwithValueOpen = (int)setSpeedOpen * (int)bandwith;
    int bandwithValueClose = (int)setSpeedClose * (int)bandwith;

    int lowSpeedLimitValue = isSpeedUpdated() - bandwithValueOpen;
    int highSpeedLimitValue = isSpeedUpdated() + bandwithValueClose;

    if (setSpeedOpen == lowSpeedLimitValue) {
        digitalWrite(relayUp, LOW);
        delay(DELAY);
        while (startTimer - millis() < openingTimer) {
            digitalWrite(relayDown, HIGH);
        }

    }

    if (setSpeedClose == highSpeedLimitValue) {
        digitalWrite(relayDown, LOW);
        delay(DELAY);
        while (startTimer - millis() < closingTimer) {
            digitalWrite(relayUp, HIGH);
        }
    }

}


void alarm() {
    hapticFeedback(500, 5, 500);
    ledFeedback(100);
}

/*helpers*/


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



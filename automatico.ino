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
int autoOn = 12;           // input signal when windows is operated manuallly from generic button


/* define variables*/
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
int hapticState = LOW;
int ledState = LOW;
int oneshotHaptic = false;
int activeLength;
int repeats;
int x = 0;
bool setOneshotHaptic;
bool buttonState;
bool setButton;



/*define software constants*/
unsigned long lastBounceTime = 0;       //last time button pin was toggled
unsigned long bounceDelay = 50;         //bounce time to filter out noise
unsigned long buttonPressLong_len = 2000;     //no of cycles to register as long button press
unsigned long buttonPressShort_len = 100;     //no of cycles to register as short button press
unsigned long ledPreviousMillis = 0;
unsigned long hapticPreviousMillis = 0;
int buttonPressedCounter = 0;
int DELAY = 20;

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
    pinMode(autoOn, OUTPUT);
}

void loop()
{

    while (ss.available() > 1) {

        if (gps.encode(ss.read())) {
        Serial.println("GPS module found");
        arm();
        }

        if (millis() > 5000 && gps.charsProcessed() < 10) {
            Serial.println("no GPS module");
            while (true);
        }

        buttonPressedCounter = 0;        //reset button press
    }
}



void arm() {

    isLocationValid();

    if (hasLocation == HIGH) {
 
        Serial.println("GPS Lock - armed");
        automaticoEnable();
    }

    else {
        alarm();
        Serial.println("alarm - no location fix");
    }

    if (automaticoEnabledState == true) {
        actionButtons();
        automaticWindow();
    }


    if (setOneshotHaptic == 1) {

        hapticFeedback(activeLength);
    }

}


int classButton(int pin, int outputPin) {

    while (digitalRead(pin) == LOW) {
        delay(100);
        buttonPressedCounter = buttonPressedCounter + 100;
        digitalWrite(outputPin, HIGH);

    }

    btnPressElapsed = buttonPressedCounter;
    digitalWrite(outputPin, LOW);

    return btnPressElapsed;
}

void automaticoEnable() { 
    
    if (classButton(buttonOnOff, 0) >= buttonPressShort_len && classButton(buttonOnOff, 0) < buttonPressLong_len && automaticoEnabledState == false) {
        automaticoEnabledState = true;
        
        digitalWrite(autoOn, HIGH);
        setOneshotHaptic = true;
        repeats = 2;
        activeLength = 500;
    }

    else if (classButton(buttonOnOff, 0) >=  buttonPressLong_len && automaticoEnabledState == true) {
        automaticoEnabledState = false;
        
        digitalWrite(autoOn, LOW);
        setOneshotHaptic = true;
        repeats = 3;
        activeLength = 200;

    }
}


void actionButtons() { 

        if (digitalRead(buttonClose) == LOW) {
                setSpeedClose = isSpeedUpdated();
                closingTimer = classButton(buttonClose, relayUp);
               
        }

        if (digitalRead(buttonOpen) == LOW) {
            setSpeedOpen = isSpeedUpdated();
            openingTimer = classButton(buttonOpen, relayDown);
        }
}

void automaticWindow() {

    unsigned long startTimer = millis();
    int speedPV = isSpeedUpdated();

    int bandwidth = 10;        //bandwidth +-10% of PV
     

    int bandwidthValueOpen = (setSpeedOpen * bandwidth)/100;
    int bandwidthValueClose = (setSpeedClose * bandwidth)/100;

    int lowSpeedLimitValue = speedPV - bandwidthValueOpen;
    int highSpeedLimitValue = speedPV + bandwidthValueClose;

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
    
    feedback(gpsLock, 100);
}

/*helpers*/



void hapticFeedback(int activeLength) {  
        
        unsigned long currentMillis = millis();
      
         if (currentMillis - hapticPreviousMillis >= (unsigned long)activeLength) {

             hapticPreviousMillis = currentMillis;

             if (x == repeats) {
                             setOneshotHaptic = false;
                             hapticState = HIGH;
                             x = 0;
             }

             if (hapticState == LOW) {
                 hapticState = HIGH;
                 x++;
             }
             else {
                 hapticState = LOW;
                 
             }
                      
             digitalWrite(haptic, hapticState);
                 
         }

}


void feedback(int pin, unsigned long interval) {

    unsigned long currentMillis = millis();


    if (currentMillis - ledPreviousMillis >= interval) {

        ledPreviousMillis = currentMillis;


        if (ledState == LOW) {
            ledState = HIGH;
        }
        else {
            ledState = LOW;
        }

        digitalWrite(pin, ledState);
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
        feedback(gpsLock, 200);
    }

}

int isSpeedUpdated() {
    if (gps.speed.isUpdated()) {
        //currentSpeed = gps.speed.kmph();
        currentSpeed = 25;
    }

    return currentSpeed;
}


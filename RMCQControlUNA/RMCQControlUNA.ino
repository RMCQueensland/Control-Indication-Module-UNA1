/* RMCQ Control and Indication Module UNA
  By David Lowe 20181101

  See Description for further detail

  Program Details:
*/
#define progName "RMCQ Control and Indication Module UNA"
#define version "0.1BETA"
#define date "12 Jan 2019"
#define author "DLowe"


#include <Servo.h>
#include <RMCQControl.h>
Servo servo200;    // create servo object to control a servo
Servo servo201A;
Servo servo201B;
Servo servo201C;

//Route comamd inputs
#define RCUNAL1 5      //RCUNAL1   M1
#define RCUNAL2 4      //RCUNAL2   P1
#define RCUNAL3 6      //RCUNAL3   M2
#define RCUNAL4 7      //RCUNAL4   P2
#define RCUNAL5 8      //RCUNAL5   P3
#define RCUNAL6 9      //RCUNAL6   P4

RMCQControl rmcqControl = RMCQControl();

// Servo variables
#define startPos  80
#define throwNormal  60
#define throwReverse  30
#define timeBetweenServoChange  20
#define servoRateOfChange  30

// Trunout Normal and Reverse positions for each servo used. Set to each individual Servo and physical setup.
#define TC200 10
#define TC201A 11
#define TC201B 12
#define TC201C 13

// Servo variables
// the Turnout is in the Normal state (HIGH). LOW for Reverse.
boolean currentServoState200 = HIGH;
boolean currentServoState201A = HIGH;
boolean currentServoState201B = HIGH;
boolean currentServoState201C = HIGH;

// Variables will change:
int inputStateRCUNAL1;             // the current reading from the input RCUNAL1
int lastinputStateRCUNAL1 = HIGH;   // the previous reading from the input RCUNAL1
int inputStateRCUNAL2;             // the current reading from the input RCUNAL2
int lastinputStateRCUNAL2 = HIGH;   // the previous reading from the input RCUNAL2
int inputStateRCUNAL3;             // the current reading from the input RCUNAL3
int lastinputStateRCUNAL3 = HIGH;   // the previous reading from the input RCUNAL3
int inputStateRCUNAL4;             // the current reading from the input RCUNAL4
int lastinputStateRCUNAL4 = HIGH;   // the previous reading from the input RCUNAL4
int inputStateRCUNAL5;             // the current reading from the input RCUNAL5
int lastinputStateRCUNAL5 = HIGH;   // the previous reading from the input RCUNAL5
int inputStateRCUNAL6;             // the current reading from the input RCUNAL6
int lastinputStateRCUNAL6 = HIGH;   // the previous reading from the input RCUNAL6

// the following variables athe long's because the time, measured in miliseconds, will quickly become a bigger number Ran can be stored in an int.
unsigned long lastDebounceTime = 0;  // the last time the output pin was toggled
unsigned long debounceDelay = 30;    // the debounce time; increase if the output flickers

//Added information for MCP23017
//Initalise the required libraries
#include <Wire.h>
#include "Adafruit_MCP23017.h"

//define the micro switch input pins from the expander (MCP pins)
#define TI200 8
#define TI201A 9
#define TI201B 10
#define TI201C 11
//define the LED output pins to the expander (MCP pins)
#define RIUNAL1 4    //M1
#define RIUNAL2 3    //P1
#define RIUNAL3 5    //M2
#define RIUNAL4 6    //P2
#define RIUNAL5 7    //P3
#define RIUNAL6 8    //P4

Adafruit_MCP23017 mcp;  //Create a instance of the MCP23017 chip


void setup() {
  Serial.begin(9600);
  Serial.println("RMCQ Standard Indentification Information");
  Serial.print("Running; "); Serial.println(progName);
  Serial.print("Version; "); Serial.println(version);
  Serial.print("Last updated; "); Serial.println(date);
  Serial.print("Programer; "); Serial.println(author);

  // Set Servo global variables
  rmcqControl.setUp(startPos, throwNormal, throwReverse, timeBetweenServoChange, servoRateOfChange);

  // set pin configuration
  pinMode(RCUNAL1, INPUT_PULLUP);
  pinMode(RCUNAL2, INPUT_PULLUP);
  pinMode(RCUNAL3, INPUT_PULLUP);
  pinMode(RCUNAL4, INPUT_PULLUP);
  pinMode(RCUNAL5, INPUT_PULLUP);
  pinMode(RCUNAL6, INPUT_PULLUP);

  // set initial state for servos
  // Set Point 200
  rmcqControl.center(servo200, startPos, TC200);
  rmcqControl.changeNormal(LOW, servo200, TC200);  // Set the Servo to Normal
  currentServoState200 = HIGH;
  // Set Point 201A
  rmcqControl.center(servo201A, startPos, TC201A);
  rmcqControl.changeReverse(HIGH, servo201A, TC201A);    // Set the Servo to Normal
  currentServoState201A = LOW;
  // Set Point 201B
  rmcqControl.center(servo201B, startPos, TC201B);
  rmcqControl.changeNormal(LOW, servo201B, TC201B);    // Set the Servo to Normal
  currentServoState201B = HIGH;
  // Set Point 201C
  rmcqControl.center(servo201C, startPos, TC201C);
  rmcqControl.changeNormal(LOW, servo201C, TC201C);    // Set the Servo to normal
  currentServoState201C = HIGH;

  //Added setup for control panel indication
  mcp.begin();      // use default address 0
  //Set Port B as inputs and apply the 100K internal pullups
  for (uint8_t pin = 8; pin  < 15; pin ++)
  {
    mcp.pinMode(pin, INPUT);
    mcp.pullUp(pin, HIGH);
  }
  //Set Port A as output and turn all the LEDs on (HIGH)
  for (uint8_t pin = 0; pin  < 7; pin ++)
  {
    mcp.pinMode(pin, OUTPUT);
    mcp.digitalWrite(pin, HIGH);
  }
  //Turn the LEDs off
  delay(200);
  for (uint8_t pin = 0; pin  < 7; pin ++)
  {
    mcp.digitalWrite(pin, LOW);
  }
}
//RMCQ library functions
//void RMCQControl::changeNormal(boolean currentState, Servo servoName, int outputPin)
//void RMCQControl::changeReverse(boolean currentState, Servo servoName, int outputPin)

/*    Route         Point
              __________ L2   P1
   T200 _____/__________ L1   M1


   T201A ____ __________ L3   M2
   T201B     \__________ L4   P2
   T201C      \_________ L5   P3
               \________ L6   P4
*/

void loop() {
  //Added routine for control panel indication
  RMCQIndicationUNA();

  // Read Route Commard L1 (M1) input
  // On L1: 200: C, 201A: NA, 201B: NA, 201C: NA.
  int RCUNAL1State = digitalRead(RCUNAL1);
  if (RCUNAL1State != lastinputStateRCUNAL1) {
    lastDebounceTime = millis();
    Serial.println("Command L1 (M1) activated");
  }
  if ((millis() - lastDebounceTime) > debounceDelay) {
    if (RCUNAL1State != inputStateRCUNAL1) {
      inputStateRCUNAL1 = RCUNAL1State;
      if (RCUNAL1State == HIGH) {
        setRCUNAL1();
      }
    }
  }
  lastinputStateRCUNAL1 = RCUNAL1State;

  // Read Route Commard L2 (P1) input
  // On L1: 200: T, 201A: NA, 201B: NA, 201C: NA.
  int RCUNAL2State = digitalRead(RCUNAL2);
  if (RCUNAL2State != lastinputStateRCUNAL2) {
    lastDebounceTime = millis();
    Serial.println("Command L2 (P1) activated");
  }
  if ((millis() - lastDebounceTime) > debounceDelay) {
    if (RCUNAL2State != inputStateRCUNAL2) {
      inputStateRCUNAL2 = RCUNAL2State;
      if (RCUNAL2State == LOW) {
        setRCUNAL2();
      }
    }
  }
  lastinputStateRCUNAL2 = RCUNAL2State;

  // Read Route Commard L3 (M2) input
  // On L1: 200: NA, 201A: C, 201B: NA, 201C: NA.
  int RCUNAL3State = digitalRead(RCUNAL3);
  if (RCUNAL3State != lastinputStateRCUNAL3) {
    lastDebounceTime = millis();
    Serial.println("Command L3 (M2) activated");
  }
  if ((millis() - lastDebounceTime) > debounceDelay) {
    if (RCUNAL3State != inputStateRCUNAL3) {
      inputStateRCUNAL3 = RCUNAL3State;
      if (RCUNAL3State == LOW) {
        setRCUNAL3();
      }
    }
  }
  lastinputStateRCUNAL3 = RCUNAL3State;

  // Read Route Commard L4 (P2) input
  // On L1: 200: NA, 201A: C, 201B: T, 201C: NA.
  int RCUNAL4State = digitalRead(RCUNAL4);
  if (RCUNAL4State != lastinputStateRCUNAL4) {
    lastDebounceTime = millis();
    Serial.println("Command L4 (P2) activated");
  }
  if ((millis() - lastDebounceTime) > debounceDelay) {
    if (RCUNAL4State != inputStateRCUNAL4) {
      inputStateRCUNAL4 = RCUNAL4State;
      if (RCUNAL4State == LOW) {
        setRCUNAL4();
      }
    }
  }
  lastinputStateRCUNAL4 = RCUNAL4State;

  // Read Route Commard L5 (P3) input
  // On L1: 200: NA, 201A: C, 201B: C, 201C: T.
  int RCUNAL5State = digitalRead(RCUNAL5);
  if (RCUNAL5State != lastinputStateRCUNAL5) {
    lastDebounceTime = millis();
    Serial.println("Command L5 (P3) activated");
  }
  if ((millis() - lastDebounceTime) > debounceDelay) {
    if (RCUNAL5State != inputStateRCUNAL5) {
      inputStateRCUNAL5 = RCUNAL5State;
      if (RCUNAL5State == LOW) {
        setRCUNAL5();
      }
    }
  }
  lastinputStateRCUNAL5 = RCUNAL5State;

  // Read Route Commard L6 (P4) input
  // On L1: 200: NA, 201A: C, 201B: C, 201C: C.
  int RCUNAL6State = digitalRead(RCUNAL6);
  if (RCUNAL6State != lastinputStateRCUNAL6) {
    lastDebounceTime = millis();
    Serial.println("Command L6 (P4) activated");
  }
  if ((millis() - lastDebounceTime) > debounceDelay) {
    if (RCUNAL6State != inputStateRCUNAL6) {
      inputStateRCUNAL6 = RCUNAL6State;
      if (RCUNAL6State == LOW) {
        setRCUNAL6();
      }
    }
  }
  lastinputStateRCUNAL6 = RCUNAL6State;

}



void setRCUNAL1() {
  // Point 200 to Closed
  rmcqControl.changeNormal(currentServoState200, servo200, TC200);
  currentServoState200 = HIGH;
}

void setRCUNAL2() {
  // Point 200 to Thrown
  rmcqControl.changeReverse(currentServoState200, servo200, TC200);
  currentServoState200 = LOW;
}

void setRCUNAL3() {
  // Point 201A to Thrown
  rmcqControl.changeReverse(currentServoState201A, servo201A, TC201A);
  currentServoState201A = LOW;
}

void setRCUNAL4() {
  // Point 201A to Closed and 201B to Thrown
  rmcqControl.changeNormal(currentServoState201A, servo201A, TC201A);
  currentServoState201A = HIGH;
  rmcqControl.changeReverse(currentServoState201B, servo201B, TC201B);
  currentServoState201B = LOW;
}

void setRCUNAL5() {
  // Point 201A to Closed and 201B to Closed and 201C to Thrown
  rmcqControl.changeNormal(currentServoState201A, servo201A, TC201A);
  currentServoState201A = HIGH;
  rmcqControl.changeNormal(currentServoState201B, servo201B, TC201B);
  currentServoState201B = HIGH;
  rmcqControl.changeReverse(currentServoState201C, servo201C, TC201C);
  currentServoState201C = LOW;
}

void setRCUNAL6() {
  // Point 201A to Closed and 201B to Closed and 201C to Closed
  rmcqControl.changeNormal(currentServoState201A, servo201A, TC201A);
  currentServoState201A = HIGH;
  rmcqControl.changeNormal(currentServoState201B, servo201B, TC201B);
  currentServoState201B = HIGH;
  rmcqControl.changeNormal(currentServoState201C, servo201C, TC201C);
  currentServoState201C = HIGH;
}

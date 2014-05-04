#include <RF24.h>
#include <RF24_config.h>
#include <nRF24L01.h>

#include <RF24Network.h>
#include <RF24Network_config.h>
#include <Sync.h>
#include <SPI.h>

#include "nodeconfig.h"
#include "printf.h"

/*
  DoorBell Button
 
 Modified from Blink example
 
 http://www.arduino.cc/en/Tutorial/Button
 */
 
#ifdef VERSION_H
#include "version.h"
#else
#define __TAG__ "Unknown"
#endif

// constants won't change. They're used here to 
// set pin numbers:
const int buttonPin = 2;     // the number of the pushbutton pin
const int ledPin =  3;      // the number of the LED pin

const uint16_t chime_node = 1; // The address of the chime node

const uint8_t radio_cepin = 9; // Pin attached to Chip Enable on RF module
const uint8_t radio_cspin = 10; // Pin attached to Chip Select on RF module

const int radioChannel = 100;  // The radio channel to use
const int serialSpeed = 57600; // The serial port speed

// variables will change:
int buttonState = 0;         // variable for reading the pushbutton status
int buttonStateLast = 0;     // Variable to compare the current button state against

// nRF24L01(+) radio using the Getting Started board
RF24 radio(radio_cepin, radio_cspin);
RF24Network network(radio);

// Our node address
uint16_t this_node;


void setup() {
  //
  // Print preamble
  //

  Serial.begin(57600);
  printf_begin();
  printf_P(PSTR("\n\rDoorBell\n\r"));
  printf_P(PSTR("VERSION: " __TAG__ "\n\r"));
  
  //
  // Pull node out of eeprom
  //
  
  // Which node are we?
  this_node = nodeconfig_read();
  printf_P(PSTR("Chime Node: %i\n\r"), chime_node);
  
  //
  // Bring up the RF network
  //
  
  SPI.begin();
  radio.begin();
  network.begin(radioChannel, /*node address*/ this_node);
  
  // initialize the LED pin as an output:
  pinMode(ledPin, OUTPUT);   
  // initialize the pushbutton pin as an input:
  pinMode(buttonPin, INPUT);     
  
  // Initialize led states
  digitalWrite(ledPin, HIGH);
}

void loop(){
  // Pump the network regularly
  network.update();
  
  // read the state of the pushbutton value:
  buttonState = digitalRead(buttonPin);

  // check if the buttons state changed
  if (buttonState != buttonStateLast) {
    // check if the pushbutton is pressed.
    // if it is, the buttonState is HIGH:
    if (buttonState == LOW) {
      // Blink LED pin
      digitalWrite(ledPin, LOW);
      bool result = sendDoorBellPress();
      
      if (!result) {
        blink(ledPin); 
      }
    } 
    else {
      // turn LED off:
      digitalWrite(ledPin, HIGH); 
    }
  }
  
  // Setup button state last for next loop.
  buttonStateLast = buttonState;
}

void blink(int pin) {
  for (int i=0; i < 3; i++) {
    slowBlink(pin, 10, 10);
  }
}

void slowBlink(int pin, int stepDistance, int stepDelay) {
  for (int i = 255; i >= 0; i -= stepDistance) {
    analogWrite(pin, i);
    delay(stepDelay);
  }
  analogWrite(pin, 0);
  for (int i = 0; i <= 255; i += stepDistance) {
    analogWrite(pin, i);
    delay(stepDelay);
  }
  analogWrite(pin, 255);
}

boolean sendDoorBellPress()
{
  return false;
}

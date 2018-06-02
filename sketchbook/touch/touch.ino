/*********************************************************
This is a library for the MPR121 12-channel Capacitive touch sensor

Designed specifically to work with the MPR121 Breakout in the Adafruit shop 
  ----> https://www.adafruit.com/products/

These sensors use I2C communicate, at least 2 pins are required 
to interface

Adafruit invests time and resources providing this open source code, 
please support Adafruit and open-source hardware by purchasing 
products from Adafruit!

Written by Limor Fried/Ladyada for Adafruit Industries.  
BSD license, all text above must be included in any redistribution
**********************************************************/

#include <Wire.h>
#include "Adafruit_MPR121.h"

// You can have up to 4 on one i2c bus but one is enough for testing!
Adafruit_MPR121 cap = Adafruit_MPR121();

// Keeps track of the last pins touched
// so we know when buttons are 'released'
uint16_t lasttouched = 0;
uint16_t currtouched = 0;
int num = 0;
int dummy = -1;
int LED = 4;
int incomingbyte = 'i';
void setup() {
  Serial.begin(9600);
  pinMode(LED, OUTPUT);
  while (!Serial) { // needed to keep leonardo/micro from starting too fast!
    delay(10);
  }
  
  //Serial.println("Adafruit MPR121 Capacitive Touch sensor test"); 
  
  // Default address is 0x5A, if tied to 3.3V its 0x5B
  // If tied to SDA its 0x5C and if SCL then 0x5D
   if (!cap.begin(0x5A)) {
    //Serial.println("MPR121 not found, check wiring?");
     while (1);
   }
  //Serial.println("MPR121 found!");
}

void Light() {
    incomingbyte = (char)Serial.read();
    if (incomingbyte=='i'){
      digitalWrite(LED, HIGH);
      }
    else if (incomingbyte=='o'){
      digitalWrite(LED, LOW);
      }
    Serial.flush();
  }

void loop() {
  // Get the currently touched pads
  currtouched = cap.touched();
  for (uint8_t i=0; i<12; i++) {
    incomingbyte = (char)Serial.read();
    if (incomingbyte=='i'){
      digitalWrite(LED, HIGH);
      }
    else if (incomingbyte=='o'){
      digitalWrite(LED, LOW);
      }
    // it if *is* touched and *wasnt* touched before, alert!
    if ((currtouched & _BV(i)) && !(lasttouched & _BV(i)) ) {
      Serial.print(i);
      Serial.print(' ');
      num=1;
      Serial.print(num);
      Serial.print(' ');
      Serial.println(' touched');
    }
    // if it *was* touched and now *isnt*, alert!
    else if (!(currtouched & _BV(i)) && (lasttouched & _BV(i)) ) {
      Serial.print(i);
      Serial.print(' ');
      num=2;
      Serial.print(num);
      Serial.print(' ');
      Serial.println(' released');
    }
    // if it *was* touched and now *isnt*, alert!
    else {
      Serial.print(dummy);
      Serial.print(' ');
      Serial.print(dummy);
      Serial.print(' ');
      Serial.println(' none');
    }
  }
  // reset our state
  lasttouched = currtouched;

  // comment out this line for detailed data from the sensor!
  return;

  // put a delay so it isn't overwhelming
  // delay(100);
}








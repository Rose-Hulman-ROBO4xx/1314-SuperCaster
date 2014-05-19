/* 
This is the code for the US sensor prototype
Sensor characteristics:
~9.8mV per inch
*/

//Imports
#include <stdlib.h>
#include "analogComp.h"

//Constants (volts)
//#define CLOSE 0.9*1023/5 //~30cm 
//#define FAR 0.8*1023/5 //~45cm

#define US_CONVERT 0.0098 //(Volts/inch)
#define AN_CONVERT 1023/5
#define CLOSE_INCHES 24 
#define FAR_INCHES 40 

#define CLOSE_VOLT US_CONVERT*CLOSE_INCHES  //
#define FAR_VOLT US_CONVERT*FAR_INCHES  // 

#define CLOSE CLOSE_VOLT*AN_CONVERT
#define FAR FAR_VOLT*AN_CONVERT

//Pinouts
#define US1_IN A1
#define LED_CLOSE 44
#define LED_GOOD 45
#define LED_FAR 46

//Variables
float US1;

void setup() {
    
  pinMode(US1_IN, INPUT);
  pinMode(LED_CLOSE,OUTPUT);
  pinMode(LED_GOOD,OUTPUT);
  pinMode(LED_FAR,OUTPUT);
  
  //Set initial values
  digitalWrite(LED_CLOSE,HIGH);
  digitalWrite(LED_GOOD,HIGH);
  digitalWrite(LED_FAR,HIGH);
  delay(500);
  digitalWrite(LED_CLOSE,LOW);
  digitalWrite(LED_GOOD,LOW);
  digitalWrite(LED_FAR,LOW);
  
  US1 = 0;
}


void loop(){
  
US1 = analogRead(US1_IN);

if (US1 < CLOSE){  
	digitalWrite(LED_CLOSE,HIGH);
	digitalWrite(LED_GOOD,LOW);
	digitalWrite(LED_FAR,LOW);
} else if(US1 > FAR) {
	digitalWrite(LED_CLOSE,LOW);
	digitalWrite(LED_GOOD,LOW);
	digitalWrite(LED_FAR,HIGH);
	}
else{
	digitalWrite(LED_CLOSE,LOW);
	digitalWrite(LED_GOOD,HIGH);
	digitalWrite(LED_FAR,LOW);
	}

}
 

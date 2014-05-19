/* 
This is the code for the IR sensor prototype
Sensor characteristics
Dist(cm) (ft)	      Voltage    Experimental Distance (ft)
0	0		0            --
10	0.33		2.3
20	0.66		2.75
30	0.98		2           0.85-0.95
40	1.31		1.52        1.3-1.4
50	1.64		1.25        1.5-1.6
60	1.97		1.1         ~1.9
70	2.30		0.9         ~2.2
80	2.62		0.8         2.4-2.5
90	2.95		0.7         ~2.6
100	3.28		0.65        ~3
110	3.61		0.60        ~3
120	3.94		0.55        ~3.4
130	4.26		0.50        
140	4.59		0.48
150	4.92		0.46        ~4
*/

//Imports
#include <stdlib.h>
#include "analogComp.h"

//Constants (volts)
//#define CLOSE 0.9*1023/5 //~30cm 
//#define FAR 0.8*1023/5 //~45cm

#define CLOSE 0.5*1023/5  //
#define FAR 0.48*1023/5  // 

//Pinouts
#define IR_REF A0
#define IR1_IN A1
#define IR2_IN A3
#define BUMP_IN 22
#define LED_CLOSE 44
#define LED_GOOD 45
#define LED_FAR 46
#define LED_BUMP 43


//Variables
float IR1;
float IR2;
volatile int IRflag = LOW;
volatile int state = LOW;

void setup() {
  
  pinMode(BUMP_IN,INPUT);   
  pinMode(IR1_IN, INPUT);
  pinMode(IR2_IN, INPUT);
  pinMode(LED_CLOSE,OUTPUT);
  pinMode(LED_GOOD,OUTPUT);
  pinMode(LED_FAR,OUTPUT);
  pinMode(LED_BUMP,OUTPUT);
  
  attachInterrupt(0,BumpDetect,CHANGE);
  analogComparator.setOn(IR1_IN,IR_REF);
  analogComparator.enableInterrupt(CliffDetect, FALLING);
  
  //Set initial values
  digitalWrite(LED_CLOSE,HIGH);
  digitalWrite(LED_GOOD,HIGH);
  digitalWrite(LED_FAR,HIGH);
  digitalWrite(LED_BUMP,HIGH);
  delay(1000);
  digitalWrite(LED_CLOSE,LOW);
  digitalWrite(LED_GOOD,LOW);
  digitalWrite(LED_FAR,LOW);
  digitalWrite(LED_BUMP,LOW);
  
  IR1 = 0;
  IR2 = 0;
}


void loop(){
  
  delay(100);
  if(IRflag){ 
    digitalWrite(LED_FAR,HIGH);
    digitalWrite(LED_GOOD,LOW);
  }
/*  
IR1 = analogRead(IR1_IN);
IR2 = analogRead(IR2_IN);

if (IR1 > CLOSE){  
	digitalWrite(LED_CLOSE,HIGH);
	digitalWrite(LED_GOOD,LOW);
	digitalWrite(LED_FAR,LOW);
} else if(IR1 < FAR) {
	digitalWrite(LED_CLOSE,LOW);
	digitalWrite(LED_GOOD,LOW);
	digitalWrite(LED_FAR,HIGH);
	}
else{
	digitalWrite(LED_CLOSE,LOW);
	digitalWrite(LED_GOOD,HIGH);
	digitalWrite(LED_FAR,LOW);
	}
*/
}

/*
ISR(ANALOG_COMP_vect)
 {
          IRflag=1; // Kick detected
 }*/
 
void CliffDetect(){
  IRflag = !IRflag;
  digitalWrite(LED_FAR,IRflag);
  digitalWrite(LED_GOOD,!IRflag);
  
}
 
void BumpDetect(){
  state = !state;
  digitalWrite(LED_BUMP,state);
  /*
  if(digitalRead(BUMP_IN)){
    digitalWrite(LED_BUMP,HIGH);
  }else{
    digitalWrite(LED_BUMP,LOW);
  }
  */
}


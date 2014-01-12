/* This is the code for the main Supercaster cart microcontroller (Arduino)
As of 10/31/13 supports:
Radio control X
Remote control 
*/

//Imports
#include <stdlib.h>
#include <EEPROM.h>

//Constants
//Radio
#define buffering 50 //what counts as straight ahead? If too small, the robot will jitter. If too large the robot will drive away from the transmitter
#define STOP_LEVEL 120
//Serial
#define FWD_LIMIT 800
#define LEFT_TURN_LIMIT 800
#define RIGHT_TURN_LIMIT 200

//Pinouts
//Radio
#define ANT_WAVEFORM_OUT 8 // set output pin
#define SPEAKER_FROM_WALKIETALKIE A1 //set input pin
#define CALIBRATE_IN 48
#define LEFT_MOTOR_PIN  10
#define RIGHT_MOTOR_PIN 9
//Serial
#define LED_READY 67
#define LED_RECIEVE 68
#define LED_SW 22
#define LEFT_MOTOR_PIN  10
#define RIGHT_MOTOR_PIN 9

//Variables
//Radio
uint16_t caliset = 0;
uint16_t voltage = 0;
//Remote
String inputString = "";
boolean stringComplete = false;
String Horz = "";
String Vert = "";
String S = "";
int i = 0;
int Horzi = 0;
int Verti = 0;
int Si = 0;

void setup() {
analogReference(DEFAULT);
Serial.begin(9600); //Rx for remote control communication and Tx for serial debugging
delay(6000); // Delay for 6s so that radio reciever can be powered on

//Define/Set pins
//Radio
pinMode(ANT_WAVEFORM_OUT, OUTPUT);
pinMode(SPEAKER_FROM_WALKIETALKIE, INPUT);
pinMode(LEFT_MOTOR_PIN,OUTPUT);
pinMode(RIGHT_MOTOR_PIN,OUTPUT);
pinMode(CALIBRATE_IN, INPUT);
digitalWrite(CALIBRATE_IN, HIGH); // enable internal pullup resistor
//Remote
inputString.reserve(200);  
pinMode(LEFT_MOTOR_PIN,OUTPUT);
pinMode(RIGHT_MOTOR_PIN,OUTPUT);
pinMode(LED_READY,OUTPUT);
pinMode(LED_RECIEVE,OUTPUT);
pinMode(LED_SW,OUTPUT);
digitalWrite(LED_READY,HIGH);
digitalWrite(LED_RECIEVE,LOW);
digitalWrite(LEFT_MOTOR_PIN,HIGH);
digitalWrite(RIGHT_MOTOR_PIN,HIGH);

//read calibration word from EEPROM
byte HByte =  EEPROM.read(1);
byte LByte =  EEPROM.read(2);
caliset = word(HByte, LByte);

if (digitalRead(CALIBRATE_IN) == LOW){ // used for calibratingu
digitalWrite(LEFT_MOTOR_PIN,HIGH);
digitalWrite(RIGHT_MOTOR_PIN,HIGH);
delay(9000); // a wait so you can back away from the robot while it is calibrating
digitalWrite(LEFT_MOTOR_PIN,HIGH);
digitalWrite(RIGHT_MOTOR_PIN,HIGH);
}
}

void loop(){
if(stringComplete){
    
    Si = StringToInt(S);
    Horzi = StringToInt(Horz);
    Verti = StringToInt(Vert);
	inputString = "";
    stringComplete = false;
	}
	
if (Si == 1){ //If Si value is high, use remote control
    digitalWrite(LED_SW,LOW);
    if(Verti > FWD_LIMIT){
      digitalWrite(LEFT_MOTOR_PIN,LOW);
      digitalWrite(RIGHT_MOTOR_PIN,LOW);
    }else if(Horzi > LEFT_TURN_LIMIT){
      digitalWrite(LEFT_MOTOR_PIN,LOW);
      digitalWrite(RIGHT_MOTOR_PIN,HIGH);
    }else if(Horzi < RIGHT_TURN_LIMIT){
      digitalWrite(LEFT_MOTOR_PIN,HIGH);
      digitalWrite(RIGHT_MOTOR_PIN,LOW);
    }else{
      digitalWrite(LEFT_MOTOR_PIN,HIGH);
      digitalWrite(RIGHT_MOTOR_PIN,HIGH);
    }   

}else{ //If Si value is not high, use radio tracking
digitalWrite(LED_SW,HIGH);
digitalWrite(ANT_WAVEFORM_OUT, HIGH);  //output antenna switching waveform
delay(1);
digitalWrite(ANT_WAVEFORM_OUT, LOW);
delay(1);
digitalWrite(ANT_WAVEFORM_OUT, HIGH);
delay(1);
digitalWrite(ANT_WAVEFORM_OUT, LOW);
delay(1);
digitalWrite(ANT_WAVEFORM_OUT, HIGH);
delay(1);
digitalWrite(ANT_WAVEFORM_OUT, LOW);
delay(1);
digitalWrite(ANT_WAVEFORM_OUT, HIGH);
delay(1);
digitalWrite(ANT_WAVEFORM_OUT, LOW);
//delay(1);
delayMicroseconds(600);
voltage = analogRead(SPEAKER_FROM_WALKIETALKIE); //read voltage from radio

if (digitalRead(CALIBRATE_IN) == LOW){           //if in calibrate mode, store voltage in EEPROM
caliset = voltage;
byte HByte = highByte(caliset);
byte LByte = lowByte(caliset);
EEPROM.write(1, HByte);
EEPROM.write(2, LByte);
delay(5000);
endprogram();
}

if (voltage > (caliset - buffering) && voltage < (caliset + buffering)) { //drive forward

//motor control code for FORWARD here
//code for a continuous roating servo is included below
digitalWrite(LEFT_MOTOR_PIN,LOW);
digitalWrite(RIGHT_MOTOR_PIN,LOW);

}


if (voltage > (caliset + buffering)){ //turn

//motor control code for TURNING here (right or left depends on antenna config.)
//code for a continuous roating servo is included below
digitalWrite(LEFT_MOTOR_PIN,HIGH);
digitalWrite(RIGHT_MOTOR_PIN,LOW);
}
if (voltage < (caliset - buffering)){  //turn the other way

//motor control code for TURNING the OTHER DIRECTION here (right or left depends on antenna config.)
//code for a continuous roating servo is included below
digitalWrite(LEFT_MOTOR_PIN,LOW);
digitalWrite(RIGHT_MOTOR_PIN,HIGH);
}

if (voltage < STOP_LEVEL){
digitalWrite(LEFT_MOTOR_PIN,HIGH);
digitalWrite(RIGHT_MOTOR_PIN,HIGH);
}
delay(5); //just a simple wait
}
}
void serialEvent(){
//Remote control serial event
  digitalWrite(LED_RECIEVE,HIGH);
  while(Serial.available()){
    char inChar = (char) Serial.read();
    if(inChar=='V'){
      Vert = inputString;
      inputString = "";
      stringComplete = true;
    }else if(inChar =='H'){
      Horz = inputString;
      inputString = "";
    }else if(inChar =='S'){
      S = inputString;
      inputString = "";
    }
    else{
      inputString += inChar;
    }
  }
  digitalWrite(LED_RECIEVE,LOW);
}

int StringToInt(String str){
//Converts a given string into an integer, used for remote control
  int num = 0;
  char charBuff[5];
  str.toCharArray(charBuff,5);
  num = atoi(charBuff);
  return num;
}

void endprogram(){
  loopy:
  goto loopy;
}


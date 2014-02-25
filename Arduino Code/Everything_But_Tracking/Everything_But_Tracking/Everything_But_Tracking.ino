/* 
This is the code for sensory array on timer interrupts
Sensor characteristics:
  Ultrasonic:
    ~9.8mV per inch
  
  Infrared:
        Dist(cm) (ft)	       Voltage    Experimental Distance (ft)
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

//Constants 
  //Sensors:
#define NUM_US 5
#define NUM_IR 0

#define US_HIT_BUFFER 15

#define CLOSE_INCHES 35 //Approximate stopping distance for US
#define FAR_INCHES 55    //Approx starting distance for US

#define AN_CONVERT 1023/5

#define IR_EDGE 0.44*AN_CONVERT  //Max range in V (~4ft if perpendicular surface)

#define US_CONVERT 0.0098 //(Volts/inch)
#define CLOSE_VOLT US_CONVERT*CLOSE_INCHES  //
#define FAR_VOLT US_CONVERT*FAR_INCHES  //
#define US_CLOSE CLOSE_VOLT*AN_CONVERT
#define US_FAR FAR_VOLT*AN_CONVERT


  //Serial:
#define FWD_ANG 180
#define FWD_LIMIT 772
#define BWD_LIMIT 252
#define LEFT_TURN_LIMIT 1000
#define RIGHT_TURN_LIMIT 20
#define MAX_SPD 1023
#define TURN_SPD MAX_SPD*0.75 //Remember: 0 = full back, 512 = stop, 1024 = full fwd
#define STOP_SPD 512

#define TEAM_NUM 32

//Pinouts
#define KILL_PIN 22

  //Sensors:
#define USFL_PIN A2
#define USFC_PIN A1
#define USFR_PIN A0
#define USLSF_PIN A3
#define USRSF_PIN A4
#define USLSB_PIN A5
#define USRSB_PIN A6
#define USB_PIN A7

#define IRFL_PIN A8
#define IRFC_PIN A9
#define IRFR_PIN A10
#define IRL_PIN A11
#define IRR_PIN A12
#define IRBL_PIN A13
#define IRBR_PIN A14

#define LED_US1 32
#define LED_US2 30
#define LED_US3 31
#define LED_US4 33
#define LED_US5 34
#define LED_US6 35
#define LED_US7 36
#define LED_US8 37

//Global Variables
  //Sensors:
boolean US_flag = 0;
boolean IR_flag = 0;
boolean timer_flag = 0;
boolean stop_flag = 0;
int US_location = 0;
int IR_location = 0;
int US_pins[] = {USFL_PIN,USFC_PIN,USFR_PIN,USLSF_PIN,USRSF_PIN,USLSB_PIN,USRSB_PIN,USB_PIN};
int IR_pins[] = {IRFL_PIN,IRFC_PIN,IRFR_PIN,IRL_PIN,IRR_PIN,IRBL_PIN,IRBR_PIN};
float US_read[] = {0,0,0,0,0,0,0,0};
int US_hitCount[] = {0,0,0,0,0,0,0,0};
float IR_read[] = {0,0,0,0,0,0,0};

int US_LEDs[] = {LED_US1,LED_US2,LED_US3,LED_US4,LED_US5,LED_US6,LED_US7,LED_US8};

  //Serial:
String inputString = "";
boolean stringComplete = false;
String Horz = "";
String Vert = "";
String S = "";
String Ang = "";
String E = "";
String T = "";
int i = 0;
int Horzi = 512;
int Verti = 512;
int Si = 0;      //Transmit State
int Ti = 0;      //Tight Turn
int Ei = 0;      //Emergency Stop
int Anglei = 180;  //Desired Angle


void setup() {
  pinMode(KILL_PIN,OUTPUT);
  digitalWrite(KILL_PIN,LOW);
  
  //Sensors:  
  int i=0;
  for(i=0; i<NUM_US; i++){
    pinMode(US_pins[i],INPUT);
    
    pinMode(US_LEDs[i],OUTPUT);
    digitalWrite(US_LEDs[i],LOW);
  }
  for(i=0;i<NUM_IR;i++){
    pinMode(IR_pins[i],INPUT);
  }
  
  //Serial:
  Serial.begin(9600);  //Arduino Joystick
  Serial1.begin(9600);  //Left Caster PIC
  Serial2.begin(9600);  //Right Caster PIC  
  inputString.reserve(200);
  
  
  //set timer1 interrupt
  TCCR1A = 0;// set entire TCCR1A register to 0
  TCCR1B = 0;// same for TCCR1B
  TCNT1  = 0;//initialize counter value to 0
  OCR1A = 1563;// = (16*10^6) / (1*1024) - 1 (must be <65536)  
  TCCR1B |= (1 << WGM12); // turn on CTC mode  
  TCCR1B |= (1 << CS12) | (1 << CS10);  // Set CS12 and CS10 bits for 1024 prescaler  
  TIMSK1 |= (1 << OCIE1A); // enable timer compare interrupt
  sei();//allow interrupts
}


void loop(){
  //Sensor check:
  if(timer_flag){
    US_location = readUS();
    //IR_location = readIR();
    timer_flag = 0;
    
    //For debug
    Serial.print("US_location: ");
    Serial.print(US_location);
    Serial.print("     ");
    //Serial.print("IR_location: ");
    //Serial.print(IR_location);
    Serial.print("   US Flag: ");
    Serial.print(US_flag);
    //Serial.print("   IR Flag: ");    
    //Serial.print(IR_flag);
    Serial.print('\n');
    
  }
  
  //Stop_flag = IR_flag|US_flag;
  //if(IR_flag){
    //killPower()
  if(US_flag){
    digitalWrite(US_LEDs[US_location], HIGH);
    RightPICSendSerial(190, STOP_SPD);
    LeftPICSendSerial(190, STOP_SPD);
  }else{
    for(int i=0; i<NUM_US; i++){
      digitalWrite(US_LEDs[i], LOW);
    }
  }
  /*
  Serial.print(Ei);
  Serial.print('E');
  Serial.print(Ti);
  Serial.print('T');
  Serial.print(Horzi);
  Serial.print('H');
  Serial.print(Verti);
  Serial.print('V');
  Serial.print(Angi);
  Serial.print("A\n");*/
  
  //Serial:  
 if(stringComplete){ 
    Ei = StringToInt(E);
    Ti = StringToInt(T);
    Si = StringToInt(S);
    Horzi = StringToInt(Horz);
    Verti = StringToInt(Vert);
    //Verti = Verti * (MAX_SPD/1024);
    Anglei = StringToInt(Ang);
    Anglei = Anglei*0.352;
   
    if((Ei == 1) && (US_flag ==0)){
      //If vert value not to extreme, and horz value is, perform an appropriate tank drive turn
      if(Verti < FWD_LIMIT){
        if(Verti > BWD_LIMIT){
          if(Horzi > LEFT_TURN_LIMIT){
              Serial.print("Horzi > LEFT_TURN_LIMIT\n");
              if(Ti){                
                RightPICSendSerial(Anglei, (TURN_SPD));
                LeftPICSendSerial(Anglei, (MAX_SPD-TURN_SPD));
              }else{
                RightPICSendSerial(Anglei, TURN_SPD);
                LeftPICSendSerial(Anglei, (STOP_SPD));
              }
          }else if(Horzi < RIGHT_TURN_LIMIT){
              Serial.print("Horzi < RIGHT_TURN_LIMIT\n");
              if(Ti){
                LeftPICSendSerial(Anglei, (TURN_SPD));
                RightPICSendSerial(Anglei, (MAX_SPD-TURN_SPD));                 
              }else{
                LeftPICSendSerial(Anglei, TURN_SPD);
                RightPICSendSerial(Anglei, (STOP_SPD));   
              }           
          }else{
            Serial.print("RIGHT_TURN_LIMIT < Horzi < LEFT_TURN_LIMIT\n");           
            Verti = Verti/2+256; 
            LeftPICSendSerial(Anglei, Verti);
            RightPICSendSerial(Anglei, Verti);  
          }  
        }else{
            Serial.print("Verti < BWD_LIMIT\n");                    
            Verti = Verti/2+256; 
            LeftPICSendSerial(Anglei, Verti);
            RightPICSendSerial(Anglei, Verti);  
        }    
      }else{
        Serial.print("Verti > FWD_LIMIT\n");           
        Verti = Verti/2+256; 
        LeftPICSendSerial(Anglei, Verti);
        RightPICSendSerial(Anglei, Verti);      
      }
    }else{
      Serial.print("Ei = 0\n");
      RightPICSendSerial(Anglei, STOP_SPD);
      LeftPICSendSerial(Anglei, STOP_SPD);
      //killPower();
    }    
    
    inputString = "";
    stringComplete = false;
 
    delay(50);
 }
  
  
}
//Reads all values of the ultrasonic sensors, returns the array index of a sensor detecting an object
int readUS(){
  int tempVal=0;    
  for(i=0;i<NUM_US;i++){
    US_read[i] = analogRead(US_pins[i]);
    if(US_read[i] < US_CLOSE){
      US_hitCount[i]++;
    }else if(US_read[i] > US_FAR){
      US_hitCount[i] = 0;
    }
  }
  
  for(i=0;i<NUM_US;i++){ 
     if(US_hitCount[i]>US_HIT_BUFFER){
        US_hitCount[i] = US_HIT_BUFFER+1;  
        US_flag = 1;
        return i;
     }
  }  
  
  US_flag = 0;
  return -1;
}
//Reads all values of the infrared sensors, returns the array index of a sensor detecting a cliff
int readIR(){
  int tempVal=0;
  for(i=0;i<NUM_IR;i++){
    tempVal = analogRead(IR_pins[i]);
    if(tempVal < IR_EDGE){
      IR_flag = 1;
      return i;
    }
  }
  IR_flag = 0;
  return -1;
}

//Serial communication protocol for the PIC on the left caster (-> T)
void LeftPICSendSerial(int angle, int spd){
      /*Serial.print("Sending to left PIC ");
      Serial.print(angle);
      Serial.print("A ");
      Serial.print(spd);
      Serial.print("S\n");*/
      Serial1.print(angle);
      delay(15);
      Serial1.print('A');
      delay(15);
      Serial1.print(spd); 
      delay(15);
      Serial1.print('S');
      return;
}
//Serial communication protocol for the PIC on the right caster (T <-)
void RightPICSendSerial(int angle, int spd){
      /*Serial.print("Sending to right PIC: ");
      Serial.print(angle);
      Serial.print("A ");
      Serial.print(spd);
      Serial.print("S\n");*/
      Serial2.print(angle);
      delay(15);
      Serial2.print('A');
      delay(15);
      Serial2.print(spd); 
      delay(15);
      Serial2.print('S');
      return;
}
//Takes a String representing an integer and converts it to an int
int StringToInt(String str){
  int num = 0;
  char charBuff[5];
  str.toCharArray(charBuff,5);
  num = atoi(charBuff);
  return num;
}

//Serial Rx from remote control
void serialEvent(){
  while(Serial.available()){
    char inChar = (char) Serial.read() - TEAM_NUM;
    if(inChar=='A'){
      Ang = inputString;
      inputString = "";
      stringComplete = true;
    }else if(inChar =='V'){
      Vert = inputString;
      inputString = "";
    }else if(inChar =='H'){
      Horz = inputString;
      inputString = "";
    }else if(inChar =='T'){
      T = inputString;
      inputString = "";
    }else if(inChar =='S'){
      S = inputString;
      inputString = "";
    }else if(inChar =='E'){
      E = inputString;
      inputString = "";
    }else if(inChar <= 57 && inChar >= 48){
      inputString += inChar;
    }
  }/*
  while(Serial1.available()){
    char a1 = (char) Serial1.read();
    Serial.print("Left PIC angle: ");
    Serial.print(a1);
    Serial.print("\n");
  }
  while(Serial2.available()){
    char a2 = (char) Serial2.read();
    Serial.print("Right PIC angle: ");
    Serial.print(a2);
    Serial.print("\n");
  }*/
}

ISR(TIMER1_COMPA_vect){//timer1 interrupt 1Hz toggles pin 13 (LED)
//generates pulse wave of frequency 1Hz/2 = 0.5kHz (takes two cycles for full wave- toggle high then toggle low)
  timer_flag=1;
}
 

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
#define NUM_US 8
#define NUM_IR 7

#define US_CONVERT 0.0098 //(Volts/inch)
#define AN_CONVERT 1023/5
#define CLOSE_INCHES 18
#define FAR_INCHES 30
#define CLOSE_VOLT US_CONVERT*CLOSE_INCHES  //
#define FAR_VOLT US_CONVERT*FAR_INCHES  //
#define US_CLOSE CLOSE_VOLT*AN_CONVERT
#define US_FAR FAR_VOLT*AN_CONVERT
#define IR_EDGE 0.44*AN_CONVERT  //~4ft   in V

  //Serial:
#define FWD_ANG 90
#define FWD_LIMIT 712
#define BWD_LIMIT 312
#define LEFT_TURN_LIMIT 912
#define RIGHT_TURN_LIMIT 112
#define TURN_SPD 1024 //Remember: 0 = full back, 512 = stop, 1024 = full fwd
#define MAX_SPD 1024
#define STOP_SPD 512

//Pinouts
  //Sensors:
#define USFL_PIN A0
#define USFC_PIN A1
#define USFR_PIN A2
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
float IR_read[] = {0,0,0,0,0,0,0};

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
int Horzi = 0;
int Verti = 0;
int Si = 0;      //Transmit State
int Ti = 0;      //Tight Turn
int Ei = 0;      //Emergency Stop
int Angi = 0;  //Desired Angle


void setup() {
  //Sensors:  
  int i=0;
  for(i=0; i<NUM_US; i++){
    pinMode(US_pins[i],INPUT);
  }
  for(i=0;i<NUM_IR;i++){
    pinMode(IR_pins[i],INPUT);
  }
  
  //Serial:
  Serial.begin(9600);  //Arduino Joystick
  Serial1.begin(9600);  //Left Caster PIC
  Serial2.begin(9600);  //Right Caster PIC  
  inputString.reserve(200);
  
  
  //set timer1 interrupt at 1Hz
  TCCR1A = 0;// set entire TCCR1A register to 0
  TCCR1B = 0;// same for TCCR1B
  TCNT1  = 0;//initialize counter value to 0
  // set compare match register for 1hz increments
  OCR1A = 1563;// = (16*10^6) / (1*1024) - 1 (must be <65536)
  // turn on CTC mode
  TCCR1B |= (1 << WGM12);
  // Set CS12 and CS10 bits for 1024 prescaler
  TCCR1B |= (1 << CS12) | (1 << CS10);  
  // enable timer compare interrupt
  TIMSK1 |= (1 << OCIE1A);
  sei();//allow interrupts
}


void loop(){
  //Sensor check:
  if(timer_flag){
    US_location = readUS();
    IR_location = readIR();
    timer_flag = 0;
    /* 
    //For debug
    Serial.print("US_location: ");
    Serial.print(US_location);
    Serial.print("     ");
    Serial.print("IR_location: ");
    Serial.print(IR_location);
    Serial.print("   US Flag: ");
    Serial.print(US_flag);
    Serial.print("   IR Flag: ");    
    Serial.print(IR_flag);
    Serial.print('\n');
    */
  }
  
  Stop_flag = IR_flag|US_flag;
  if(IR_flag){
    RightPICSendSerial(90, STOP_SPD);
    LeftPICSendSerial(90, STOP_SPD);
    //killPower()
  }else if(US_flag){
    //Find direction NOT to move in
  }  
  
  //Serial:
    if(stringComplete){
      Ei = StringToInt(E);
      Ti = StringToInt(T);
      Si = StringToInt(S);
      Horzi = StringToInt(Horz);
      Verti = StringToInt(Vert);
      Angi = StringToInt(Ang);
      Angi = Angi*0.352;
      
      if(Ei == 0){
        //If vert value not to extreme, and horz value is, perform an appropriate tank drive turn
        if(Verti < FWD_LIMIT){
          if(Verti > BWD_LIMIT){
            if(Horzi > LEFT_TURN_LIMIT){
                RightPICSendSerial(Angi, TURN_SPD);
                LeftPICSendSerial(Angi, (STOP_SPD));
            }else if(Horzi < RIGHT_TURN_LIMIT){
                LeftPICSendSerial(Angi, TURN_SPD);
                RightPICSendSerial(Angi, (STOP_SPD));
            }else{
              LeftPICSendSerial(Anglei, Verti);
              RightPICSendSerial(Anglei, Verti);  
            }  
          }else{
              LeftPICSendSerial(Anglei, Verti);
              RightPICSendSerial(Anglei, Verti);  
          }  
          //Send angle and y-direction values to the pics
        }else{
          LeftPICSendSerial(Angi, Verti);
          RightPICSendSerial(Angi, Verti);      
        }
       //Emergency Stop: (NOT CURRENTLY IMPLEMENTED TO ENGAGE BREAKS)
      }else{
        RightPICSendSerial(Angi, STOP_SPD);
        LeftPICSendSerial(Angi, STOP_SPD);
        //killPower();
      }    
  
      inputString = "";
      stringComplete = false;
    }
  
  
}
//Reads all values of the ultrasonic sensors, returns the array index of a sensor detecting an object
int readUS(){
  int tempVal=0;
  for(i=0;i<NUM_US;i++){
    tempVal = analogRead(US_pins[i]);
    if(tempVal < US_CLOSE){
      US_flag = 1;
      return i;
    }else if(tempVal > US_FAR){
      US_flag = 0;
    }
  }
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
    char inChar = (char) Serial.read();
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
    }else{
      inputString += inChar;
    }
  }
}

ISR(TIMER1_COMPA_vect){//timer1 interrupt 1Hz toggles pin 13 (LED)
//generates pulse wave of frequency 1Hz/2 = 0.5kHz (takes two cycles for full wave- toggle high then toggle low)
  timer_flag=1;
}
 

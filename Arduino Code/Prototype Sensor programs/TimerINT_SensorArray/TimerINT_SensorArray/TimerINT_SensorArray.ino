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
#define MAX_SPD 1023
#define STOP_SPD 512

#define NUM_US 0
#define NUM_IR 5
#define NUM_B 0

#define CLOSE_INCHES 30
#define FAR_INCHES 60
#define US_CONVERT 0.0098 //(Volts/inch)

#define CLOSE_VOLT US_CONVERT*CLOSE_INCHES  //
#define FAR_VOLT US_CONVERT*FAR_INCHES  // 

#define AN_CONVERT 1023/5
#define US_CLOSE CLOSE_VOLT*AN_CONVERT
#define US_FAR FAR_VOLT*AN_CONVERT

//#define IR_EDGE 0.44*AN_CONVERT  //~4ft   in V
#define IR_VOLT .5
#define IR_EDGE IR_VOLT*AN_CONVERT

//Pinouts
/*
#define USFL_PIN A2
#define USFC_PIN A1
#define USFR_PIN A0
#define USLSF_PIN A3
#define USRSF_PIN A4
#define USLSB_PIN A5
#define USRSB_PIN A6
#define USB_PIN A7*/

#define USFL_PIN A2
#define USFC_PIN A1
#define USFR_PIN A0
#define USLSF_PIN A3
#define USRSF_PIN A4
#define USLSB_PIN A5
#define USRSB_PIN A6
#define USB_PIN A7

#define IRFL_PIN A9
#define IRFC_PIN A8
#define IRFR_PIN A10
#define IRL_PIN A11
#define IRR_PIN A12
#define IRBL_PIN A13
#define IRBR_PIN A14

#define BFL_PIN 24
#define BFC_PIN 25
#define BFR_PIN 26
#define BLSF_PIN 27
#define BRSF_PIN 28
#define BLSB_PIN 29
#define BRSB_PIN 30
#define BB_PIN 31

#define LED_IR1 40
#define LED_IR2 41
#define LED_IR3 42
#define LED_IR4 43
#define LED_IR5 44
#define LED_IR6 45
#define LED_IR7 46
/*
#define LED_US1 32
#define LED_US2 30
#define LED_US3 31
#define LED_US4 33
#define LED_US5 34
#define LED_US6 35
#define LED_US7 36
#define LED_US8 37*/
#define US_FLAG_LED 33

#define KILL_PIN 22

/*
#define LED_US_CLOSE 44
#define LED_GOOD 45
#define LED_IR_FAR 46*/

//Global Variables

boolean timer_flag = 0;
boolean US_flag = 0;
boolean IR_flag = 0;

int US_location = 0;
int IR_location = 0;
int US_pins[] = {USFL_PIN,USFC_PIN,USFR_PIN,USLSF_PIN,USRSF_PIN,USLSB_PIN,USRSB_PIN,USB_PIN};
int IR_pins[] = {IRFL_PIN,IRFC_PIN,IRFR_PIN,IRL_PIN,IRR_PIN,IRBL_PIN,IRBR_PIN};
float US_read[] = {0,0,0,0,0,0,0,0};
float IR_read[] = {0,0,0,0,0,0,0};

boolean B_flag = 0;
int B_location = 0;
int B_pins[] = {BFL_PIN,BFC_PIN,BFR_PIN,BLSF_PIN,BRSF_PIN,BLSB_PIN,BRSB_PIN,BB_PIN};

int IR_LEDs[] = {LED_IR1,LED_IR2,LED_IR3,LED_IR4,LED_IR5,LED_IR6,LED_IR7};
//int US_LEDs[] = {LED_US1,LED_US2,LED_US3,LED_US4,LED_US5,LED_US6,LED_US7,LED_US8};

void setup() {  
  int i;  
  Serial.begin(9600);
  Serial1.begin(9600);  //Left Caster PIC
  Serial2.begin(9600);  //Right Caster PIC  
  
  
  //for(i=0; i< NUM_US; i++){
  for(i=0; i< NUM_US; i++){  
    pinMode(US_pins[i],INPUT);
    //pinMode(US_LEDs[i],OUTPUT);
    //digitalWrite(US_LEDs[i],LOW);
  }
  
  pinMode(US_FLAG_LED,OUTPUT);
  digitalWrite(US_FLAG_LED,LOW);
  
  //for(i=0;i<NUM_IR;i++){
  for(i=0; i< NUM_IR; i++){
    pinMode(IR_pins[i],INPUT);
    pinMode(IR_LEDs[i],OUTPUT);
    digitalWrite(IR_LEDs[i],LOW);
  }
  
  for(i=0; i< NUM_B; i++){
    pinMode(B_pins[i],INPUT);
  }
  pinMode(KILL_PIN,OUTPUT);
  digitalWrite(KILL_PIN,LOW);
  
 /*
  pinMode(LED_US_CLOSE,OUTPUT);
  pinMode(LED_GOOD,OUTPUT);
  pinMode(LED_IR_FAR,OUTPUT);
  /*
  //Set initial values
  digitalWrite(LED_US_CLOSE,HIGH);
  digitalWrite(LED_GOOD,HIGH);
  digitalWrite(LED_IR_FAR,HIGH);
  delay(500);
  digitalWrite(LED_US_CLOSE,LOW);
  digitalWrite(LED_GOOD,LOW);
  digitalWrite(LED_IR_FAR,LOW);
  */
  
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
  if(timer_flag){
    US_location = readUS();
    IR_location = readIR();
    //B_location = readB();  
    timer_flag = 0;
    
    //Serial.print("\n");
    
    //Serial.print("US_location: ");
    //Serial.print(US_location);
    //Serial.print("     ");
    //Serial.print("  IR_location: ");
    //Serial.print(IR_location);
    //Serial.print("   US Flag: ");
    //Serial.print(US_flag);
    //Serial.print("   IR Flag: ");    
    //Serial.print(IR_flag);
    //Serial.print('\n');
  }
  /*if(IR_flag){
    //RightPICSendSerial(90, STOP_SPD);
    //LeftPICSendSerial(90, STOP_SPD);
    //killPower()
  }else if(US_flag){
    
    RightPICSendSerial(90, STOP_SPD);
    LeftPICSendSerial(90, STOP_SPD);
  }else{
    RightPICSendSerial(90, MAX_SPD);
    LeftPICSendSerial(90, MAX_SPD);
  }*/
  
  
  if(IR_flag){
    digitalWrite(IR_LEDs[IR_location], HIGH);
    //digitalWrite(KILL_PIN,HIGH);
    
    LeftPICSendSerial(180, 512);
    RightPICSendSerial(180, 512);
    delay(100);
  }else{
    for(int i=0; i<NUM_IR; i++){
      digitalWrite(IR_LEDs[i], LOW);
    }
    digitalWrite(KILL_PIN,LOW);
    LeftPICSendSerial(180, 850);
    RightPICSendSerial(180, 850);
  }
  if(B_flag){
    //do something
  }
  if(US_flag){
    digitalWrite(US_FLAG_LED, HIGH);
  }else{
    //for(int i=0; i<NUM_US; i++){
      digitalWrite(US_FLAG_LED, LOW);
    //}
  }
    /*
  digitalWrite(LED_US_CLOSE,US_flag);
  digitalWrite(LED_IR_FAR,IR_flag);
  digitalWrite(LED_GOOD,!(US_flag|IR_flag));*/
  delay(200);
}

int readB(){
  for(int i=0;i<NUM_B;i++){
    if(digitalRead(B_pins[i])==0){ 
        B_flag = 1;
        return i;
    }
  }
  B_flag = 0;
  return -1;
}

int readUS(){
  int tempVal=0;
  float vltg = 0;
  float inch = 0;
  US_flag = 0;
  for(int i=0;i<NUM_US;i++){    
    tempVal = analogRead(US_pins[i]);
    
    /*Serial.print(" P: ");
    Serial.print(US_pins[i]);
    Serial.print(" V: ");
    Serial.print(tempVal);
    Serial.print(" CV: ");
    Serial.print(US_CLOSE);
    vltg = tempVal/(AN_CONVERT);
    Serial.print(" Vltg: ");
    Serial.print(vltg);
    Serial.print(" In: ");
    inch = vltg/US_CONVERT;
    Serial.print(inch);*/
    
    if(tempVal < US_CLOSE){
      US_flag = 1;
      return i;
    }else if(tempVal > US_FAR){
      //US_flag = 0;
    }
  }  
  return -1;
}

int readIR(){
  int tempVal=0;
  for(int i=0;i<NUM_IR;i++){
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

ISR(TIMER1_COMPA_vect){
//generates pulse wave of frequency 
  timer_flag=1;
}
 

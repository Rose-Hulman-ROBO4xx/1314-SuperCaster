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
#include <Servo.h> 
#include <EEPROM.h>


//Constants 
  //Sensors:
#define NUM_US 2
#define NUM_IR 0

#define US_HIT_BUFFER 15

#define CLOSE_INCHES 30 //Approximate stopping distance for US
#define FAR_INCHES 40    //Approx starting distance for US

#define AN_CONVERT 1023/5

#define IR_EDGE 0.5*AN_CONVERT  //Max range in V (~4ft if perpendicular surface)

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

//////ANT STUFF
#define ANT_WAVEFORM 8
#
 
    // variable to store the servo position 
#define MIN_POS 1000 //120 Degrees
#define MAX_POS 2100  //240 Degrees
#define DELAY 5
#define DEFAULT_SERVO_M 1550
#define N180_DEG_M 1550
#define DEFAULT_SERVO 90 
#define STEP 2
#define SERVO_TURN 20
#define TURN_ANGLE_M_MAX 1800
#define TURN_ANGLE_M_MIN 1200
#define MIN_ANG 120
#define MAX_ANG 240

#define SPEAKER_FROM_WALKIETALKIE A15 //set input pin
#define CALIBRATE_IN 48
#define RADIO_POWER 46
#define RADIO_VOL_UP 47
#define LEFT_MOTOR_PIN  10
#define RIGHT_MOTOR_PIN 9
#define SERVO_PIN 3

#define buffering 15 //what counts as straight ahead? If too small, the robot will jitter. If too large the robot will drive away from the transmitter
#define sample_delay 5 //Number of edges detected before polling for value
#define antenna_sample_size 20 //Number of samples taken before decision is made.
#define STOP_LEVEL_MAX 120
#define STOP_LEVEL_MIN 50



uint16_t caliset = 0;
volatile uint16_t voltage = 0;
volatile int edge_count = 0;

int goal_ang_micro = DEFAULT_SERVO_M;
volatile boolean finished_move = 1;
volatile int interrupt_count_servo = 0;
volatile int current_pos_micro = N180_DEG_M;
boolean move_enabled = 1;
boolean toggle = 0;
volatile int estate = LOW;
int angle_2_casters = 180;
Servo antenna_servo;
boolean ant_read_flag = false;
boolean samples_full = false;
int num_samples_taken = 0;
uint16_t voltageReadings[antenna_sample_size];


//////



//Global Variables
  //Sensors:
boolean US_flag = 0;
boolean IR_flag = 0;
boolean timer_flag = 0;
int sensor_timer_count = 0;
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
  
  Serial.println("In setup");
  pinMode(ANT_WAVEFORM,OUTPUT);
  pinMode(SPEAKER_FROM_WALKIETALKIE, INPUT);
  pinMode(LEFT_MOTOR_PIN,OUTPUT);
  pinMode(RIGHT_MOTOR_PIN,OUTPUT);
  pinMode(CALIBRATE_IN, INPUT);
  pinMode(RADIO_POWER,OUTPUT);
  pinMode(RADIO_VOL_UP,OUTPUT);
  digitalWrite(CALIBRATE_IN, HIGH);
  digitalWrite(LEFT_MOTOR_PIN,HIGH);
  digitalWrite(RIGHT_MOTOR_PIN,HIGH);

  //Serial.println("Set Output mode");
  antenna_servo.attach(3);  // attaches the servo on pin 9 to the servo object
  antenna_servo.writeMicroseconds(current_pos_micro);
  //Serial.println("Set up Servo,attempting timer");
  
  byte HByte =  EEPROM.read(1);
  byte LByte =  EEPROM.read(2);
  caliset = word(HByte, LByte);
  Serial.print("EEPROM Calibration number: ");
  Serial.print(caliset);
  Serial.println(" If you haven't calibrated yet, you need to for it to work");
  
  cli();
//  //set timer1 interrupt
//  TCCR1A = 0;// set entire TCCR1A register to 0
//  TCCR1B = 0;// same for TCCR1B
//  TCNT1  = 0;//initialize counter value to 0
//  OCR1A = 1563;// = (16*10^6) / (1*1024) - 1 (must be <65536)  
//  TCCR1B |= (1 << WGM12); // turn on CTC mode  
//  TCCR1B |= (1 << CS12) | (1 << CS10);  // Set CS12 and CS10 bits for 1024 prescaler  
//  TIMSK1 |= (1 << OCIE1A); // enable timer compare interrupt
//  
  //set timer2 interrupt at 8kHz
  TCCR2A = 0;// set entire TCCR2A register to 0
  TCCR2B = 0;// same for TCCR2B
  TCNT2  = 0;//initialize counter value to 0
  // set compare match register for 8khz increments
  OCR2A = 250;// = (16*10^6) / (1000*64) - 1 (must be <256)
  // turn on CTC mode
  TCCR2A |= (1 << WGM21);
  // Set CS22 bit for 64 prescaler
  TCCR2B |= (1 << CS22);   
  // enable timer compare interrupt
  TIMSK2 |= (1 << OCIE2A);
  sei();//allow interrupts
}


void loop(){
  


////////ANT Stuff
  if (digitalRead(CALIBRATE_IN) == LOW){    //if in calibrate mode, store voltage in EEPROM
    delay(5000);
    while(ant_read_flag == 0){
      delay(1);
    }
    caliset = voltage;
    byte HByte = highByte(caliset);
    byte LByte = lowByte(caliset);
    EEPROM.write(1, HByte);
    EEPROM.write(2, LByte);
    delay(5000);
    while(1){
      delay(1);
    }
  }
  
  
  angle_2_casters = map(current_pos_micro,MIN_POS,MAX_POS,MIN_ANG,MAX_ANG);
  if(samples_full){
    handleAntennaReadings();
  }
 
}
//Reads all values of the ultrasonic sensors, returns the array index of a sensor detecting an object
void handleAntennaReadings(){
    samples_full = 0;
    int num_stop = 0;
    int num_hold = 0;
    int num_increase = 0;
    int num_decrease = 0;
    for(int i = 0; i < antenna_sample_size; i+=1){
    if (voltageReadings[i] < STOP_LEVEL_MAX && voltageReadings[i] > STOP_LEVEL_MIN){
      num_stop+=1;
    } else {
      if (voltageReadings[i] > (caliset - buffering) && voltageReadings[i] < (caliset + buffering)) { //drive forward
        num_hold+=1;
    }
   
    if (voltageReadings[i] < (caliset -buffering)){ //turn  voltage > (caliset + buffering)
    num_decrease+=1;

    }
    
    if (voltageReadings[i] > (caliset + buffering)){  //turn the other way
    num_increase+=1;

    }
  }
 }
 
 samples_full = 0; //Data processed, reset flag, and tell interupt it can begin to fill buffer up again.
 num_samples_taken = 0;
 
 int sample_array_f[4] = {num_stop, num_hold, num_increase,num_decrease};
 char case_num = 0;
 for(int i = 0; i < 4; i+=1){
   if(sample_array_f[i] > sample_array_f[case_num]) case_num = i;
 }
 switch (case_num) {
  case 0:
     Serial.print("Stop Movement ");
    break;
   case 1:
     Serial.print("Hold angle ");
     break;
   case 2:
     Serial.print("Decrease angle ");
      if (current_pos_micro > MIN_POS)
      goal_ang_micro = current_pos_micro - SERVO_TURN;
      else current_pos_micro = MIN_POS;
     break;
   case 3:
     Serial.print("Increase angle ");
      if (current_pos_micro < MAX_POS)
      goal_ang_micro = current_pos_micro + SERVO_TURN;
      else goal_ang_micro = MAX_POS;
      break;
   default:
     Serial.print("Error ");
     break;
 }
 
  Serial.print("In, Cal\t");
  Serial.print(voltage);
  Serial.print("\t");
  Serial.print(caliset);
  Serial.print(" Position ");
  Serial.print(current_pos_micro);
  Serial.print(" goal ");
  Serial.print(goal_ang_micro);
  Serial.print(" Agreed ");
  Serial.println(sample_array_f[case_num]);
  
//  Serial.print(" Cast Ang ");
//  Serial.print(angle_2_casters);
  
//  RightPICSendSerial(angle_2_casters, STOP_SPD);
//  LeftPICSendSerial(angle_2_casters, STOP_SPD); 
  
}


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

//ISR(TIMER1_COMPA_vect){//timer1 interrupt 1Hz toggles pin 13 (LED)
////generates pulse wave of frequency 1Hz/2 = 0.5kHz (takes two cycles for full wave- toggle high then toggle low)
//  timer_flag=1;
//}
ISR(TIMER2_COMPA_vect){//timer2 interrupt 1kHz
  digitalWrite(ANT_WAVEFORM,toggle);
  toggle = toggle^1;
  edge_count+=1;
  if (edge_count >= sample_delay*2){
    estate = !estate;
    edge_count = 0;
    voltage = analogRead(SPEAKER_FROM_WALKIETALKIE);
    if (num_samples_taken < antenna_sample_size){
    voltageReadings[num_samples_taken] = voltage;
    num_samples_taken +=1;
    } else {
      samples_full = 1;
    }
    ant_read_flag = 1;
  }
  //SERVO Handling
  if (interrupt_count_servo >= DELAY){
    if (move_enabled){
      if (current_pos_micro > goal_ang_micro ){
        current_pos_micro-=STEP;
        finished_move = 0;
      } else if (current_pos_micro < goal_ang_micro) {
        current_pos_micro+=STEP;
        finished_move = 0;
      } else {
        finished_move = 1;
      }
    }
    interrupt_count_servo = 0;
    antenna_servo.writeMicroseconds(current_pos_micro);
    //
  } else interrupt_count_servo+=1;
  //moveantenna
  //antenna_servo.writeMicroseconds(current_pos_micro);
  //Antenna Reading
  //Sensor Timer
  if (sensor_timer_count == 100) {
    sensor_timer_count = 0;
    timer_flag = 1;
  } else sensor_timer_count +=1;
}
 

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

/////////////////////////////Imports
#include <stdlib.h>
#include <Servo.h> b
#include <EEPROM.h>

/////////////////////////Constants 
  //Sensors:
#define NUM_US 8
#define NUM_IR 5
#define NUM_B 6

#define US_HIT_BUFFER 5
#define IR_HIT_BUFFER 1
#define SENSOR_READING_DELAY 100 //delay between sensor readings in ms

#define CLOSE_INCHES 30 //Approximate stopping distance for US
#define FAR_INCHES 50    //Approx starting distance for US

#define AN_CONVERT 1023/5

#define IR_VOLT 0.5
//#define IR_VOLT_CASTER 0.55
#define IR_EDGE IR_VOLT*AN_CONVERT  //Max range in V (~4ft if perpendicular surface)
//#define IR_EDGE_CASTER IR_VOLT_CASTER*AN_CONVERT  //Max range in V (~4ft if perpendicular surface)

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

#define TEAM_NUM 0

//Pinouts
#define KILL_PIN 22
#define RC_PIN 46

  //Sensors:
#define USFL_PIN A2
#define USFC_PIN A1
#define USFR_PIN A0
#define USLSF_PIN A3
#define USRSF_PIN A4
#define USLSB_PIN A5
#define USRSB_PIN A6
#define USB_PIN A7

#define USFL_EN 12
#define USFC_EN 9
#define USFR_EN 13
#define USLSF_EN 8
#define USRSF_EN 11
#define USLSB_EN 7
#define USRSB_EN 10
#define USB_EN 6

#define IRFL_PIN A8
#define IRFC_PIN A9
#define IRFR_PIN A10
#define IRL_PIN A11
#define IRR_PIN A12
#define IRBL_PIN A13
#define IRBR_PIN A14

#define BFL_PIN 23
#define BFC_PIN 25
#define BFR_PIN 27
#define BLSF_PIN 29
#define BRSF_PIN 31
#define BLSB_PIN 33
#define BRSB_PIN 35
#define BB_PIN 37

#define LED_US_FLAG 30
#define LED_IR_FLAG 32
#define LED_BP_FLAG 34



//////Tracking defines
//Pins
#define ANT_WAVEFORM 2
#define SPEAKER_FROM_WALKIETALKIE A15 //set input pin
#define CALIBRATE_IN 44
#define RADIO_POWER 24
#define RADIO_VOL_UP 26
#define RADIO_VOL_DWN 28
#define SERVO_PIN 3
 
// Servo defines
#define MIN_POS 750 //120 Degrees. Servo PWM in microseconds required to reach minimum usable angle (120 at current settings)
#define MAX_POS 1850  //240 Degrees. Servo PWM in microseconds required to reach maximum usable angle (240 at current settings)
#define DELAY 2 //delay between PWM writes to servo. Slows down turn
#define DEFAULT_SERVO_M 1275 //Default servo angle in microseconds PWM.
#define DEFAULT_SERVO 90 //default servo angle in degrees
#define STEP 5  //amount servo adds when trying to reach goal angle. in microseconds
#define SERVO_TURN 60 //amount to add to PWM values to sevo when tracking. in microseconds
#define MIN_ANG 120 //servo min in degrees
#define MAX_ANG 240 //servo max in degrees

//tracking parameters
#define buffering 10 //what counts as straight ahead? If too small, the robot will jitter. If too large the robot will drive away from the transmitter
#define sample_delay 10 //Number of edges detected before polling for value
#define antenna_sample_size 3  //Number of samples taken before decision is made.
#define STOP_LEVEL_MAX 120
#define STOP_LEVEL_MIN 0

//Enumerators for direction determination
#define STOP_MOV 1
#define HOLD_ANG 2
#define INC_ANG 3
#define DEC_ANG 4
#define ERROR_CALC -1


////////////////////////////Global Variables
  //Sensors:
boolean US_flag = 0;
boolean IR_flag = 0;
boolean B_flag = 0;
boolean timer_flag = 0;
int sensor_timer_count = 0;
boolean stop_flag = 0;

int US_location = 0;
int IR_location = 0;
int B_location = 0;
int US_grpCount = 0;

int US_pins[] = {USFL_PIN,USFC_PIN,USFR_PIN,USLSF_PIN,USRSF_PIN,USLSB_PIN,USRSB_PIN,USB_PIN};
int US_grp1[] = {USFL_PIN,USRSF_PIN,USLSB_PIN};
int US_grp2[] = {USFR_PIN,USLSF_PIN,USRSB_PIN};
int US_grp3[] = {USFC_PIN,USB_PIN};

int US_enables[] = {USFL_EN,USRSF_EN,USLSB_EN,USFR_EN,USLSF_EN,USRSB_EN,USFC_EN,USB_EN};

int IR_pins[] = {IRFL_PIN,IRFC_PIN,IRFR_PIN,IRL_PIN,IRR_PIN,IRBL_PIN,IRBR_PIN};
int B_pins[] = {BFL_PIN,BFC_PIN,BFR_PIN,BLSF_PIN,BRSF_PIN,BLSB_PIN,BRSB_PIN,BB_PIN};

float US_read[] = {0,0,0,0,0,0,0,0};
int US_hitCount[] = {0,0,0,0,0,0,0,0};
float IR_read[] = {10,10,10,10,10,10,10};
int IR_hitCount[] = {0,0,0,0,0,0,0};

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
int trackSpeed = 0;


//Tracking Variables

uint16_t caliset = 0;
volatile uint16_t voltage_1 = 0;
volatile uint16_t voltage_2 = 0;
volatile int edge_count = 0;

int goal_ang_micro = DEFAULT_SERVO_M;
volatile boolean finished_move = 1;
volatile int interrupt_count_servo = 0;
volatile int current_pos_micro = DEFAULT_SERVO_M;
boolean move_enabled = 1; //0 locks antenna to default pos
boolean toggle = 0;
boolean second_sample_flag = 0;
int angle_2_casters = 180;
Servo antenna_servo;
boolean ant_read_flag = false;
boolean samples_full = false;
int num_samples_taken = 0;
uint16_t voltageReadings_1[antenna_sample_size]; //array for rising edge samples
uint16_t voltageReadings_2[antenna_sample_size]; //array for falling edge samples
boolean drive_command = STOP_MOV;

void setup() {
  
  pinMode(KILL_PIN,OUTPUT); //start kill pin
  digitalWrite(KILL_PIN,LOW);
  
  pinMode(RC_PIN,INPUT_PULLUP);
  
  //Sensors:  
  int i=0;
  for(i=0; i<NUM_US; i++){
    pinMode(US_pins[i],INPUT);    
    pinMode(US_enables[i],OUTPUT);
    
    
    //Enable group 1 initially
    if(i<3){
       digitalWrite(US_enables[i],HIGH);
    }else{
       digitalWrite(US_enables[i],LOW);
    }  
  }
  
  for(i=0;i<NUM_IR;i++){ //setup IR
    pinMode(IR_pins[i],INPUT);
  }
  
  for(i=0; i< NUM_B; i++){ //setup bump sensors
    pinMode(B_pins[i],INPUT);
  }
  
  pinMode(LED_US_FLAG,OUTPUT);   //debug LEDs
  digitalWrite(LED_US_FLAG,LOW);
  
  pinMode(LED_IR_FLAG,OUTPUT);  
  digitalWrite(LED_IR_FLAG,LOW);
  
  pinMode(LED_BP_FLAG,OUTPUT);  
  digitalWrite(LED_BP_FLAG,LOW);
  
  //Serial:
  Serial.begin(9600);  //Arduino Remote Control
  Serial1.begin(9600);  //Left Caster PIC
  Serial2.begin(9600);  //Right Caster PIC  
  inputString.reserve(200);
  
  //Antenna Pins
  pinMode(ANT_WAVEFORM,OUTPUT);
  pinMode(SPEAKER_FROM_WALKIETALKIE, INPUT);
  pinMode(CALIBRATE_IN, INPUT);
  pinMode(RADIO_POWER,OUTPUT);
  pinMode(RADIO_VOL_UP,OUTPUT);
  pinMode(RADIO_VOL_DWN,OUTPUT);
  digitalWrite(CALIBRATE_IN, HIGH);
  
  powerUpRadio();

  antenna_servo.attach(3);  // attaches the servo on pin 9 to the servo object
  antenna_servo.writeMicroseconds(current_pos_micro);
  
  byte HByte =  EEPROM.read(1);
  byte LByte =  EEPROM.read(2);
  caliset = word(HByte, LByte);
  
  cli(); //disable interupts
  
   //set timer2 interrupt at 1kHz
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
  
  Serial.print('1');
  delay(100);
}




void loop(){
  //Sensor check:  
  
  
  Serial.print('1'); // Tell Remote Control that master arduino is ready to communicate 
  delay(1);
  
  if(stringComplete){// if arduino has recived full command, process new command and set appropriate variables
      Ei = StringToInt(E);
      Ti = StringToInt(T);
      Si = StringToInt(S);
      Horzi = StringToInt(Horz);
      Verti = StringToInt(Vert);
      //Verti = Verti * (MAX_SPD/1024);
      Anglei = StringToInt(Ang);
      inputString = "";
      stringComplete = false;
 
      
    if(Ei==0){ // emergency stop flag, kill power
       killPower();
       
    }
    
    if(Si==1){    // tracking enabled, enter tracking
      //Run Tracking and enable all safety sensors
      if(timer_flag){  
        updateTrackingSensors();
      }
      
      //Antenna Readings, no caster movement, calculate tracking angle
      angle_2_casters = map(current_pos_micro,MIN_POS,MAX_POS,MAX_ANG,MIN_ANG);
        if(samples_full){//if antenna buffer is full process data
          if(Ti) move_enabled = 1; //determine tracking mode
          else move_enabled = 0;
          handleAntennaReadings();
          if(!Ti) {current_pos_micro = DEFAULT_SERVO_M;
          goal_ang_micro = DEFAULT_SERVO_M;}
        }
        
      if((!B_flag)&&(!US_flag)&&(!IR_flag)&&(Ei==1)){
        trackSpeed = Verti; //process tracking speed      
       
       if (Ti) {
       //Strafe
       
        RightPICSendSerial(angle_2_casters, trackSpeed);//set angle to servo angle
        LeftPICSendSerial(angle_2_casters, trackSpeed);
       }  else {
         
         if(drive_command == DEC_ANG) {//right turn
          RightPICSendSerial(180, STOP_SPD);
          LeftPICSendSerial(180, trackSpeed);
       } else if (drive_command == INC_ANG) {//turn left
          RightPICSendSerial(180, trackSpeed);
          LeftPICSendSerial(180, STOP_SPD);
       } else if(drive_command == HOLD_ANG) {//hold straight
          RightPICSendSerial(180, trackSpeed);
          LeftPICSendSerial(180, trackSpeed);        
       } else {
         RightPICSendSerial(angle_2_casters, STOP_SPD); //stop casters if no signal detected
        LeftPICSendSerial(angle_2_casters, STOP_SPD);
       }
       }      
      }else{
        RightPICSendSerial(angle_2_casters, STOP_SPD); //stop for sensor reading
        LeftPICSendSerial(angle_2_casters, STOP_SPD);
      }
      
      
   }else{ //Si is 0, go into remote control, only checking bump and cliff sensors       
     if(timer_flag){
       updateRCSensors();
     }
      //Serial:  
      Anglei = Anglei*0.352;
      goal_ang_micro = DEFAULT_SERVO_M;
      if((!B_flag)&&(!IR_flag)&&(Ei == 1)){
        //If vert value not to extreme, and horz value is, perform an appropriate tank drive turn
        if(Verti < FWD_LIMIT){ //process controller input to set caster to correct values
          if(Verti > BWD_LIMIT){
            if(Horzi > LEFT_TURN_LIMIT){ 
                if(Ti){                
                  RightPICSendSerial(Anglei, (TURN_SPD));
                  LeftPICSendSerial(Anglei, (MAX_SPD-TURN_SPD)); 
                }else{
                  RightPICSendSerial(Anglei, TURN_SPD);
                  LeftPICSendSerial(Anglei, (STOP_SPD));
                }
            }else if(Horzi < RIGHT_TURN_LIMIT){
                if(Ti){
                  LeftPICSendSerial(Anglei, (TURN_SPD));
                  RightPICSendSerial(Anglei, (MAX_SPD-TURN_SPD));            
                }else{
                  LeftPICSendSerial(Anglei, TURN_SPD);
                  RightPICSendSerial(Anglei, (STOP_SPD));   
                }           
            }else{
              LeftPICSendSerial(Anglei, STOP_SPD);
              RightPICSendSerial(Anglei, STOP_SPD);  
            }  
          }else{
              Verti = Verti/2+256; 
              LeftPICSendSerial(Anglei, Verti);
              RightPICSendSerial(Anglei, Verti);  
          }    
        }else{
          Verti = Verti/2+256; 
          LeftPICSendSerial(Anglei, Verti);
          RightPICSendSerial(Anglei, Verti);      
        }  
      }else{
        LeftPICSendSerial(Anglei, STOP_SPD);
        RightPICSendSerial(Anglei, STOP_SPD);  
      }  
      delay(10);   
     }
  }
}
  
void killPower(){ //kills power to robot.
  digitalWrite(KILL_PIN,HIGH);
  RightPICSendSerial((Anglei*0.352), STOP_SPD);
  LeftPICSendSerial((Anglei*0.352), STOP_SPD);
  
}

void updateTrackingSensors(){ //called in loop, updates tracking sensors checked every .1 seconds
  IR_location = readIR();
  US_location = readUS();
  B_location = readB();
  timer_flag = 0;      
  
  //any flags trigger appropriate responses
  
  if(IR_flag){
    killPower();
  }
  if(B_flag){
    
    digitalWrite(LED_BP_FLAG,B_flag);
    RightPICSendSerial(angle_2_casters, STOP_SPD);
    LeftPICSendSerial(angle_2_casters, STOP_SPD);
    delay(200);
  }else if(US_flag){      
    digitalWrite(LED_US_FLAG,US_flag);      
    RightPICSendSerial(angle_2_casters, STOP_SPD);
    LeftPICSendSerial(angle_2_casters, STOP_SPD);
  }else{
    digitalWrite(LED_US_FLAG,LOW);
    digitalWrite(LED_BP_FLAG,LOW);
  }
}

void updateRCSensors(){ //Updates remote control sensors, flags appropriate response checked every .1 seconds 
  IR_location = readIR();
  B_location = readB();
  timer_flag = 0;      
  
  //Stop_flag = IR_flag|US_flag;
  if(IR_flag){
    killPower();
  }
  if(B_flag){
    //digitalWrite(US_LEDs[US_location], HIGH);
    digitalWrite(LED_BP_FLAG,B_flag);
    RightPICSendSerial((Anglei*0.352), STOP_SPD);
    LeftPICSendSerial((Anglei*0.352), STOP_SPD);
    delay(200);
  }else{
    digitalWrite(LED_BP_FLAG,LOW);   
  }
}

//Read all of the bump sensor values, return the array index of the triggered bump sensor. 
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

//Reads all values of the ultrasonic sensors, returns the array index of a sensor detecting an object
int readUS(){
  int tempVal=0;      
  //Read thrid US group
    
  if(US_grpCount == 2){
    //Read group 3
    for(i=0;i<2;i++){
      US_read[i+6] = analogRead(US_grp3[i]);      
    }    
    //Disable group 3
    for(i=0;i<3;i++){
       digitalWrite(US_enables[i+6],LOW);
    }
    //Enable group 1
    for(i=0;i<3;i++){
       digitalWrite(US_enables[i],HIGH);
    }          
    US_grpCount = 0;      
  
  //Read second US group 
  }else if(US_grpCount == 1){
    //Read group 2
    for(i=0;i<3;i++){
      US_read[i+3] = analogRead(US_grp2[i]);
      
    }
    //Disable group 2
    for(i=0;i<3;i++){
       digitalWrite(US_enables[i+3],LOW);
    }
    //Enable group 3
    for(i=0;i<3;i++){
       digitalWrite(US_enables[i+6],HIGH);
    }      
    US_grpCount++;
    
  //Read first US group
  }else if(US_grpCount == 0){
    //Read group 1
    for(i=0;i<3;i++){
      US_read[i] = analogRead(US_grp1[i]);
    }    
    
    //Disable group 1
    for(i=0;i<3;i++){
       digitalWrite(US_enables[i],LOW);
    }
    //Enable group 2
    for(i=0;i<3;i++){
       digitalWrite(US_enables[i+3],HIGH);
    }    
    US_grpCount++;
  }
  
  delay(15);
    
  for(i=0;i<NUM_US;i++){ 
     if(US_read[i] < US_CLOSE){
        US_hitCount[i]++;
     }else if(US_read[i] > US_FAR){
        US_hitCount[i] = 0;
     }
      
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
      spd = 1023-spd;
      Serial1.print(angle);
      delay(15);
      Serial1.print('A');
      delay(15);
      Serial1.print(spd); 
      delay(15);
      Serial1.print('S');
/*      Serial.print("L: ang ");
      Serial.print(angle);
      Serial.print('\n');
      */
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
    char inChar = (char) Serial.read() - TEAM_NUM;
    
    //Serial.print(inChar);
    //Serial.print('\n');
    
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
  }
}

void handleAntennaReadings(){
    int num_stop = 0;
    int num_hold = 0;
    int num_increase = 0;
    int num_decrease = 0;
    for(int i = 0; i < antenna_sample_size; i+=1){


    int determined_move = determine_direction_from_sample(voltageReadings_1[i],voltageReadings_2[i]);
    switch(determined_move) {
      case STOP_MOV:
        num_stop+=1;
        break;
      case HOLD_ANG:
         num_hold+=1;
         break;
       case INC_ANG:
         num_increase+=1;
         break;
       case DEC_ANG: 
          num_decrease+=1;
          break;
        default:
          break;
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
     //Serial.print("Stop Movement ");
     drive_command = STOP_MOV;
    break;
   case 1:
     //Serial.print("Hold angle ");
     drive_command = HOLD_ANG;
     break;
   case 3:
     //Serial.print("Decrease angle ");
      if (current_pos_micro > MIN_POS)
      goal_ang_micro = current_pos_micro - SERVO_TURN;
      else current_pos_micro = MIN_POS;
      drive_command = DEC_ANG;
     break;
   case 2:
     //Serial.print("Increase angle ");
      if (current_pos_micro < MAX_POS)
      goal_ang_micro = current_pos_micro + SERVO_TURN;
      else goal_ang_micro = MAX_POS;
      drive_command = INC_ANG;
      break;
   default:
     //Serial.print("Error ");
     break;
 }
 
  
}

int determine_direction_from_sample(uint16_t voltageReading_rise, uint16_t voltageReading_fall) {
  int diff = voltageReading_rise-voltageReading_fall;
    if (voltageReading_rise < STOP_LEVEL_MAX ){ //stop movement
      return STOP_MOV;
    } else {
      if (diff < buffering &&  diff > -1*buffering) //drive forward
      return HOLD_ANG;
    
    else if (diff > 0)//turn  voltage > (caliset + buffering)
    return DEC_ANG;

    else if (diff < 0 )//turn the other way
    return INC_ANG;
  }
  return ERROR_CALC;
}



ISR(TIMER2_COMPA_vect){//timer2 interrupt @ 1kHz
  //Toggle switching waveform
  digitalWrite(ANT_WAVEFORM,toggle);
  toggle = toggle^1;
  
  edge_count+=1; // add count to num of edges caused
  
  if (second_sample_flag && samples_full != 1) {//if a sample on rising edge was taken, but the buffer is not full, take a sample for falling edge
   voltage_2 =  analogRead(SPEAKER_FROM_WALKIETALKIE);
   voltageReadings_2[num_samples_taken] = voltage_2;
   num_samples_taken +=1;
   second_sample_flag = 0;
  }
  
  if (edge_count >= sample_delay*2){ //if the num of edges counted is above the delay, take a rising edge reading
    edge_count = 0;
    voltage_1 = analogRead(SPEAKER_FROM_WALKIETALKIE);
    if (num_samples_taken < antenna_sample_size){
    voltageReadings_1[num_samples_taken] = voltage_1;
    second_sample_flag = 1;
    } else {
      samples_full = 1;
    }
    ant_read_flag = 1;
  }
  
  
  //SERVO Handling. Handle servo movements
  if (interrupt_count_servo >= DELAY){ // if count from last servo handling is above delay, move servo
    if (move_enabled){ // increase current position if it is not at goal angle
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
    interrupt_count_servo = 0; //clear flag
    if (current_pos_micro > MAX_POS) current_pos_micro = MAX_POS; //constrain servo
    if (current_pos_micro < MIN_POS) current_pos_micro = MIN_POS;
    antenna_servo.writeMicroseconds(current_pos_micro); //set servo
    //
  } else interrupt_count_servo+=1;
  
  //flag for sensor readings
  if (sensor_timer_count >= SENSOR_READING_DELAY) {
    sensor_timer_count = 0;
    timer_flag = 1;
  } else sensor_timer_count +=1;
}


void powerUpRadio() {
  boolean turnOff = true; //default to turn the radio off
  digitalWrite(RADIO_VOL_UP,HIGH);
  delay(150); //delay till sound comes on (if on)
  int testOn = analogRead(SPEAKER_FROM_WALKIETALKIE);
  if (testOn > STOP_LEVEL_MAX){ //the radio is already on
    turnOff = false;
  } 
  delay(350); // finish hold
  digitalWrite(RADIO_VOL_UP,LOW); // release volume button
  delay(500);
  
  if (turnOff){ //if radio is not on
  
  digitalWrite(RADIO_POWER,HIGH); //power up
  delay(1500);
  digitalWrite(RADIO_POWER,LOW);
  delay(1000);
  
  digitalWrite(RADIO_VOL_UP,HIGH);//next three blocks increase volume
  delay(500);
  digitalWrite(RADIO_VOL_UP,LOW);
  delay(500);
  
  digitalWrite(RADIO_VOL_UP,HIGH);
  delay(500);
  digitalWrite(RADIO_VOL_UP,LOW);
  delay(500);
  
  digitalWrite(RADIO_VOL_UP,HIGH);
  delay(500);
  digitalWrite(RADIO_VOL_UP,LOW);
  delay(500);
  }
  
}
 

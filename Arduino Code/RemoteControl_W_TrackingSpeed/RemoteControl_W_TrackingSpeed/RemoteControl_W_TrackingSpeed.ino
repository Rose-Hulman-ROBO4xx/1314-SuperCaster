/*
This code for the Joystick of the supercaster cart.
Communication Stream (Serial):
(Emergency Stop)E[[[(Transmit)S]]](Tight Turn)T(Joystick Horizontal)H(Joystick Vertical)V(Caster Angle)A
ex: OEOT512H512V512A
*/

#define PIN_LED_TURN 67 //led 4
#define PIN_LED_POWER 68 //led 5
#define PIN_LED_TRANSMIT 69 //led 6

#define PIN_CONTRAST_ANALOG A8
#define PIN_HORZ_ANALOG 0
#define PIN_VERT_ANALOG 1

#define PIN_ESTOP 22
#define PIN_ANGLE_POT A7
#define PIN_TRACKING_TOGGLE 2
#define PIN_TURN_TOGGLE 3

#define TEAM_NUM 0
#define RXWATCHDOG 20

#define MIN_TRACK_SPEED 10 // MIN and MAX track speed are percentages of max speed
#define MAX_TRACK_SPEED 90

#include <LiquidCrystal.h>

// initialize the library with the numbers of the interface pins
LiquidCrystal lcd(14, 15, 16, 17, 18, 19, 20);
int Horz;
int Vert;
int Angle;
int Tracking;
int Turn;
int EStop;
int RxReady;
int RxCnt=0;
char thousand;
char hundreds;
char tens;
char ones;

void LCDUpdate();
void sendNumber(int);
void sendChar(char);
void sendOneNumber(int);

void setup(){
  Serial.begin(9600);
  lcd.begin(16, 2);
  lcd.setCursor(0,0);
  lcd.print("Tx");
  analogReference(DEFAULT);
  pinMode(PIN_LED_TURN, OUTPUT);
  pinMode(PIN_LED_POWER, OUTPUT);
  pinMode(PIN_LED_TRANSMIT, OUTPUT);
  
  pinMode(PIN_ESTOP,INPUT_PULLUP);
  pinMode(PIN_TRACKING_TOGGLE, INPUT_PULLUP);
  pinMode(PIN_TURN_TOGGLE, INPUT_PULLUP);
}

void loop(){
  digitalWrite(PIN_LED_POWER,HIGH);    
  Horz = analogRead(PIN_HORZ_ANALOG);
  Vert = analogRead(PIN_VERT_ANALOG);
  Angle = analogRead(PIN_ANGLE_POT);
  Tracking = digitalRead(PIN_TRACKING_TOGGLE);
  Turn = digitalRead(PIN_TURN_TOGGLE);
  EStop = digitalRead(PIN_ESTOP);
  EStop = 1;
  
  RxCnt++;
  if(RxCnt>RXWATCHDOG){
     RxReady = 0;
  }  
    
  if(RxReady){
    
    if(Tracking==1){      
        if((Angle*0.0976)<MIN_TRACK_SPEED){
          Angle = MIN_TRACK_SPEED*10.24;
        }
        else if((Angle*0.0976) > MAX_TRACK_SPEED){
          Angle = MAX_TRACK_SPEED*10.24;
        }
    }
    
    if (Tracking==0) {
      //if(RxReady==1){
        //Serial.print(digitalRead(PIN_ESTOP));
        sendOneNumber(EStop);
        sendChar('E');
        sendOneNumber(Tracking);    
        sendChar('S');
        sendOneNumber(Turn);
        sendChar('T');
        sendNumber(Horz);
        sendChar('H');
        sendNumber(Vert);
        sendChar('V');
        sendNumber(Angle);
        sendChar('A');
      //}
    }else{
        sendOneNumber(EStop);
        sendChar('E');
        sendOneNumber(Tracking);    
        sendChar('S');
        sendOneNumber(0);
        sendChar('T');
        sendNumber(512);
        sendChar('H');
        sendNumber(Angle);
        sendChar('V');
        sendNumber(512);
        sendChar('A');
    }
  }
    
    LCDUpdate();
    
    //if (!Tracking){
      //digitalWrite(PIN_LED_TRANSMIT,HIGH);
    //}else{
     // digitalWrite(PIN_LED_TRANSMIT,LOW);
    //}
    //if (!Turn){
     // digitalWrite(PIN_LED_TURN,HIGH);
    //}else{
    //  digitalWrite(PIN_LED_TURN,LOW);
    //}
    delay(100);
  
}
    
void LCDUpdate(){
  int LCDAng = 0;
  if (Tracking==0) {
    lcd.setCursor(0,0);
    lcd.print("                    ");
    lcd.setCursor(0,0);
    lcd.print("Tx:");
    lcd.print(RxReady);
    lcd.print("  S:");
    lcd.print(EStop);  
    lcd.setCursor(10,0);
    lcd.print("H:");
    lcd.print(Horz);  
    lcd.setCursor(0,1);
    lcd.print("                    ");
    lcd.setCursor(0, 1);  
    lcd.print("A:");  
    LCDAng = Angle * 0.3515;  
    lcd.print(LCDAng);  
    lcd.setCursor(6,1);
    lcd.print("T:");
    lcd.print(Turn);    
    lcd.setCursor(10,1);
    lcd.print("V:");
    lcd.print(Vert);
    
  }else{
    lcd.setCursor(0,0);
    lcd.print("                    ");
    lcd.setCursor(0,0);
    lcd.print("Track:");     
    lcd.print(RxReady);
    lcd.print(" Stop:");
    lcd.print((1-EStop));  
    lcd.setCursor(0,1);    
    lcd.print("                    ");
    lcd.setCursor(0, 1);  
    lcd.print("   Speed:");
    LCDAng = Angle*0.098;
    lcd.print(LCDAng); 
    lcd.print("%");
  }
  
}

void sendNumber(int num){
  int temp = num;
  
  thousand = temp/1000;
  temp = temp%1000;
  hundreds = temp/100;
  temp = temp%100;
  tens = (char)temp/10;
  ones = (char)temp%10;
  
  thousand += (48 + TEAM_NUM);
  hundreds += (48 + TEAM_NUM);
  tens += (48 + TEAM_NUM);
  ones += (48 + TEAM_NUM);
  
  Serial.print(thousand);
  Serial.print(hundreds);
  Serial.print(tens);
  Serial.print(ones);
}

void sendOneNumber(int num){
  ones = (48 + TEAM_NUM + num);
  Serial.print(ones);
}

void sendChar(char chara){
 char temp = chara + TEAM_NUM;
 Serial.print(temp);  
}

void serialEvent(){
  while(Serial.available()){
    char inChar = (char) Serial.read() - TEAM_NUM;
    if(inChar <= 57 && inChar >= 48){
      RxReady = inChar-48;      
      RxCnt=0;
    }
  }
}

#include <LiquidCrystal.h>
#include <stdlib.h>

#define KILL_PIN 22

#define FWD_ANG 90
#define FWD_LIMIT 772
#define BWD_LIMIT 252
#define LEFT_TURN_LIMIT 1000
#define RIGHT_TURN_LIMIT 20

#define MAX_SPD  1023
#define TURN_SPD  MAX_SPD/4
#define STOP_SPD 512

#define TEAM_NUM 32

LiquidCrystal lcd(14,15,16,17,18,19,20);
//char inByte = 0;
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
int Anglei = 0;  //Desired Angle

void setup(){    
  pinMode(KILL_PIN,OUTPUT);
  digitalWrite(KILL_PIN,LOW);  
  
  Serial.begin(9600);  //Arduino Joystick
  Serial1.begin(9600);  //Left Caster PIC
  Serial2.begin(9600);  //Right Caster PIC
  
  inputString.reserve(200);
  /*
  lcd.begin(16,2);
  lcd.setCursor(0,0);
  lcd.print("Rx: ");*/
  
  Serial.print("Arduino Ready\n");
}

void loop(){
  //delay(1000);
  //stringComplete = true;
  if(stringComplete){
    //Serial.print("Message recieved");
    /*
    lcd.setCursor(0,0);
    lcd.print("                 ");
    lcd.setCursor(0,0);
    
    lcd.print(Ang);
    lcd.print(" ");
    lcd.print(Horz);
    //lcd.print(" ");
    lcd.setCursor(0,1);
    lcd.print("                 ");
    lcd.setCursor(0,1);
    lcd.print(Verti);
    lcd.print("  ");    
    lcd.print(Ei);
    lcd.print(" ");
    lcd.print(Ti);
    lcd.print(" ");
    lcd.print(Si);*/
    
    Ei = StringToInt(E);
    Ti = StringToInt(T);
    Si = StringToInt(S);
    Horzi = StringToInt(Horz);
    Verti = StringToInt(Vert);
    //Verti = Verti * (MAX_SPD/1024);
    Anglei = StringToInt(Ang);
    Anglei = Anglei*0.352;
   
    
    Serial.print(Ei);
    Serial.print('E');
    Serial.print(Ti);
    Serial.print('T');
    Serial.print(Horzi);
    Serial.print('H');
    Serial.print(Vert);
    Serial.print('V');
    Serial.print(Anglei);
    Serial.print("A\n");
  
    //DEBUG ONLY: SET E-STOP TO OFF
    //if (Verti > 1024){
      //Verti = 0;
    //}
    //Ei = 1;
    /*
    Horzi = 512;
    Verti = Verti + 100;
    Anglei = 180;
    LeftPICSendSerial(Anglei, Verti);*/
     
    if(Ei == 1){
      if(Verti < FWD_LIMIT){
        if(Verti > BWD_LIMIT){     
           //If vert value not to extreme, and horz value is, perform an appropriate tank drive turn
          if(Horzi > LEFT_TURN_LIMIT){
              Serial.print("Horzi > LEFT_TURN_LIMIT\n");
              RightPICSendSerial(Anglei, TURN_SPD);
              LeftPICSendSerial(Anglei, (STOP_SPD));
          }else if(Horzi < RIGHT_TURN_LIMIT){
              Serial.print("Horzi < RIGHT_TURN_LIMIT\n");
              LeftPICSendSerial(Anglei, TURN_SPD);
              RightPICSendSerial(Anglei, (STOP_SPD));          
          }else{
            Serial.print("RIGHT_TURN_LIMIT < Horzi < LEFT_TURN_LIMIT\n");
            LeftPICSendSerial(Anglei, Verti);
            RightPICSendSerial(Anglei, Verti);  
          }  
        }else{
            Serial.print("Verti < BWD_LIMIT\n");         
            LeftPICSendSerial(Anglei, Verti);
            RightPICSendSerial(Anglei, Verti);  
        }    
      }else{
        Serial.print("Verti > FWD_LIMIT\n");
        LeftPICSendSerial(Anglei, Verti);
        RightPICSendSerial(Anglei, Verti);      
      }
    }else{
      Serial.print("Ei = 0\n");
      RightPICSendSerial(Anglei, STOP_SPD);
      LeftPICSendSerial(Anglei, STOP_SPD);
      killPower();
    }    
    
    inputString = "";
    stringComplete = false;
    
    delay(100);
  }
}
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
void LeftPICSendSerial(int angle, int spd){
      spd = spd/2+256;
      Serial.print("Sending to left PIC ");
      Serial.print(angle);
      Serial.print("A ");
      Serial.print(spd);
      Serial.print("S\n");
      Serial1.print(angle);
      delay(15);
      Serial1.print('A');
      delay(15);
      Serial1.print((MAX_SPD-spd)); 
      delay(15);
      Serial1.print('S');
      delay(15);
      return;
}
void RightPICSendSerial(int angle, int spd){
      spd = spd/2*(0.83)+299;
      Serial.print("Sending to right PIC: ");
      Serial.print(angle);
      Serial.print("A ");
      Serial.print(spd);
      Serial.print("S\n");
      Serial2.print(angle);
      delay(15);
      Serial2.print('A');
      delay(15);
      Serial2.print(spd); 
      delay(15);
      Serial2.print('S');
      delay(15);
      return;
}
int StringToInt(String str){
  int num = 0;
  char charBuff[5];
  str.toCharArray(charBuff,5);
  num = atoi(charBuff);
  return num;
}
void killPower(){
  digitalWrite(KILL_PIN,HIGH);
}
    

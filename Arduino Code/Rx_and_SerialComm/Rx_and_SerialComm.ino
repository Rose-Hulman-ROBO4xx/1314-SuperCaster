#include <LiquidCrystal.h>
#include <stdlib.h>

#define LED_READY 67
#define LED_RECIEVE 68
#define LED_TEST 22

#define FWD_ANG 90

#define FWD_LIMIT 612
#define BWK_LIMIT 412
#define LEFT_TURN_LIMIT 212
#define RIGHT_TURN_LIMIT 812
#define TURN_SPD 1024 //Remember: 0 = full back, 512 = stop, 1024 = full fwd
#define MAX_SPD 1024
#define STOP_SPD 512

//LiquidCrystal lcd(14,15,16,17,18,19,20);
//char inByte = 0;
String inputString = "";
boolean stringComplete = false;
String Horz = "";
String Vert = "";
String S = "";
String Angle = "";
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
  Serial.begin(9600);  //Arduino Joystick
  Serial1.begin(9600);  //Left Caster PIC
  Serial2.begin(9600);  //Right Caster PIC
  
  inputString.reserve(200);
  
  //lcd.begin(16,2);
  //lcd.setCursor(0,0);
  //lcd.print("Reciever");
  pinMode(LED_READY,OUTPUT);
  pinMode(LED_RECIEVE,OUTPUT);
  pinMode(LED_TEST,OUTPUT);
  digitalWrite(LED_READY,HIGH);
  digitalWrite(LED_RECIEVE,LOW);
  digitalWrite(LED_TEST,HIGH);
}

void loop(){
  if(stringComplete){
    /*lcd.setCursor(0,1);
    lcd.print("                 ");
    lcd.setCursor(0,1);
    lcd.print(S);
    lcd.print(" ");
    lcd.print(Horz);
    lcd.print(" ");
    lcd.print(Vert);
    lcd.print("  ");*/
    //Serial.println(S);
    //Serial.println(Horz);
    //Serial.println(Vert);
    
    Ei = StringToInt(E);
    Ti = StringToInt(T);
    Si = StringToInt(S);
    Horzi = StringToInt(Horz);
    Verti = StringToInt(Vert);
    Anglei = StringToInt(Angle);
      
    if(Ei == 0){
      //If vert value not to extreme, and horz value is, perform an appropriate tank drive turn
      if(Verti < FWD_LIMIT){
        if(Verti > BWD_LIMIT){
          if(Horzi < LEFT_TURN_LIMIT){
              RightPICSendSerial(Anglei, TURN_SPD);
              LeftPICSendSerial(Anglei, (MAX_SPD-TURN_SPD));
          }else if(Horzi > RIGHT_TURN_LIMIT)P{
              LeftPICSendSerial(Anglei, TURN_SPD);
              RightPICSendSerial(Anglei, (MAX_SPD-TURN_SPD));
          }
        }
      }else{
        LeftPICSendSerial(Anglei, Verti);
        RightPICSendSerial(Anglei, Verti);      
      }
    }else{
      RightPICSendSerial(Anglei, STOP_SPD);
      LeftPICSendSerial(Anglei, STOP_SPD);
      //killPower();
    }    
    
     if(Si == 1){
        digitalWrite(LED_TEST,HIGH);
     } else {
       digitalWrite(LED_TEST,LOW);
     }
      
   
    
    inputString = "";
    stringComplete = false;
  }
}
void serialEvent(){
  digitalWrite(LED_RECIEVE,HIGH);
  while(Serial.available()){
    char inChar = (char) Serial.read();
    if(inChar =='A'){
      Angle = inputString;
      inputString = "";
      stringComplete = true;
    else if(inChar=='V'){
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
    else{
      inputString += inChar;
    }
  }
  digitalWrite(LED_RECIEVE,LOW);
}
void LeftPICSendSerial(int angle, int spd){
      Serial1.print(angle);
      delay(15);
      Serial1.print('A');
      delay(15);d
      Serial1.print(spd); 
      delay(15);
      Serial1.print('S');
      return;
}
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
int StringToInt(String str){
  int num = 0;
  char charBuff[5];
  str.toCharArray(charBuff,5);
  num = atoi(charBuff);
  return num;
}
//void killPower();
  
  

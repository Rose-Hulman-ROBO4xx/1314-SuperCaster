#include <LiquidCrystal.h>
#include <stdlib.h>

#define LED_READY 67
#define LED_RECIEVE 68
#define LEFT_MOTOR_PIN  9
#define RIGHT_MOTOR_PIN 10
#define FWD_LIMIT 800
#define LEFT_TURN_LIMIT 800
#define RIGHT_TURN_LIMIT 200
#define LED_TEST 22

LiquidCrystal lcd(14,15,16,17,18,19,20);
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
int Si = 0;
int Ti = 0;
int Ei = 0;
int Anglei = 0;

void setup(){  
  Serial.begin(9600);
  
  inputString.reserve(200);
  
  lcd.begin(16,2);
  lcd.setCursor(0,0);
  lcd.print("Reciever");
  pinMode(LEFT_MOTOR_PIN,OUTPUT);
  pinMode(RIGHT_MOTOR_PIN,OUTPUT);
  pinMode(LED_READY,OUTPUT);
  pinMode(LED_RECIEVE,OUTPUT);
  pinMode(LED_TEST,OUTPUT);
  digitalWrite(LED_READY,HIGH);
  digitalWrite(LED_RECIEVE,LOW);
  digitalWrite(LEFT_MOTOR_PIN,HIGH);
  digitalWrite(RIGHT_MOTOR_PIN,HIGH);
  digitalWrite(LED_TEST,HIGH);
}

void loop(){
  if(stringComplete){
    lcd.setCursor(0,1);
    lcd.print("                 ");
    lcd.setCursor(0,1);
    lcd.print(S);
    lcd.print(" ");
    lcd.print(Horz);
    lcd.print(" ");
    lcd.print(Vert);
    lcd.print("  ");
    Serial.println(S);
    Serial.println(Horz);
    Serial.println(Vert);
    
    Ei = StringToInt(E);
    Ti = StringToInt(T);
    Si = StringToInt(S);
    Horzi = StringToInt(Horz);
    Verti = StringToInt(Vert);
    Anglei = StringToInt(Angle);
    
    
    if(Verti > FWD_LIMIT){
      Serial.println("Forward");
      digitalWrite(LEFT_MOTOR_PIN,LOW);
      digitalWrite(RIGHT_MOTOR_PIN,LOW);
    }else if(Horzi > LEFT_TURN_LIMIT){
      Serial.println("Left");
      digitalWrite(LEFT_MOTOR_PIN,LOW);
      digitalWrite(RIGHT_MOTOR_PIN,HIGH);
    }else if(Horzi < RIGHT_TURN_LIMIT){
      Serial.println("Right");
      digitalWrite(LEFT_MOTOR_PIN,HIGH);
      digitalWrite(RIGHT_MOTOR_PIN,LOW);
    }else{
      Serial.println("Stop");
      digitalWrite(LEFT_MOTOR_PIN,HIGH);
      digitalWrite(RIGHT_MOTOR_PIN,HIGH);
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
    }else if(inChar=='V'){
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
  digitalWrite(LED_RECIEVE,LOW);
}

int StringToInt(String str){
  int num = 0;
  char charBuff[5];
  str.toCharArray(charBuff,5);
  num = atoi(charBuff);
  return num;
}
  
  

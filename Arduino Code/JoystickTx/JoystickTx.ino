/*
This code for the Joystick of the supercaster cart.
Communication Stream (Serial):
(Emergency Stop)E(Transmit)S(Tight Turn)T(Joystick Horizontal)H(Joystick Vertical)V(Caster Angle)A

*/

#define PIN_LED_1 64
#define PIN_LED_2 65
#define PIN_LED_3 66
#define PIN_LED_TURN 67 //led 4
#define PIN_LED_POWER 68 //led 5
#define PIN_LED_TRANSMIT 69 //led 6

#define PIN_RIGHT_BUTTON 2
#define PIN_LEFT_BUTTON 3

#define PIN_CONTRAST_ANALOG 8
#define PIN_HORZ_ANALOG 0
#define PIN_VERT_ANALOG 1

#define PIN_ESTOP 40
#define PIN_ANGLE_POT A7
#define PIN_TRANSMIT_TOGGLE PIN_RIGHT_BUTTON
#define PIN_TURN_TOGGLE PIN_LEFT_BUTTON

#include <LiquidCrystal.h>

// initialize the library with the numbers of the interface pins
LiquidCrystal lcd(14, 15, 16, 17, 18, 19, 20);
int Horz;
int Vert;
int Angle;
int Transmit;

void setup(){
  Serial.begin(9600);
  lcd.begin(16, 2);
  analogReference(DEFAULT);
  pinMode(PIN_LED_1, OUTPUT);
  pinMode(PIN_LED_2, OUTPUT);
  pinMode(PIN_LED_3, OUTPUT);
  pinMode(PIN_LED_TURN, OUTPUT);
  pinMode(PIN_LED_POWER, OUTPUT);
  pinMode(PIN_LED_TRANSMIT, OUTPUT);
  
  pinMode(PIN_ESTOP,INPUT_PULLUP);
  pinMode(PIN_TRANSMIT_TOGGLE, INPUT_PULLUP);
  pinMode(PIN_TURN_TOGGLE, INPUT_PULLUP);
}

void loop(){
  digitalWrite(PIN_LED_POWER,HIGH);
  
  Horz = analogRead(PIN_HORZ_ANALOG);
  Vert = analogRead(PIN_VERT_ANALOG);
  Angle = analogRead(PIN_ANGLE_POT);
  Transmit = digitalRead(PIN_TRANSMIT_TOGGLE);

  if (Transmit==0) {
    //Serial.print(digitalRead(PIN_ESTOP));
    Serial.print(0);
    Serial.print('E');
    Serial.print(Transmit);    
    Serial.print('S');
    Serial.print(digitalRead(PIN_TURN_TOGGLE));
    Serial.print('T');
    Serial.print(Horz);
    Serial.print('H');
    Serial.print(Vert);
    Serial.print('V');
    Serial.print(Angle);
    Serial.print('A');
  }  
  
  lcd.setCursor(0,0);
  lcd.print("Tranciever");
  lcd.setCursor(0, 1);
  lcd.print("H:");
  lcd.print(Horz);
  lcd.print("  ");
  lcd.setCursor(8, 1);
  lcd.print("V:");
  lcd.print( Vert);
  lcd.print("  ");
  if (!Transmit)){
    digitalWrite(PIN_LED_TRANSMIT,HIGH);
  }else{
    digitalWrite(PIN_LED_TRANSMIT,LOW);
  }
  if (!digitalRead(PIN_LEFT_BUTTON)){
    digitalWrite(PIN_LED_TURN,HIGH);
  }else{
    digitalWrite(PIN_LED_TURN,LOW);
  }
  delay(100);
}
    

#define PIN_LED_1 64
#define PIN_LED_2 65
#define PIN_LED_3 66
#define PIN_LED_4 67
#define PIN_LED_5 68
#define PIN_LED_6 69
#define PIN_RIGHT_BUTTON 2
#define PIN_LEFT_BUTTON 3
#define PIN_SELECT_BUTTON 24
#define PIN_CONTRAST_ANALOG 8
#define PIN_HORZ_ANALOG 0
#define PIN_VERT_ANALOG 1
#define PIN_HORZ_ANALOG 0
#define PIN_VERT_ANALOG 1
#include <LiquidCrystal.h>

// initialize the library with the numbers of the interface pins
LiquidCrystal lcd(14, 15, 16, 17, 18, 19, 20);
int Horz;
int Vert;

void setup(){
  Serial.begin(9600);
  lcd.begin(16, 2);
  analogReference(DEFAULT);
  pinMode(PIN_LED_1, OUTPUT);
  pinMode(PIN_LED_2, OUTPUT);
  pinMode(PIN_LED_3, OUTPUT);
  pinMode(PIN_LED_4, OUTPUT);
  pinMode(PIN_LED_5, OUTPUT);
  pinMode(PIN_LED_6, OUTPUT);
  pinMode(PIN_SELECT_BUTTON, INPUT_PULLUP);
  pinMode(PIN_LEFT_BUTTON, INPUT_PULLUP);
  pinMode(PIN_RIGHT_BUTTON, INPUT_PULLUP);
}

void loop(){
  digitalWrite(PIN_LED_5,HIGH);
  Horz = analogRead(PIN_HORZ_ANALOG);
  Vert = analogRead(PIN_VERT_ANALOG);

  if (digitalRead(PIN_RIGHT_BUTTON) == 0) {
    Serial.print(digitalRead(PIN_LEFT_BUTTON));
    Serial.print('S');
    Serial.print(Horz);
    Serial.print('H');
    Serial.print(Vert);
    Serial.print('V');
  }  
  //Serial.print('HV');
  
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
  if (!digitalRead(PIN_SELECT_BUTTON)){
    digitalWrite(PIN_LED_2,HIGH);
  }else{
    digitalWrite(PIN_LED_2,LOW);
  }
  if (!digitalRead(PIN_LEFT_BUTTON)){
    digitalWrite(PIN_LED_4,HIGH);
  }else{
    digitalWrite(PIN_LED_4,LOW);
  }
  if (!digitalRead(PIN_RIGHT_BUTTON)){
    digitalWrite(PIN_LED_6,HIGH);
  }else{
    digitalWrite(PIN_LED_6,LOW);
  }  
  delay(100);
}
    

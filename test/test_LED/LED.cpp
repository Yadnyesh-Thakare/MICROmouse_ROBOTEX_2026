#include <Arduino.h>

#define BTNL 14
#define BTNR 35

#define LEDL 12
#define LEDR 32

void setup(){
  Serial.begin(115200);
  pinMode(LEDL, OUTPUT);
  pinMode(LEDR, OUTPUT);
  pinMode(BTNL, INPUT);
  pinMode(BTNR, INPUT);  
}

void loop(){

  if(digitalRead(BTNL)){
    digitalWrite(LEDL, HIGH);
  }
  if(digitalRead(BTNR)){
    digitalWrite(LEDR, HIGH);
  }

  digitalWrite(LEDL, LOW);
  digitalWrite(LEDR, LOW);
}
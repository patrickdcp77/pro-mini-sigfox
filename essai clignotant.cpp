#include <Arduino.h>  

#define led A0 // Définition des variables, mettre A0 ou 14
int temp=500;

void setup() {

  Serial.begin(9600);
  Serial.println("src/essai clignotant.cpp");
  pinMode(led, OUTPUT);  //Définition de la variable led en OUTPUT
  }

void loop() {
  digitalWrite(led,1);    //# Code .....
  delay(temp);
  digitalWrite(led,0);
  delay(temp);
}

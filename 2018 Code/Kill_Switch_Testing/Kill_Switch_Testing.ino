#include <Servo.h>

Servo pitch;
void setup(){
  pinMode(A1, INPUT);
  pinMode(8, OUTPUT);
  Serial.begin(9600);
  Serial.println("Starting. ");
  pitch.attach(9);
}


void loop(){
  boolean killed = (digitalRead(A1) == LOW);
  double voltage = digitalRead(A1);
  Serial.print("Voltage equals: ");
  if(digitalRead(A1) == LOW){
    Serial.println("kill switch hit");
    Serial.println("Sending a high signal to the load Arduino.");
    digitalWrite(8, HIGH);
    pitch.write(110);
    delay(1000);
  }
  else{
    Serial.println("Returned low");
    pitch.write(45);
  }
  
  delay(2000);
}

#include <Servo.h>
const int PWM_PIN = 3;
const double VOLTAGE_DIVIDER_PRE_PCC = 14.327;

Servo pitch;


void setup() {
  // put your setup code here, to run once:
  pinMode(PWM_PIN, OUTPUT);
  pitch.write(55); //Startup pitch
  analogWrite(PWM_PIN, 50);

}

void loop() {
  // put your main code here, to run repeatedly:
  analogWrite(PWM_PIN, 50);
  double prePCCVoltage = VOLTAGE_DIVIDER_PRE_PCC*((double)analogRead(A2))*5.0/1023.0;
  Serial.print("PrePCC Voltage is: ");
  Serial.println(prePCCVoltage);
  delay(1000);

}

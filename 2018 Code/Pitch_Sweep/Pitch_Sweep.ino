//11MS 25-48
//12MS 41-65
//13MS 46-70


#include <Servo.h>
const int MINIMUM_PITCH_RANGE = 46;
const int MAXIMUM_PITCH_RANGE = 70;
const int PWM_PIN = 3;
const double VOLTAGE_DIVIDER_TURBINE = 13.015;

Servo pitch;

const int SECONDS_PER_PITCH = 7;

void setup() {
  Serial.begin(9600);
  pinMode(PWM_PIN, OUTPUT);
  // put your setup code here, to run once:
  pitch.attach(9);
  analogWrite(PWM_PIN, 255);
  pitch.write(55); //Startup pitch

}

void loop() {
  // put your main code here, to run repeatedly:
  double turbineVoltage;
  Serial.println("Starting the pitch sweep now. ");
  for(int i = MINIMUM_PITCH_RANGE; i <= MAXIMUM_PITCH_RANGE; i+=3){
    pitch.write(i);
    analogWrite(PWM_PIN, 255);
    delay(SECONDS_PER_PITCH*1000);
    Serial.print("Pitch is: ");
    Serial.println(i);
    turbineVoltage = VOLTAGE_DIVIDER_TURBINE*((double)analogRead(A0))*5.0/1023.0;
    Serial.print("Reading turbine Voltage as: ");
    Serial.println(turbineVoltage);
  }
  delay(SECONDS_PER_PITCH*1000*2);
  Serial.println("Starting a new loop of the pitch sweep now. ");

}

//TODO: 
//PROTECT AGAINST CASE WHERE DUTY CYCLE IS 1 AND THEORETICAL VOLTAGE CAN NEVER BE ACTUAL VOLTAGE BECAUSE OF SYSTEM LOSSES


#include <Servo.h>

const int PWM_PIN = 3; 
const int SERVO_PITCH_PIN = 9; 
const int LOWER_VOLTAGE_LIMIT = 10;
const int UPPER_VOLTAGE_LIMIT = 15;
const int MAINTAIN_VOLTAGE_IN = 13;
const double VOLTAGE_DIVIDER_LOAD = 14.327;
const int DUTY_CYCLE_RATIO = 5;
const int MINIMUM_PITCH_RANGE = 25;
const int MAXIMUM_PITCH_RANGE = 75;
static int currentPitch;


double turbineVoltageBefore;
const int STARTUP_PITCH = 55; //Need to verify pitch for new turbine -> should be verified now
const double VOLTAGE_DIVIDER_TURBINE = 13.015; //This should be the same as last year so we are good
const double THEORETICAL_VS_ACTUAL_VOLTAGE_BUFFER = .3;
const double VOLTAGE_DIVIDER_PRE_PCC = 14.327;

void determinePitch(double turbineVoltage);
int calculateDutyCycle(double);
void stabilizeVoltageGivenDutyCycle(int dutyCycle, double desiredVoltage);


Servo pitch;

void setup(){
  Serial.begin(9600);
  Serial.println("Starting the test");
  pinMode(PWM_PIN, OUTPUT);
  turbineVoltageBefore = -1;
  pitch.attach(SERVO_PITCH_PIN);
  pitch.write(STARTUP_PITCH);
  currentPitch = STARTUP_PITCH;
  analogWrite(PWM_PIN, 50);
  delay(3000);
  
}

void loop(){
  double turbineVoltage;
  int dutyCycle;
  double theoreticalVoltage;
  turbineVoltage = VOLTAGE_DIVIDER_TURBINE*((double)analogRead(A0))*5.0/1023.0;
  if(turbineVoltageBefore!= -1){ //This is not the first voltage reading.
    determinePitch(turbineVoltage);
  }
  //Else do nothing because it's only the first loop
  //Now calculate the duty cycle
  
  
  dutyCycle = calculateDutyCycle(turbineVoltage);
  theoreticalVoltage = turbineVoltage*double(dutyCycle)/255.0;
  if(dutyCycle >= 0 && dutyCycle <= 255){
    analogWrite(PWM_PIN, dutyCycle);
  }
  else{
    if(dutyCycle < 0){
      dutyCycle = 0;
      theoreticalVoltage = 0;
      analogWrite(PWM_PIN, 0);
    }
    if(dutyCycle > 255){
      dutyCycle = 255;
      theoreticalVoltage = turbineVoltage;
      analogWrite(PWM_PIN, 255);
    }
  }
  stabilizeVoltageGivenDutyCycle(dutyCycle, 5); //This is the theoretical output voltage
  double prePCCVoltage = VOLTAGE_DIVIDER_PRE_PCC*((double)analogRead(A2))*5.0/1023.0;
  Serial.print("prePCC voltage is: ");
  Serial.println(prePCCVoltage);
  
  
  //Move to the t+1 loop domain
  turbineVoltageBefore = turbineVoltage;
}

void determinePitch(double turbineVoltage){
  
  //Case 1:
  if(turbineVoltage < LOWER_VOLTAGE_LIMIT){
    if(currentPitch <= MAXIMUM_PITCH_RANGE){
      pitch.write(currentPitch+1);
      currentPitch++;
    }
  }
  else if(turbineVoltage> LOWER_VOLTAGE_LIMIT && turbineVoltage < MAINTAIN_VOLTAGE_IN){ //Case 2
    if(turbineVoltage>=turbineVoltageBefore){
      //Do nothing
    }
    else{
      if(currentPitch <= MAXIMUM_PITCH_RANGE){
        pitch.write(currentPitch+1);
        currentPitch++;
      }
    }
    
  }
  else if(turbineVoltage >= MAINTAIN_VOLTAGE_IN && turbineVoltage < UPPER_VOLTAGE_LIMIT){  //Case 3
    
    if(turbineVoltage>=turbineVoltageBefore){
      if(currentPitch >= MINIMUM_PITCH_RANGE){
        pitch.write(currentPitch-1);
        currentPitch--;
      }
    }
    else{
      //Do nothing
    }
    
  }
  else if(turbineVoltage>=UPPER_VOLTAGE_LIMIT){
    if(currentPitch >= MINIMUM_PITCH_RANGE+3){
      pitch.write(currentPitch - 3);
      currentPitch -= 3;
    }
    else if(currentPitch >= MINIMUM_PITCH_RANGE+2){

      pitch.write(currentPitch - 2);
      currentPitch -= 2;
    }
    else if(currentPitch >= MINIMUM_PITCH_RANGE+2){
      pitch.write(currentPitch-1);
      currentPitch-=1;
    }
  }
  
  else{
    
    //For debugging
    Serial.println("Edge case we have not accounted for. ");
    Serial.print("Turbine Voltage is: ");
    Serial.println(turbineVoltage);
    Serial.print("And voltage before is: ");
    Serial.println(turbineVoltageBefore);
    //For debugging
  }
  delay(10);
}

int calculateDutyCycle(double turbineVoltage){
  if(turbineVoltage > 5){
    
    int dutyCycle = int(17397*pow(turbineVoltage, -2.538));
    //For debugging
    Serial.print("Calculating a duty cycle of: ");
    Serial.println(dutyCycle);
    
    if(dutyCycle > 255){
      return 255;
    }
    else if(dutyCycle < 0){
      return 1;
    }
    else{
      return dutyCycle;
    }
    
    
  }
  else{
    
    //For debugging
    Serial.println("Input voltage less than 5. Sending a fully on duty cycle. ");
    //For debugging
    
    return 255; //Max duty cycle 
  }
  
  
}



void stabilizeVoltageGivenDutyCycle(int dutyCycle, double desiredVoltage){
  
  
  int iterations = 0;
  bool flag = true;
  while(iterations < 1000 && flag == true){
    flag = false;
    double prePCCVoltage = VOLTAGE_DIVIDER_LOAD*((double)analogRead(A2))*5.0/1023.0;
    if(VOLTAGE_DIVIDER_LOAD*((double)analogRead(A2))*5.0/1023.0 < desiredVoltage - THEORETICAL_VS_ACTUAL_VOLTAGE_BUFFER){
      flag = true;
      if(dutyCycle < 255){
        analogWrite(PWM_PIN, ++dutyCycle);
      }
      else{
        //DO NOTHING
      }
    }
    else if(VOLTAGE_DIVIDER_LOAD*((double)analogRead(A2))*5.0/1023.0 > desiredVoltage + THEORETICAL_VS_ACTUAL_VOLTAGE_BUFFER){
      flag = true;
      if(dutyCycle > 1){
        analogWrite(PWM_PIN, --dutyCycle);
      }
      else{
        //DO NOTHING
      }
    }
       
    iterations++;
  }
}



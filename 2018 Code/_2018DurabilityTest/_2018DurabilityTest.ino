//TODO: 
//PROTECT AGAINST CASE WHERE DUTY CYCLE IS 1 AND THEORETICAL VOLTAGE CAN NEVER BE ACTUAL VOLTAGE BECAUSE OF SYSTEM LOSSES


#include <Servo.h>

const int PWM_PIN = 3; 
const int SERVO_PITCH_PIN = 9; 
const int LOWER_VOLTAGE_LIMIT = 105;
const int UPPER_VOLTAGE_LIMIT = 32;
const int MAINTAIN_VOLTAGE_IN = 22;
const double VOLTAGE_DIVIDER_LOAD = 14.327;
const int DUTY_CYCLE_RATIO = 5;
const int MINIMUM_PITCH_RANGE = 45;
const int MAXIMUM_PITCH_RANGE = 95;
static int currentPitch;
const double MAX_VOLTAGE = 40;
const int BRAKE_PITCH = 110;


double turbineVoltageBefore;
const int DURABILITY_PITCH = 55; //Need to verify pitch for new turbine -> should be verified now
const double VOLTAGE_DIVIDER_TURBINE = 13.015; //This should be the same as last year so we are good
const double THEORETICAL_VS_ACTUAL_VOLTAGE_BUFFER = .3;
const double VOLTAGE_DIVIDER_PRE_PCC = 14.327;

void determinePitch(double turbineVoltage);
int calculateDutyCycle(double);
void stabilizeVoltageGivenDutyCycle(int dutyCycle, double desiredVoltage);
void safetyMaxVoltageCheck();
double averageTurbineVoltage();


Servo pitch;

void setup(){
  Serial.begin(9600);
  Serial.println("Starting the test");
  pinMode(PWM_PIN, OUTPUT);
  turbineVoltageBefore = -1;
  pitch.attach(SERVO_PITCH_PIN);
  pitch.write(DURABILITY_PITCH);
  currentPitch = DURABILITY_PITCH;
  analogWrite(PWM_PIN, 255);
  delay(2000);
  
}

void loop(){
  pitch.write(DURABILITY_PITCH);
  currentPitch = DURABILITY_PITCH;
  double turbineVoltage;
  int dutyCycle;
  double theoreticalVoltage;
  turbineVoltage = VOLTAGE_DIVIDER_TURBINE*((double)analogRead(A0))*5.0/1023.0;
  //Else do nothing because it's only the first loop
  //Now calculate the duty cycle
  safetyMaxVoltageCheck();
  
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
  delay(3000);
  
  
  //Move to the t+1 loop domain
  turbineVoltageBefore = turbineVoltage;
}

void determinePitch(double turbineVoltage){
  
  //Case 1:
  if(turbineVoltage < LOWER_VOLTAGE_LIMIT){
    if(currentPitch > MINIMUM_PITCH_RANGE){
      pitch.write(currentPitch-1);
      currentPitch--;
    }
  }
  else if(turbineVoltage> LOWER_VOLTAGE_LIMIT && turbineVoltage < MAINTAIN_VOLTAGE_IN){ //Case 2
    if(turbineVoltage>=turbineVoltageBefore){
      //Do nothing
    }
    else{
      if(currentPitch > MINIMUM_PITCH_RANGE){
        pitch.write(currentPitch-1);
        currentPitch--;
      }
    }
    
  }
  else if(turbineVoltage >= MAINTAIN_VOLTAGE_IN && turbineVoltage < UPPER_VOLTAGE_LIMIT){  //Case 3
    
    if(turbineVoltage>=turbineVoltageBefore){
      if(currentPitch < MAXIMUM_PITCH_RANGE){
        pitch.write(currentPitch+1);
        currentPitch++;
      }
    }
    else{
      //Do nothing
    }
    
  }
  else if(turbineVoltage>=UPPER_VOLTAGE_LIMIT){
    if(currentPitch < MAXIMUM_PITCH_RANGE-3){
      pitch.write(currentPitch + 3);
      currentPitch += 3;
    }
    else if(currentPitch < MAXIMUM_PITCH_RANGE-2){

      pitch.write(currentPitch + 2);
      currentPitch += 2;
    }
    else if(currentPitch < MAXIMUM_PITCH_RANGE-1){
      pitch.write(currentPitch+1);
      currentPitch+=1;
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
  delay(30);
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
  while(iterations < 100 && flag == true){
    safetyMaxVoltageCheck();
    flag = false;
    double prePCCVoltage = VOLTAGE_DIVIDER_LOAD*((double)analogRead(A2))*5.0/1023.0;
    //Serial.println(prePCCVoltage);
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
   // delay(7);
  }
}


void safetyMaxVoltageCheck(){
  double turbineVoltage = averageTurbineVoltage();
  if(turbineVoltage >= MAX_VOLTAGE){
    Serial.println("Voltage crossing threshold. Breaking the turbine. ");
    pitch.write(BRAKE_PITCH);
    currentPitch = BRAKE_PITCH;
    delay(15000);
  }
  else{
    Serial.println("Voltage not crossing threshold. ");
  }
  
}

double averageTurbineVoltage(){
  double totalVoltage = 0;
  for(int i = 0; i < 10; i++){
    totalVoltage+=VOLTAGE_DIVIDER_TURBINE*((double)analogRead(A0))*5.0/1023.0;
  }
  return totalVoltage/10;

}


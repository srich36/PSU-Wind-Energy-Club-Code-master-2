//TODO: 
//     1. Test EEPROM memory. 
//     2. TEST REFACTORED CODE
//     3. IMPLEMENT EXPERIMENTAL DUTY CYCLE FUNCTION
//     4. ERROR HANDLING FOR PITCHING INCREMENTS PAST MINIMUM OR MAXIMUM PITCH ANGLES



#include <Servo.h>
#include <EEPROM.h>

//All pins are by default input so you don't have to specify input,and you do not on analog pins.

//You do not have to configure for analogread input when reading a voltage from something. If you want to read a voltage it has to be connected to these
//pins because they can have a range of values.

//however, it is still good to confiure as input to see what your pins are, because even though it configures them as digital input, analogread converts them
//to analog

//Analogwrite also sets the pin to output when called




////////////////////////////////////
/////Note to self: analogWrite and read are analog functions and thus default to analog pins
///// while digitalWrite and read are digital functions and thus expect digital pins as default
///// however, you can say A0 in digitalread to make it read the analog pin
/////////////




//****************************************************//
//******************Arduino Pin constants*************//
//****************************************************//
const int PWM_PIN = 3;               //Digital
const int LOAD_ARDUINO_PIN = 8;      //Digital 
const int SERVO_PITCH_PIN = 9;        //Digital
//const int KILL_SWITCH_PIN = 2      //Analog
const int TURBINE_VOLTAGE_PIN = 0;    //Analog
const int PWM_CONVERSION = 255;       //the arduino operates on a 0-255 scale for pwm so the duty cycle needs to be within this range
const int FIVE_TO_SEVEN_PITCH_ANGLE = 50;
const int SEVEN_TO_TEN_PITCH_ANGLE = 48;
const int TEN_PLUS_INITIAL_PITCH_ANGLE = 31;
const double SIGNAL_BUFFER = .01;
const double THEORETICAL_VS_ACTUAL_VOLTAGE_BUFFER = .3;
static int currentPitch;
static boolean breakedInCompetition = false;
const int CYCLES_PER_THRESHOLD_CROSS = 500;
static boolean analog50 = false;
static boolean analog255 = false;

//****************************************************//
//****************Control System Constants************//
//****************************************************//

const int BRAKE_PITCH = 110; //Need to verify pitch for new turbine -> should be verified now
const int STARTUP_PITCH = 55; //Need to verify pitch for new turbine -> should be verified now
const int MINIMUM_PITCH_ANGLE = 27;
const int MAXIMUM_PITCH_ANGLE= 90;
const double VOLTAGE_DIVIDER_TURBINE = 13.015; //This should be the same as last year so we are good
const double VOLTAGE_DIVIDER_LOAD = 14.327;//This needs to be calculated for our new load
const double VOLTAGE_DIVIDER_PRE_PCC = 14.327;
const double VOLTAGE_DIFFERENT_BUFFER = 3.5;
const double LOAD_VOLTAGE_BUFFER = 2.0;
const double MAX_VOLTAGE = 45;
const int RESISTANCE = 50; //Is this a constant or does it change? 
const double POWER_AT_11MS = 20;
const double VOLTAGE_AT_11_MS = 37;
const double VOLTAGE_AT_11_MS_BUFFER = 2.5;

Servo pitch;

//****************************************************//
//****************Function Prototypes*****************//
//****************************************************//

double inferWindSpeed(double voltageIn, double RPM, double Power);
//int determineOptimumPitch(double);
void processDisconnectedState();
int calculateTheoreticalDutyCycle11(double resistance, double power, double turbineVoltage);
double getRPMfromVoltageIn(double VoltageIn);
double calculatePowerFromRPM(double RPM);
void pitchToPitchAngleBucket(double windSpeed);
boolean determineDisconnect(double, double);
double calculateTheoreticalOutputVoltage(double voltageIn, int dutyCycle);
void stabilizeVoltageGivenDutyCycle(int dutyCycle, double desiredVoltage); 
void pitchToMaintainVoltage(double turvineVoltage);
double averageLoadVoltage();
double averageTurbineVoltage();
double averagePrePCCVoltage();
int caculateExperimentalDutyCycle(double theoreticalDutyCycle);
boolean disconnectBetterCheck();


void setup(){
  Serial.begin(9600);
  
  //Change the timer settings
  TCCR2B = TCCR2B & 0b11111000 | 0x02;
  
  pinMode(PWM_PIN, OUTPUT);  //Configures PWM pin as output
  pinMode(LOAD_ARDUINO_PIN, OUTPUT);
  pinMode(A0, INPUT);        //For the turbine voltage
  pinMode(A2, INPUT);        //For the prePCC voltage
  pinMode(A6, INPUT);        //For the load voltage
  
  //pinMode(servoPitch, OUTPUT);  Don't think I need this //configures servo pin as output
  pitch.attach(SERVO_PITCH_PIN);
  Serial.println("Beginning the control system code");
  
  //CHECKING IF KILL SWITCH IS HIT
  if(digitalRead(A1) == HIGH){  //because the circuit is normally closed. High means the kill switch is not hit
    digitalWrite(LOAD_ARDUINO_PIN, LOW);
    /*
    if(EEPROM.read(0)){
      pitch.write(EEPROM.read(0+1));
      Serial.print("At pitch: ");
      Serial.println(EEPROM.read(0+1));
      currentPitch = EEPROM.read(0+1);
    }
    else{
      pitch.write(STARTUP_PITCH);
      Serial.print("At pitch: ");
      Serial.println(STARTUP_PITCH);
      currentPitch = STARTUP_PITCH;
    }*/
    pitch.write(STARTUP_PITCH);
    Serial.println(STARTUP_PITCH);
    currentPitch = STARTUP_PITCH;
  }
  else{   //The kill switch is hit
    //digitalWrite(LOAD_ARDUINO_PIN, HIGH);
    pitch.write(BRAKE_PITCH);
    currentPitch = BRAKE_PITCH;
  }
  //if the kill switch is hit don't pitch at all. 
  
  //CHECKING IF KILL SWITCH IS HIT
}

void loop(){

  Serial.print("Current pitch equal to: ");
  Serial.println(currentPitch);
  //delay(2000);
  //For debugging
  Serial.print("Starting a new loop current pitch is currently equal to: ");
  Serial.println(currentPitch);
  //For debugging
  

  //****************************************************//
  //*****Variables to be used in the control system*****//
  //****************************************************//

  double turbineVoltage;
  double loadVoltage, prePCCVoltage;
  double theoreticalOutputVoltage;
  double inferredWindSpeed;
  boolean breakNeeded;
  double RPM;
  double power;
  int theoreticalDutyCycle, experimentalDutyCycle;

  if(digitalRead(A1) == LOW ){ //Kill switch is hit, don't do anything on this loop

    Serial.println("Kill switch hit.");
    //NEEDS TESTING
    double turbineVoltage = averageTurbineVoltage();
    if(turbineVoltage > 7.2){
      digitalWrite(LOAD_ARDUINO_PIN, HIGH);
      Serial.println("Kill switch is hit. Turbine is braking or is already braked.");
      breakNeeded = true;
      processDisconnectedState(breakNeeded);
    }
    else{
      Serial.println("Not enough power to pitch. not sending a signal.");
    }
  }
  else{
    digitalWrite(LOAD_ARDUINO_PIN, LOW);
    if(currentPitch == BRAKE_PITCH){
      pitch.write(STARTUP_PITCH);
      currentPitch = STARTUP_PITCH;
    }
    Serial.println("Kill switch not hit. ");
    
    //Kill switch not hit
    breakNeeded = false;
    //Kill switch not hit
    
    //Reading in turbine, pre-pcc, and load voltage
    turbineVoltage = averageTurbineVoltage();
    //prePCCVoltage = averagePrePCCVoltage();
    loadVoltage = averageLoadVoltage();
    //Reading in turbine, pre-pcc, and load voltage
  
    //CHECKING FOR AND PROCESSING A DISCONNECT
    breakNeeded = determineDisconnect(loadVoltage, turbineVoltage);
    processDisconnectedState(breakNeeded);
    //CHECKING FOR AND PROCESSING A DISCONNECT
    
    if(!breakNeeded){
      if(turbineVoltage < 5){
        
      }
      else{
      
        //For testing
        Serial.print("Reading in a turbine voltage of: ");
        Serial.println(turbineVoltage);
        Serial.print("Reading in a load voltage of: ");
        Serial.println(loadVoltage);
        Serial.print("Reading in a prePCC voltage of: ");
        Serial.println(prePCCVoltage);
       // delay(3000);
        //For testing
        
        //ONLY PROCESS THE REST OF THE CODE IF A BREAK IS NOT NEEDED
    
        //Calculating RPM and Power from turbine voltage
        RPM = getRPMfromVoltageIn(turbineVoltage);
        power = calculatePowerFromRPM(RPM); 
        //Calculating RPM and Power from turbine Voltage
        
               
        //Getting the estimated wind speed
        inferredWindSpeed = inferWindSpeed(turbineVoltage, RPM, power);
        //Getting the estimated wind speed
        Serial.print("Estimating the wind speed to be: ");
        Serial.println(inferredWindSpeed);
        delay(2000);
      
        //Pitching to the corresponding pitch at the estimated wind speed
        if(turbineVoltage < VOLTAGE_AT_11_MS - VOLTAGE_AT_11_MS_BUFFER){
          pitchToPitchAngleBucket(inferredWindSpeed);
          Serial.println("Sending a duty cycle of 255");
        }
        else{
          //MIGHT NEED TO INCLUDE BUFFER HERE NEED TO CHECK
          //analogWrite(PWM_PIN, 255);
          for(int i = 0; i < 25; i++){
            Serial.println("Estimating wind speed to be greater than 11MS");
            if(turbineVoltage > VOLTAGE_AT_11_MS){
              if(currentPitch < MAXIMUM_PITCH_ANGLE){
                pitch.write(++currentPitch);
              }
            }
              else if(turbineVoltage < VOLTAGE_AT_11_MS){
                if(currentPitch > MINIMUM_PITCH_ANGLE){
                  pitch.write(--currentPitch);
                }
              }
            delay(30);
            
          }
          
        }
        //Pitching to the corresponding pitch at the estimated wind speed

        
        if(turbineVoltage >= MAX_VOLTAGE){
          if(currentPitch < BRAKE_PITCH - 3){
            pitch.write(currentPitch+3);
            currentPitch+=3;
          }
        }
      
        
      }//ELSE IF TURBINE VOLTAGE > 5
    } //ELSE !breakNeed
  //delay(5000);
  } //ELSE KILL NOT SWITCH HIT
  
} //VOID LOOP





//********************************************************************************************************************//
//********************************************************************************************************************//
//********************************************************************************************************************//
//*********************************************ADDITIONAL FUNCTIONS***************************************************//
//********************************************************************************************************************//
//********************************************************************************************************************//
//********************************************************************************************************************//





double calculateTheoreticalOutputVoltage(double voltageIn, int PWMdutyCycle){
  return voltageIn*double(PWMdutyCycle/255.0);
}


void processDisconnectedState(boolean disconnected){
  if(disconnected){
    Serial.println("Braking the turbine!");
    breakedInCompetition = true;
    EEPROM.write(0,breakedInCompetition);
    if(currentPitch!= BRAKE_PITCH){
      EEPROM.write(0+1,currentPitch);
    }
    double turbineVoltage = averageTurbineVoltage();
    if(turbineVoltage > 7){
      digitalWrite(LOAD_ARDUINO_PIN, HIGH);
    }
    pitch.write(BRAKE_PITCH);
    currentPitch = BRAKE_PITCH;
    delay(4000); //DELAY SO IT DOES NOT PITCH BACK HERE
  }
  else{
    breakedInCompetition = false;
    digitalWrite(LOAD_ARDUINO_PIN, LOW);
    EEPROM.write(0,breakedInCompetition);
  }
  return;
}

double inferWindSpeed(double voltageIn, double RPM, double power){
  
  
   //For debugging
  //Serial.print("Input voltage of: ");
  //Serial.println(voltageIn);
  Serial.print("Calculating a power of: ");
  Serial.println(power);
  Serial.print("Estimating a wind speed of: ");
  //For debugging
  if(power == 0){
    Serial.println("0");
    return 0;
  }
  //double windSpeed = .187*power-0.000956*pow(power,2)+4.35;
  if(voltageIn>=11 && voltageIn < 22){
    return 6.01;
  }
  else if(voltageIn>=22 && voltageIn < 30){
    return 8.01;
  }
  else if(voltageIn >= 30){
    return 11.01;
  }
  else if(voltageIn <= 3){
    return 0;
  }
  else if(voltageIn > 3 && voltageIn < 11){
    return 5.01;
  }
  
}



double getRPMfromVoltageIn(double turbineVoltage){
  
  //For debugging
  Serial.print("Input voltage of: ");
  Serial.println(turbineVoltage);
  Serial.print("Calculating an RPM of: ");
  
  //For debugging
  double rpm = 108*turbineVoltage-462;
  Serial.print("Calculated RPM as: ");
  if(rpm > 0){
    Serial.println(rpm);
    return rpm;
  }
  else{
    Serial.println("0");
    return 0;
  }
}

double calculatePowerFromRPM(double RPM){
  double power = -.0000000000797*pow(RPM,3)+.0000111*pow(RPM,2)-.00497*RPM-1.91;
  if(power > 0){
    return power;
  }
  else{
    return 0;
  }
}

int calculateTheoreticalDutyCycle(double resistance, double power, double turbineVoltage){
  double theoreticalDutyCycle = PWM_CONVERSION*int(sqrt(resistance*power)/turbineVoltage);

  Serial.print("Theoretical duty calculated as: ");
  Serial.println(theoreticalDutyCycle);
 // delay(3000);

  //ERROR HANDLING
  if(theoreticalDutyCycle > 255){
    return 255;
  }
  else if(theoreticalDutyCycle < 0){
    return 0;
  }
  else
    return theoreticalDutyCycle;
  //ERROR HANDLING
}

void pitchToPitchAngleBucket(double windSpeed){
  Serial.print("Wind speed passed into this function: ");
  Serial.println(windSpeed);
  if(windSpeed > 5 && windSpeed < 7){
    Serial.println("In  the wind speed 5 to 7 ranging. Pitching to this angle. ");
    pitch.write(FIVE_TO_SEVEN_PITCH_ANGLE);
    currentPitch = FIVE_TO_SEVEN_PITCH_ANGLE;
  }
  else if(windSpeed>=7 && windSpeed < 10){
    Serial.println("In  the wind speed 7 to 10 range. Pitching to this angle. ");
    pitch.write(SEVEN_TO_TEN_PITCH_ANGLE);
    currentPitch = SEVEN_TO_TEN_PITCH_ANGLE;
  }
  else if(windSpeed > 10){
    Serial.println("In  the wind speed 10+ range. Pitching to this angle. ");
    pitch.write(TEN_PLUS_INITIAL_PITCH_ANGLE);
    currentPitch = TEN_PLUS_INITIAL_PITCH_ANGLE;
  }
  else{
    Serial.println("Calculating a wind speed of less than 5. Not pitching at all. ");
  }

}


boolean determineDisconnect(double loadVoltage, double prePCCVoltage){
  double totalPrePCCVoltages = prePCCVoltage;
  double totalLoadVoltages = loadVoltage;
  for(int i = 0; i < 9; i++){
    totalPrePCCVoltages += VOLTAGE_DIVIDER_PRE_PCC*((double)analogRead(A2))*5.0/1023.0;
    totalLoadVoltages += VOLTAGE_DIVIDER_LOAD*((double)analogRead(A6))*5.0/1023.0;
    delay(10);
  }
  double averageLoadVoltage = totalLoadVoltages/=10;
  double averagePrePCCVoltage = totalPrePCCVoltages/=10;
  if(averagePrePCCVoltage-averageLoadVoltage > VOLTAGE_DIFFERENT_BUFFER || averageLoadVoltage - averagePrePCCVoltage > VOLTAGE_DIFFERENT_BUFFER){
    Serial.println("System detects a PCC disconnect. Checking in further detail. ");
    boolean checkedFurther = disconnectBetterCheck();
    delay(500);
    return checkedFurther;
  }
  return false;
}

//INPUTS: Duty Cycle and desired output voltage
//ALGORITHM: Checks if the actual output voltage is within a buffer of the theoretical one. If it is greater
//           than the desired output + buffer, drop the duty cycle by one and test again. If it is less than
//           the desired output - buffer, increase the duty cycle by one and test again. 
//
//CONSTRAINTS: DUTY CYCLE CAN NEVER BE MORE THAN 255 OR LESS THAN 0

//THIS FUNCTION IS NOT USED

/*
void stabilizeVoltageGivenDutyCycle(int dutyCycle, double desiredVoltage){
  
  
  int iterations = 0;
  bool flag = true;
  while(iterations < 100 && flag == true){
    flag = false;
    if(VOLTAGE_DIVIDER_LOAD*((double)analogRead(A2))*5.0/1023.0 < desiredVoltage - THEORETICAL_VS_ACTUAL_VOLTAGE_BUFFER){
      flag = true;
      if(dutyCycle < 255){
        analogWrite(PWM_PIN, ++dutyCycle);
      }
      else{
        break;
      }
    }
    else if(VOLTAGE_DIVIDER_LOAD*((double)analogRead(A2))*5.0/1023.0 > desiredVoltage + THEORETICAL_VS_ACTUAL_VOLTAGE_BUFFER){
      flag = true;
      if(dutyCycle > 0){
        analogWrite(PWM_PIN, --dutyCycle);
      }
      else{
        break;
      }
    }
       
    iterations++;
  }
}
*/

//THIS FUNCTION IS NOT USED

void pitchToMaintainVoltage(double turbineVoltage){
  //PROBABLY SHOULD TAKE AN AVERAGE HERE, PROBABLY SHOULD BE TAKING AVERAGES FOR ALL THE VOLTAGES TO BE HONEST
  if(averageTurbineVoltage() > VOLTAGE_AT_11_MS){
    if(currentPitch < MAXIMUM_PITCH_ANGLE){
      pitch.write(++currentPitch);
    }
  }
  else{
    if(currentPitch > MINIMUM_PITCH_ANGLE){
      pitch.write(--currentPitch);
    }
  }
}

double averageTurbineVoltage(){
  double totalVoltage = 0;
  for(int i = 0; i < 10; i++){
    totalVoltage+=VOLTAGE_DIVIDER_TURBINE*((double)analogRead(A0))*5.0/1023.0;
  }
  return totalVoltage/10;

}

double averageLoadVoltage(){
  double totalVoltage = 0;
  for(int i = 0; i < 10; i++){
    totalVoltage+=VOLTAGE_DIVIDER_LOAD*((double)analogRead(A6))*5.0/1023.0;
  }
  return totalVoltage/10;

}

double averagePrePCCVoltage(){
  double totalVoltage = 0;
  for(int i = 0; i < 10; i++){
    totalVoltage+=VOLTAGE_DIVIDER_PRE_PCC*((double)analogRead(A2))*5.0/1023.0;
  }
  return totalVoltage/10;

}

int calculateExperimentalDutyCycle(double theoreticalDutyCycle){
  return theoreticalDutyCycle;                                        //THIS NEEDS TO BE IMPLEMENTED ACCORDING TO THE DATA AFTER 11MS
}


boolean disconnectBetterCheck(){
  double averageOfAveragePrePCCVoltages;
  double averageOfAverageLoadVoltages;
  double totalAveragePrePCCVoltages = 0;
  double totalAverageLoadVoltages = 0;
  for(int i = 0; i < 10; i++){
    totalAveragePrePCCVoltages+=averagePrePCCVoltage();
    totalAverageLoadVoltages+=averageLoadVoltage();
    delay(15);

  }
  averageOfAveragePrePCCVoltages = totalAveragePrePCCVoltages/10;
  Serial.print("Average PrePCCVoltages: ");
  Serial.println(averageOfAveragePrePCCVoltages);
  averageOfAverageLoadVoltages = totalAverageLoadVoltages / 10;
  Serial.print("Average of load voltages");
  Serial.println(averageOfAverageLoadVoltages);
  if(averageOfAveragePrePCCVoltages-averageOfAverageLoadVoltages > VOLTAGE_DIFFERENT_BUFFER || averageOfAverageLoadVoltages - averageOfAveragePrePCCVoltages > VOLTAGE_DIFFERENT_BUFFER){
    if(averageOfAverageLoadVoltages < LOAD_VOLTAGE_BUFFER){
      return true;
    }
  }
  else{
    return false;
  }
  
}


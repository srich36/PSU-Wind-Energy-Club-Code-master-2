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

//****************************************************//
//******************Questions to Ask:*************//
//****************************************************//
//
//1. Is resistance constant? If not, which resistance should I use? Because it is always changing in the equation and optimal power outputs. 
//A: the effective resistance is changing through the duty cycle. The r in the equations is that constant 50
//
//2. How are we determining wind speed? Using the data that has a given Vin and given power output at
//A: Gven inputs of voltage, RPM, and power, we can estimate the wind speed 
//
//3. On the resistance power curve data plot, are those optimal pitch angles at the given wind speed all at a 50ohm resistance? Is that what we are operating at constantly?
//A: We are just doing buckets of pitch angles for a given wind speed
//
//4. So we want to fix power, do we need to find the RPM at the optimal pitch angle at 11 m/s and calculate that power
// Then keep that is constant in the duty cycle equation? Because if so, then we need to test more at 11m/s because we don't have
// data there at a 50 ohm resistance.
//A: Yes the power at 11m/s is a constant we will maintain.



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
const int FIVE_TO_SEVEN_PITCH_ANGLE = 60;
const int SEVEN_TO_TEN_PITCH_ANGLE = 56;
const int TEN_PLUS_INITIAL_PITCH_ANGLE = 50;
const double SIGNAL_BUFFER = .01;
const double THEORETICAL_VS_ACTUAL_VOLTAGE_BUFFER = .3;
static int currentPitch;
static boolean breakedInCompetition = false;
const int CYCLES_PER_THRESHOLD_CROSS = 500;

//****************************************************//
//****************Control System Constants************//
//****************************************************//

const int BRAKE_PITCH = 110; //Need to verify pitch for new turbine -> should be verified now
const int STARTUP_PITCH = 55; //Need to verify pitch for new turbine -> should be verified now
const int MINIMUM_USABLE_PITCH_RANGE = 25;
const int MAXIMUM_USABLE_PITCH_RANGE = 80;
const double VOLTAGE_DIVIDER_TURBINE = 13.015; //This should be the same as last year so we are good
const double VOLTAGE_DIVIDER_LOAD = 14.327;//This needs to be calculated for our new load
const double VOLTAGE_DIVIDER_PRE_PCC = 14.327;
const double VOLTAGE_DIFFERENT_BUFFER = .5;
const double LOAD_VOTLAGE_BUFFER = .5;
const double MAX_VOLTAGE = 45;
const int RESISTANCE = 50; //Is this a constant or does it change? 
const double POWER_AT_11MS = 45.5;
const double VOLTAGE_AT_11_MS = 40;
const double VOLTAGE_AT_11_MS_BUFFER = 1;

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


void setup(){
  Serial.begin(9600);
  
  //Change the timer settings
  //TCCR2B = TCCR2B & 0b11111000 | 0x01;
  
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
    if(EEPROM.read(0)){
      pitch.write(EEPROM.read(0+1));
      currentPitch = EEPROM.read(0+1);
    }
    else{
      pitch.write(STARTUP_PITCH);
      currentPitch = STARTUP_PITCH;
    }
  }
  else{   //The kill switch is hit
    pitch.write(BRAKE_PITCH);
    digitalWrite(LOAD_ARDUINO_PIN, HIGH);
  }
  //if the kill switch is hit don't pitch at all. 
  
  //CHECKING IF KILL SWITCH IS HIT

  analogWrite(PWM_PIN, 50);  
}

void loop(){
  
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
    digitalWrite(LOAD_ARDUINO_PIN, HIGH);
    Serial.println("Kill switch is hit. Turbine is braking or is already braked.");
    breakNeeded = true;
    processDisconnectedState(breakNeeded);
  }
  else{
    Serial.println("Kill switch not hit. ");
    
    //Kill switch not hit
    breakNeeded = false;
    //Kill switch not hit
    
    //Reading in turbine, pre-pcc, and load voltage
    turbineVoltage = averageTurbineVoltage();
    prePCCVoltage = averagePrePCCVoltage();
    loadVoltage = averageLoadVoltage();
    //Reading in turbine, pre-pcc, and load voltage

    //CHECKING FOR AND PROCESSING A DISCONNECT
    breakNeeded = determineDisconnect(loadVoltage, prePCCVoltage);
    processDisconnectedState(breakNeeded);
    //CHECKING FOR AND PROCESSING A DISCONNECT
    
    if(!breakNeeded){
      if(turbineVoltage < 5){
        analogWrite(PWM_PIN, 50); //TALK TO PEYMAN AND MILTON FOR THIS
      }
      else{
      
      
        //For testing
        Serial.print("Reading in a turbine voltage of: ");
        Serial.println(turbineVoltage);
        Serial.print("Reading in a load voltage of: ");
        //Serial.println(loadVoltage);
        Serial.print("Reading in a prePCC voltage of: ");
        Serial.println(prePCCVoltage);
        delay(3000);
        //For testing
        
        //PCC Disconnect
        
        //PCC Disconnect block
           //ONLY PROCESS THE REST OF THE CODE IF A BREAK IS NOT NEEDED
    
        //Calculating RPM and Power from turbine voltage
        RPM = getRPMfromVoltageIn(turbineVoltage);
        power = calculatePowerFromRPM(RPM); 
        //Calculating RPM and Power from turbine Voltage
        
        //For testing
        Serial.print("Calculated power to be: ");
        Serial.println(power);                         //Only need to output power because power comes from RPM
        //For testing
               
        //Getting the estimated wind speed
        inferredWindSpeed = inferWindSpeed(turbineVoltage, RPM, power);
        //Getting the estimated wind speed
      
        //Pitching to the corresponding pitch at the estimated wind speed
        if(turbineVoltage < VOLTAGE_AT_11_MS - VOLTAGE_AT_11_MS_BUFFER){
          pitchToPitchAngleBucket(inferredWindSpeed);
          analogWrite(PWM_PIN, 255);
        }
        else{
          for(int i = 0; i < CYCLES_PER_THRESHOLD_CROSS; i++){
            theoreticalDutyCycle = calculateTheoreticalDutyCycle(RESISTANCE, POWER_AT_11MS, turbineVoltage);
            theoreticalOutputVoltage = double((theoreticalDutyCycle/255.0))*turbineVoltage;
            experimentalDutyCycle = calculateExperimentalDutyCycle(theoreticalDutyCycle);
            
            if(experimentalDutyCycle > 255){
              experimentalDutyCycle = 255;
              theoreticalOutputVoltage = turbineVoltage;
              analogWrite(PWM_PIN, experimentalDutyCycle);
            }
            else if(experimentalDutyCycle < 0){
              experimentalDutyCycle = 0;
              theoreticalOutputVoltage = 0;
              analogWrite(PWM_PIN, experimentalDutyCycle);
            }
            else{
              analogWrite(PWM_PIN, experimentalDutyCycle); //EXPERIMENTAL                
              stabilizeVoltageGivenDutyCycle(experimentalDutyCycle, theoreticalOutputVoltage);  
              pitchToMaintainVoltage(turbineVoltage);                                                    //NEED TO ONLY DO THIS IF IT'S WITHIN A BUFFER
            }
          }
          
        }
        //Pitching to the corresponding pitch at the estimated wind speed
        if(turbineVoltage >= MAX_VOLTAGE){
          pitch.write(currentPitch-3);
          currentPitch-=3;
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
    digitalWrite(LOAD_ARDUINO_PIN, HIGH);
    pitch.write(BRAKE_PITCH);
    currentPitch = BRAKE_PITCH;
    delay(1000); //DELAY SO IT DOES NOT PITCH BACK HERE
  }
  else{
    breakedInCompetition = false;
    digitalWrite(LOAD_ARDUINO_PIN, HIGH);
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
  double windSpeed = .187*power-0.000956*pow(power,2)+4.35;
  if(windSpeed > 0){
    Serial.println(windSpeed);
    return windSpeed;
  }
  else{
    return 0;
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
  delay(3000);

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
    delay(1);
  }
  double averageLoadVoltage = totalLoadVoltages/=10;
  double averagePrePCCVoltage = totalPrePCCVoltages/=10;
  if(averagePrePCCVoltage-averageLoadVoltage > VOLTAGE_DIFFERENT_BUFFER && averageLoadVoltage < LOAD_VOTLAGE_BUFFER){
    Serial.println("System detects a PCC disconnect");
    delay(1000);
    return true;
  }
  return false;
}

//INPUTS: Duty Cycle and desired output voltage
//ALGORITHM: Checks if the actual output voltage is within a buffer of the theoretical one. If it is greater
//           than the desired output + buffer, drop the duty cycle by one and test again. If it is less than
//           the desired output - buffer, increase the duty cycle by one and test again. 
//
//CONSTRAINTS: DUTY CYCLE CAN NEVER BE MORE THAN 255 OR LESS THAN 0

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

void pitchToMaintainVoltage(double turbineVoltage){
  //PROBABLY SHOULD TAKE AN AVERAGE HERE, PROBABLY SHOULD BE TAKING AVERAGES FOR ALL THE VOLTAGES TO BE HONEST
  if(averageTurbineVoltage() > VOLTAGE_AT_11_MS){
    if(currentPitch > MINIMUM_USABLE_PITCH_RANGE){
      pitch.write(--currentPitch);
    }
  }
  else{
    if(currentPitch < MAXIMUM_USABLE_PITCH_RANGE){
      pitch.write(++currentPitch);
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
  return theoreticalDutyCycle;                                        //THIS NEEDS TO BE IMPLEMENTED ACCORDING TO THE DATA
}




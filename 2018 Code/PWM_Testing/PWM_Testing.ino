const int PWMPin = 3;
const double VOLTAGE_DIVIDER_TURBINE = 13.015; //This should be the same as last year so we are good
const double VOLTAGE_DIVIDER_LOAD = 14.327;

void setup()
{
  
  TCCR2B = TCCR2B & 0b11111000 | 0x04;    // set timer 2 divisor to 1 for PWM frequency of 31372.55 Hz 
  pinMode(PWMPin, OUTPUT);
  Serial.begin(9600);
  Serial.println("Beginning test of PWM functionality in 10 seconds");
  delay(10000);
}

void writePWMVoltage(double PWMdutycycle){
  Serial.print("Sending a duty cycle of "); 
  Serial.println(PWMdutycycle);
  analogWrite(PWMPin,PWMdutycycle);
  double turbineVoltage = VOLTAGE_DIVIDER_TURBINE*((double)analogRead(A0))*5.0/1023.0;
  double loadVoltage = VOLTAGE_DIVIDER_LOAD*((double)analogRead(A2))*5.0/1023.0;
    //Reading in turbine and load voltage
    
    
    //For testing
  Serial.print("Reading in a turbine voltage of: ");
  Serial.println(turbineVoltage);
  Serial.print("Reading in a load voltage of: ");
  Serial.println(loadVoltage);
  delay(5000);
}


void loop()
{
  writePWMVoltage(255);
}



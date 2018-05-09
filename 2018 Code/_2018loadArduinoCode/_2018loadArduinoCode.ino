//TODO: 
//      1. figure out how to send the control signal to the arduino to break
//      2. Confirm both the voltage divider constants
//      3. Implement the kill switch in the hardware and test to make sure it is reading it correctly
//      4. Confirm constant buffer values
//     



const double VOLTAGE_DIVIDER_LOAD= 14.327; // Need to confirm both of these values
const int MOSFET_SIGNAL_PIN = 3;
const int CONTROL_ARDUINO_PIN = 10;      //Generic pin values that we can figure out
const double LOAD_VOLTAGE_BUFFER = 1.5;
const int ARDUINO_TURN_ON_TIME = 10000;
static boolean brakedBefore;



boolean determineDisconnect(double loadVoltage, double turbineVoltage);
void setup(){
  Serial.begin(9600);
  pinMode(MOSFET_SIGNAL_PIN, OUTPUT);
  //pinMode(KILL_SWITCH_PIN, INPUT);
  pinMode(CONTROL_ARDUINO_PIN, INPUT);
  digitalWrite(MOSFET_SIGNAL_PIN, LOW);
  brakedBefore = false;
}

void loop(){
  
  
  double loadVoltage = VOLTAGE_DIVIDER_LOAD*((double)analogRead(A0))*5.0/1023.0;  //Attach the load voltage reading on A1
  
  
  //For debugging
  Serial.print("Reading the load voltage as: ");
  Serial.println(loadVoltage);
  //For debugging
  
  boolean braked = (digitalRead(CONTROL_ARDUINO_PIN) == HIGH); //Has to be high
  if(braked){
    Serial.println("Setting braked ");
    digitalWrite(MOSFET_SIGNAL_PIN, HIGH);
    Serial.println("Recieving a high signal");
    delay(ARDUINO_TURN_ON_TIME);  
    brakedBefore = true;
  }
  //THIS SHOULD WORK ONLY IF 
  else if(!braked && loadVoltage > LOAD_VOLTAGE_BUFFER && brakedBefore){
    Serial.println("Recieving a low signal");
    digitalWrite(MOSFET_SIGNAL_PIN, LOW);

  }
  
  
}
  
  

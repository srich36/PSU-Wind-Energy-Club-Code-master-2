//TODO: 
//      1. figure out how to send the control signal to the arduino to break
//      2. Confirm both the voltage divider constants
//      3. Implement the kill switch in the hardware and test to make sure it is reading it correctly
//      4. Confirm constant buffer values
//     


const int MOSFET_SIGNAL_PIN = 3;
const int CONTROL_SIGNAL_PIN = 10;
const int CONTROL_ARDUINO_TURN_ON_TIME = 3000;
const double VOLTAGE_DIVIDER_LOAD = 14.317;
const double LOAD_VOLTAGE_BUFFER = 1.5;
static boolean crossedThreshold;




void setup(){
  pinMode(MOSFET_SIGNAL_PIN, OUTPUT);
  pinMode(CONTROL_SIGNAL_PIN, INPUT);
  crossedThreshold = false;
  // pinMode(KILL_SWITCH_PIN, INPUT);
  digitalWrite(MOSFET_SIGNAL_PIN, LOW);
}

void loop(){
  
  if(digitalRead(CONTROL_SIGNAL_PIN) == HIGH){
      
    digitalWrite(MOSFET_SIGNAL_PIN, HIGH); //turn on the mosfet
    delay(CONTROL_ARDUINO_TURN_ON_TIME);                         //HANDLES KILL SWITCH CASE
    crossedThreshold = true;
    
  }
  else{
    if(VOLTAGE_DIVIDER_LOAD*((double)analogRead(A0))*5.0/1023.0 <= 0+LOAD_VOLTAGE_BUFFER && crossedThreshold){ //THIS COVERS THE PCC DISCONNECT CODE. 
      digitalWrite(MOSFET_SIGNAL_PIN, HIGH);
    }
    else{
      digitalWrite(MOSFET_SIGNAL_PIN, LOW);
    
    }
  }
}
  
  



//TODO: 
//      1. figure out how to send the control signal to the arduino to break
//      2. Confirm both the voltage divider constants
//      3. Implement the kill switch in the hardware and test to make sure it is reading it correctly
//      4. Confirm constant buffer values
//     


const int MOSFET_SIGNAL_PIN = 3;
const int CONTROL_SIGNAL_PIN = 10;




void setup(){
  pinMode(MOSFET_SIGNAL_PIN, OUTPUT);
  pinMode(CONTROL_SIGNAL_PIN, INPUT);
 // pinMode(KILL_SWITCH_PIN, INPUT);
 //digitalWrite(MOSFET_SIGNAL_PIN, LOW);
}

void loop(){
  
  if(digitalRead(CONTROL_SIGNAL_PIN) == HIGH){
      
    digitalWrite(MOSFET_SIGNAL_PIN, HIGH); //turn on the mosfet
    //delay(10000);                         //Don't need to wait 10 seconds, just need to make sure the control sends low if it is not braked
    digitalWrite(MOSFET_SIGNAL_PIN, LOW); //turn mosfet back off
  }
  else{
    digitalWrite(MOSFET_SIGNAL_PIN, LOW);
  }
}
  
  



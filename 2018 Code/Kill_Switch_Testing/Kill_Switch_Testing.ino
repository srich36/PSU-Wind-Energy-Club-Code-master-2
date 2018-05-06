
void setup(){
  pinMode(A1, INPUT);
  pinMode(8, OUTPUT);
  Serial.begin(9600);
  Serial.println("Starting. ");
}


void loop(){
  boolean killed = (digitalRead(A1) == LOW);
  double voltage = digitalRead(A1);
  Serial.print("Voltage equals: ");
  if(digitalRead(A1) == HIGH){
    Serial.println("Returned high");
    Serial.println("Sending a high signal to the load Arduino.");
    digitalWrite(8, HIGH);
    delay(1000);
  }
  else{
    Serial.println("Returned low");
  }
  Serial.println(voltage);
  Serial.println(killed);
  delay(1000);
}

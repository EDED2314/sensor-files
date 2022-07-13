
int led = 13;
void setup() {
  
  pinMode(led, OUTPUT);
  Serial.begin(9600);
  
}

void loop() {
  // put your main code here, to run repeatedly:
  digitalWrite(led, HIGH);   
  delay(1000);               
  Serial.println("blink...");
  digitalWrite(led, LOW);    
  delay(1000); 
}

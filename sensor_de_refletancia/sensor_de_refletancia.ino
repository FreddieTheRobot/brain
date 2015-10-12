int pinoSensor1 = 2;
//int pinoSensor2 = A2;
int valorSensor1 = 0;
//int valorSensor2 = 0;
const int motor1 = 6;
const int dirA = 7; //direcao do motor A - HIGH ou LOW

void setup() {
  Serial.begin(9600);
  pinMode(motor1, OUTPUT);
  pinMode(dirA, OUTPUT);
}

void loop() {
  valorSensor1 = analogRead(pinoSensor1);
  //valorSensor2 = analogRead(pinoSensor2);
  Serial.print("Pino 1: ");
  Serial.print("\t");
  Serial.println(valorSensor1);
  //Serial.print("Pino 2: ");
  //Serial.print("\t");
  //Serial.println(valorSensor2);  
  delay(500);  

  digitalWrite(dirA, HIGH);
  analogWrite(motor1,100);



}

const int motorA = 5; //velocidade motor A - de 0 a 255
const int motorB = 6; //velocidade motor B - de 0 a 255
const int dirA = 7; //direcao do motor A - HIGH ou LOW
const int dirB = 8; //direcao do motor B - HIGH ou LOW


void setup() {
  pinMode(motorA, OUTPUT);
  pinMode(motorB, OUTPUT);
  pinMode(dirA, OUTPUT);
  pinMode(dirB, OUTPUT);
}

void loop() {
  digitalWrite(dirA, HIGH); //SENTIDO DE ROTACAO
  digitalWrite(dirB, LOW);
  analogWrite(motorA, 200); //VELOCIDADE
  analogWrite(motorB, 200);
  
  delay(1000);
  
  analogWrite(motorA, 0);
  analogWrite(motorB, 0);
  
  delay(1000);
  
  digitalWrite(dirA, LOW);
  digitalWrite(dirB, HIGH);
  analogWrite(motorA, 200);
  analogWrite(motorB, 200);
  
  delay(1000);

  analogWrite(motorA, 0);
  analogWrite(motorB, 0);
  
  delay(1000);
}

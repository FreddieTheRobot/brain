// Sensor de refletancia
int pinoSensor1 = 1;//Sensor de refletancia esquerdo
int pinoSensor2 = 2;//Sensor de refletancia direito
int valorSensor1 = 0;//Valor sensor de refletancia esquerdo
int valorSensor2 = 0;//Valor sensor de refletancia direito
// Motor
const int motorA = 5; //velocidade motor A - de 0 a 255
const int motorB = 6; //velocidade motor B - de 0 a 255
const int dirA = 7; //direcao do motor A - HIGH ou LOW
const int dirB = 8; //direcao do motor B - HIGH ou LOW

void setup() {
  Serial.begin(9600);

  pinMode(motorA, OUTPUT);
  pinMode(motorB, OUTPUT);
  pinMode(dirA, OUTPUT);
  pinMode(dirB, OUTPUT);
}

void loop() {
  valorSensor1 = analogRead(pinoSensor1);
  valorSensor2 = analogRead(pinoSensor2);
  
  Serial.print("Pino 1: ");
  Serial.print("\t");
  Serial.println(valorSensor1);

  Serial.print("Pino 2: ");
  Serial.print("\t");
  Serial.println(valorSensor2);

  if(valorSensor1 < 300 && valorSensor2 < 300) {
    digitalWrite(dirB, HIGH); //SENTIDO DE ROTACAO
    analogWrite(motorB, 200); //VELOCIDADE
    digitalWrite(dirA, LOW); //SENTIDO DE ROTACAO
    analogWrite(motorA, 200); //VELOCIDADE
  } else {
    analogWrite(motorB, 0);
    analogWrite(motorA, 0);
  }

  delay(500);
}

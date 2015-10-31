// Sensor de refletancia
#define pinoSensor1 A1 //Sensor de refletancia direito
#define pinoSensor2 A2 //Sensor de refletancia esquerdo

// Motores
#define M_Direito 5 //Motor Direito(A) Vel 0 - 255
#define M_Esquerdo 6 //Motor Esquerdo(B) Vel 0 - 255
#define dirD 7 //Direcao do motor A - HIGH (Frente) ou LOW (Tras)
#define dirE 8 //Direcao do motor B - HIGH (Frente) ou LOW (Tras)

#define DIR_D_NORMAL LOW
#define DIR_D_REVERSO HIGH
#define DIR_E_NORMAL HIGH
#define DIR_E_REVERSO LOW

// Velocidade normal
#define VEL_D_NORMAL 150
#define VEL_E_NORMAL 100

// Velocidade maxima
#define VEL_MAXIMA 180

// Refletancia mínima.
// Abaixo disso, o sensor está SOBRE a faixa.
#define LIMITE 450

// Aciona motores
// Caso queira apensa ver os valores dos sensores no monitor serial,
// sem que os motores sejam acionados, coloque o valor 0.
#define ENGINES_ON 1



// MENOR o valor => está no BRANCO
// MAIOR o valor => está no PRETO
int valorSensor1 = 0; //Valor sensor de refletancia esquerdo
int valorSensor2 = 0; //Valor sensor de refletancia direito

//PID
float Kp = 0.5;
float Ki = 0;
float Kd = 0;

int valorFora = 0;
float error = 0;
float previousError = 0;
float totalError = 0;
float derivative = 0;

float power = 0;

float P;
float I;
float D;


void setup() {
  Serial.begin(9600);

  pinMode(M_Esquerdo, OUTPUT);
  pinMode(M_Direito, OUTPUT);
  pinMode(dirD, OUTPUT);
  pinMode(dirE, OUTPUT);

  Para();

  delay(5000);

  MoveParaFrente();
}

void Para()
{
  analogWrite(M_Esquerdo, 0);
  analogWrite(M_Direito, 0);
}


void MoveParaFrente()
{
#if ENGINES_ON == 1
  digitalWrite(dirD, DIR_D_NORMAL);
  analogWrite(M_Direito, VEL_D_NORMAL);
  digitalWrite(dirE, DIR_E_NORMAL);
  analogWrite(M_Esquerdo, VEL_E_NORMAL);
#endif
}


void loop() {
  lerSensores();

  corrigeCurso();
  PID();

  //delay(250);
}



void lerSensores() {
  valorSensor1 = analogRead(pinoSensor1);
  valorSensor2 = analogRead(pinoSensor2);
  
#if ENGINES_ON == 0
    Serial.print("valorSensor1 = ");
    Serial.print(valorSensor1);
    
    Serial.print("\t");
    Serial.print("valorSensor2 = ");
    Serial.print(valorSensor2);

    Serial.print("\t");
    Serial.print("valorFora = ");
    Serial.print(valorFora);
    
    Serial.print("\t");
    Serial.print("previousError = ");
    Serial.print(previousError);
    
    Serial.print("\t");
    Serial.print("error = ");
    Serial.print(error);
    
    Serial.print("\t");
    Serial.print("totalError = ");
    Serial.print(totalError);
        
    Serial.print("\t");
    Serial.print("power = ");
    Serial.println(power);
    
    delay(1000);
#endif
}


void corrigeCurso() {
//    if(valorSensor1 < LIMITE && valorSensor2 < LIMITE) {
//      valorFora = 0;
//      //MoveParaFrente();
//    }
  
    if(valorSensor2 > LIMITE) { 
      // Sensor ESQUERDO entrou na faixa

      valorFora = 1;
   }

   if(valorSensor1 > LIMITE) {
      // Sensor DIREITO entrou na faixa

      valorFora = -1;
   }
}


void PID() {
//  if(valorFora == 0) {
//    error = 0;
//    totalError = 0;
//    derivative = 0;
//    previousError = 0;
//    power = 0;
//    return;
//  }
  
  error = abs(((valorSensor1 + valorSensor2)/2) - LIMITE);
  totalError += error;
  derivative = error - previousError;
  previousError = error;

  P = (Kp * error);
  I = (Ki * totalError);
  D = (Kd * derivative);
  
  power = P + I + D;

  if(power > 100) {
    power = 100;
  }

  if(power < -100) {
    power = -100;
  }

  power = power * valorFora;

#if ENGINES_ON == 1
  if(power < 0) {
      // Move o robô para a esquerda

      //digitalWrite(dirD, DIR_D_NORMAL);
      analogWrite(M_Direito, VEL_D_NORMAL);
      //digitalWrite(dirE, DIR_E_REVERSO);
      analogWrite(M_Esquerdo, VEL_E_NORMAL - abs(int(power)));    
  } else if (power > 0) {
      // Move o robô para a direita
      
      //digitalWrite(dirD, DIR_D_REVERSO);
      analogWrite(M_Direito, VEL_D_NORMAL - int(power));
      //digitalWrite(dirE, DIR_E_NORMAL);
      analogWrite(M_Esquerdo, VEL_E_NORMAL);
  } else {
      // Move para frente
      
      //digitalWrite(dirD, DIR_D_REVERSO);
      analogWrite(M_Direito, VEL_D_NORMAL);
      //digitalWrite(dirE, DIR_E_NORMAL);
      analogWrite(M_Esquerdo, VEL_E_NORMAL);
  }
#endif

}


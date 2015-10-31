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
#define LIMITE 780

// Aciona motores
// Caso queira apensa ver os valores dos sensores no monitor serial,
// sem que os motores sejam acionados, coloque o valor 0.
#define ENGINES_ON 1



// MENOR o valor => está no BRANCO
// MAIOR o valor => está no PRETO
int valorSensor1 = 0; //Valor sensor de refletancia esquerdo
int valorSensor2 = 0; //Valor sensor de refletancia direito

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
  
  #if ENGINES_ON == 1
    corrigeCurso();
  #endif

  //delay(250);
}



void lerSensores() {
  valorSensor1 = analogRead(pinoSensor1);
  valorSensor2 = analogRead(pinoSensor2);

  if (ENGINES_ON == 0) {
    Serial.print("valorSensor1 = ");
    Serial.print(valorSensor1);
    Serial.print("\t");
    Serial.print("valorSensor2 = ");
    Serial.println(valorSensor2);
    delay(1000);
  }
}


void corrigeCurso() {
    if(valorSensor1 < LIMITE && valorSensor2 < LIMITE) {
      MoveParaFrente();
    }
  
    if(valorSensor2 > LIMITE) { 
      // Sensor ESQUERDO entrou na faixa
      // Move o robô para a esquerda

      digitalWrite(dirD, DIR_D_NORMAL);
      analogWrite(M_Direito, VEL_D_NORMAL * 0.8);
      digitalWrite(dirE, DIR_E_REVERSO);
      analogWrite(M_Esquerdo, VEL_E_NORMAL * 0.8);
   }

   if(valorSensor1 > LIMITE) {
      // Sensor DIREITO entrou na faixa
      // Move o robô para a direita
      
      digitalWrite(dirD, DIR_D_REVERSO);
      analogWrite(M_Direito, VEL_D_NORMAL * 0.8);
      digitalWrite(dirE, DIR_E_NORMAL);
      analogWrite(M_Esquerdo, VEL_E_NORMAL * 0.8);
   }
}




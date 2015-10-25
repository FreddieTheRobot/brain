// Sensor de refletancia
#define pinoSensor1 A1 //Sensor de refletancia direito
#define pinoSensor2 A2 //Sensor de refletancia esquerdo

int valorSensor1 = 0; //Valor sensor de refletancia esquerdo
int valorSensor2 = 0; //Valor sensor de refletancia direito

// Motores
#define M_Direito 5 //Motor Direito(A) Vel 0 - 255
#define M_Esquerdo 6 //Motor Esquerdo(B) Vel 0 - 255
#define dirD 7 //Direcao do motor A - HIGH (Frente) ou LOW (Tras)
#define dirE 8 //Direcao do motor B - HIGH (Frente) ou LOW (Tras)

#define DIR_D_NORMAL LOW
#define DIR_D_REVERSO HIGH
#define DIR_E_NORMAL HIGH
#define DIR_E_REVERSO LOW

// Debug sensores
#define DEBUG_SENSORES 1

// Velocidade normal
#define VEL_NORMAL 100

// Velocidade maxima
#define VEL_MAXIMA 180

// Refletancia mínima.
// Abaixo disso, o sensor está SOBRE a faixa.
#define REFLETANCIA_MIN 850


int V_Direito, V_Esquerdo;

int mode;

void setup() {
  Serial.begin(9600);

  pinMode(M_Esquerdo, OUTPUT);
  pinMode(M_Direito, OUTPUT);
  pinMode(dirD, OUTPUT);
  pinMode(dirE, OUTPUT);


  // Para debugar valores dos sensores no terminal, 
  // com o cabo conectado à placa, utilize mode = 1.
  mode = 0; // 0 = motores funcionam; 1 = motores parados

  Para();

  if(mode != DEBUG_SENSORES) {
    PFrente();
  }
}

void Para()
{
  analogWrite(M_Esquerdo, 0);
  analogWrite(M_Direito, 0);
}


void PFrente()
{
  digitalWrite(dirD, DIR_D_NORMAL);
  analogWrite(M_Direito, VEL_NORMAL);
  digitalWrite(dirE, DIR_E_NORMAL);
  analogWrite(M_Esquerdo, VEL_NORMAL);
}


void loop() {

  segueLinha();

  Serial.print("valorSensor1 = ");
  Serial.print(valorSensor1);
  Serial.print("\t");
  Serial.print("valorSensor2 = ");
  Serial.print(valorSensor2);

  delay(1500);
}


void segueLinha() {

  if(mode != DEBUG_SENSORES) {
    PFrente();
  }

  lerSensores();

  if(valorSensor1 == 1023 && valorSensor2 == 1023) {
    Para();
  } else {
    if(mode != DEBUG_SENSORES) {
       corrigeCurso();
    }
  }

}

void corrigeCurso() {
    if(valorSensor1 < REFLETANCIA_MIN) { // Sensor DIREITO entrou na faixa
      digitalWrite(dirD, DIR_D_REVERSO);
      analogWrite(M_Direito, VEL_MAXIMA);
      digitalWrite(dirE, DIR_E_NORMAL);
      analogWrite(M_Esquerdo, VEL_MAXIMA);
   }

   if(valorSensor2 < REFLETANCIA_MIN) { // Sensor ESQUERDO entrou na faixa
      digitalWrite(dirE, DIR_E_REVERSO);
      analogWrite(M_Esquerdo, VEL_MAXIMA);
      digitalWrite(dirD, DIR_D_NORMAL);
      analogWrite(M_Direito, VEL_MAXIMA);
   }
}


void lerSensores() {
  valorSensor1 = analogRead(pinoSensor1);
  valorSensor2 = analogRead(pinoSensor2);
}

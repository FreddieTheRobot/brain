// Sensor de refletancia
#define pinoSensor1 A1 //Sensor de refletancia direito
#define pinoSensor2 A2 //Sensor de refletancia esquerdo

//Sensor ultrassonico
#define echoPin 13 //Pino 13 recebe o pulso do echo
#define trigPin 12 //Pino 12 envia o pulso para gerar o echo

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
#define LIMITE_OTICO 780

// Distancia minima em cm
// Abaixo disso, o robo para por 10 segundos
#define LIMITE_SONICO 10

// Aciona motores
// Caso queira apenas ver os valores dos sensores no monitor serial,
// sem que os motores sejam acionados, coloque o valor 0.
#define ENGINES_ON 1



// MENOR o valor => está no BRANCO
// MAIOR o valor => está no PRETO
int valorSensor1 = 0; //Valor sensor de refletancia esquerdo
int valorSensor2 = 0; //Valor sensor de refletancia direito

long duracao; //Valor para o calculo da distancia
long distancia; //Valor da distancia em cm

void setup() {
  Serial.begin(9600);

  pinMode(echoPin, INPUT); // define o pino 13 como entrada (recebe)
  pinMode(trigPin, OUTPUT); // define o pino 12 como saida (envia)

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

  if(detectouObstaculo()) {
    Para();
    delay(10000);
  }

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
    Serial.print("\t");
    Serial.print("duracao = ");
    Serial.println(duracao);
    Serial.print("\t");
    Serial.print("distancia = ");
    Serial.println(distancia);
    delay(1000);
  }
}


void corrigeCurso() {
    if(valorSensor1 < LIMITE_OTICO && valorSensor2 < LIMITE_OTICO) {
      MoveParaFrente();
    }
  
    if(valorSensor2 > LIMITE_OTICO && valorSensor1 < LIMITE_OTICO) { 
      // Sensor ESQUERDO entrou na faixa
      // Move o robô para a esquerda

      
      digitalWrite(dirD, DIR_D_NORMAL);
      analogWrite(M_Direito, VEL_D_NORMAL * 1.5);
      digitalWrite(dirE, DIR_E_REVERSO);
      analogWrite(M_Esquerdo, VEL_E_NORMAL * 0.75);
   }

   if(valorSensor1 > LIMITE_OTICO && valorSensor2 < LIMITE_OTICO) {
      // Sensor DIREITO entrou na faixa
      // Move o robô para a direita
      
      digitalWrite(dirD, DIR_D_REVERSO);
      analogWrite(M_Direito, VEL_D_NORMAL * 0.75);
      digitalWrite(dirE, DIR_E_NORMAL);
      analogWrite(M_Esquerdo, VEL_E_NORMAL * 1.5);
   }

  if(valorSensor1 > LIMITE_OTICO && valorSensor2 > LIMITE_OTICO){
      //Sensor DIREITO E ESQUERDO NA FAIXA
      //Da ré

      digitalWrite(dirD, DIR_D_REVERSO);
      analogWrite(M_Direito, VEL_D_NORMAL);
      digitalWrite(dirE, DIR_E_REVERSO);
      analogWrite(M_Esquerdo, VEL_E_NORMAL);
  }

}


bool detectouObstaculo() {
  //seta o pino 12 com um pulso baixo "LOW" ou desligado ou ainda 0  
  digitalWrite(trigPin, LOW);  
  // delay de 2 microssegundos  
  delayMicroseconds(2);  
  //seta o pino 12 com pulso alto "HIGH" ou ligado ou ainda 1  
  digitalWrite(trigPin, HIGH);  
  //delay de 10 microssegundos  
  delayMicroseconds(10);  
  //seta o pino 12 com pulso baixo novamente  
  digitalWrite(trigPin, LOW);  
  //pulseInt lê o tempo entre a chamada e o pino entrar em high  
  duracao = pulseIn(echoPin,HIGH);  
  //Esse calculo é baseado em s = v.t, lembrando que o tempo vem dobrado  
  //porque é o tempo de ida e volta do ultrassom
  //e a velocidade do som é de aproximadamente 29.1 microsegundos por centimetro
  distancia = (duracao/2)/29.1;

  if(distancia < LIMITE_SONICO) {
    return true;
  } else {
    return false;
  }
}




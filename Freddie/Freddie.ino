// Sensor de refletancia
#define pinoSensor1 A1            //Sensor de refletancia direito
#define pinoSensor2 A2            //Sensor de refletancia esquerdo

//Sensor ultrassonico
#define echoPin 13                //Pino 13 recebe o pulso do echo
#define trigPin 12                //Pino 12 envia o pulso para gerar o echo

// Motores
#define M_Direito 5               // Pino da velocidade de rotação do motor direito
#define M_Esquerdo 6              // Pino da velocidade de rotação do motor esquerdo
#define dirD 7                    // Pino do sentido de rotação do motor direito
#define dirE 8                    // Pino do sentido de rotação do motor esquerdo

// Sentido de rotação dos motores
#define DIR_D_NORMAL LOW          // Rotação para frente do motor direito
#define DIR_D_REVERSO HIGH        // Rotação para trás do motor direito
#define DIR_E_NORMAL HIGH         // Rotação para frente do motor esquerdo
#define DIR_E_REVERSO LOW         // Rotação para trás do motor esquerdo

// Velocidade normal
#define VEL_D_NORMAL 150 * 0.72   // Velociade normal do motor direito
#define VEL_E_NORMAL 103 * 0.72   // Velociade normal do motor esquerdo
#define VEL_D_CURVA VEL_D_NORMAL  // Velociade na curva do motor direito
#define VEL_E_CURVA VEL_E_NORMAL  // Velociade na curva do motor esquerdo

// Refletancia mínima.
// Abaixo disso, o sensor está SOBRE a faixa.
#define LIMITE_OTICO 780          // Limite de refletância

// Distancia minima em cm
// Abaixo disso, o robo para por 10 segundos
#define LIMITE_SONICO 10          // Limite sônico

// Aciona motores
// Caso queira apenas ver os valores dos sensores no monitor serial,
// sem que os motores sejam acionados, coloque o valor 0.
#define ENGINES_ON 1              // Flag para controle dos motores (debug)



// MENOR o valor => está no BRANCO
// MAIOR o valor => está no PRETO
int valorSensor1 = 0;             // Valor sensor de refletancia esquerdo
int valorSensor2 = 0;             // Valor sensor de refletancia direito

long duracao;                     // Valor para o calculo da distancia
long distancia;                   // Valor da distancia em cm



void setup() {
  Serial.begin(9600);

  pinMode(echoPin, INPUT);        // define o pino 13 como entrada (recebe)
  pinMode(trigPin, OUTPUT);       // define o pino 12 como saida (envia)

  pinMode(M_Esquerdo, OUTPUT);    // configura o pino de velocidade do motor esquerdo
  pinMode(M_Direito, OUTPUT);     // configura o pino de velocidade do motor direito
  pinMode(dirD, OUTPUT);          // configura o pino de sentido de rotação do motor direito
  pinMode(dirE, OUTPUT);          // configura o pino de sentido de rotaçãodo motor esquerdo

  Para();                         // Para o robô

  delay(3000);

  MoveParaFrente();               // Inicia o movimento
}


void loop() {

  SeguePista();                   // Lê sensores e faz correção da trajetória

  // Se detectou o obstáculo, para por 10s e reinicia o movimento.
  if (detectouObstaculo()) {
    Para();
    delay(10000);
    int i = 0;
    while (i < 10000) {
      SeguePista();
      i++;
    }
  }

}

// Lê sensores e faz correção da trajetória
void SeguePista() {
  lerSensores();

#if ENGINES_ON == 1
  corrigeCurso();
#endif
}

// Para os motores
void Para()
{
  analogWrite(M_Esquerdo, 0);
  analogWrite(M_Direito, 0);
}

// Movo o robô para frente
void MoveParaFrente()
{
#if ENGINES_ON == 1
  digitalWrite(dirD, DIR_D_NORMAL);
  analogWrite(M_Direito, VEL_D_NORMAL);
  digitalWrite(dirE, DIR_E_NORMAL);
  analogWrite(M_Esquerdo, VEL_E_NORMAL);
#endif
}

// Move o robô para a esquerda
// - aumentando a velocidade do motor direito
// - diminuindo a velocidada do motor esquerdo
// - invertendo o sentido de rotação do motor esquerdo
void MoveParaEsquerda() {
  digitalWrite(dirD, DIR_D_NORMAL);
  analogWrite(M_Direito, VEL_D_CURVA * 1.5);
  digitalWrite(dirE, DIR_E_REVERSO);
  analogWrite(M_Esquerdo, VEL_E_CURVA * 0.75);
}

// Move o robô para a direita
// - aumentando a velocidade do motor esquerdo
// - diminuindo a velocidada do motor direito
// - invertendo o sentido de rotação do motor direito
void MoveParaDireita() {
  digitalWrite(dirD, DIR_D_REVERSO);
  analogWrite(M_Direito, VEL_D_CURVA * 0.75);
  digitalWrite(dirE, DIR_E_NORMAL);
  analogWrite(M_Esquerdo, VEL_E_CURVA * 2.3);
}

// Movo o robô para trás
void MoveParaTras() {
  digitalWrite(dirD, DIR_D_REVERSO);
  analogWrite(M_Direito, VEL_D_NORMAL);
  digitalWrite(dirE, DIR_E_REVERSO);
  analogWrite(M_Esquerdo, VEL_E_NORMAL);
}

// Lê os sensores óticos
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


// Corrige o curso de acordo com os valores obtidos dos sensores óticos
void corrigeCurso() {
  // Ambos os sensores estão sobre a superfície clara
  // Move para frente
  if (valorSensor1 < LIMITE_OTICO && valorSensor2 < LIMITE_OTICO) {
    MoveParaFrente();
  }

  // Sensor ESQUERDO entrou na faixa
  // Move o robô para a esquerda
  if (valorSensor2 > LIMITE_OTICO && valorSensor1 < LIMITE_OTICO) {
    MoveParaEsquerda();
  }

  // Sensor DIREITO entrou na faixa
  // Move o robô para a direita
  if (valorSensor1 > LIMITE_OTICO && valorSensor2 < LIMITE_OTICO) {
    MoveParaDireita();
  }

  // Ambos os sensores estão sobre a superfície escura
  // Move para trás até a situação se reverter
  if (valorSensor1 > LIMITE_OTICO && valorSensor2 > LIMITE_OTICO) {
    while (valorSensor1 > LIMITE_OTICO && valorSensor2 > LIMITE_OTICO) {
      for (int i = 0; i < 11000; i++) {
        MoveParaTras();
      }
      lerSensores();
    }
  }

}

// Verifica se há obstáculo, utilizando os valores obtidos do sensor ultrassônico
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
  duracao = pulseIn(echoPin, HIGH);
  //Esse calculo é baseado em s = v.t, lembrando que o tempo vem dobrado
  //porque é o tempo de ida e volta do ultrassom
  //e a velocidade do som é de aproximadamente 29.1 microsegundos por centimetro
  distancia = (duracao / 2) / 29.1;

  if (distancia < LIMITE_SONICO) {
    return true;
  } else {
    return false;
  }
}

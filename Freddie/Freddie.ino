// Sensor de refletancia
#define pinoSensor1 = A1;//Sensor de refletancia direito
#define pinoSensor2 = A2;//Sensor de refletancia esquerdo
int valorSensor1 = 0;//Valor sensor de refletancia esquerdo
int valorSensor2 = 0;//Valor sensor de refletancia direito
// Motores
#define M_Direito 5//Motor Direito(A) Vel 0 - 255
#define M_Esquerdo 6//Motor Esquerdo(B) Vel 0 - 255
#define dirD 7//Direcao do motor A - HIGH (Frente) ou LOW (Tras)
#define dirE 8//Direcao do motor B - HIGH (Frente) ou LOW (Tras)
// PID
float avgSensor = 180;// Media da leitura dos sensores

float Kp = 0.5;//Inicializa Kp (Ganho proporcional)
float Ki = 0.00015;//Inicializa Ki (Ganho Integral)
float Kd = 5;//Inicializa Kd (Ganho Derivativo)

float erro = 0;//Inicializa o erro
float erroAnterior = 0;//Inicializa erroAnterior
float totalErro = 0;//Inicializa totalErro

float power = 0;//Inicializa power

int V_Direito, V_Esquerdo;//Declara V_Direito e V_Esquerdo

void setup() { //Inicia o setup
  Serial.begin(9600);//Inicializa o serial para verificar valores

  pinMode(M_Esquerdo, OUTPUT);//Configura o pino 6 como OUTPUT
  pinMode(M_Direito, OUTPUT);//Configura o pino 5 como OUTPUT
  pinMode(dirD, OUTPUT);//Configura o pino 6 como OUTPUT
  pinMode(dirE, OUTPUT);//Configura o pino 6 como OUTPUT

  Para();//Para o robo

  PFrente();//Move o robo para frente com velocidade 100
}//Fim do setup

void Para()//Declara a funcao Para
{
  analogWrite(M_Esquerdo, 0);//Velocidade do motor esquerdo de 0
  analogWrite(M_Direito, 0);//Velocidade do motor direito de 0
}//Fim da declaracao

void PFrente() //Declara a funcao PFrente (Para Frente)
{
  digitalWrite(dirE, HIGH);//Sentido de rotacao
  analogWrite(M_Direito, 100);//Velocidade de 100
  digitalWrite(dirD, LOW);//Sentido de rotacao
  analogWrite(M_Esquerdo, 100);//Velocidade de 100
}//Fim da declaracao

void loop() {//Inicia o loop

  segueLinha();//Chama a funcao segueLinha
  //Linhas 57 a 67 para verificar os valores obtidos
  Serial.print("valorSensor1 = ");
  Serial.print(valorSensor1);
  Serial.print("\t");
  Serial.print("valorSensor2 = ");
  Serial.print(valorSensor2);
  Serial.print("\t");
  Serial.print("erro = ");
  Serial.print(erro);
  Serial.print("\t");
  Serial.print("power = ");
  Serial.println(power);

  delay(1500);
}//Fim do loop

void segueLinha(void) {//Declara a funcao segueLinha
   PID_program();//Chama a funcao PID_program

   analogWrite(M_Esquerdo, V_Esquerdo);
   //Muda a velocidade de M_Esquerdo para o resultado de PID_program
   analogWrite(M_Direito, V_Direito);
   //Muda a velocidade de M_Direito para o resultado de PID_program
}//Fim da declaracao

void PID_program()//Declara afuncao PID_program
{
    lerSensores();//Chama a funcao lerSensores

    erroAnterior = erro;//Salva o valor do erro anterior para o diferencial
    erro = avgSensor - 180;//Calcula quanto o robo se desviou do centro
    totalErro += erro;//Acumula o erro para integral

    power = (Kp*erro) + (Kd*(erro-erroAnterior)) + (Ki*totalErro);
    //Aplica o PID

    //if( power > 0 ) { power = 100.0; }
    //if( power < 0 ) { power = -100.0; }

    if(power < 0) // Vira a esquerda
    {
      V_Direito = 100;//Mantem a velocidade de 100
      V_Esquerdo = 100 - abs(int(power));//Calcula a nova velocidade
    }

    else // Vira a direita
    {
      V_Direito = 100 - int(power);//Calcula a nova velocidade
      V_Esquerdo = 100;//Mantem a velocidade de 100
    }
}//Fim da declaracao

void lerSensores() {//Declara a funcao lerSensores
  valorSensor1 = analogRead(pinoSensor1);//Salva o valor do sensor direito
  valorSensor2 = analogRead(pinoSensor2);//Salva o valor do sensor esquerdo

  avgSensor = (valorSensor1 + valorSensor2) / 2;// Media da leitura dos sensores
}//Fim da declaracao

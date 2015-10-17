// Sensor de refletancia
int pinoSensor1 = A1;//Sensor de refletancia esquerdo
int pinoSensor2 = A2;//Sensor de refletancia direito
int valorSensor1 = 0;//Valor sensor de refletancia esquerdo
int valorSensor2 = 0;//Valor sensor de refletancia direito
// Motor
const int Left_Speed = 6; //velocidade motor A - de 0 a 255
const int Right_Speed = 5; //velocidade motor B - de 0 a 255
const int dirA = 7; //direcao do motor A - HIGH ou LOW
const int dirB = 8; //direcao do motor B - HIGH ou LOW

float avgSensor = 180; // Average sensor reading

float Kp = 0.5;   // Max deviation = 8-4.5 = 3.5 ||  255/3.5 = 72
float Ki = 0.00015;
float Kd = 5;

float error = 0;
float previousError = 0;
float totalError = 0;

float power = 0;

int PWM_Right, PWM_Left;



void setup() {
  Serial.begin(9600);
  
  pinMode(Left_Speed, OUTPUT);
  pinMode(Right_Speed, OUTPUT);
  pinMode(dirA, OUTPUT);
  pinMode(dirB, OUTPUT);
  
  Stop();
  
  Forward();
}

void Stop() 
{
  analogWrite(Left_Speed,0);
  analogWrite(Right_Speed, 0);
}

void Forward()
{
  digitalWrite(dirB, HIGH); //SENTIDO DE ROTACAO
  analogWrite(Right_Speed, 100); //VELOCIDADE
  digitalWrite(dirA, LOW); //SENTIDO DE ROTACAO
  analogWrite(Left_Speed, 100); //VELOCIDADE
}

void loop() {

  lineFollow();

  Serial.print("valorSensor1 = ");
  Serial.print(valorSensor1);
  Serial.print("\t");
  Serial.print("valorSensor2 = ");
  Serial.print(valorSensor2);
  Serial.print("\t");
  Serial.print("error = ");
  Serial.print(error);
  Serial.print("\t");
  Serial.print("power = ");
  Serial.println(power);

  delay(300);
}

void lineFollow(void) {
   PID_program();
   
   analogWrite(Left_Speed, PWM_Left);
   analogWrite(Right_Speed, PWM_Right); 
}


void PID_program()
{ 
    lerSensores();
    
    previousError = error; // save previous error for differential 
    error = avgSensor - 180; // Count how much robot deviate from center
    totalError += error; // Accumulate error for integral
    
    power = (Kp*error) + (Kd*(error-previousError)) + (Ki*totalError);
    
    //if( power > 0 ) { power = 100.0; }
    //if( power < 0 ) { power = -100.0; }
    
    if(power < 0) // Turn left
    {
      PWM_Right = 100;
      PWM_Left = 100 - abs(int(power));
    }
    
    else // Turn right
    {
      PWM_Right = 100 - int(power);
      PWM_Left = 100;
    }  
}


void lerSensores() {
  valorSensor1 = analogRead(pinoSensor1);
  valorSensor2 = analogRead(pinoSensor2);

  avgSensor = (valorSensor1 + valorSensor2) / 2;
}

#include <PID_v1.h> //import pid library
#include <AFMotor.h> //import AFMotor library

#define PIN_INPUT A5 //definim el pin A5 per a llegir la velocitat
AF_DCMotor motor4(4); //motor 4
AF_DCMotor motor3(3); //motor 3
AF_DCMotor motor2(2); //motor 2
AF_DCMotor motor1(1); //motor 1

double Setpoint, Input, Output; //variables per al PID

double Kp=.5, Ki=.05, Kd=.05; // valors dels parametres proporcional, integral i derivatiu
PID myPID(&Input, &Output, &Setpoint, Kp, Ki, Kd, REVERSE); //inicialitzem el pid

int encoder_pin = 3; //per a llegir el rotary encoder
int rpm; // per calcular velocitat de gir
volatile byte pulses; //pulsos fets sobre el encoder
unsigned long tempsVell; //temps vell 
unsigned int holes = 20; // el encoder te 20 forats


//funcio per a comptar els forats del encoder llegits
void counter()
{
   pulses++; // incrementem els pulsos/forats llegits
}



void setup()
{
  motor4.setSpeed(200); //posem el speed a 200 per a tots els motors
  motor4.run(RELEASE);
  motor3.setSpeed(200);
  motor3.run(RELEASE);
  motor2.setSpeed(200);
  motor2.run(RELEASE);
  motor1.setSpeed(200);
  motor1.run(RELEASE);
  
  Input = analogRead(PIN_INPUT); //llegim la velocitat inicial
  Setpoint = 10; //posem el setpoint inicialment a 10
  Serial.begin(9600);
  myPID.SetMode(AUTOMATIC);
  Serial.print("Input");
  Serial.print("\t");
  Serial.print("Output");


   pinMode(encoder_pin, INPUT);
   attachInterrupt(0, counter, FALLING); //interrupcions ens activaran la funcio counter per comptar forats del encoder

   //inicialitzem els pulsos, els rpm i el temps a 0
   pulses = 0; 
   rpm = 0;
   tempsVell = 0;
}


void loop()
{

   if (millis() - tempsVell >= 1000) {
    detachInterrupt(0); //aturem interrupcions per a fer els calculs
    rpm = (60 * 1000 / holes ); //calcul rpm
    rpm= rpm / (millis() - tempsVell)* pulses;
    tempsVell  = millis();
    pulses = 0;
    attachInterrupt(0, counter, FALLING); //un cop fets els calculs tornem a iniciar la interrupcio
   }
      
  Input = rpm;
  Serial.println();
  Serial.print(Input);
  Serial.print("\t");
  myPID.Compute(); //un cop canvit el input, cridem la funcio per a recalcular el output
  //canviem el output per als 4 motors
  motor4.setSpeed(Output); 
  motor4.run(REVERSE);
  motor3.setSpeed(Output);
  motor3.run(REVERSE);
  motor2.setSpeed(Output);
  motor2.run(REVERSE);
  motor1.setSpeed(Output);
  motor1.run(REVERSE);
  
  Serial.print(Output);
}

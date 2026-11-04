// #include <Arduino.h>
#include "Mux.h"
using namespace admux;
Mux mux(Pin(26, INPUT, PinType::Analog), Pinset(13, 12, 14, 27));
//--------------------------------------------------------------
#include <Ultrasonic.h>
// Khởi tạo cảm biến siêu âm với chân Trigger là 33 và chân Echo là 25
Ultrasonic ultrasonic(33, 25);
float distance;
//--------------------------------------------------------
#include <TB6612_ESP32.h>

#define PWMA 15 
#define AIN2 2 
#define AIN1 0 // ESP32 Pin D13 to TB6612FNG Pin AIN1
#define STBY 4 // ESP32 Pin D33 to TB6612FNG Pin STBY

#define BIN1 16 // ESP32 Pin D12 to TB6612FNG Pin BIN1
#define BIN2 17// ESP32 Pin D27 to TB6612FNG Pin BIN2
#define PWMB 18 // ESP32 Pin D25 to TB6612FNG Pin PWMB

const int offsetA = 1;
const int offsetB = 1;

Motor motor1 = Motor(AIN1, AIN2, PWMA, offsetA, STBY,5000 ,8,1 );
Motor motor2 = Motor(BIN1, BIN2, PWMB, offsetB, STBY,5000 ,8,2 );
//--------------------------------------------------------
#define BOTON    35
#define GO       32

#define LED_ON  25
#define LED     33

long int sensores[16];  int sensor;
long int digital[16];
long int lectura_fondo[16];
long int lectura_linea[16];
long int umbral[16];
//int umbral=750;
long int sumap, suma, pos, poslast, position;

int linea=0;
int barra=1;

//////////////// pid/////////////////////
float KP = 0.18;      // Hệ số Proportional 
float KD = 4;         // Hệ số Derivative 
float KI = 0.002;     // Hệ số Integral 
//-----------------------------------------------
int vel = 85;        // Tốc độ tối đa của robot (0-255).

int veladelante = 175; // Tốc độ di chuyển phía trước (forward).230
int velatras = 115;   // Tốc độ di chuyển phía sau (backward).140

////////////////////////////////////////
/// datos para la integral
int error1=0;
int error2=0;
int error3=0;
int error4=0;
int error5=0;
int error6=0;
/////////////////////////

///////////variable PID///////////////
int proporcional=0;
int integral=0;
int derivativo=0;
int diferencial=0;
int last_prop;
int setpoint=650;
//-----------------------------------

int counter_R;


void setup()
{
  Serial.begin(115200);

  pinMode(BOTON,INPUT);
  pinMode(GO,INPUT);

  pinMode(LED,OUTPUT);
  pinMode(LED_ON,OUTPUT);

//---------------------------------------------
  digitalWrite(LED_ON,1);
  digitalWrite(LED,1);
//--------------------------------------------
while(digitalRead(BOTON));
for(int i=0;i<50;i++){
  fondos();
  digitalWrite(LED,0); delay(20);  digitalWrite(LED,1);  delay(20);
}
while(digitalRead(BOTON));
for(int i=0;i<50;i++){
  lineas();
  digitalWrite(LED,0); delay(20);  digitalWrite(LED,1);  delay(20);
}

promedio();
while(digitalRead(BOTON));
  digitalWrite(LED,0);
}
//============================================================


void loop()
{
 // float distance = ultrasonic.read();

//---------------------------------------------
while(true){
  int go=digitalRead(GO);

  lectura();
  
  frenos();
  PID();
  maze();

  if(go==0){  break;}
}


//-------------------STOP-------------------------
 while(true){
   int go=digitalRead(BOTON);
    motor1.brake();
    motor2.brake();
    if(go==0) break;
}
  
}
void PID(){
  proporcional=pos-setpoint;
  derivativo=proporcional-last_prop;
  integral=error1+error2+error3+error4+error5+error6;
  last_prop=proporcional;
  error6=error5;
  error5=error4;
  error4=error3;
  error3=error2;
  error2=error1;
  error1=proporcional;
  int diferencial=(proporcional*KP) + (derivativo*KD) + (integral*KI);

  if(diferencial > vel) diferencial=vel;
  else if(diferencial < -vel) diferencial=-vel;

  (diferencial < 0) ? motores(vel, vel+diferencial) : motores(vel-diferencial, vel);
}

  /* Proyecto 2 Segundo Módulo.ino*/

#include "ESP32_IDE.h"
#include "ESP32Servo.h"

// Definiciones
#define ENC5A E0
#define ENC5B E1
#define ENC6A E2
#define ENC6B E3

#define DIR5 S0
#define PWM5 S1
#define DIR6 S4
#define PWM6 S5

#define pi 3.1415928
volatile int contador5=0,contador6=0;
byte ant5=0, act5=0,ant6=0, act6=0;
int n5=0, n6=0;

float velA[2];
float velD[2];
float suma[2];
int contador_suma[2];
float previous_error[2]={0,0};
float integral[2];
float conversion[]={724.29,724.29};

bool inicio = false;
unsigned long t_inicio5=0, t_vuelta5,t_inicio6=0, t_vuelta6;

int velocidad=0;

int output;
double Kp=2.7, Ki=0.02246893314, Kd=0.02214;
double actual_delt, SetPoint;
double error, derivative, error_abs;
unsigned long  dt=10;

//Configuración
void setup(){
  //Entradas
  pinMode(ENC5A,INPUT);
  pinMode(ENC5B,INPUT);
  pinMode(ENC6A,INPUT);
  pinMode(ENC6B,INPUT);
  
  //Salidas
  Serial.begin(9600);

  pinMode(DIR5,OUTPUT);
  pinMode(DIR6,OUTPUT);

  attachInterrupt(digitalPinToInterrupt(ENC5A),encoder5,CHANGE);
  attachInterrupt(digitalPinToInterrupt(ENC5B),encoder5,CHANGE);
  attachInterrupt(digitalPinToInterrupt(ENC6A),encoder6,CHANGE);
  attachInterrupt(digitalPinToInterrupt(ENC6B),encoder6,CHANGE);

  velA[0]=0;
  velA[1]=0;

  velD[0]=0;
  velD[1]=0;
}

void loop()
{ 
  // put your main code here, to run repeatedly:
  if (Serial.available())
  {
    delay(50);
    String code = "";
    while (Serial.available())
    {
      char q;
      q = Serial.read();
      if (q != '\n')
      {
      code = code + q; 
      }
    }
    String pulso = "";
    pulso=pulso+code[1]+code[2]+code[3]+code[4];
    velocidad=pulso.toInt();
    switch (code[0])
    {
      case '5':
        velD[0] = velocidad;
        integral[0]=0;
        previous_error[0]=0;
        break;
      case '6':
        velD[1] = velocidad;
        integral[1]=0;
        previous_error[1]=0;
        break;
      default:
        Serial.println("Cámara, envía algo correcto");
        break;
    }
  }

  if((micros()-t_inicio5)>500000)
  {
    velA[0]=0;
  }
  
  if(velD[0]!=velA[0])
  {
    suma[0]=velA[0]+suma[0];
    contador_suma[0] = contador_suma[0] + 1;
    if(contador_suma[0]>=20)
    {
      Serial.print("Vel D Mot 5: ");
      Serial.print(velD[0]);
      Serial.print("   Vel A Mot 5: ");
      Serial.println(suma[0]/20);
      contador_suma[0]=0;
      suma[0]=0;
    }
    error = float(velD[0]) - float(velA[0]);
    derivative = (error - previous_error[0])/10;
    integral[0] = integral[0] + (error*10);
    output=(2*error) + (1*derivative)+(0.02*integral[0]);
    Serial.println(output);
    if(output<0)
    {
      digitalWrite(DIR5,LOW);
    }
    else
    {
      digitalWrite(DIR5,HIGH);
    }
    output = abs(output);
    output = map(output,0,724.29,0,254);
    output = constrain(output,0,254);
    analogWrite(PWM5,output);
    previous_error[0] = error;
    delay(10);
  }

  if((micros()-t_inicio6)>500000)
  {
    velA[1]=0;
  }

  if(velD[1]!=velA[1])
  {
    suma[1]=velA[1]+suma[1];
    contador_suma[1] = contador_suma[1] + 1;
    if(contador_suma[1]>=20)
    {
      Serial.print("Vel D Mot 6: ");
      Serial.print(velD[1]);
      Serial.print("   Vel A Mot 6: ");
      Serial.println(suma[1]/20);
      contador_suma[1]=0;
      suma[1]=0;
    }
    error = float(velD[1]) - float(velA[1]);
    derivative = (error - previous_error[1])/10;
    integral[1] = integral[1] + (error*10);
    output=(2*error) + (1*derivative) + (0.02*integral[1]);
    if(output<0)
    {
      digitalWrite(DIR6,LOW);
    }
    else
    {
      digitalWrite(DIR6,HIGH);
    }
    output = abs(output);
    output = map(output,0,724.29,0,254);
    output = constrain(output,0,254);
    analogWrite(PWM6,output);
    previous_error[1] = error;
    delay(10);
  }
}

void encoder5()
{
  ant5 = act5;
  if(digitalRead(ENC5A)==1) bitSet(act5,0); else bitClear(act5,0);
  if(digitalRead(ENC5B)==1) bitSet(act5,1); else bitClear(act5,1);
  
  if(ant5==3 && act5==1) contador5++;
  if(ant5==1 && act5==0) contador5++;
  if(ant5==0 && act5==2) contador5++;
  if(ant5==2 && act5==3) contador5++;
  
  if(ant5==1 && act5==3) contador5--;
  if(ant5==0 && act5==1) contador5--;
  if(ant5==2 && act5==0) contador5--;
  if(ant5==3 && act5==2) contador5--;
  
  if (abs(contador5)>64)
  {
    t_vuelta5=micros()-t_inicio5;
    t_inicio5=micros();
    if(contador5>0)
    {
      velA[0]=(1.0/t_vuelta5)*1000000*(2*pi);
    }
    else
    {
      velA[0]=-(1.0/t_vuelta5)*1000000*(2*pi);
    }
    contador5=0;
  }
}

void encoder6(){
  ant6 = act6;
  if(digitalRead(ENC6A)==1) bitSet(act6,0); else bitClear(act6,0);
  if(digitalRead(ENC6B)==1) bitSet(act6,1); else bitClear(act6,1);
  
  if(ant6==3 && act6==1) contador6++;
  if(ant6==1 && act6==0) contador6++;
  if(ant6==0 && act6==2) contador6++;
  if(ant6==2 && act6==3) contador6++;
  
  if(ant6==1 && act6==3) contador6--;
  if(ant6==0 && act6==1) contador6--;
  if(ant6==2 && act6==0) contador6--;
  if(ant6==3 && act6==2) contador6--;

  if (abs(contador6)>64)
  {
    t_vuelta6=micros()-t_inicio6;
    t_inicio6=micros();
    if(contador6>0)
    {
      velA[1]=(1.0/t_vuelta6)*1000000*(2*pi);
    }
    else
    {
      velA[1]=-(1.0/t_vuelta6)*1000000*(2*pi);
    }
    contador6=0;
  }
}

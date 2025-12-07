#include <QTRSensors.h>


//---------------------
#include "BluetoothSerial.h"

String device_name = "BRENS";

#if !defined(CONFIG_BT_ENABLED) || !defined(CONFIG_BLUEDROID_ENABLED)
#error Bluetooth is not enabled! Please run make menuconfig to and enable it
#endif

// Check Serial Port Profile
#if !defined(CONFIG_BT_SPP_ENABLED)
#error Serial Port Profile for Bluetooth is not available or not enabled. It is only available for the ESP32 chip.
#endif
String btCommand = "";   // variable globale ou statique

BluetoothSerial SerialBT;

//---------------------

#define ENA 5
#define IN1 18
#define IN2 19
#define IN3 21
#define IN4 22
#define ENB 23
#define BlueLed 17
#define Button 34
const int sensorCount = 8;
const uint8_t sensorPins[] = {13,12,14,27,26,25,33,32};
uint16_t sensorValues[sensorCount];
uint16_t Threshold[sensorCount];
int binaryValue[sensorCount];
int pos=0;
int casee=0;
bool binaryButton=false;
QTRSensors qtr;

// PID
float Kp = 0.28;
float Ki = 0;
float Kd = 0.6;

float error = 0;
float previousError = 0;
float derivativeFiltered = 0;  
int state=0;
int stateAllBlack=0;

// 🔥 FILTRE PASSE-BAS
float alpha = 0.7;   // entre 0 et 1

float output = 0;
int baseSpeed = 240;
int MAX_MOTOR_SPEED = 255;


void setup() {
  Serial.begin(9600);

  SerialBT.begin(device_name);  
  qtr.setTypeRC();
  qtr.setSensorPins(sensorPins, sensorCount);

  pinMode(ENA, OUTPUT);
  pinMode(IN1, OUTPUT);
  pinMode(IN2, OUTPUT);
  pinMode(IN3, OUTPUT);
  pinMode(IN4, OUTPUT);
  pinMode(ENB, OUTPUT);
  pinMode(BlueLed,OUTPUT);
  pinMode(Button,INPUT);
  //---- CALIBRATION ----
  Serial.println("Calibrating...");
  for (int i = 0; i < 200; i++) {
    qtr.calibrate();
    delay(10);
  }
  digitalWrite(BlueLed,HIGH);
  delay(500);
  digitalWrite(BlueLed,LOW);
  Serial.println("Calibration done.");

  // Calcul du seuil pour chaque capteur
  for (int i = 0; i < sensorCount; i++) {
    Threshold[i] =
      (qtr.calibrationOn.maximum[i] + qtr.calibrationOn.minimum[i]) / 2;
  }
  digitalWrite(BlueLed,HIGH);
  delay(3000);
}


void loop() {
  getButtonValue();
  if(binaryButton)
  {
      getBinaryValue();
      switch(state)
      {
        case 0: //ka7la lawla
        circulate();
        if(binaryValue[1]==0  && binaryValue[3]==0 && binaryValue[4]==0 && binaryValue[6]==0)
        {
          state=1; 
        }
        break;    
        case 1:
        circulate();
        SerialBT.println("circulate");
        if(binaryValue[0]==1 && binaryValue[1]==1 && binaryValue[2]==1 && binaryValue[3]==1 && binaryValue[4]==1 && binaryValue[5]==1 && binaryValue[6]==1 && binaryValue[7]==1)
        {
          state=2;
        }
        break;


        case 2: //ka7la Thenya
        forward(baseSpeed);
        SerialBT.println("ka7la thenya");
        if(binaryValue[0]==0 && binaryValue[1]==0  && binaryValue[3]==1 && binaryValue[4]==1 && binaryValue[6]==0 && binaryValue[7]==0)
        {
          state=3;
        }
        break;
        case 3:
        circulate();
        SerialBT.println("circulate");
        if(binaryValue[0]==1 && binaryValue[1]==1 && binaryValue[2]==1 && binaryValue[3]==1 && binaryValue[4]==1 && binaryValue[5]==1 && binaryValue[6]==1 && binaryValue[7]==1)
        {
          state=4;
        }
        break;


        case 4: //ka7la theltha
        forward(baseSpeed);
        SerialBT.println("ka7la theltha");
        if(binaryValue[0]==0 && binaryValue[1]==0  && binaryValue[3]==1 && binaryValue[4]==1  && binaryValue[6]==0 && binaryValue[7]==0)
        {
          state=5;
        }
        break;


        case 5:
        circulate();
        SerialBT.println("circulate");
        if(binaryValue[0]==1 && binaryValue[1]==1 && binaryValue[2]==1 && binaryValue[3]==1 && binaryValue[4]==1 && binaryValue[5]==1 && binaryValue[6]==1 && binaryValue[7]==1)
        {
          state=6;
        }
        break;

        case 6: //ka7la rab3a
        SerialBT.println("ka7la rab3a");
        state=111;
        break;

        case 111:
        SerialBT.println("stop");
        stop();
        break;

        default :
        SerialBT.println();
      }
  }
}

void forward(int speed)
{
  run_Motor_Left(speed);
  run_Motor_Right(speed);
}
void CalculateError()
{
  pos=qtr.readLineBlack(sensorValues);
  error=pos-3500;

}

// ---- PID AVEC FILTRE PASSE-BAS α ----
void PIDCalculator() {

  float P = Kp * error;

  static float integral = 0;
  integral += error;
  float I = Ki * integral;

  // Dérivée brute
  float derivative = error - previousError;

  // 🔥 DERIVÉE FILTRÉE (LOW-PASS FILTER)
  derivativeFiltered =
        alpha * derivative +
        (1.0 - alpha) * derivativeFiltered;

  float D = Kd * derivativeFiltered;

  output = P + I + D;

  previousError = error;
}



// ---- APPLICATION SUR LES MOTEURS ----
void changeMotorSpeed() {
  int leftSpeed  = baseSpeed + output;
  int rightSpeed = baseSpeed - output;

  // Saturation
  leftSpeed  = constrain(leftSpeed,  -255, MAX_MOTOR_SPEED);
  rightSpeed = constrain(rightSpeed, -255, MAX_MOTOR_SPEED);

  run_Motor_Left(leftSpeed);
  run_Motor_Right(rightSpeed);
}



// ---- MOTORS ----
void run_Motor_Left(int speed) {
  if(speed<0)
  {
    analogWrite(ENA, (speed)*(-1));
    digitalWrite(IN1, HIGH);
    digitalWrite(IN2, LOW);
  }
  else
  {
    analogWrite(ENA, speed);
    digitalWrite(IN1, LOW);
    digitalWrite(IN2, HIGH);
  }
  
}

void run_Motor_Right(int speed) {
  if(speed<0){
    analogWrite(ENB, (speed)*(-1));
    digitalWrite(IN3, LOW);
    digitalWrite(IN4, HIGH);
  }
  else 
  {
    analogWrite(ENB, speed);
    digitalWrite(IN3, HIGH);
    digitalWrite(IN4, LOW);
  }
}





void circulate() {
  CalculateError();
  PIDCalculator();
  changeMotorSpeed();
}


void forward()
{
  run_Motor_Left(baseSpeed);
  run_Motor_Right(baseSpeed);
}

void stop()
{
  analogWrite(ENA,0);
  analogWrite(ENB,0);
}
void getBinaryValue()
{
  qtr.readLineBlack(sensorValues);
  for(int i=0;i<sensorCount;i++)
  {
    if(sensorValues[i]>Threshold[i])
    binaryValue[i]=1;
    else 
    binaryValue[i]=0;
  }

} 

void getButtonValue()
{
  int button=analogRead(Button);
  if(button==4095)
    binaryButton=!binaryButton;
}

void sharpRight()
{

}

void sharpLeft()
{

}
void right()
{
  
}

void left()
{

}



// void integrateBluetoothPID() {

//   if (SerialBT.available()) {
//     String command = SerialBT.readStringUntil('\n');
//     // Découpe la commande (exemple : "kp 0.26")
//     int spaceIndex = command.indexOf(' ');
    
//     if(spaceIndex > 0){
//       String paramName = command.substring(0, spaceIndex);
//       String paramValueString = command.substring(spaceIndex + 1);

//       // Conversion string -> float
//       float valueFloat = paramValueString.toFloat();

//       // Conversion float -> entier
//       int valueInt = (int)valueFloat;

//       // Exemple : modifier un PID
//       if(paramName == "kp") 
//         Kp= valueInt;
//       if(paramName =="kd")
//         Kd=valueInt;
//       if(paramName=="ki")
//         Ki=valueInt;
//     }
//   }
// }


void printBinaryValue()
{
  String ch = "";  // Une seule chaîne
  for(int i=0;i<sensorCount;i++)
  {
    ch += String(binaryValue[i]) + " ";  
  }
  SerialBT.println(ch);
}
#include <Arduino.h>
#include "BluetoothSerial.h"
#include <Servo.h>
BluetoothSerial SerialBT;
byte BTData;

Servo myservo;  // create servo object to control a servo
int servoPin = 12;

// GPIOs DC motor
int DCmotorInput1 = 16;               // IN1
int DCmotorInput2 = 17;               // IN2
int DCmotorInput3 = 18;               // IN3
int DCmotorInput4 = 19;               // IN4
int EnableDCmotor = 5;                // ENA
int EnableDCmotor2 = 4;               // ENB

// Configuring the PWM values
const int PWM_Frequency = 5000;
const int PWM_Channel1 = 0;           // PWM Channel for motor 1
const int PWM_Channel2 = 1;  
const int PWM_Channel3 = 2;         // PWM Channel for motor 2
const int PWM_Resolution = 8;
int dutyCycle = 150;
Servo servo1;
void setup() {
  Serial.begin(115200);
  delay(2000);
 
  

  // Configuring the output GPIOs
  pinMode(DCmotorInput1, OUTPUT);
  pinMode(DCmotorInput2, OUTPUT);
  pinMode(DCmotorInput3, OUTPUT);
  pinMode(DCmotorInput4, OUTPUT);
  pinMode(EnableDCmotor, OUTPUT);
  pinMode(EnableDCmotor2, OUTPUT);
  pinMode(servoPin, OUTPUT);
  
  
  
  // Configuring the PWM output for motor 1
  ledcSetup(PWM_Channel1, PWM_Frequency, PWM_Resolution);
  ledcAttachPin(EnableDCmotor, PWM_Channel1);

  // Configuring the PWM output for motor 2
  ledcSetup(PWM_Channel2, PWM_Frequency, PWM_Resolution);
  ledcAttachPin(EnableDCmotor2, PWM_Channel2);

  ledcSetup(PWM_Channel3, PWM_Frequency, PWM_Resolution);
  ledcAttachPin(servoPin, PWM_Channel3);

  // Starts the Bluetooth device
  SerialBT.begin();
  Serial.println("Bluetooth Bağlantısı Başlatıldı! Bağlanmayı Dene...");



}

void loop() {
  if (SerialBT.available()) {
    BTData = SerialBT.read();
  }

  if (BTData == 'L') {
    
    servo1.write(servoPin,45);
    Serial.println(0);
delay(10);
  }

  if (BTData == 'R') {
    
   servo1.write(servoPin,135);
    Serial.println(15);
delay(10);
  }

  if (BTData == 'S') {
    // Stops both DC motors
    Serial.println("Motor Durdu");
    digitalWrite(DCmotorInput1, LOW);
    digitalWrite(DCmotorInput2, LOW);
    digitalWrite(DCmotorInput3, LOW);
    digitalWrite(DCmotorInput4, LOW);
    delay(10);
  }

  if (BTData == 'F') {
    // Moves both DC motors forward with speed according to PWM duty cycle
    Serial.println("İleri Gidiliyor");
    digitalWrite(DCmotorInput1, LOW);
    digitalWrite(DCmotorInput2, HIGH); 
    digitalWrite(DCmotorInput3, HIGH);
    digitalWrite(DCmotorInput4, LOW);
    ledcWrite(PWM_Channel1, dutyCycle);
    ledcWrite(PWM_Channel2, dutyCycle);
    delay(10);
  }

  if (BTData == 'B') {
    // Moves both DC motors backwards with speed according to PWM duty cycle
    Serial.println("Geri Gidiliyor");
    digitalWrite(DCmotorInput1, HIGH);
    digitalWrite(DCmotorInput2, LOW); 
    digitalWrite(DCmotorInput3, LOW);
    digitalWrite(DCmotorInput4, HIGH);
    ledcWrite(PWM_Channel1, dutyCycle);
    ledcWrite(PWM_Channel2, dutyCycle);
    delay(10);
  }

  if (BTData == 'I') {
    // Increments the PWM duty cycle by 10
    dutyCycle = dutyCycle + 10;

    // Limits the PWM duty cycle value to a maximum of 250
    if (dutyCycle >= 250) {
      dutyCycle = 250;
    }

    Serial.print("I ");
    Serial.println(dutyCycle);
    ledcWrite(PWM_Channel1, dutyCycle);
    ledcWrite(PWM_Channel2, dutyCycle);
  }

  if (BTData == 'D') {
    // Decrements the PWM duty cycle by 10
    dutyCycle = dutyCycle - 10;

    // Limits the PWM duty cycle value to a minimum of 170
    if (dutyCycle <= 0) {
      dutyCycle = 0;
    }
    
    Serial.print("Hız Seviyesi: ");
    Serial.println(dutyCycle);
    ledcWrite(PWM_Channel1, dutyCycle);
    ledcWrite(PWM_Channel2, dutyCycle);
  }
}

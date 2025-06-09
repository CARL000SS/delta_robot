#include <Arduino.h>
#include <math.h>
#include "PID.h"
#include "Encoder.h"

#define START_BOTON 32

bool startFlag = false;

unsigned long previousMillis = 0;
unsigned long previousMillis1s = 0;

float output = 0.0;

int cont = 0;
int cont2 = 0;

const int motorCount = 1;

HardwareSerial SerialESP(2);

Motor motor[motorCount] = {
    // Kp, Ki, Kd,       RPWM, LPWM,     CHA1, CHA2,         ENCODER_A, ENCODER_B, POT_PIN,           START_POT,    REDUCTION
    // {4.0, 0.0, 0.0,       15, 2,          1, 2,              22, 21, 33,                              -14.0,     50},  // motor jetson
    // {4.0, 0.0, 0.0,       5, 4,          3, 4,               12, 13, 25,                              -5.0,     40}   // motor fuente 
    {6.0, 0.05, 0.6,      2, 15,         5, 6,               12, 13, 34,                              -14.0,     10}  // motor tontos
};

void gainsBySerial() {
  if (Serial.available() > 0) {
    String recibido = Serial.readStringUntil('\n');
    recibido.trim();  // Elimina espacios y retornos de carro
    
    // Dividir la cadena en los nueve gains flotantes
    float gains[9];
    int index = 0;
    int lastComma = -1;
    
    for (int i = 0; i < recibido.length(); i++) {
      if (recibido.charAt(i) == ',' || i == recibido.length() - 1) {
        String valorStr = recibido.substring(lastComma + 1, i + (i == recibido.length() - 1 ? 1 : 0));
        gains[index] = valorStr.toFloat();  // Cambiado a toFloat()
        lastComma = i;
        index++;
        if (index >= 9) break;
      }
    }

    for (int i = 0; i < motorCount; i++) {
      motor[i].pid.setGains(gains[motorCount*i], gains[motorCount*i+1], gains[motorCount*i+2]); //Kp, ki, kd
    }
  }
}

void setpointsBySerial() {
  if (motorCount == 2) {
    if (SerialESP.available() >= motorCount) {
      motor[0].pid.setpoint = SerialESP.read() - 85;
      motor[1].pid.setpoint = SerialESP.read() - 85;
    }
  } else {
    if (Serial.available() >= motorCount) {
      motor[0].pid.setpoint = Serial.read() - 70;
    }
  }
}

void setup() {
  Serial.begin(115200);
  SerialESP.begin(115200, SERIAL_8N1, 16, 17); // RX=16, TX=17

  for (int i = 0; i < motorCount; i++) {
    motor[i].setup(i);
  }

  setupInterruptPin();

  pinMode(START_BOTON, INPUT_PULLDOWN);
}

void loop() {
  unsigned long currentMillis = millis();

  if (digitalRead(START_BOTON) == HIGH) {
    startFlag = !startFlag;
    Serial.println(startFlag ? "START" : "STOP");
    delay(250);
  }

  // gainsBySerial();
  setpointsBySerial();


  if (!startFlag) {

    for (int i = 0; i < motorCount; i++) {
      motor[i].stop();
    }

    if (cont >= 1000) {
      cont = 0;
      
      Serial.print("Pot0: "); Serial.print(analogRead(motor[0].POT_PIN) * (270.0 / 4095.0)-135.0);
      Serial.print("\t\tPot1: "); Serial.print(analogRead(motor[1].POT_PIN) * (270.0 / 4095.0)-135.0);
      Serial.println();
      delay(10);
    }
    cont += 1;

  } else if (currentMillis - previousMillis >= 10) { // 10 ms
    previousMillis = currentMillis;

    for (int i = 0; i < motorCount; i++) {
      
      if (motor[i].inPosition) {

        if (motor[i].firstStart) {
          motor[i].pid.position = 0;
          motor[i].posicionGrados = 0;
          motor[i].pid.setpoint = 0;
          motor[i].resetEncoder();
          motor[i].firstStart = false;
        }

        motor[i].checkMotorPosition();
        // output = motor[i].pid.control();
        output = constrain(motor[i].pid.control(), -20, 20);

      } else {

        motor[i].checkPot();

        if (motor[i].posicionGrados < motor[i].START_POT + 2 && motor[i].posicionGrados > motor[i].START_POT - 2) {
          motor[i].stop();
          motor[i].inPosition = true;
          Serial.println();
          Serial.println(" # # # # # Initial Position Set for motor " + String(i) + " # # # # #");
          Serial.println();
          delay(50);
        } else {
          output = constrain(motor[i].pid.control(), -80, 80); // Constrain output to avoid saturation
        }
      }

      motor[i].writePWM(output);

    }

    if (cont2 >= 100) {
      cont2 = 0;
      Serial.println();
      Serial.println("- - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -");
      for (int i = 0; i < motorCount; i++) {
        Serial.print("PWM: "); Serial.print(motor[i].writeOutput);
        Serial.print("\t\tPosition: "); Serial.print(motor[i].posicionGrados);
        Serial.print("\t\tSetpoint: "); Serial.print(motor[i].pid.setpoint);
        Serial.println();
      }
    }
    cont2 += 1;



  } else {}
}
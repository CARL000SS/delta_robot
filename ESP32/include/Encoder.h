#ifndef ENCODER_READER_H
#define ENCODER_READER_H

#include <Arduino.h>

const int frecuencia = 2000;
const int resolucion = 8;
const int MAX_DUTY_CYCLE = (int)(pow(2, resolucion) - 1);
#define PULSOS_POR_REV 2000

struct Motor {  
  PID pid;

  int RPWM;
  int LPWM;
  int CHA1;
  int CHA2;
  
  int ENCODER_A;
  int ENCODER_B;

  int POT_PIN;
  float START_POT;

  volatile long encoderCount = 0;
  static long lastCount;
  float posicionGrados = 0;

  int writeOutput = 0;

  bool inPosition = false;
  bool firstStart = true;

  int reduction;


  Motor(float kp_, float ki_, float kd_, int RPWM_, int LPWM_, int CHA1_, int CHA2_, int ENCODER_A_, int ENCODER_B_, int POT_PIN_, float _START_POT_, int _reduction_)
        : pid(kp_, ki_, kd_), RPWM(RPWM_), LPWM(LPWM_), CHA1(CHA1_), CHA2(CHA2_), ENCODER_A(ENCODER_A_), ENCODER_B(ENCODER_B_), POT_PIN(POT_PIN_), START_POT(_START_POT_), reduction(_reduction_) {}

  void setup(int i) {
    ledcSetup(CHA1, frecuencia, resolucion);
    ledcAttachPin(LPWM, CHA1);

    ledcSetup(CHA2, frecuencia, resolucion);
    ledcAttachPin(RPWM, CHA2);
    
    pinMode(ENCODER_A, INPUT_PULLUP);
    pinMode(ENCODER_B, INPUT_PULLUP);

    Serial.print("[");
    Serial.print(i);
    Serial.println("] Debug info:");
  
    Serial.print("  Pins -> RPWM: "); Serial.print(RPWM);
    Serial.print(", LPWM: "); Serial.print(LPWM);
    Serial.print(", CHA1: "); Serial.print(CHA1);
    Serial.print(", CHA2: "); Serial.println(CHA2);
  
    Serial.print("  Encoder A: "); Serial.print(ENCODER_A);
    Serial.print(", Encoder B: "); Serial.println(ENCODER_B);

    pid.setpoint = START_POT;
  }

  void writePWM(int value) {
    writeOutput = saturate(value);
    
    if (writeOutput > 0) {
      ledcWrite(CHA1, writeOutput);
      ledcWrite(CHA2, 0);
    } else if (writeOutput < 0) {
      ledcWrite(CHA1, 0);
      ledcWrite(CHA2, -writeOutput);
    } else {
      stop();
    }
  }

  int saturate(int u) {
    if (u > MAX_DUTY_CYCLE) {
        u = MAX_DUTY_CYCLE;
    } else if (u < -MAX_DUTY_CYCLE) {
        u = -MAX_DUTY_CYCLE;
    }
    return u;
  }

  void stop() {
    ledcWrite(CHA1, 0);
    ledcWrite(CHA2, 0);
  }

  void resetEncoder() {
    encoderCount = 0;
    lastCount = encoderCount;
  }

  void checkMotorPosition() {
    if (encoderCount != lastCount) {
      posicionGrados = (((float)encoderCount * 360.0) / PULSOS_POR_REV) / reduction;
      pid.position = posicionGrados;
      lastCount = encoderCount;
    }
  }

  void checkPot() {
    posicionGrados = analogRead(POT_PIN) * (270.0 / 4095.0)-135.0;
    pid.position = posicionGrados;
  }
};

long Motor::lastCount = 0;

extern Motor motor[1];



const int8_t transitionTable[16] = {
  0, -1, +1, 0,
  +1, 0, 0, -1,
  -1, 0, 0, +1,
  0, +1, -1, 0
};

volatile uint8_t prevEncoderState0 = 0;
void IRAM_ATTR readEncoder0() {
  bool A = GPIO.in & (1 << motor[0].ENCODER_A);
  bool B = GPIO.in & (1 << motor[0].ENCODER_B);
  uint8_t currentState0 = (A << 1) | B; // new state (2 bits)
  uint8_t transition0 = (prevEncoderState0 << 2) | currentState0; // 4 bits
  motor[0].encoderCount += transitionTable[transition0];
  prevEncoderState0 = currentState0;
}

volatile uint8_t prevEncoderState1 = 0;
void IRAM_ATTR readEncoder1() {
  bool A = GPIO.in & (1 << motor[1].ENCODER_A);
  bool B = GPIO.in & (1 << motor[1].ENCODER_B);
  uint8_t currentState1 = (A << 1) | B; // new state (2 bits)
  uint8_t transition1 = (prevEncoderState1 << 2) | currentState1; // 4 bits
  motor[1].encoderCount += transitionTable[transition1];
  prevEncoderState1 = currentState1;
}




extern const int motorCount;
void setupInterruptPin() {

  if (motorCount == 2) {

    attachInterrupt(digitalPinToInterrupt(motor[0].ENCODER_A), readEncoder0, CHANGE);
    attachInterrupt(digitalPinToInterrupt(motor[0].ENCODER_B), readEncoder0, CHANGE);

    attachInterrupt(digitalPinToInterrupt(motor[1].ENCODER_A), readEncoder1, CHANGE);
    attachInterrupt(digitalPinToInterrupt(motor[1].ENCODER_B), readEncoder1, CHANGE);

  } else {

    attachInterrupt(digitalPinToInterrupt(motor[0].ENCODER_A), readEncoder0, CHANGE);
    attachInterrupt(digitalPinToInterrupt(motor[0].ENCODER_B), readEncoder0, CHANGE);

  }
  

}

#endif

#ifndef PID_H
#define PID_H

class PID {
  public:
    PID(float kp, float ki, float kd): _kp(kp), _ki(ki), _kd(kd),
    _previousError(0), _integral(0), _dt(0.01), setpoint(90), position(0) {}

    void debugger(int num) {
      Serial.print("PID instance created: "); Serial.println(num);
      Serial.print("Kp: "); Serial.println(_kp);
      Serial.print("Ki: "); Serial.println(_ki);
      Serial.print("Kd: "); Serial.println(_kd);
      Serial.print("dt: "); Serial.println(_dt);
      Serial.println();
    }

    void setGains(float kp,float ki,float kd) {
      _kp = constrain(kp, 0.0, 20.0);
      _ki = constrain(ki, 0.0, 1.0);
      _kd = constrain(kd, 0.0, 2.0);

      Serial.print("GAINS - Kp = ");
      Serial.print(_kp);
      Serial.print(", Ki = ");
      Serial.print(_ki);
      Serial.print(", Kd = ");
      Serial.println(_kd);
      
      // Warnings:
      if (kp != _kp) {
        Serial.print("Kp constrained to: ");
        Serial.println(_kp);
      }
      if (ki != _ki) {
          Serial.print("Ki constrained to: ");
          Serial.println(_ki);
      }
      if (kd != _kd) {
          Serial.print("Kd constrained to: ");
          Serial.println(_kd);
      }
      Serial.println();
    }

    float control() {
      float error = setpoint - position;
      _integral += error * _dt;
      float derivative = (error - _previousError) / _dt;

      float output = _kp * error + _ki * _integral + _kd * derivative;

      _previousError = error;

      return output;
    }

    float setpoint, position;
    float _kp, _ki, _kd;

  private:
    float _previousError;
    float _integral;
    float _dt;
};

#endif

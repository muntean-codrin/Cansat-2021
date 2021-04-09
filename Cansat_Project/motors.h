int motorPWM = 100;

struct CansatMotor
{
  int pin1, pin2, pinE;
  volatile int encoderCounter, encoderMultiplier;
  volatile int targetTicks = 0;

  CansatMotor(int _pin1, int _pin2, int _pinE)
  {
    pin1 = _pin1;
    pin2 = _pin2;
    pinE = _pinE;
    encoderCounter = 0;
    encoderMultiplier = 1;

    pinMode(pin1, OUTPUT);
    pinMode(pin2, OUTPUT);
    pinMode(pinE, INPUT);
  }



  void Spin(bool forward)
  {
    analogWrite(pin1, 0);
    analogWrite(pin2, 0);
    encoderMultiplier = forward ? 1 : -1;

    if (forward)
    {
      analogWrite(pin1, motorPWM);
      digitalWrite(pin2, LOW);
    }
    else
    {
      digitalWrite(pin1, LOW);
      analogWrite(pin2, motorPWM);
    }
  }

  void Stop()
  {
    analogWrite(pin1, 0);
    analogWrite(pin2, 0);
  }

  int GetEncoder()
  {
    return encoderCounter;
  }

  void MoveTo(int ticks)
  {
    bool forward = false;
    if(ticks - encoderCounter > 0)
    {
      forward = true;
    }
    Spin(forward);
    targetTicks = ticks;
    
  }
};

CansatMotor motors[3] = {
  {5, 6, 25},
  {9, 10, 38},
  {12, 17, 3}
};


void encoder0Handler()
{
  if(motors[0].encoderCounter == motors[0].targetTicks)
  {
    motors[0].Stop();
  }
  motors[0].encoderCounter += motors[0].encoderMultiplier;
}

void encoder1Handler()
{
  if(motors[1].encoderCounter == motors[1].targetTicks)
  {
    motors[1].Stop();
  }
  motors[1].encoderCounter += motors[1].encoderMultiplier;
}

void encoder2Handler()
{
  if(motors[2].encoderCounter == motors[2].targetTicks)
  {
    motors[2].Stop();
  }
  motors[2].encoderCounter += motors[2].encoderMultiplier;
}

void SetupMotors()
{
  attachInterrupt(digitalPinToInterrupt(motors[0].pinE), encoder0Handler, CHANGE);
  attachInterrupt(digitalPinToInterrupt(motors[1].pinE), encoder1Handler, CHANGE);
  attachInterrupt(digitalPinToInterrupt(motors[2].pinE), encoder2Handler, CHANGE);
}

#ifndef SIMPLE_STEPPER_H
#define SIMPLE_STEPPER_H

#include <Arduino.h>

class SimpleStepper{
public:
  SimpleStepper(int stepPin, int dirPin, float stepsPerDegree);

  void begin();
  void setHome();
  void update();
  void startMove(float moveDegrees, float minSpeed, float maxSpeed, float acceleration);
  float getAngle();
  float getWrappedAngle(); //Current angle wrapped to 0 - 360
  float shortestMoveTo(float targetAngle);
  long degreesToSteps(float degrees);
  bool isDone();
  float getVelocityDegPerSec(); // Current calculated velocity
  float getAccelerationDegPerSec2(); // Current calculated acceleration

private:
  int _stepPin;
  int _dirPin;
  float _stepsPerDegree;

  long _currentPosition = 0;
  long _startPosition = 0;
  long _targetPosition = 0;

  float _minSpeed = 0;
  float _maxSpeed = 0;
  float _acceleration = 0;

  float _currentVelocitySteps = 0;
  float _currentAccelerationSteps = 0;

  unsigned long _lastStepTime = 0;
  int _direction = 1;
  bool _moving = false;

  float wrapAngle(float angle);// Keeps angle between 0 and 360
};

#endif
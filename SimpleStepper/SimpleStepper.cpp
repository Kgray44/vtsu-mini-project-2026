#include "SimpleStepper.h"
#include <math.h>

// =====================================================
// CONSTRUCTOR
// =====================================================
SimpleStepper::SimpleStepper(int stepPin, int dirPin, float stepsPerDegree){
  _stepPin = stepPin;
  _dirPin = dirPin;
  _stepsPerDegree = stepsPerDegree;
}

// =====================================================
// SETUP MOTOR
// =====================================================
void SimpleStepper::begin(){
  pinMode(_stepPin, OUTPUT);
  pinMode(_dirPin, OUTPUT);

  digitalWrite(_stepPin, LOW);
}

// =====================================================
// SET CURRENT POSITION AS HOME
// =====================================================
void SimpleStepper::setHome(){
  _currentPosition = 0;
}

// =====================================================
// CONVERT DEGREES TO STEPS
// =====================================================
long SimpleStepper::degreesToSteps(float degrees){
  return round(degrees * _stepsPerDegree);
}

// =====================================================
// START A MOVEMENT
// =====================================================
void SimpleStepper::startMove(float moveDegrees, float minSpeed, float maxSpeed, float acceleration){
  long moveSteps = degreesToSteps(moveDegrees);

  // Nothing to move
  if (moveSteps == 0){
    _moving = false;
    _currentVelocitySteps = 0;
    _currentAccelerationSteps = 0;
    return;
  }

  // Remember starting position
  _startPosition = _currentPosition;

  // Calculate final position
  _targetPosition = _currentPosition + moveSteps;

  // Save movement settings
  _minSpeed = minSpeed;
  _maxSpeed = maxSpeed;
  _acceleration = acceleration;

  // Determine direction
  if (moveSteps > 0){
    _direction = 1;
    digitalWrite(_dirPin,HIGH);
  }

  else{
    _direction = -1;
    digitalWrite(_dirPin,LOW);
  }

  _lastStepTime = micros();
  _moving = true;
}

// =====================================================
// MOTOR CONTROLLER
//
// CALL THIS CONSTANTLY
// =====================================================
void SimpleStepper::update(){
  // Motor isn't moving
  if (!_moving){
    _currentVelocitySteps = 0;
    _currentAccelerationSteps = 0;
    return;
  }

  // How far have we gone?
  long stepsTraveled = abs(_currentPosition - _startPosition);

  // How far is left?
  long stepsRemaining = abs(_targetPosition - _currentPosition);

  // Are we finished?
  if (stepsRemaining == 0){
    _moving = false;
    _currentVelocitySteps = 0;
    _currentAccelerationSteps = 0;
    return;
  }

  // ===================================================
  // ACCELERATION
  //
  // v = sqrt(v0^2 + 2as)
  // ===================================================
  float accelSpeed = sqrt((_minSpeed * _minSpeed) + (2.0 * _acceleration * stepsTraveled));

  // ===================================================
  // DECELERATION
  //
  // Same equation,
  // using distance remaining
  // ===================================================
  float decelSpeed = sqrt((_minSpeed * _minSpeed) + (2.0 * _acceleration * stepsRemaining));

  // ===================================================
  // CHOOSE ALLOWED SPEED
  // ===================================================
  float speed = min(accelSpeed, decelSpeed);
  speed = min(speed, _maxSpeed);

  // ===================================================
  // DETERMINE CURRENT ACCELERATION
  // ===================================================

  // Cruising at max speed
  if (_maxSpeed <= accelSpeed && _maxSpeed <= decelSpeed){
    _currentAccelerationSteps = 0;
  }

  // Accelerating
  else if (accelSpeed <= decelSpeed){
    _currentAccelerationSteps = _acceleration * _direction;
  }

  // Decelerating
  else {
    _currentAccelerationSteps = -_acceleration * _direction;
  }

  // Save signed velocity
  _currentVelocitySteps = speed * _direction;

  // ===================================================
  // SPEED -> TIME BETWEEN STEPS
  //
  // T = 1 / speed
  // ===================================================
  unsigned long stepPeriod = 1000000.0 / speed;

  // ===================================================
  // CHECK IF IT IS TIME FOR ANOTHER STEP
  // ===================================================
  unsigned long now = micros();

  if (now - _lastStepTime >= stepPeriod){
    _lastStepTime = now;

    // Produce ONE STEP pulse
    digitalWrite(_stepPin, HIGH);
    delayMicroseconds(5);
    digitalWrite(_stepPin, LOW);

    // Update our known position
    _currentPosition += _direction;

    // Check if finished
    if (_currentPosition == _targetPosition ){
      _moving = false;
      _currentVelocitySteps = 0;
      _currentAccelerationSteps = 0;
    }
  }
}

// =====================================================
// CURRENT POSITION
// =====================================================
float SimpleStepper::getAngle(){
  return _currentPosition/_stepsPerDegree;
}

// =====================================================
// WRAP ANGLE TO 0 - 360
// =====================================================
float SimpleStepper::wrapAngle(float angle){
  angle = fmod(angle, 360.0);
  if (angle < 0) angle += 360.0;
  return angle;
}

// =====================================================
// CURRENT POSITION FROM 0 - 360
// =====================================================
float SimpleStepper::getWrappedAngle(){
  return wrapAngle(getAngle());
}

// =====================================================
// SHORTEST PATH TO AN ABSOLUTE ANGLE
// =====================================================
float SimpleStepper::shortestMoveTo(float targetAngle){
  float currentAngle = getWrappedAngle();
  targetAngle = wrapAngle(targetAngle);

  float difference = targetAngle - currentAngle;

  // Too far clockwise?
  if (difference > 180.0){difference -= 360.0;}

  // Too far counterclockwise?
  if (difference < -180.0){difference += 360.0;}

  return difference;
}

// =====================================================
// DONE SIGNAL
// =====================================================
bool SimpleStepper::isDone(){
  return !_moving;
}

// =====================================================
// CURRENT VELOCITY
// steps/sec -> degrees/sec
// =====================================================
float SimpleStepper::getVelocityDegPerSec(){
  return _currentVelocitySteps/_stepsPerDegree;
}

// =====================================================
// CURRENT ACCELERATION
// steps/sec^2 -> degrees/sec^2
// =====================================================
float SimpleStepper::getAccelerationDegPerSec2(){
  return _currentAccelerationSteps/_stepsPerDegree;
}
#include "SimpleStepper.h"

// =====================================================
// MOTOR SETTINGS
// =====================================================
const float MOTOR_STEPS = 200.0;
const int MICROSTEPS = 2;

// =====================================================
// BASE GEARING
// =====================================================
const int BASE_DRIVEN_GEAR = 100;
const int BASE_DRIVER_GEAR = 10;

const float BASE_GEAR_RATIO = (float)BASE_DRIVEN_GEAR/BASE_DRIVER_GEAR;
const float BASE_STEPS_PER_DEGREE =(MOTOR_STEPS*MICROSTEPS*BASE_GEAR_RATIO)/360.0;

// =====================================================
// STENCIL GEARING
// =====================================================
const int STENCIL_DRIVEN_GEAR = 100;
const int STENCIL_DRIVER_GEAR = 10;

const float STENCIL_GEAR_RATIO = (float)STENCIL_DRIVEN_GEAR/STENCIL_DRIVER_GEAR;
const float STENCIL_STEPS_PER_DEGREE = (MOTOR_STEPS*MICROSTEPS*STENCIL_GEAR_RATIO)/360.0;

// =====================================================
// PINS
// =====================================================
const int BASE_STEP_PIN = 8;
const int BASE_DIR_PIN  = 7;

const int STENCIL_STEP_PIN = 18;
const int STENCIL_DIR_PIN  = 1;

// =====================================================
// CREATE MOTOR CONTROLLERS
// =====================================================
SimpleStepper base(BASE_STEP_PIN,BASE_DIR_PIN,BASE_STEPS_PER_DEGREE);
SimpleStepper stencil(STENCIL_STEP_PIN,STENCIL_DIR_PIN,STENCIL_STEPS_PER_DEGREE);

// =====================================================
// MASTER MOTION SETTINGS
//
// Motor with longest movement gets these values.
// Other motor is scaled automatically.
// =====================================================
const float MASTER_MIN_SPEED = 100.0;
const float MASTER_MAX_SPEED = 1000.0;
const float MASTER_ACCEL = 100.0;

// =====================================================
// SERIAL INPUT
// =====================================================

String serialInput = "";
bool moveInProgress = false; // True while a coordinated move is running

void setup()
{
  Serial.begin(115200);

  // Set up both motor controllers
  base.begin();
  stencil.begin();

  // Current positions become HOME
  base.setHome();
  stencil.setHome();

  Serial.println();
  Serial.println("=============================");
  Serial.println(" TWO MOTOR TEST CONTROLLER");
  Serial.println("=============================");
  Serial.println();

  printInstructions();
}

void loop(){
  // Motor controllers always get updated
  base.update();
  stencil.update();

  readSerialInput();

  // ---------------------------------------------------
  // LIVE MOTION DATA
  // ---------------------------------------------------
  static unsigned long lastPrint = 0;

  if (moveInProgress && millis() - lastPrint >= 250){
    lastPrint = millis();
    printMotionData();
  }

  // ---------------------------------------------------
  // CHECK IF BOTH ARE FINISHED
  // ---------------------------------------------------
   if (moveInProgress && coordinatedMoveDone()){
    moveInProgress = false;

    Serial.println();
    Serial.println("BOTH MOTORS FINISHED!");
    Serial.print("Base Final: ");
    Serial.print(base.getWrappedAngle(),2); Serial.print(" deg");
    Serial.print("    Stencil Final: ");
    Serial.print(stencil.getWrappedAngle(),2); Serial.println(" deg");
    Serial.println();

    printInstructions();
  }
}

// =====================================================
// SERIAL INPUT
//
// Enter:
//
// 15,-120
//
// First number  = Base target
// Second number = Stencil target
// =====================================================

void readSerialInput()
{
  while (Serial.available())
  {
    char incoming = Serial.read();
    // End of line
    if (incoming == '\n' || incoming == '\r'){
      // Ignore empty lines
      if (serialInput.length() == 0) continue;

      float baseTarget;
      float stencilTarget;

      // Read two numbers separated by comma
      int valuesRead = sscanf(serialInput.c_str(),"%f,%f",&baseTarget,&stencilTarget);

      // Valid command
      if (valuesRead == 2){
        if (moveInProgress){
          Serial.println("Motors are still moving.");
        }
        else{
          Serial.println();
          Serial.print("New Target -> Base: ");
          Serial.print(baseTarget);
          Serial.print(" deg   Stencil: ");
          Serial.print(stencilTarget);
          Serial.println(" deg");

          moveBothTo(baseTarget,stencilTarget);
        }
      }

      // Invalid command
      else{
        Serial.println("Invalid input. Use: BASE,STENCIL");
        Serial.println("Example: 15,-120");
      }

      // Clear input
      serialInput = "";
    }

    else{
      // Add character to command
      serialInput += incoming;
    }
  }
}

// =====================================================
// LIVE MOTION DISPLAY
// =====================================================
void printMotionData(){
  // ---------------- BASE ----------------
  Serial.print("Base: ");
  Serial.print(base.getWrappedAngle(),2);
  Serial.print(" deg");
  Serial.print(" | V: ");
  Serial.print(base.getVelocityDegPerSec(),2);
  Serial.print(" deg/s");
  Serial.print(" | A: ");
  Serial.print(base.getAccelerationDegPerSec2(),2);
  Serial.print(" deg/s^2");

  // ---------------- STENCIL ----------------
  Serial.print("   ||   Stencil: ");
  Serial.print(stencil.getWrappedAngle(),2);
  Serial.print(" deg");
  Serial.print(" | V: ");
  Serial.print(stencil.getVelocityDegPerSec(),2
  );
  Serial.print(" deg/s");
  Serial.print(" | A: ");
  Serial.print(stencil.getAccelerationDegPerSec2(),2);
  Serial.println(" deg/s^2");
}

// =====================================================
// PRINT USER INSTRUCTIONS
// =====================================================
void printInstructions(){
  Serial.println("Enter target angles as:");
  Serial.println("BASE,STENCIL");
  Serial.println();
  Serial.println("Example: 15,-120");
  Serial.println();
}

// =====================================================
// COORDINATED MOVE
//
// Give this the desired FINAL ANGLES.
//
// It:
// 1. Finds shortest movements
// 2. Finds movement ratio
// 3. Scales speed
// 4. Scales acceleration
// 5. Starts both motors
//
// They finish together.
// =====================================================
void moveBothTo(float baseTargetAngle,float stencilTargetAngle){
  // Find shortest movements
  float baseMove = base.shortestMoveTo(baseTargetAngle);
  float stencilMove = stencil.shortestMoveTo(stencilTargetAngle);

  // Convert movements to motor steps
  long baseSteps = abs(base.degreesToSteps(baseMove));
  long stencilSteps = abs(stencil.degreesToSteps(stencilMove));

  // Find longest movement
  long longestMove = max(baseSteps,stencilSteps);

  // Nothing needs to move
  if (longestMove == 0){
    Serial.println("Already at target positions.");
    return;
  }

  // ===================================================
  // MOVEMENT RATIOS
  // ===================================================
  float baseRatio = (float)baseSteps/longestMove;
  float stencilRatio =(float)stencilSteps/longestMove;

  // ===================================================
  // START BASE
  // ===================================================
  if (baseSteps > 0){
    base.startMove(baseMove, MASTER_MIN_SPEED*baseRatio, MASTER_MAX_SPEED*baseRatio, MASTER_ACCEL*baseRatio);
  }

  // ===================================================
  // START STENCIL
  // ===================================================
  if (stencilSteps > 0){
    stencil.startMove(stencilMove, MASTER_MIN_SPEED*stencilRatio, MASTER_MAX_SPEED*stencilRatio, MASTER_ACCEL*stencilRatio);
  }

  // Tell the main program a move is now active
  moveInProgress = true;
}

// =====================================================
// BOTH MOTORS DONE?
// =====================================================
bool coordinatedMoveDone(){
  return base.isDone() && stencil.isDone();
}


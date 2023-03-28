#include <Wire.h>
#include <Zumo32U4.h>

Zumo32U4Motors motors;
Zumo32U4ButtonA buttonA;
Zumo32U4ButtonB buttonB;
Zumo32U4Encoders encoders;
Zumo32U4LCD screen;  
Zumo32U4LineSensors lineSensors;

int leftEnc, rightEnc, avg, diff;
int currentStep;
int lineSensorValues[5]; 


void setup() {
  // put your setup code here, to run once:
  screen.clear();
  Serial.begin(9600);
  lineSensors.initFiveSensors();
  doLineSensorCalibration();
  screen.gotoXY(0,0);
  screen.clear();
  screen.print("Press B.");
  buttonB.waitForButton();
  delay(500);
  currentStep = 0;
}


void doLineSensorCalibration()
{
  buttonA.waitForRelease(); 
  screen.clear();
  screen.gotoXY(0,0);
  screen.print("calibrating"); 
  screen.gotoXY(0,1);
  screen.print("A->Done");
  while (!buttonA.isPressed())
  {
    lineSensors.calibrate(); 
    delay(5);
  }
  buttonA.waitForRelease(); 
}


void loop() {
  screen.clear();
  updateStepInLowerRightCornerOfScreen();
  if (currentStep == 0)
  {
    moveOneStepStraightTowards(4000);
  }
  if (currentStep == 1)
  {
    moveOnALine(6500);
    //push ball
  }
  if (currentStep == 2)
  {
    moveOnALine(-7000);
  }
  if (currentStep == 3)
  {
   turnOneStepTowardsAngle(-600);
  }
  if (currentStep == 4)
  {
    moveOneStepStraightTowards(3500);
  }
  if (currentStep == 5)
  {
   turnOneStepTowardsAngle(700);
  }
  if (currentStep == 6)
  {
    moveOneStepStraightTowards(900);
    //stuff cube under right ramp
  }
  if (currentStep == 7)
  {
    moveOneStepStraightTowards(-1000);
  }
  if (currentStep == 8)
  {
   turnOneStepTowardsAngle(-1400);
  }
  if (currentStep == 9)
  {
    moveOneStepStraightTowards(1890);
  }
  if (currentStep == 10)
  {
   turnOneStepTowardsAngle(1250);
  }
  if (currentStep == 11)
  {
    moveOneStepStraightTowards(7000);
  }
  if (currentStep == 12)
  {
    moveOneStepStraightTowards(-1000);
  }
  if (currentStep == 13)
  {
   turnOneStepTowardsAngle(2400);
  }
  if (currentStep == 14)
  {
    moveOneStepStraightTowards(8500);
  }
  if (currentStep == 15)
  {
   turnOneStepTowardsAngle(2600);
  }
  if (currentStep == 16)
  {
    moveOneStepStraightTowards(5800);
    //to the center
  }
  if (currentStep == 17)
  {
     screen.clear();
     screen.gotoXY(0,0);
     screen.print("Press B");
     buttonB.waitForButton();
     resetAndAdvanceToNextStep();
     currentStep = 0; 
  }
}


void measureEncoders()
{
  leftEnc = encoders.getCountsLeft();
  rightEnc = encoders.getCountsRight();
  avg = (leftEnc + rightEnc)/2;
  diff = leftEnc - rightEnc;
}


void moveOnALine(int target)
{
  int pos = lineSensors.readLine(lineSensorValues,true);
  screen.clear();
     screen.gotoXY(0,0);
     screen.print(pos);
     updateStepInLowerRightCornerOfScreen();
  measureEncoders();
  int error = avg - target;
  int power = abs(error)/4;  
  if (power > 330) 
  {
    power = 330;
  }
  if (power < 105) 
  {
    power = 105;
  }
  // ----------------
  if (abs(error) > 20) 
  {
    int leftMotorPower;
    if (avg < target) 
    {
      leftMotorPower = power;
      updateDirectionIndicatorOnScreen("^");
    }
    if (avg > target) 
    {
      leftMotorPower = -1*power;
      updateDirectionIndicatorOnScreen("v");
    }
    int rightMotorPower = leftMotorPower;  
    if(pos <= 2005)
    {
      setLEDs(LOW,LOW,HIGH);
      rightMotorPower = leftMotorPower * 1.2;
    }
    else if(pos <= 1995)
    {
      setLEDs(HIGH,LOW,LOW);
      rightMotorPower = leftMotorPower * 0.9;
    }
    else
    {
      setLEDs(LOW,HIGH,LOW);
    }
    
    motors.setSpeeds(leftMotorPower,rightMotorPower);
  }
  else 
  {
    resetAndAdvanceToNextStep();
  }
}


void moveOneStepStraightTowards(int target)
{
  measureEncoders();
  int error = avg-target;
  
  screen.gotoXY(0,0); 
  screen.print(avg);
  screen.gotoXY(0,1);
  screen.print(target);
  
  int power = abs(error)/4;  
  if (power > 360)  
  {
    power = 360;
  }
  if (power < 50) 
  {
    power = 75;
  }
  // ----------------
  if (abs(error) > 20) 
  {
    int leftMotorPower;
    if (avg < target) 
    {
      leftMotorPower = power;
      updateDirectionIndicatorOnScreen("^");
    }
    if (avg > target) 
    {
      leftMotorPower = -1*power;
      updateDirectionIndicatorOnScreen("v");
    }
    // ---------  Correct for left/right drift -> see Encoders 1
    int rightMotorPower = leftMotorPower;  // assume we go straight
    if (leftMotorPower>0) // driving forwards.
    {
      if (diff < -7) // correct if drifting left
      {
          rightMotorPower = leftMotorPower * 0.95;
      }
      if (diff > 7) // correct if drifting right
      {
          rightMotorPower = leftMotorPower * 1.10; //updated from 1.05 to 1.10 in version 1.1
      }
    }
    else // course correction if driving backwards! New in version 1.1
    {
      if (diff < -4) // correct if drifting left
      {
          rightMotorPower = leftMotorPower * 1.10; //updated from 1.05 to 1.10 in version 1.1
      }
      if (diff > 4) // correct if drifting right
      {
          rightMotorPower = leftMotorPower * 0.90;
      }
    }
    // ------------
    motors.setSpeeds(leftMotorPower,rightMotorPower);
  }
  else // if we are at the target...
  {
    resetAndAdvanceToNextStep();
  }
}



void turnOneStepTowardsAngle(int target)
{
  measureEncoders();
  int error = diff-target;

  screen.gotoXY(0,0);  // draw diff and target on screen.
  screen.print(diff);
  screen.gotoXY(0,1);
  screen.print(target);
//  screen.print(error); // alternative to "target," if you prefer.
   
  // ------------ P-control -> see "Encoders 2"
  int power = abs(error)/4;
  if (power > 319)
  {
    power = 319;
  }
  if (power < 100) // min power that actually moves the robot.
  {
    power = 100;
  }
  // ------------
  if (abs(error) > 20) // within 20 is "close enough." - adjust as needed.
  {
    if (diff < target) // we need to turn right to get to target.
    {
       motors.setSpeeds(+1*power,-1*power);
       updateDirectionIndicatorOnScreen(">");
    }
    else // we need to turn left to get to target
    {
       motors.setSpeeds(-1*power,+1*power);
       updateDirectionIndicatorOnScreen("<");
    }
  }
  else
  {
    resetAndAdvanceToNextStep();
  }
}

/**
 * process when we reach target to set up for the next step in our multi-step
 * procedure.
 */
void resetAndAdvanceToNextStep()
{
    encoders.getCountsAndResetLeft();
    encoders.getCountsAndResetRight();
    currentStep = currentStep + 1;
    motors.setSpeeds(0,0);
}

/**
 * draws an arrow (or whatever dir is) in the upper right corner of the screen.
 */
void updateDirectionIndicatorOnScreen(String dir)
{
  screen.gotoXY(7,0);
  screen.print(dir);
}

/**
 * puts the step number into the lower right corner of screen. 
 */
void updateStepInLowerRightCornerOfScreen()
{
  screen.gotoXY(6,1); // print the current step in the lower right corner.
  screen.print(currentStep); 
}
/** 
 * Change the state of all three LEDs - call this method with three HIGH/LOW values to set all three LEDs at once. 
 * For example, "setLEDs(HIGH,LOW,HIGH);" would turn on the yellow and red LEDs.
 */
void setLEDs(int Y, int G, int R)
{
    ledYellow(Y);
    ledGreen(G);
    ledRed(R);
}

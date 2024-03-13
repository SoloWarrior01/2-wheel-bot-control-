/***************************************************************
   Motor driver definitions
   
   Add a "#elif defined" block to this file to include support
   for a particular motor driver.  Then add the appropriate
   #define near the top of the main ROSArduinoBridge.ino file.
   
   *************************************************************/


  void initMotorController() {
    // digitalWrite(RIGHT_MOTOR_ENABLE, HIGH);
    // digitalWrite(LEFT_MOTOR_ENABLE, HIGH);
  }
  
  void setMotorSpeed(int i, int spd) {
    unsigned int reverse = 0;
    if(spd < 0)
    {
      spd = -spd;
      reverse = 1;
    }
    else if(spd>255)
    {
      spd = 0;
    }
    if (i == LEFT) { 
      analogWrite(LEFT_MOTOR_PWM, spd);
      if      (reverse == 0) { digitalWrite(LEFT_MOTOR_FORWARD, HIGH); digitalWrite(LEFT_MOTOR_BACKWARD, LOW); }
      else if (reverse == 1) { digitalWrite(LEFT_MOTOR_BACKWARD, HIGH); digitalWrite(LEFT_MOTOR_FORWARD, LOW); }
    }
    else if(i == RIGHT) {
      analogWrite(RIGHT_MOTOR_PWM, spd);
      if      (reverse == 0) { digitalWrite(RIGHT_MOTOR_FORWARD, HIGH); digitalWrite(RIGHT_MOTOR_BACKWARD, LOW); }
      else if (reverse == 1) { digitalWrite(RIGHT_MOTOR_BACKWARD, HIGH); digitalWrite(RIGHT_MOTOR_FORWARD, LOW); }
    }
  }
  
  void setMotorSpeeds(int leftSpeed, int rightSpeed) {
    setMotorSpeed(LEFT, leftSpeed);
    setMotorSpeed(RIGHT, rightSpeed);
  }
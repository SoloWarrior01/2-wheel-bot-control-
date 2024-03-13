/***************************************************************
   Motor driver function definitions - by James Nugen
   *************************************************************/

#define RIGHT_MOTOR_BACKWARD 7
#define LEFT_MOTOR_BACKWARD  8
#define RIGHT_MOTOR_FORWARD  6
#define LEFT_MOTOR_FORWARD   9
#define RIGHT_MOTOR_PWM 10
#define LEFT_MOTOR_PWM 11

void initMotorController();
void setMotorSpeed(int i, int spd);
void setMotorSpeeds(int leftSpeed, int rightSpeed);
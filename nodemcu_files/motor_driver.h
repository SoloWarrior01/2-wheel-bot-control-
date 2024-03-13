/***************************************************************
   Motor driver function definitions - by James Nugen
   *************************************************************/

#define RIGHT_MOTOR_BACKWARD D7
#define LEFT_MOTOR_BACKWARD  D2
#define RIGHT_MOTOR_FORWARD  D8
#define LEFT_MOTOR_FORWARD   D3
#define RIGHT_MOTOR_PWM 10
#define LEFT_MOTOR_PWM D4

void initMotorController();
void setMotorSpeed(int i, int spd);
void setMotorSpeeds(int leftSpeed, int rightSpeed);
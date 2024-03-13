#include <util/atomic.h>
#include "SerialTransfer.h"

/* Serial port baud rate */
#define BAUDRATE     57600

/* Maximum PWM signal */
#define MAX_PWM        255

#define RPM_SCALE 500

#if defined(ARDUINO) && ARDUINO >= 100
#include "Arduino.h"
#else
#include "WProgram.h"
#endif

/* Include definition of serial commands */
#include "commands.h"

#include "motor_driver.h"

/* Encoder driver function definitions */
#include "encoder_driver.h"

/* PID parameters and functions */
#include "diff_controller.h"

/* Run the PID loop at 30 times per second */
#define PID_RATE           1000     // Hz
/* Convert the rate into an interval */
const int PID_INTERVAL = 1000 / PID_RATE;

/* Track the next time we make a PID calculation */
unsigned long nextPID = PID_INTERVAL;
/* Stop the robot if it hasn't received a movement command
 in this number of milliseconds */
#define AUTO_STOP_INTERVAL 10000
long lastMotorCommand = AUTO_STOP_INTERVAL;

SerialTransfer myTransfer;

char arr[20];
char success[30]="OK";
char failure[30]="ER";
char bd[30]="57600";
char encRead[30];
char motOut[30];

// A pair of varibles to help parse serial commands (thanks Fergs)
int arg = 0;
int index = 0;

// Variable to hold an input character
char chr;

// Variable to hold the current single-character command
char cmd;

// Character arrays to hold the first and second arguments
char argv1[16];
char argv2[16];
char argv3[16];

// The arguments converted to integers
long arg1;
long arg2;
long arg3;

void setup()
{
  Serial.begin(57600);
  myTransfer.begin(Serial);
  attachInterrupt(digitalPinToInterrupt(LEFT_ENC_PIN_A),
                  interruptLeftEncoder,RISING);
  attachInterrupt(digitalPinToInterrupt(RIGHT_ENC_PIN_A),
                  interruptRightEncoder,RISING);

  pinMode(LEFT_ENC_PIN_A,INPUT);
  pinMode(LEFT_ENC_PIN_B,INPUT);
  pinMode(RIGHT_ENC_PIN_A,INPUT);
  pinMode(RIGHT_ENC_PIN_B,INPUT);
  pinMode(LEFT_MOTOR_PWM,OUTPUT);
  pinMode(RIGHT_MOTOR_PWM,OUTPUT);
  pinMode(LEFT_MOTOR_BACKWARD,OUTPUT);
  pinMode(RIGHT_MOTOR_BACKWARD,OUTPUT);
  pinMode(LEFT_MOTOR_FORWARD,OUTPUT);
  pinMode(RIGHT_MOTOR_FORWARD,OUTPUT);
}

int count(int x)
{
  int c=0;
  while(x)
  {
    c++;
    x/=10;
  }
  return c;
}

void sendSuccess()
{
    uint16_t sendSize = 0;
    sendSize = myTransfer.txObj(success, sendSize);
    myTransfer.sendData(sendSize);
}

void sendFailure()
{
    uint16_t sendSize = 0;
    sendSize = myTransfer.txObj(failure, sendSize);
    myTransfer.sendData(sendSize);
}

void sendBaud()
{
    uint16_t sendSize = 0;
    sendSize = myTransfer.txObj(bd, sendSize);
    myTransfer.sendData(sendSize);
}

int runCommand(){
  int i = 0;
  char *p = argv1;
  char *str;
  int pid_args[4];
  arg1 = atoi(argv1);
  arg2 = atoi(argv2);
  arg3 = atoi(argv3);

  if(cmd=='b')
  {
    sendBaud();
  }
  else if(cmd=='r')
  {
      resetPID();
      kp = arg1;
      ki = arg2;
      kd = arg3;
      sendSuccess();
  }
  else if(cmd=='m')
  {
    /* Reset the auto stop timer */
    lastMotorCommand = millis();
    if (arg1 == 0 && arg2 == 0) {
      setMotorSpeeds(0, 0);
      resetPID();
      moving = 0;
    }
    else 
    {
      if(moving==0)
        resetPID();
      moving = 1;
    }
    leftPID.TargetVelocity = arg1;
    rightPID.TargetVelocity = arg2;
    leftPID.PrevT = micros();
    rightPID.PrevT = micros();
    sendSuccess();
  }
  else if(cmd=='o')
  {
    /* Reset the auto stop timer */
    lastMotorCommand = millis();
    resetPID();
    moving = 0; // Sneaky way to temporarily disable the PID
    setMotorSpeeds(arg1, arg2);
    sendSuccess();
  }
  else
  {
    sendFailure();
  }
}


/* Clear the current command parameters */
void resetCommand() {
  cmd = NULL;
  memset(argv1, 0, sizeof(argv1));
  memset(argv2, 0, sizeof(argv2));
  memset(argv3, 0, sizeof(argv3));
  arg1 = 0;
  arg2 = 0;
  arg3 = 0;
  arg = 0;
  index = 0;
}

void loop()
{ 
  if(myTransfer.available())
  {
    uint16_t recSize = 0;
    uint16_t sendSize = 0;
    
    recSize = myTransfer.rxObj(arr, recSize);
    for(int i=0; i < 20; i++)
    {
       char chr = arr[i];
       if (chr == 13) {
        if (arg == 1) argv1[index] = NULL;
        else if (arg == 2) argv2[index] = NULL;
        else if(arg == 3) argv3[index] = NULL;
          runCommand();
          resetCommand();
       }
      // Use spaces to delimit parts of the command
      else if (chr == ' ') {
        // Step through the arguments
        if (arg == 0) arg = 1;
        else if (arg == 1)  {
          argv1[index] = NULL;
          arg = 2;
          index = 0;
        }
        else if (arg == 2) {
          argv2[index] = NULL;
          arg = 3;
          index = 0;
        }
        continue;
      }
      else {
        if (arg == 0) {
          // The first arg is the single-letter command
          cmd = chr;
        }
        else if (arg == 1) {
          // Subsequent arguments can be more than one character
          argv1[index] = chr;
          index++;
        }
        else if (arg == 2) {
          argv2[index] = chr;
          index++;
        }
        else if (arg == 3) {
          argv3[index] = chr;
          index++;
        }
      }
    }
  }

if (millis() > nextPID) {
  updatePID();
  nextPID += PID_INTERVAL;
}

// Check to see if we have exceeded the auto-stop interval
if ((millis() - lastMotorCommand) > AUTO_STOP_INTERVAL) {;
  setMotorSpeeds(0, 0);
  moving = 0;
  resetPID();
}
  delay(1000/30);
}
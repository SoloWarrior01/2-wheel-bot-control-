/* Functions and type-defs for PID control.

   Taken mostly from Mike Ferguson's ArbotiX code which lives at:
   
   http://vanadium-ros-pkg.googlecode.com/svn/trunk/arbotix/
*/

/* PID setpoint info For a Motor */
typedef struct {
  float TargetVelocity;    // target speed in ticks per frame
  float Velocity;                  // encoder count
  float PrevVel;                  // last encoder count
  long PrevT;
  float PrevErr;
  float VelFilt;
  float ITerm;
  float PrevInput;                // last input
  int output;                    // last motor setting
}
SetPointInfo;

SetPointInfo leftPID, rightPID;

/* PID Parameters */
float kp = 30;
float ki = 100;
float kd = 0.05;
float IMAX = 150;
float IMIN = -150;
unsigned char moving = 0; // is the base in motion?

/*
* Initialize PID variables to zero to prevent startup spikes
* when turning PID on to start moving
* In particular, assign both Encoder and PrevEnc the current encoder value
* See http://brettbeauregard.com/blog/2011/04/improving-the-beginner%E2%80%99s-pid-initialization/
* Note that the assumption here is that PID is only turned on
* when going from stop to moving, that's why we can init everything on zero.
*/
void resetPID(){
   leftPID.ITerm = 0;
   rightPID.ITerm = 0;
}

/* PID routine to compute the next motor commands */
void doPID(SetPointInfo * p) {

  long currT = micros();
  float deltaT = ((float) (currT - p->PrevT))/1.0e6;
  p->PrevT = currT;

  // Convert count/s to RPM
  float v1 = p->Velocity/RPM_SCALE;

  p->VelFilt = 0.854*p->VelFilt + 0.0728*v1 + 0.0728*p->PrevVel;
  p->PrevVel = v1;

  // Set a target
  float vt = p->TargetVelocity;

  // Compute the control signal u
  float e = vt - p->VelFilt;
  float u = 0.0;
  float errdiff = e - p->PrevErr;
  float inputdiff = p->VelFilt - p->PrevInput;
  p->ITerm = p->ITerm + e*deltaT;

  if(ki*p->ITerm>IMAX)
    p->ITerm = IMAX/ki;
  else if(ki*p->ITerm<IMIN)
    p->ITerm = IMIN/ki;
  u = kp*e + ki*p->ITerm - kd*(inputdiff/deltaT);

  // Set the motor speed and direction
  int pwr = (int)(u);
  
  if (pwr >= MAX_PWM)
    pwr = MAX_PWM;
  else if (pwr <= -MAX_PWM)
    pwr = -MAX_PWM;


  p->PrevInput = p->VelFilt;
  p->PrevErr = e;
  p->output = pwr;
}

/* Read the encoder values and call the PID routine */
void updatePID() {
  /* Read the encoders */
  leftPID.Velocity = readEncoder(LEFT);
  rightPID.Velocity = readEncoder(RIGHT);
  
  if (!moving){
    if (leftPID.PrevInput != 0 || rightPID.PrevInput != 0) resetPID();
    return;
  }

  /* Compute PID update for each motor */
  doPID(&rightPID);
  doPID(&leftPID);

  /* Set the motor speeds accordingly */
  setMotorSpeeds(leftPID.output, rightPID.output);
}
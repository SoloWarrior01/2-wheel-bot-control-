/* *************************************************************
   Encoder definitions
   
   Add an "#ifdef" block to this file to include support for
   a particular encoder board or library. Then add the appropriate
   #define near the top of the main ROSArduinoBridge.ino file.
   
   ************************************************************ */
   
volatile float left_velocity_i = 0;
volatile float right_velocity_i = 0;
volatile long left_prevT_i = 0;
volatile long right_prevT_i = 0;
  
/* Wrap the encoder reading function */
float readEncoder(int i) {
  float left_vel=0;
  float right_vel=0;
  noInterrupts();
  left_vel = left_velocity_i;
  right_vel = right_velocity_i;
  interrupts();
  if (i == LEFT) return left_vel;
  else if(i == RIGHT) return right_vel;
}

/* Wrap the encoder reset function */
void resetEncoder(int i) {
//    if (i == LEFT){
//      left_enc_pos=0;
//      return;
//    } else if(i == RIGHT){ 
//      right_enc_pos=0;
//      return;
//    }
}

/* Wrap the encoder reset function */
void resetEncoders() {
  resetEncoder(LEFT);
  resetEncoder(RIGHT);
}

void ICACHE_RAM_ATTR interruptLeftEncoder(){
 // Read encoder B when ENCA rises
 int b = digitalRead(LEFT_ENC_PIN_B);
 int increment = 0;
 if(b>0){
   // If B is high, increment forward
   increment = 1;
 }
 else{
   // Otherwise, increment backward
   increment = -1;
 }

 //Compute velocity with method 2
 long currT = micros();
 float deltaT = ((float) (currT - left_prevT_i))/1.0e6;
 left_velocity_i = -1*increment/deltaT;
 left_prevT_i = currT;
}

void ICACHE_RAM_ATTR interruptRightEncoder(){
 // Read encoder B when ENCA rises
 int b = digitalRead(RIGHT_ENC_PIN_B);
 int increment = 0;
 if(b>0){
   // If B is high, increment forward
   increment = 1;
 }
 else{
   // Otherwise, increment backward
   increment = -1;
 }

 //Compute velocity with method 2
 long currT = micros();
 float deltaT = ((float) (currT - right_prevT_i))/1.0e6;
 right_velocity_i = increment/deltaT;
 right_prevT_i = currT;
}
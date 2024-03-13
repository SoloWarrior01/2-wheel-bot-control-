/* *************************************************************
   Encoder driver function definitions - by James Nugen
   ************************************************************ */
   
   
//below can be changed, but should be PORTD pins; 
//otherwise additional changes in the code are required
#define LEFT_ENC_PIN_A D0  //pin 2
#define LEFT_ENC_PIN_B D1  //pin 3

//below can be changed, but should be PORTC pins
#define RIGHT_ENC_PIN_A D5  //pin A4
#define RIGHT_ENC_PIN_B D6   //pin A5
   
float readEncoder(int i);
void resetEncoder(int i);
void resetEncoders();

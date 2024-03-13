#include <math.h>
#include <ESP8266WiFi.h>
#include <ESP8266WiFiMulti.h>
#include <ros.h>
#include <geometry_msgs/Twist.h>
#include <std_msgs/UInt8.h>
// The min amount of PWM the motors need to move. Depends on the battery, motors and controller.
//The max amount is defined by PWMRANGE in Arduino.h
#define PWM_MIN 220
#define RPM_SCALE 500
#define MAX_PWM 255

#define PID_RATE           1000     // Hz
const int PID_INTERVAL = 1000 / PID_RATE;
unsigned long nextPID = PID_INTERVAL;
#define AUTO_STOP_INTERVAL 10000
long lastMotorCommand = AUTO_STOP_INTERVAL;

#include "commands.h"
#include "motor_driver.h"
/* Encoder driver function definitions */
#include "encoder_driver.h"
/* PID parameters and functions */
#include "diff_controller.h"


// Declare functions
void setupPins();
void setupSerial();
void setupWiFi();
bool rosConnected();
void onTwist(const geometry_msgs::Twist &msg);
void ICACHE_RAM_ATTR interruptLeftEncoder();
void ICACHE_RAM_ATTR interruptRightEncoder();

#ifndef ACCESS_POINT_SSID
ESP8266WiFiMulti wifi;
#endif

// ROS serial server
IPAddress server(192, 168, 32, 109);
ros::NodeHandle node;
ros::Subscriber<geometry_msgs::Twist> sub("/omni_vel", &onTwist);
bool _connected = false;


void setup()
{
  setupPins();
  setupSerial();
  setupWiFi();

  // Connect to rosserial socket server and init node. (Using default port of 11411)
  Serial.printf("Connecting to ROS serial server at %s\n", server.toString().c_str());
  node.getHardware()->setConnection(server);
  node.initNode();
  node.subscribe(sub);
}

void setupPins()
{
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
  stop();

delay(2000);
}

void setupSerial()
{
  Serial.begin(57600);
  Serial.println();
}

void setupWiFi()
{
#ifdef ACCESS_POINT_SSID

  WiFi.disconnect();
  Serial.println("Creating Wifi network");
  if (WiFi.softAP(ACCESS_POINT_SSID))
  {
    Serial.println("Wifi network created");
    Serial.print("SSID: ");
    Serial.println(WiFi.softAPSSID());
    Serial.print("IP:   ");
    Serial.println(WiFi.softAPIP());
  }

#else

  WiFi.softAPdisconnect();
  WiFi.disconnect();
  WiFi.mode(WIFI_STA);
  wifi.addAP("ASUS_X00RD", "Golu-Molu");

  Serial.println("Connecting to Wifi");
  while (wifi.run() != WL_CONNECTED)
  {
    delay(500);
    Serial.print(".");
  }

  Serial.println("\nWiFi connected");
  Serial.print("SSID: ");
  Serial.println(WiFi.SSID());
  Serial.print("IP:   ");
  Serial.println(WiFi.localIP());

#endif
}

void stop()
{
  setMotorSpeeds(0, 0);
}

void onTwist(const geometry_msgs::Twist &msg)
{
  if (!_connected)
  {
    stop();
    return;
  }

  float arg1 = msg.linear.x;
  float arg2 = msg.linear.y;

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
    Serial.print(arg1);
    Serial.print(" ");
    Serial.println(arg2);
}

void loop()
{
  if (!rosConnected())
    stop();
    
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
  node.spinOnce();
}

bool rosConnected()
{
  // If value changes, notify via LED and console.
  bool connected = node.connected();
  if (_connected != connected)
  {
    _connected = connected;
    Serial.println(connected ? "ROS connected" : "ROS disconnected");
  }
  return connected;
}
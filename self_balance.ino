#include <Wire.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BNO055.h>
#include <utility/imumaths.h>
Adafruit_BNO055 bno = Adafruit_BNO055(55);            //instantiate IMU
// motor pin defs
#define dir_A   12 // motor shield channel A direction
#define spd_A   3  // channel A speed 
#define brk_A   9  // channel A brake 
#define dir_B   13 // motor shield channel B direction 
#define spd_B   11 // channel B speed  
#define brk_B   8  // channel B brake 
// reserved motor pins
#define curr_A  A0 // channel A current sensing
#define curr_B  A1 // channel B current sensing
// sensor pin defs
#define IR_L    A2 //left IR sensor
#define IR_R    A3 //right IR sensor
int movespd;       //general movement speed of motor
unsigned long prev_time;
// pid constants
const float kp = 0.05;
const float kd = 0.15;
const float ki = 0.0000;                              //not using integral error, only PD
// pid errors
float integral_error = 0;
float deriv_error = 0;
float error = 0;
float prev_error = 0;

void setup(void) 
{
  pinMode(spd_A, OUTPUT);                             // motor control outputs
  pinMode(brk_A, OUTPUT);
  pinMode(dir_A, OUTPUT);
  pinMode(spd_B, OUTPUT);
  pinMode(brk_B, OUTPUT);
  pinMode(dir_B, OUTPUT);
  Serial.begin(9600);
  Serial.println("Orientation Sensor Test"); Serial.println("");
  
  if(!bno.begin()) {                                  //fail check imu init
    Serial.print("Ooops, no BNO055 detected ... Check your wiring or I2C ADDR!");
    while(1);
  }
  delay(1000); 
  bno.setExtCrystalUse(true);
}

// motor movement setting
void setMovement(bool dirL, bool enL, int spdL, bool dirR, bool enR, int spdR) {
  // channel A motor on right side
  digitalWrite(dir_A, dirR);
  digitalWrite(brk_A, enR);
  analogWrite(spd_A, spdR);
  // channel B motor on left side
  digitalWrite(dir_B, dirL);
  digitalWrite(brk_B, enL);
  analogWrite(spd_B, spdL);
}

// directional motor setup
void forward() {
  setMovement(HIGH, LOW, movespd, HIGH, LOW, movespd);        // move forward 
}
void reverse() { 
  setMovement(LOW, LOW, movespd, LOW, LOW, movespd);          // move backward
}

// pid control loop
void loop(void) 
{ 
  sensors_event_t event;                                      // get new sensor event
  bno.getEvent(&event);
  // pid control algorithm
  unsigned long current_time = millis();
  unsigned long elapsed_time = current_time - prev_time;      // get elapsed time btw errors
  float setpoint = 5.7;                                       // set point in degrees, !=0 for inclines
  float pitch = event.orientation.y;                          // y-axis pitch tilt in degrees, negative = forward
  error = setpoint - pitch;                                   // error is setpoint offset
  integral_error += error*elapsed_time;                       // integral error is cumulative for all time
  deriv_error = (error - prev_error)/elapsed_time;            // derivative error is change in error per time
  prev_error = error;                                         // reset error per round
  prev_time = current_time;                                   // reset timer per round
  float pid = kp*error + kd*deriv_error + ki*integral_error;  // total linear pid output
  float pid2 = 255*(2/PI)*atan(pid);                          // adjusted pid output to arctan curve capped at 255

  if(pitch < setpoint) {        // if robot is leaning forward, move wheels forward
    movespd = abs(int(pid2));   // use magnitude for motor speed
    forward();
  }
  if(pitch > setpoint) {        // if robot is leaning backward, move wheels backward
    movespd = abs(int(pid2));   // use magnitude for motor speed
    reverse();
  }
  
  Serial.println(pid2);         // show scaled pid values
  
  delay(5);                     // control at 200Hz
}

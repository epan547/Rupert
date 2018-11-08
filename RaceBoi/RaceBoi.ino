// This code should help you get started with your balancing robot.
// The code performs the following steps
// Calibration phase:
//  In this phase the robot should be stationary lying on the ground.
//  The code will record the gyro data for a couple of seconds to zero
//  out any  drift.
//gyro
//  The robot has a hardcoded angle offset between the lying down and the
//    standing up configuration.  This offset can be modified in Balance.cpp around the lines below:
//
//      // this is based on coarse measurement of what I think the angle would be resting on the flat surface. 
//      // this corresponds to 94.8 degrees
//      angle = 94827-6000;
//
// Waiting phase:
//  The robot will now start to integrate the gyro over time to estimate
//  the angle.  Once the angle gets within +/- 3 degrees of vertical,
//  we transition into the armed phase.  A buzzer will sound to indicate
//  this transition.
//
// Armed phase:
//  The robot is ready to go, however, it will not start executing its control
//  loop until the angle leaves the region of [-3 degrees, 3 degrees].  This
//  allows you to let go of your robot, and it won't start moving until it's started
//  to fall just a little bit.  Once it leaves the region around vertical, it enters
//  the controlled phase.
//
// Controlled phase:
//  Here you can implement your control logic to do your balancing (or any of the
//  other Olympic events.


#include <Balboa32U4.h>
#include "Balance.h"

#define METERS_PER_CLICK 3.141592*80.0*(1/1000.0)/12.0/(162.5)
#define MOTOR_MAX 300
#define MAX_SPEED 0.75  // m/s
#define FORTY_FIVE_DEGREES_IN_RADIANS 0.78

extern int32_t angle_accum;
extern int32_t speedLeft;
extern int32_t driveLeft;
extern int32_t distanceRight;
extern int32_t speedRight;
extern int32_t distanceLeft;
extern int32_t distanceRight;

// Setting Control Parameter Values
float Ki = -89;
float Kp = -80;
float Kd = -.11;
float Jp = 16;
float Ji = 60;
float Kv = 0;

// Setting up important equation entities
int PWM_L = 0;
int PWM_R = 0;

float pos_t = 0;
float vR_set, vL_set = 0;

float vL, vR, totalDistanceLeft, totalDistanceRight;
float vD = 0;
float leftMotorPWM = 0;
float rightMotorPWM = 0;

float imu_ax_average = 0.0;
float alpha_imu_ax = 0.1;

float dt = 0;
float totalv_R = 0;
float totalv_L = 0;

float thetad = 0;
float thetad_accum = 0;
float pos_desired = 0;
float prev_t = millis();
float cur_t = 0;

void balanceDoDriveTicks();

extern int32_t displacement;
int32_t prev_displacement=0;

LSM6 imu;
Balboa32U4Motors motors;
Balboa32U4Encoders encoders;
Balboa32U4Buzzer buzzer;
Balboa32U4ButtonA buttonA;

void updatePWMs(float totalDistanceLeft, float totalDistanceRight, float vL, float vR, float angleRad, float angleRadAccum, float dt) {
  /* You will fill this function in with your code to run the race.  The inputs to the function are:
   *    totalDistanceLeft: the total distance travelled by the left wheel (meters) as computed by the encoders
   *    totalDistanceRight: the total distance travelled by the right wheel (meters) as computed by the encoders
   *    vL: the velocity of the left wheel (m/s) measured over the last 10ms
   *    vR: the velocity of the right wheel (m/s) measured over the last 10ms
   *    angleRad: the angle in radians relative to vertical (note: not the same as error)
   *    angleRadAccum: the angle integrated over time (note: not the same as error)
   */
  // Correction velocity

  // Defining position by averaging R & L wheel positions
  pos_t = (totalDistanceLeft + totalDistanceRight)/2;
  // Defining desired position
  pos_desired += dt * .3; //Increasing this number increases speed but reduces stability
  // Definng desired angle (Position Control)
  thetad = Kd * (pos_t-pos_desired);
  // Acruing total angle
  thetad_accum += thetad * dt;

  
  // Angle Control
  vD = Kp*(thetad-angleRad) + Ki*(thetad_accum-angleRadAccum);


  // Velocity Control
  totalv_R = totalv_R + (vD-vR)*dt;
  totalv_L = totalv_L + (vD-vL)*dt;
  
  PWM_L = Jp*(vD-vL) + Ji*(totalv_L);
  PWM_R = Jp*(vD-vR) + Ji*(totalv_R);
  
  // Set PWMs
  leftMotorPWM = PWM_L;
  rightMotorPWM = 1.1105*PWM_R;`//Slighlty adjusted to account for wheel velocity discrepancies
  
}

uint32_t prev_time;

void setup()
{
  Serial.begin(9600);
  //angle = 94900;
  prev_time = 0;
  ledYellow(0);
  ledRed(1);
  balanceSetup();
  ledRed(0);
  angle_accum = 0;
  ledGreen(0);
  ledYellow(0);
}

extern int16_t angle_prev;
int16_t start_flag = 0;
int16_t armed_flag = 0;
int16_t start_counter = 0;
void lyingDown();
extern bool isBalancingStatus;
extern bool balanceUpdateDelayedStatus;

void newBalanceUpdate()
{
  static uint32_t lastMillis;
  uint32_t ms = millis();

  if ((uint32_t)(ms - lastMillis) < UPDATE_TIME_MS) { return; }
  balanceUpdateDelayedStatus = ms - lastMillis > UPDATE_TIME_MS + 1;
  lastMillis = ms;

  // call functions to integrate encoders and gyros
  balanceUpdateSensors();
  imu_ax_average = alpha_imu_ax*imu.a.x + (1 - alpha_imu_ax)*imu_ax_average;
  if (imu_ax_average < 0)
  {
    lyingDown();
    isBalancingStatus = false;
  }
  else
  {
    isBalancingStatus = true;
  }
}


void loop()
{
  uint32_t cur_time = 0;
  static uint32_t prev_print_time = 0;   // this variable is to control how often we print on the serial monitor
  static float angle_rad;                // this is the angle in radians
  static float angle_rad_accum = 0;      // this is the accumulated angle in radians
  static float error_ = 0;      // this is the accumulated velocity error in m/s
  static float error_left_accum = 0;      // this is the accumulated velocity error in m/s
  static float error_right_accum = 0;      // this is the accumulated velocity error in m/s

  cur_time = millis();                   // get the current time in miliseconds



  newBalanceUpdate();                    // run the sensor updates. this function checks if it has been 10 ms since the previous 
  
  if(angle > 3000 || angle < -3000)      // If angle is not within +- 3 degrees, reset counter that waits for start
  {
    start_counter = 0;
  }

  bool shouldPrint = cur_time - prev_print_time > 105;
  if(shouldPrint)   // do the printing every 105 ms. Don't want to do it for an integer multiple of 10ms to not hog the processor
  {
        Serial.print(angle_rad);  
        Serial.print("\t");  
        Serial.print(angle_rad_accum);  
        Serial.print("\t");
        Serial.print(leftMotorPWM);
        Serial.print("\t");
        Serial.print(rightMotorPWM);
        Serial.print("\t");
        Serial.print(vL);
        Serial.print("\t");
        Serial.print(vR);
        Serial.print("\t");
        Serial.print(totalDistanceLeft);
        Serial.print("\t");
        Serial.println(totalDistanceRight);

        prev_print_time = cur_time;
/* Uncomment this and comment the above if doing wireless
        Serial1.print(angle_rad);  
        Serial1.print("\t");
        Serial1.print(angle_rad_accum);  
        Serial1.print("\t");
        Serial1.print(PWM_left);
        Serial1.print("\t");
        Serial1.print(PWM_right);
        Serial1.print("\t");
        Serial1.print(vL);
        Serial1.print("\t");
        Serial1.println(vR);
       */
  }

  float delta_t = (cur_time - prev_time)/1000.0;

  // handle the case where this is the first time through the loop
  if (prev_time == 0) {
    delta_t = 0.01;
  }
  
  // every UPDATE_TIME_MS, check if angle is within +- 3 degrees and we haven't set the start flag yet
  if(cur_time - prev_time > UPDATE_TIME_MS && angle > -3000 && angle < 3000 && !armed_flag && !start_flag)   
  {
    // increment the start counter
    start_counter++;
    // If the start counter is greater than 30, this means that the angle has been within +- 3 degrees for 0.3 seconds, then set the start_flag
    if(start_counter > 30)
    {
      armed_flag = 1;
      buzzer.playFrequency(DIV_BY_10 | 445, 1000, 15);
    }
  }

  // angle is in millidegrees, convert it to radians and subtract the desired theta
  angle_rad = ((float)angle)/1000/180*3.14159;

  // only start when the angle falls outside of the 3.0 degree band around 0.  This allows you to let go of the
  // robot before it starts balancing
  if(cur_time - prev_time > UPDATE_TIME_MS && (angle < -3000 || angle > 3000) && armed_flag)   
  {
    start_flag = 1;
    armed_flag = 0;
  }

  // every UPDATE_TIME_MS, if the start_flag has been set, do the balancing
  if(cur_time - prev_time > UPDATE_TIME_MS && start_flag)
  {
    // set the previous time to the current time for the next run through the loop
    prev_time = cur_time;

    // speedLeft and speedRight are just the change in the encoder readings
    // wee need to do some math to get them into m/s
    vL = METERS_PER_CLICK*speedLeft/delta_t;
    vR = METERS_PER_CLICK*speedRight/delta_t;

    totalDistanceLeft = METERS_PER_CLICK*distanceLeft;
    totalDistanceRight = METERS_PER_CLICK*distanceRight;
    angle_rad_accum += angle_rad*delta_t;

    updatePWMs(totalDistanceLeft, totalDistanceRight, vL, vR, angle_rad, angle_rad_accum, delta_t);

    // if the robot is more than 45 degrees, shut down the motor
    if(false && start_flag && fabs(angle_rad) > FORTY_FIVE_DEGREES_IN_RADIANS)
    {
      // reset the accumulated errors here
      angle_rad_accum = 0.0;
      start_flag = 0;   /// wait for restart
      prev_time = 0;
      motors.setSpeeds(0, 0);
    } else if(start_flag) {
      motors.setSpeeds((int)leftMotorPWM, (int)rightMotorPWM);
    }
  }

  // kill switch
  if (buttonA.getSingleDebouncedPress())
  {
      motors.setSpeeds(0,0);
      while(!buttonA.getSingleDebouncedPress());
  }
}

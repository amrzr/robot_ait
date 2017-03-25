/**************************************
 * Case Study Project
 * Service Robot by 
 * TH Koeln 
 * Automation and IT
 * Authors:
 *           Atanu Mazumdar
 *           Kihoon Lee
 *           Eray Evran
 * Guided By:
 *           Prof. Dr. Elena Algorri
 **************************************/
#if (ARDUINO >= 100)
 #include <Arduino.h>
#else
 #include <WProgram.h>
#endif

//ROS headers
#include <ros.h>
#include <geometry_msgs/Vector3Stamped.h>
#include <geometry_msgs/Twist.h>
#include <sensor_msgs/Imu.h>
#include <ros/time.h>
#include "robot_specs.h"

//MPU9050 headers 
#include "quaternionFilters.h"
#include "MPU9250.h"
#define MPU9250_ADDRESS  0x68

// Encoder Pin Settings
#define encodPinA1      3     // encoder A pin
#define encodPinB1      8     // encoder B pin
#define encodPinA2      18
#define encodPinB2      7

#define LOOPTIME        100   // PID loop time(ms)
#define SMOOTH      10
#define LED_PIN 13
#define sign(x) (x > 0) - (x < 0)

unsigned long lastMilli = 0;       // loop timing 
unsigned long lastMilliPub = 0;
double rpm_req1 = 0;
double rpm_req2 = 0;
double rpm_act1 = 0;
double rpm_act2 = 0;
double rpm_req1_smoothed = 0;
double rpm_req2_smoothed = 0;
boolean direction1 = true;
boolean direction2 = true;
int prev_direction1 = 2;
int prev_direction2 = 2;
int PWM_val1 = 0;
int PWM_val2 = 0;
int temp_PWM_val1=0;
int temp_PWM_val2=0;
volatile long count1 = 0;          // rev counter
volatile long count2 = 0;
long countAnt1 = 0;
long countAnt2 = 0;
int16_t ax, ay, az;
float gx, gy, gz;
// PID Constants
float Kp =   0.2;
float Kd =   0;
float Ki =   0.01;
MPU9250 myIMU;

bool blinkState = false;
ros::NodeHandle nh;

void I2CwriteByte(uint8_t Address, uint8_t Register, uint8_t Data)
{
  // Set register address
  Wire.beginTransmission(Address);
  Wire.write(Register);
  Wire.write(Data);
  Wire.endTransmission();
}

void handle_cmd( const geometry_msgs::Twist& cmd_msg) {
  double x = cmd_msg.linear.x;
  double z = cmd_msg.angular.z;
  
  if (z == 0) {     // go straight
    // convert m/s to rpm
    rpm_req1 = x*60*(-1)/(pi*wheel_diameter);
    rpm_req2 = rpm_req1;
  }
  else if (x == 0) {
    // convert rad/s to rpm
    rpm_req1 = z*(-1)*track_width*60/(wheel_diameter*pi*2);
    rpm_req2 = -rpm_req1;
  }
  else {
    rpm_req2 = x*(-1)*60/(pi*wheel_diameter)-z*track_width*60/(wheel_diameter*pi*2);
    rpm_req1 = x*(-1)*60/(pi*wheel_diameter)+z*track_width*60/(wheel_diameter*pi*2);
  }
}

// ROS Message Topics declaration
ros::Subscriber<geometry_msgs::Twist> sub("cmd_vel", handle_cmd);
geometry_msgs::Vector3Stamped rpm_msg;
geometry_msgs::Vector3Stamped imu_msg;
//sensor_msgs::Imu imu_msg;
ros::Publisher rpm_pub("rpm", &rpm_msg);
ros::Publisher imu_pub("imu", &imu_msg);
ros::Time current_time;
ros::Time last_time;

char frameid[]="imu";

void encoder1() {
  if (digitalRead(encodPinA1) == digitalRead(encodPinB1)) count1++;
  else count1--;
}
void encoder2() {
  if (digitalRead(encodPinA2) == digitalRead(encodPinB2)) count2--;
  else count2++;
}

// Set the motor index, direction, and speed
// Motor index should either be a 0 or 1
// Direction should be either true for forward or false for backwards
// Speed should range between 0 and 127 (inclusivly)
void SetSpeed(int MotorIndex, boolean Forward, int Speed)
{
  // Validate motor index
  if(MotorIndex < 0 || MotorIndex > 2)
    return;
  // Validate speed
  if(Speed < 0)
    Speed = 0;
  else if(Speed > 127)
    Speed = 127;
  // Send the "set" command based on the motor
  // Note that we do not accelerate to the
  // speed, we just instantly set it
  unsigned char SendByte = 0;
  if(MotorIndex == 0)
    SendByte = 0xC2;
  else if(MotorIndex == 1)
    SendByte = 0xCA;
  else if(MotorIndex == 2)
    SendByte = 0xF0;
  // If we go backwards, the commands are the same
  // but minus one
  if(!Forward)
    SendByte--;
  // Send the set speed command byte
  Serial2.write(SendByte);
  // Send the speed data byte
  Serial2.write(Speed);  
}

void setup() 
{
  Serial2.begin(19200); //Serial interface for Motor Driver
  Wire.begin(); //I2C begin
  count1 = 0;
  count2 = 0;
  countAnt1 = 0;
  countAnt2 = 0;
  rpm_req1 = 0;
  rpm_req2 = 0;
  rpm_act1 = 0;
  rpm_act2 = 0;
  PWM_val1 = 0;
  PWM_val2 = 0;
  temp_PWM_val1=0;
  temp_PWM_val2=0;
  nh.getHardware()->setBaud(115200);
  nh.initNode(); 
  nh.subscribe(sub);
  nh.advertise(rpm_pub);
  nh.advertise(imu_pub);
  imu_msg.header.frame_id=frameid;
  pinMode(encodPinA1, INPUT); 
  pinMode(encodPinB1, INPUT); 
  digitalWrite(encodPinA1, HIGH);                // turn on pullup resistor
  digitalWrite(encodPinB1, HIGH);
  attachInterrupt(1, encoder1, RISING);
  pinMode(encodPinA2, INPUT); 
  pinMode(encodPinB2, INPUT); 
  digitalWrite(encodPinA2, HIGH);                // turn on pullup resistor
  digitalWrite(encodPinB2, HIGH);
  attachInterrupt(5, encoder2, RISING);
  SetSpeed(0, true, 0);
  SetSpeed(1, true, 0); 
  // MPU Setup
  I2CwriteByte(MPU9250_ADDRESS,26,0x08);
  myIMU.calibrateMPU9250(myIMU.gyroBias, myIMU.accelBias); 
  myIMU.initMPU9250();
  myIMU.initAK8963(myIMU.factoryMagCalibration);
  myIMU.getAres();
  myIMU.getGres();
  myIMU.getMres();
  myIMU.magBias[0]=195.69;//165.72;
  myIMU.magBias[1]=42.17;//-110.7;
  myIMU.magBias[2]=-366.88;//-200.43;
  myIMU.magScale[0]=0.73;//0.95;
  myIMU.magScale[1]=1.26;//0.94;
  myIMU.magScale[2]=1.21;//1.13; 
}
 

void loop() {
  nh.spinOnce();
  unsigned long timex = millis();
  if(timex-lastMilli>= LOOPTIME)   {      // enter timed loop
    getMotorData(timex-lastMilli);
    PWM_val1 = updatePid(1, PWM_val1, rpm_req1, rpm_act1);
    PWM_val2 = updatePid(2, PWM_val2, rpm_req2, rpm_act2);
    if(temp_PWM_val1!=PWM_val1 || temp_PWM_val2!=PWM_val2)
    {
    if(PWM_val1 > 0) direction1 = true;
    else if(PWM_val1 < 0) direction1 = false;
    if (rpm_req1 == 0) temp_PWM_val1=0;
    if(PWM_val2 > 0) direction2 = true;
    else if(PWM_val2 < 0) direction2 = false;
    if (rpm_req2 == 0) temp_PWM_val2=0;
    temp_PWM_val1=PWM_val1;
    temp_PWM_val2=PWM_val2;
    SetSpeed(0, direction1, abs(temp_PWM_val1));
    SetSpeed(1, direction2, abs(temp_PWM_val2));
    }
    publishRPM(timex-lastMilli);
    publishACGY();
    lastMilli = timex;
  }
  if(timex-lastMilliPub >= LOOPTIME) {
    publishRPM(timex-lastMilliPub);
    lastMilliPub = timex;
  }
}

void getMotorData(unsigned long timex)  {
 rpm_act1 = double((count1-countAnt1)*60*1000)/double(timex*encoder_pulse*gear_ratio);
 rpm_act2 = double((count2-countAnt2)*60*1000)/double(timex*encoder_pulse*gear_ratio);
 countAnt1 = count1;
 countAnt2 = count2;
}

int updatePid(int id, int command, double targetValue, double currentValue) {
  double pidTerm = 0;                            // PID correction
  double error = 0;
  double new_pwm = 0;
  double new_cmd = 0;
  static double last_error1 = 0;
  static double last_error2 = 0;
  static double int_error1 = 0;
  static double int_error2 = 0;
  
  error = targetValue-currentValue;
  if (id == 1) {
    int_error1 += error;
    pidTerm = Kp*error + Kd*(error-last_error1) + Ki*int_error1;
    last_error1 = error;
  }
  else {
    int_error2 += error;
    pidTerm = Kp*error + Kd*(error-last_error2) + Ki*int_error2;
    last_error2 = error;
  }
  new_pwm = constrain(double(command)*MAX_RPM/127.0 + pidTerm, -MAX_RPM, MAX_RPM);
  new_cmd = 127.0*new_pwm/MAX_RPM;
  return int(new_cmd);
}

void publishRPM(unsigned long timex) {
  rpm_msg.header.stamp = nh.now();
  rpm_msg.vector.x = -rpm_act1;
  rpm_msg.vector.y = -rpm_act2;
  rpm_msg.vector.z = double(timex)/1000;
  rpm_pub.publish(&rpm_msg);
  nh.spinOnce();
}

void publishACGY() {
  if (myIMU.readByte(MPU9250_ADDRESS, INT_STATUS) & 0x01)
  {
    myIMU.readAccelData(myIMU.accelCount);  // Read the x/y/z adc values  
    myIMU.ax = (float)myIMU.accelCount[0] * myIMU.aRes; // - myIMU.accelBias[0];
    myIMU.ay = (float)myIMU.accelCount[1] * myIMU.aRes; // - myIMU.accelBias[1];
    myIMU.az = (float)myIMU.accelCount[2] * myIMU.aRes; // - myIMU.accelBias[2];
    myIMU.readGyroData(myIMU.gyroCount);
    myIMU.gx = (float)myIMU.gyroCount[0] * myIMU.gRes;
    myIMU.gy = (float)myIMU.gyroCount[1] * myIMU.gRes;
    myIMU.gz = (float)myIMU.gyroCount[2] * myIMU.gRes;
    myIMU.readMagData(myIMU.magCount);  // Read the x/y/z adc values
    myIMU.mx = (float)myIMU.magCount[0] * myIMU.mRes * myIMU.factoryMagCalibration[0] - myIMU.magBias[0];
    myIMU.my = (float)myIMU.magCount[1] * myIMU.mRes * myIMU.factoryMagCalibration[1] - myIMU.magBias[1];
    myIMU.mz = (float)myIMU.magCount[2] * myIMU.mRes * myIMU.factoryMagCalibration[2] - myIMU.magBias[2];
  }
  
  myIMU.updateTime();
  MahonyQuaternionUpdate(myIMU.ax, myIMU.ay, myIMU.az, myIMU.gx * DEG_TO_RAD,
                         myIMU.gy * DEG_TO_RAD, myIMU.gz * DEG_TO_RAD, myIMU.my,
                         myIMU.mx, myIMU.mz, myIMU.deltat);
  myIMU.yaw   = atan2(2.0f * (*(getQ()+1) * *(getQ()+2) + *getQ()
                    * *(getQ()+3)), *getQ() * *getQ() + *(getQ()+1)
                    * *(getQ()+1) - *(getQ()+2) * *(getQ()+2) - *(getQ()+3)
                    * *(getQ()+3));
  myIMU.pitch = -asin(2.0f * (*(getQ()+1) * *(getQ()+3) - *getQ()
                    * *(getQ()+2)));
  myIMU.roll  = atan2(2.0f * (*getQ() * *(getQ()+1) + *(getQ()+2)
                    * *(getQ()+3)), *getQ() * *getQ() - *(getQ()+1)
                    * *(getQ()+1) - *(getQ()+2) * *(getQ()+2) + *(getQ()+3)
                    * *(getQ()+3));
  myIMU.pitch *= RAD_TO_DEG;
  myIMU.yaw   *= RAD_TO_DEG;
  myIMU.yaw  -= 1.66;
  myIMU.roll *= RAD_TO_DEG;
  gx=myIMU.roll;
  gy=myIMU.pitch;
  gz=myIMU.yaw+360;    
  imu_msg.header.stamp=nh.now(); 
  /*imu_msg.orientation.x=gx; 
   * imu_msg.orientation.y=gy; 
   * imu_msg.orientation.z=gz; 
   * imu_msg.linear_acceleration.x=ax; 
   * imu_msg.linear_acceleration.y=ay;
   imu_msg.linear_acceleration.z=az;*/
  imu_msg.vector.x=gx;
  imu_msg.vector.y=gy;
  imu_msg.vector.z=gz;
  imu_pub.publish(&imu_msg);
  nh.spinOnce();
}




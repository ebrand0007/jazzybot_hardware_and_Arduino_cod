/*Changelog: 
 *   Jul 16 2017 -        Addded encoder ros publisher
 *   Jul 23 2017 -        Added jointstate ros publiser
 *   Aug 26 2017 -        Forked codebase to new/simpler implentation. 
 *     -Goal: get Arduino to process interupts faster, and only publish minimal encoder and raw_vel_cmd topics
 *     - added jazzy_base_arduino_code/jazzy_base_arduino-v2.x sub dir to work on new code base
 *     - Last compatiable codebase for linobot/jbot was git tag 2017-Jul-23-v1.01 
 *     -   using jazzy_base_arduino_code/jazzy_base_arduino subdir
 *     - Codechange: 
 *     -   drive_robot_raw_callback( int pwm_left, int left_timeout, int pwm_right, int right_timeout
 *     -       timeout params are oh-crap timeout parms to stop motors after timeout is exceed, to stop motors after certain after a threashold, 
 *     -       in event serial connection lost sync
 *  Sept 3 2017: - Added -255->255 max/min values to drive_robot_raw_callback
 *               - version 2017-Aug-26-v2.1.1
 *               - Removed excessive debug logging in drive_robot_raw_callback & drive_robot
 *  Oct 20 2017: - added delay(30) after spinOnce() in main loop
 *               - version 2017-10-20-v2.1.1
 *   
*/

//Version
const String firmware_version = "Arduino Firmware Version: jazzy_base_arduino-v2.x/2017-10-20-v2.1.1";
#define DEBUG 0

#include <Wire.h>
#if (ARDUINO >= 100)
#include <Arduino.h>
#else
#include <WProgram.h>
#endif

//Version of code
#define ARDUINO_CODE_VERSION 

//arduino intrupt pins:
//https://www.arduino.cc/en/Reference/AttachInterrupt
//Board                                   Digital Pins Usable For Interrupts
//Uno, Nano, Mini, other 328-based        2, 3
//Mega, Mega2560, MegaADK                 2, 3, 18, 19, 20, 21
//Micro, Leonardo, other 32u4-based       0, 1, 2, 3, 7
//Zero                                    all digital pins, except 4
//MKR1000 Rev.1                           0, 1, 4, 5, 6, 7, 8, 9, A1, A2
//Due                                     all digital pins
//teensey                                 all digital pins
//don't change this if you followed the schematic diagram
//ENCODER PINS
#define left_encoder_a 19 //interupt for left encoder
#define left_encoder_b 18 //interupt for left encoder
#define right_encoder_a 3 //interupt for right encoder 
#define right_encoder_b 2 //interupt for right encoder

//don't change this if you followed the schematic diagram
//MOTOR PINS
//Left Motor
#define left_motor_direction 7
#define left_motor_pwm 8
//Right Motor
#define right_motor_direction 9
#define right_motor_pwm 10

/* TODO: Striving to make this compatible with ros_control
 * http://wiki.ros.org/ros_control#Examples
*/

//for dtostr() string handeling
#include <stdlib.h>

//Note: Must export CPATH="./libraries/ros_lib" to use these local libraries"
#include <ros.h>
#include <ros/time.h>

//header for Encoder messages
#include <ros_arduino_msgs/Encoders.h>
//header file for drive_robot_raw_callback subscriber
#include "jbot2_msgs/jbot2_pwm.h"
//header files for imu
#include <geometry_msgs/Vector3.h>
#include "imu/imu_configuration.h"
#include <ros_arduino_msgs/RawImu.h>



#if defined(WIRE_T3)
#include <i2c_t3.h>
#else
#include <Wire.h>
#endif


//Pick one option below to use interupts or not
#define ENCODER_OPTIMIZE_INTERRUPTS
//#define ENCODER_DO_NOT_USE_INTERRUPTS
#include "Encoder/Encoder.h"

#define IMU_PUBLISH_RATE 10 //hz
#define ENCODER_RATE 10 //hz

//create an Encoder object
Encoder left_encoder(left_encoder_a, left_encoder_b);
Encoder right_encoder(right_encoder_a, right_encoder_b);

//function prototypes
void check_imu();
void publish_imu();

//callback function prototypes for subscribers
void drive_robot_raw_callback (const jbot2_msgs::jbot2_pwm& jbot2_pwm_msg);

unsigned long previous_drive_robot_raw_callback_time =0; //timer raw_command_callback timeout threashold
unsigned long previous_imu_time = 0;
unsigned long previous_encoder_time = 0;
unsigned long drive_robot_motor_timeout=0; //time in millisec at which set motor pwm  signals to zero. Reset when drive_robot_raw_callback drive_robot is called

bool is_first = true;

char buffer[50]; //For ROS log info

ros::NodeHandle nh;
//ros::NodeHandle nh_private_("~");

// ROS subscriber msgs
//ROS raw wheel pwm subsciber messages
ros::Subscriber<jbot2_msgs::jbot2_pwm> sub_drive_robot_raw("raw_pwm",drive_robot_raw_callback);

// ROS imu publishers msgs
ros_arduino_msgs::RawImu raw_imu_msg;
//raw_imu_msg.header.frame_id = "linoimu_link";
ros::Publisher raw_imu_pub("raw_imu", &raw_imu_msg);

// ROS encoder publishers msgs
ros_arduino_msgs::Encoders encoders_msg;
ros::Publisher pub_encoders("encoders", &encoders_msg);

void setup()
{
  pinMode(left_motor_pwm, OUTPUT);
  pinMode(left_motor_direction, OUTPUT);
  pinMode(right_motor_pwm, OUTPUT);
  pinMode(right_motor_direction, OUTPUT);
  //STOP Motors
  analogWrite(right_motor_pwm, 0);
  analogWrite(left_motor_pwm, 0);
  
  nh.initNode();
  nh.getHardware()->setBaud(57600);
  
  //POS subscribers
  //TOD0: delete nh.subscribe(pid_sub);
  //TODO: delete nh.subscribe(cmd_sub);
  //Ros topoic for Raw pwm signasls to drive motors
  nh.subscribe(sub_drive_robot_raw);

  //ROS publishers
  //TODO: delete nh.advertise(raw_vel_pub);
  nh.advertise(raw_imu_pub);

  
  //Raw Encoder Tick Messages from hardware
  encoders_msg.header.frame_id = "lino_wheelencoders";
  nh.advertise(pub_encoders);

  
  while (!nh.connected())
  {
    nh.spinOnce();
  }
  nh.loginfo("Connected to microcontroller...");
  nh.loginfo("ROS Arduino IMU started.");
  nh.loginfo(firmware_version.c_str());
  
#if defined(WIRE_T3)
  Wire.begin(I2C_MASTER, 0x00, I2C_PINS_18_19, I2C_PULLUP_INT, I2C_RATE_400);
#else
  Wire.begin();
#endif
  delay(5);
  
  drive_robot_motor_timeout=millis(); //set motor timeout to now()
}

void loop()
{

  //this block stops the motor when no command is received //TODO: modify this to work with drive_robot_raw_callback
  //if ((millis() - previous_command_time) >= 400)
  if ( millis() > drive_robot_motor_timeout ) 
  {
    //STOP Motors
    //TODO: delete: drive_robot(left_motor.pwm, right_motor.pwm);  //TODO: should these be zero??
    analogWrite(right_motor_pwm, 0);
    analogWrite(left_motor_pwm, 0);  
  }


  //this block publishes the IMU data based on defined rate
  if ((millis() - previous_imu_time) >= (1000 / IMU_PUBLISH_RATE))
  {
    //sanity check if the IMU exits
    if (is_first)
    {
    check_imu();
    }
    else
    {
    //publish the IMU data
    publish_imu();
    }
    previous_imu_time = millis();
  }
  
  //this block publishes the encoder readings. change DEBUG to 0 if you don't want to display debugging
  if ((millis() - previous_encoder_time) >= (1000 / ENCODER_RATE))
  {

      if(DEBUG)
      {
      sprintf (buffer, "Encoded Right: %d", right_encoder.read());
      nh.logdebug(buffer);
      sprintf (buffer, "Encoder Left: %d", left_encoder.read());
      nh.loginfo(buffer);
      //TODO: change above to ns.logdebug - http://wiki.ros.org/rosserial/Overview/Logging
      }
      encoders_msg.left = left_encoder.read();
      encoders_msg.right = right_encoder.read();
      encoders_msg.header.stamp = nh.now();
      pub_encoders.publish(&encoders_msg);    
      previous_encoder_time = millis();
  }
   
  //call all the callbacks waiting to be called
  nh.spinOnce();
  //Pause for a bit
  delay(30); //in milli sec; 10hz ros topic update cycle = 100 millisec, so stay below that
}



/* call back for wringing rew pwm signals to motors
 *  msg format:
 *   pwm_left, pwm_right:  raw int -255 to 255  values to send to to pins
 *   duration:    :        milliseconds timeout threshhold(oh-crap) to reset motors back to 0 if now new pwm values are recieved
 * 
*/
void drive_robot_raw_callback(const jbot2_msgs::jbot2_pwm& jbot2_pwm_msg)
{
  //this functions spins the left and right wheel based on a defined speed in PWM  
  //change left motor direction
  int pwm_left=jbot2_pwm_msg.left_pwm;
  int pwm_right=jbot2_pwm_msg.right_pwm; 
  int duration=jbot2_pwm_msg.duration;  // in milli sec 


  //Debugging
  /*sprintf (buffer, "  Recieved left_pwm: %d", pwm_left);
    nh.loginfo(buffer);  
    sprintf (buffer, "  Recieved right_pwm: %d", pwm_right);
    nh.loginfo(buffer);
  */ 
  
  previous_drive_robot_raw_callback_time=millis();
  //Time at which to reset motors in milisec - ohcrap situation
  drive_robot_motor_timeout=previous_drive_robot_raw_callback_time+duration;   
  
  
  //forward
  if (pwm_left >= 0)
  {
    //set max
    if ( pwm_left > 255 ) 
      pwm_left=255;
    digitalWrite(left_motor_direction, HIGH);
  }
  //reverse
  else
  {
    //set min
    if ( pwm_left < -255 ) 
      pwm_left=-255;   
    digitalWrite(left_motor_direction, LOW);
  }
  //spin the motor
  analogWrite(left_motor_pwm, abs(pwm_left));
  
  //change right motor direction
  //forward
  if (pwm_right >= 0)
  {
    //set max
    if ( pwm_right > 255 ) 
      pwm_right=255;
    digitalWrite(right_motor_direction, HIGH);
  }
  //reverse
  else
  {
    //set min
    if ( pwm_right < -255 ) 
      pwm_right=-255;
    digitalWrite(right_motor_direction, LOW);
  }
  //spin the motor
  analogWrite(right_motor_pwm, abs(pwm_right));
}

void check_imu()
{
  //this function checks if IMU is present
  raw_imu_msg.accelerometer = check_accelerometer();
  raw_imu_msg.gyroscope = check_gyroscope();
  raw_imu_msg.magnetometer = check_magnetometer();

  if (!raw_imu_msg.accelerometer)
  {
    nh.logerror("Accelerometer NOT FOUND!");
  }

  if (!raw_imu_msg.gyroscope)
  {
    nh.logerror("Gyroscope NOT FOUND!");
  }

  if (!raw_imu_msg.magnetometer)
  {
    nh.logerror("Magnetometer NOT FOUND!");
  }
  
  is_first = false;
}

void publish_imu()
{
  //this function publishes raw IMU reading
  raw_imu_msg.header.stamp = nh.now();
  raw_imu_msg.header.frame_id = "imu_link"; //TODO: this should be a param, not hard set
  //measure accelerometer
  if (raw_imu_msg.accelerometer)
  {
    measure_acceleration();
    raw_imu_msg.raw_linear_acceleration = raw_acceleration;
  }

  //measure gyroscope
  if (raw_imu_msg.gyroscope)
  {
    measure_gyroscope();
    raw_imu_msg.raw_angular_velocity = raw_rotation;
  }
  
  //measure magnetometer
  if (raw_imu_msg.magnetometer)
  {
    measure_magnetometer();
    raw_imu_msg.raw_magnetic_field = raw_magnetic_field;
  }
  //publish raw_imu_msg object to ROS
  raw_imu_pub.publish(&raw_imu_msg);
}


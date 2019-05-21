// Car control node for ROS
// Jon Eivind Stranden 2019 @ NTNU
// Part of Master thesis - Self-driving truck with wireless charging (SINTEF 2019)

#include <PWMServo.h>
#include <Wire.h>
#include <OctoWS2811.h>
#include "SparkFun_BNO080_Arduino_Library.h"
#include <ros.h>
#include <ros/time.h>
#include <sensor_msgs/Imu.h>
#include <geometry_msgs/Twist.h>
#include <std_msgs/Int16.h>

//// IMU ////
BNO080 IMU;
uint32_t last_time = 0;

//// Servo ////
PWMServo steering_servo, throttle_servo, gear_servo;  // Servo objects

// Define PWM pins
int steering_servo_pin = 20;
int throttle_servo_pin = 21;
int gear_servo_pin = 22;

// Variables for storing servo positions
int steering_pos = 90; 
int throttle_pos = 90;
int gear_pos = 90; 

//// LED STRIP ////
// Note: led strip is connected to pin2
const int num_leds = 4;
int led_mode = 0;

// LED colors
#define RED    0xFF0000
#define GREEN  0x00FF00
#define BLUE   0x0000FF
#define YELLOW 0xFFFF00
#define PINK   0xFF1088
#define ORANGE 0xE05800
#define WHITE  0xFFFFFF
#define BLACK  0x000000

DMAMEM int displayMemory[num_leds*6];
int drawingMemory[num_leds*6];
const int config = WS2811_GRB | WS2811_800kHz;
OctoWS2811 leds(num_leds, displayMemory, drawingMemory, config);

float long_loop_timer = millis();
float short_loop_timer = millis();
boolean long_time_passed = false;
boolean short_time_passed = false;

// For looping LED sequences
void led_control(int mode){
  if((millis() - long_loop_timer) >= 1000.0){
    long_loop_timer = millis();
    long_time_passed = !long_time_passed;
  }
  if((millis() - short_loop_timer) >= 100.0){
    short_loop_timer = millis();
    short_time_passed = !short_time_passed;
  }
  // Off
  if(mode==0){
    for (int i=0; i < leds.numPixels(); i++) {
      leds.setPixel(i, BLACK);
    }
    leds.show();
  } 
  // Green
  else if(mode==1){
    for (int i=0; i < leds.numPixels(); i++) {
      leds.setPixel(i, GREEN);  
    }
    leds.show();
  }
  else if(mode==2 && long_time_passed){
    for (int i=0; i < leds.numPixels(); i++) {
      leds.setPixel(i, GREEN);  
    }
    leds.show();
  } 
  else if(mode==2 && !long_time_passed && short_time_passed){
    for (int i=0; i < leds.numPixels(); i++) {
      leds.setPixel(i, BLACK);  
    }
    leds.show();
  } 
  // Blue
  else if(mode==3){
    for (int i=0; i < leds.numPixels(); i++) {
      leds.setPixel(i, BLUE);  
    }
    leds.show();
  }
  else if(mode==4 && long_time_passed){
    for (int i=0; i < leds.numPixels(); i++) {
      leds.setPixel(i, BLUE);  
    }
    leds.show();
  } 
  else if(mode==4 && !long_time_passed && short_time_passed){
    for (int i=0; i < leds.numPixels(); i++) {
      leds.setPixel(i, BLACK);  
    }
    leds.show();
  } 
  // Red
  else if(mode==5){
    for (int i=0; i < leds.numPixels(); i++) {
      leds.setPixel(i, RED);  
    }
    leds.show();
  }   
  else if(mode==6 && long_time_passed){
    for (int i=0; i < leds.numPixels(); i++) {
      leds.setPixel(i, RED);  
    }
    leds.show();
  } 
  else if(mode==6 && !long_time_passed && short_time_passed){
    for (int i=0; i < leds.numPixels(); i++) {
      leds.setPixel(i, BLACK);  
    }
    leds.show();
  }   
  long_time_passed = false;
  short_time_passed = false;
}


//// ROS ////
ros::NodeHandle nh;
ros::Time last_heartbeat_received;
bool is_jetson_running = true;

// ROS callback
void controlCallback(const geometry_msgs::Twist& twist_msg){
  steering_pos  = twist_msg.angular.z;
  throttle_pos = twist_msg.linear.x;
  gear_pos = twist_msg.linear.z;
}

void heartbeatCallback(const std_msgs::Int16& Int16_msg){
  last_heartbeat_received = nh.now();
}

void ledCallback(const std_msgs::Int16& Int16_msg){
  led_mode = Int16_msg.data;
}

//Publisher
sensor_msgs::Imu imu_msg;
ros::Publisher pub_imu("imu_data", &imu_msg);

//Subscriptions
ros::Subscriber<geometry_msgs::Twist> sub_twist("car_cmd", &controlCallback );
ros::Subscriber<std_msgs::Int16> sub_heartbeat("heartbeat", &heartbeatCallback );
ros::Subscriber<std_msgs::Int16> sub_led("led_mode", &ledCallback );


void setup() {

  // Start leds
  leds.begin();

  //Start I2C
  Wire.begin();

  //Start IMU
  IMU.begin();

  delay(1000);

  Serial.begin(57600);

  //Increase I2C data rate to 400kHz
  Wire.setClock(400000); 

  //Send data update every 50ms
  IMU.enableRotationVector(50); 
  IMU.enableGyro(50);
  IMU.enableLinearAccelerometer(50);

  //Attach servos on PWM pins to servo objects with min/max values
  steering_servo.attach(steering_servo_pin, 1000, 2000);
  throttle_servo.attach(throttle_servo_pin, 1000, 2000);
  gear_servo.attach(gear_servo_pin, 1000, 2000);

  nh.initNode();
  nh.advertise(pub_imu);
  nh.subscribe(sub_twist);
  nh.subscribe(sub_heartbeat);
  nh.subscribe(sub_led);
  
}

void loop(){

  led_control(led_mode);
    
  if((nh.now().sec - last_heartbeat_received.sec) <= 1){ // Check if heartbeat has been received within the last sec 
    steering_servo.write(steering_pos);
    throttle_servo.write(throttle_pos);
    gear_servo.write(gear_pos);
  } 
  else {
    steering_servo.write(90);
    throttle_servo.write(90);
    gear_servo.write(90);
  }

  if (IMU.dataAvailable() == true && nh.connected() && millis() - last_time >= 50 ){

    last_time = millis();
    
    imu_msg.header.stamp = nh.now();
    
    imu_msg.orientation.x = IMU.getQuatI();
    imu_msg.orientation.y = IMU.getQuatJ();
    imu_msg.orientation.z = IMU.getQuatK();
    imu_msg.orientation.w = IMU.getQuatReal();

    imu_msg.angular_velocity.x = IMU.getGyroX();
    imu_msg.angular_velocity.y = IMU.getGyroY();
    imu_msg.angular_velocity.z = IMU.getGyroZ();    

    imu_msg.linear_acceleration.x = IMU.getLinAccelX();
    imu_msg.linear_acceleration.y = IMU.getLinAccelY();
    imu_msg.linear_acceleration.z = IMU.getLinAccelZ();
    
    //Serial.println(imu_msg.orientation.z);
    pub_imu.publish(&imu_msg);  
  }
  nh.spinOnce();
  delay(1);
}

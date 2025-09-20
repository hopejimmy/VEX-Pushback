#include "pros/abstract_motor.hpp"
#include "pros/imu.hpp"
#include "pros/rtos.hpp"
#include "lemlib/api.hpp"
#include "robot-config.h"
#include "cstdlib"
#include <sys/types.h>
#include <time.h>










//proportional correction 时间走直线
void runtime(float velo, float setpoint,int tt, float kp){
  timer_t t1;
  int st=1;
  float current_speed = velo;
  float min_speed = 15;
  int brake_start_ms = 300;  // 提前多少毫秒开始减速
  while(st){
      float theta=imu.get_rotation();
      float error=(setpoint-theta);
      float leftspeed=velo+error*kp;
      float rightspeed=velo-error*kp;
      int remaining_time = tt - t1;


          // 保持最小速度，避免过早停止
           if (fabs(current_speed) < min_speed) {
            current_speed = (velo > 0) ? min_speed : -min_speed;
           }
        

        // 限幅（确保速度在 -100~100 之间）
        if (leftspeed > 100) leftspeed = 100;
        else if (leftspeed < -100) leftspeed = -100;
        
        if (rightspeed > 100) rightspeed = 100;
        else if (rightspeed < -100) rightspeed = -100;
leftMotors.move_velocity(leftspeed*6);
rightMotors.move_velocity(rightspeed*6);
pros::delay(tt);
      st=0;
  }
  // 制动
leftMotors.set_brake_mode(pros::MotorBrake::brake);
rightMotors.set_brake_mode(pros::MotorBrake::brake);
leftMotors.brake();
rightMotors.brake();
}













void rundis(float velo, float setpoint, int dis, float kp){
leftMotors.set_zero_position(0);
rightMotors.set_zero_position(0);
  float theta=chassis.getPose().theta;
  float error = setpoint - theta;
  int dts=1;

  while(dts){
  float leftpos = leftMotors.get_position();
  float rightpos = rightMotors.get_position();
  float avg_encoder = (fabs(leftpos) + fabs(rightpos)) / 2;
    float remaining = dis - avg_encoder;
    
    // 动态减速
    float current_speed = velo;
    if(remaining <=(fabs(velo)*3)){
      if(current_speed >= velo) {
        current_speed = velo;}
      if(fabs(current_speed) <= 15){
       current_speed = (velo > 0) ? 15 : -15;  
      }      
      current_speed = velo * (remaining / (fabs(velo)*3));

    }
    float theta=chassis.getPose().theta;
    // P控制
    // float error = setpoint - theta;
    float leftspeed = current_speed + error * kp;
    float rightspeed = current_speed - error * kp;



    // 手动限幅
    if(leftspeed >= 100) {
      leftspeed = 100;}
    if(leftspeed <= -100) {
      leftspeed = -100;}

    if(rightspeed >= 100) {
      rightspeed = 100;}
    if(rightspeed <= -100){
      rightspeed = -100;} 

    // 驱动电机
leftMotors.move_velocity(leftspeed*6);
rightMotors.move_velocity(rightspeed*6);
    
    
    if(avg_encoder>= dis){
   dts=0;
    }
    else{
      dts=1;
    }
  }

  // 直接急停（原版停止方式）
leftMotors.set_brake_mode(pros::MotorBrake::brake);
rightMotors.set_brake_mode(pros::MotorBrake::brake);
leftMotors.brake();
rightMotors.brake();
}
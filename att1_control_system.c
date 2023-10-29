/*
 * File:          att1_control_system.c
 * Date:          22/10/23
 * Description:   Atividade 1
 * Author:        WebisD
 * Modifications:
 */

#include <webots/robot.h>
#include <webots/distance_sensor.h>
#include <webots/motor.h>
#include <stdio.h>
#include <math.h>

#define TIME_STEP 64
#define MAX_SPEED 6.4
#define SPEED 2
#define MIN_SPEED 0.9

void delay(int time_milisec) {
  double currentTime, initTime, Timeleft;
  double timeValue = (double)time_milisec/1000;
  initTime = wb_robot_get_time();
  Timeleft = 0.00;

  while (Timeleft < timeValue){
    currentTime = wb_robot_get_time();
    Timeleft=currentTime-initTime;
    wb_robot_step(TIME_STEP);
  }
}

int main(int argc, char **argv) {
  /* necessary to initialize webots stuff */
  wb_robot_init();
  
  //Set motors
  WbDeviceTag left_motor_front = wb_robot_get_device("front left wheel");
  WbDeviceTag right_motor_front = wb_robot_get_device("front right wheel");
  WbDeviceTag left_motor_back = wb_robot_get_device("back left wheel");
  WbDeviceTag right_motor_back = wb_robot_get_device("back right wheel");
  // Set motor to infinity mode
  wb_motor_set_position(left_motor_front, INFINITY);
  wb_motor_set_position(right_motor_front, INFINITY);
  wb_motor_set_position(left_motor_back, INFINITY);
  wb_motor_set_position(right_motor_back, INFINITY);
  
  double left_speed = SPEED;
  double right_speed = SPEED;
  
  wb_motor_set_velocity(left_motor_front, left_speed);
  wb_motor_set_velocity(right_motor_front, right_speed);
  wb_motor_set_velocity(left_motor_back, left_speed);
  wb_motor_set_velocity(right_motor_back, right_speed);
  
  // Get sensors
  WbDeviceTag ds_so3 = wb_robot_get_device("so3"); //front sensors
  WbDeviceTag ds_so4 = wb_robot_get_device("so4"); //front sensors
  WbDeviceTag ds_so5 = wb_robot_get_device("so5");
  WbDeviceTag ds_so6 = wb_robot_get_device("so6");
  WbDeviceTag ds_so7 = wb_robot_get_device("so7");
  // Enable sensors
  wb_distance_sensor_enable(ds_so3, TIME_STEP);
  wb_distance_sensor_enable(ds_so4, TIME_STEP);
  wb_distance_sensor_enable(ds_so5, TIME_STEP);
  wb_distance_sensor_enable(ds_so6, TIME_STEP);
  wb_distance_sensor_enable(ds_so7, TIME_STEP);

  
  double sens6_value, sens7_value, sens4_value, sens3_value, sens5_value, front_sensor_value;
  double wall_dist = 200.0;
  double error, dif_error, old_error, integral, power, sensor_value, dist, dist_front;
  error = integral = old_error = power = 0.0;
  
  double p_gain = 0.01;   // PID
  double i_gain = 0.001;  // algorithm
  double d_gain = 0.005;  // constants
  
  int begin=1, timestemp = 0; // Window controller for integral controller
  
  while (wb_robot_step(TIME_STEP) != -1) {
    if(begin) delay(8000);
    begin=0;
    printf("--------------------------------\n");
    
    if(timestemp == 10){ // Create new window for integral controller
      integral = 0.0;
      timestemp=0;
    }
 
    // Read the sensors
    sens6_value = wb_distance_sensor_get_value(ds_so6);
    sens7_value = wb_distance_sensor_get_value(ds_so7);
    sens4_value = wb_distance_sensor_get_value(ds_so4);
    sens3_value = wb_distance_sensor_get_value(ds_so3);
    sens5_value = wb_distance_sensor_get_value(ds_so5);
    
    // Get biggest value in front sensors
    front_sensor_value = sens4_value > sens3_value ? sens4_value : sens3_value;
    dist_front = (1024 - front_sensor_value);
    
    // Wall is ahead. Do a 90 degree turn
    if(dist_front<70){
        right_speed = SPEED;
        left_speed = -SPEED;
        wb_motor_set_velocity(left_motor_front, left_speed);
        wb_motor_set_velocity(right_motor_front, right_speed);
        wb_motor_set_velocity(left_motor_back, left_speed);
        wb_motor_set_velocity(right_motor_back, right_speed);
        delay(2000);
    }
    
    //Get biggest value in right sensors and change K_p accordingly
    if(sens5_value >= sens6_value && sens5_value >= sens7_value){
      sensor_value = sens5_value;
      p_gain = 0.03;
    }
    if(sens6_value >= sens5_value && sens6_value>=sens7_value){
      sensor_value = sens6_value;
      p_gain = 0.02;
    }
    else{
      sensor_value = sens7_value;
      p_gain = 0.01;
    }
    
    printf("p_gain: %f, i_gain: %f, d_gain: %f\n",p_gain,i_gain,d_gain);
    printf("S7: %f, S6: %f, S5: %f, S4: %f, S3: %f\n", (1024 - sens7_value), (1024 - sens6_value), (1024-sens5_value), (1024 - sens4_value), (1024 - sens3_value));
    
    // PID algorithm
    dist = (1024 - sensor_value);
    error = wall_dist - dist;
    
    integral = integral + error;
    dif_error= error - old_error;
    old_error = error;
    power = p_gain*error + i_gain*integral + d_gain*dif_error;
    
    printf("Dist: %f \t Error: %f \t Power: %f\n", dist, error, power);
    
    // Change velocity of the right wheel based on the result of the PID
    right_speed = SPEED + power;
    left_speed = SPEED;
    
    //Limit min and max speed
    if (right_speed < MIN_SPEED) right_speed = MIN_SPEED;
    if (right_speed > MAX_SPEED) right_speed = MAX_SPEED;
    
    
    printf("Left: %f  Right: %f\n", left_speed, right_speed);
    
    //Set the velocity of the motors wheels
    wb_motor_set_velocity(left_motor_front, left_speed);
    wb_motor_set_velocity(right_motor_front, right_speed);
    wb_motor_set_velocity(left_motor_back, left_speed);
    wb_motor_set_velocity(right_motor_back, right_speed);
    
    timestemp+=1;
    printf("--------------------------------\n");

  };

  /* This is necessary to cleanup webots resources */
  wb_robot_cleanup();

  return 0;
}

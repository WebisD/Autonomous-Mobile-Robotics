/*
 * File:          att4_mapping_system.c
 * Date:
 * Description:
 * Author:
 * Modifications:
 */

/*
 * You may need to add include files like <webots/distance_sensor.h> or
 * <webots/motor.h>, etc.
 */
#include <math.h>
#include <stdio.h>
#include <stdlib.h>
#include <time.h>
#include <string.h>
#include <webots/robot.h>
#include <webots/motor.h>
#include <webots/distance_sensor.h>
#include <webots/gps.h>
#include <webots/inertial_unit.h>

/*
 * You may want to add macros here.
 */
// maximal speed allowed
#define MAX_SPEED 5.24
#define TIME_STEP 64
#define SPEED 2
#define MAX_SENSOR_NUMBER 16
#define MAX_SENSOR_VALUE 1024
#define MIN_DISTANCE 1.0
#define WHEEL_WEIGHT_THRESHOLD 100
#define MATRIX_SIZE 20

// Obstaclle // Unknown // Free
#define BLACK 1
#define GRAY 0.5
#define WHITE 0

#define SENSOR_HIT 0.8
#define SENSOR_MISS 0.45

double mapM[MATRIX_SIZE+2][MATRIX_SIZE+2];

// structure to store the data associated to one sensor
typedef struct {
  WbDeviceTag device_tag;
  double wheel_weight[2];
} SensorData;

typedef struct {
    double x;
    double y;
} Point;

typedef struct {
    int x;
    int y;
} IntPoint;

IntPoint currentPos;

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

//Update sensor read based on robot rotation
Point rotatePoint(Point A, Point B, double angle) {
    Point C;
    double angle_rad = angle * M_PI / 180.0; // Convert angle to radians

    // Perform the rotation
    C.x = (B.x - A.x) * cos(angle_rad) - (B.y - A.y) * sin(angle_rad) + A.x;
    C.y = (B.x - A.x) * sin(angle_rad) + (B.y - A.y) * cos(angle_rad) + A.y;

    return C;
}

// enum to represent the state of the robot
typedef enum { FORWARD, LEFT, RIGHT } State;

// how much each sensor affects the direction of the robot
static SensorData sensors[MAX_SENSOR_NUMBER] = {
  {.wheel_weight = {150, 0}}, {.wheel_weight = {200, 0}}, {.wheel_weight = {300, 0}}, {.wheel_weight = {600, 0}},
  {.wheel_weight = {0, 600}}, {.wheel_weight = {0, 300}}, {.wheel_weight = {0, 200}}, {.wheel_weight = {0, 150}},
  {.wheel_weight = {0, 0}},   {.wheel_weight = {0, 0}},   {.wheel_weight = {0, 0}},   {.wheel_weight = {0, 0}},
  {.wheel_weight = {0, 0}},   {.wheel_weight = {0, 0}},   {.wheel_weight = {0, 0}},   {.wheel_weight = {0, 0}}};


// bresenham Line algorithm
void bresenhamLine(int x1, int y1, int x2, int y2, int steps) {
    int dx, dy, sx, sy, err, e2;
    //steps++;

    // Calculate differences and direction of movement
    dx = abs(x2 - x1);
    dy = abs(y2 - y1);
    sx = (x1 < x2) ? 1 : -1;
    sy = (y1 < y2) ? 1 : -1;

    // Initialize error
    err = dx - dy;
    
    bool obstacle = true;
    bool pass_one = false;

    while (steps--) {
        if(x1<=0 || y1<=0 || x1>=21 || y1>=21){
          //printf("(WALL)");
          break;
        }
    
        // Draw the current point
        //printf("(%d, %d) ", x1, y1);

        // Check if the end point is reached
        if (x1 == x2 && y1 == y2) {
            obstacle = false;
            break;
        }
        if(pass_one /*&& mapM[x1][y1]<=0.5*/){
          //printf("(%d %d): %.2f", x1, y1, mapM[x1][y1]);
          
          /*float priorLogOdds = mapM[x1][y1];
          float odds = exp(priorLogOdds);  // Convert from log odds to odds
          float probability = odds / (1 + odds);  // Convert from odds to probability
          float updatedProbability = probability * SENSOR_MISS;  // Update probability based on sensor measurement
          float updatedOdds = updatedProbability / (1 - updatedProbability);  // Convert back to odds*/
          float updatedOdds = SENSOR_MISS/(1 - SENSOR_MISS);
          
          if(updatedOdds != 0){
            mapM[x1][y1] += log(updatedOdds);  // Convert back to log odds  
          }
          //printf(" in %.2f\n", mapM[x1][y1]); 
        }
        pass_one = true;
        
        // Update error term and coordinates
        e2 = 2 * err;
        if (e2 > -dy) {
            err -= dy;
            x1 += sx;
        }
        if (e2 < dx) {
            err += dx;
            y1 += sy;
        }
    }
    
    if(obstacle){
      //float priorLogOdds = mapM[x1][y1];
      /*if(mapM[x1][y1] < 0){
        mapM[x1][y1] = 0.5;
      }*/
      /*if(mapM[x1][y1] > 87.0){
        priorLogOdds = 87.0;
      }*/
      //printf("(%d %d): %.2f", x1, y1, mapM[x1][y1]);
      
      /*float odds = exp(priorLogOdds);  // Convert from log odds to odds
      float probability = odds / (1 + odds);  // Convert from odds to probability
      float updatedProbability = probability * SENSOR_HIT;  // Update probability based on sensor measurement
      float updatedOdds = updatedProbability / (1 - updatedProbability);  // Convert back to odds*/
      float updatedOdds = SENSOR_HIT/(1 - SENSOR_HIT);
      if(updatedOdds != 0.0){
        mapM[x1][y1] += log(updatedOdds);  // Convert back to log odds  
      }
      //printf(" -> %.2f\n", mapM[x1][y1]);
    }
    else if (true /*&& mapM[x1][y1]<=0.5*/){
      /*float priorLogOdds = mapM[x1][y1];
      float odds = exp(priorLogOdds);  // Convert from log odds to odds
      float probability = odds / (1 + odds);  // Convert from odds to probability
      float updatedProbability = probability * SENSOR_MISS;  // Update probability based on sensor measurement
      float updatedOdds = updatedProbability / (1 - updatedProbability);  // Convert back to odds*/
      float updatedOdds = SENSOR_MISS/(1 - SENSOR_MISS);
      if(updatedOdds != 0){
        mapM[x1][y1] += log(updatedOdds);  // Convert back to log odds  
      }
    }

    //printf("\n");
}

// Final target of the sensor based on current robot position
int lut_sensors_dist[MAX_SENSOR_NUMBER][2] = {
  {0,-10}, {-7,-8}, {-9,-5}, {-10,-2}, {-10,2}, {-9,5}, {-7,8}, {0,10}, {0,10}, {7,8}, {9,5}, {10,2}, {10,-2}, {9,-5}, {7,-8}, {0,-10}
};

void occupancyGridMapping(int robotX, int robotY, double robotTheta, double distSensor[]){
  for(int i=0; i<MAX_SENSOR_NUMBER; i++){    
    if(i==15 || i==8){
      continue;
    }
    float roundedNumber = ceil(distSensor[i]);
    int firstDigit = (int)fabs(roundedNumber);  // Take absolute value to handle negative numbers
    while (firstDigit >= 10) {
        firstDigit /= 10;
    }
    /*if((i==1 || i==6 || i==9 || i==14) && firstDigit>1){
      firstDigit--;
    }*/
    if(distSensor[i] >= 990.0){
      firstDigit = 10;
    }
    int steps = firstDigit+1;
    
    Point roboPoint = {robotX, robotY};
    Point endPoint = {robotX+lut_sensors_dist[i][0], robotY+lut_sensors_dist[i][1]};
    endPoint = rotatePoint(roboPoint, endPoint, robotTheta);
    
    //printf("s0%d | (%d,%d) -> (%d, %d)", i, robotX, robotY, (int)round(endPoint.x), (int)round(endPoint.y));
    //printf("[%d]:\n", firstDigit);
    //bresenhamLine(robotX, robotY, robotX+lut_sensors_dist[i][0], robotY+lut_sensors_dist[i][1], steps);
    bresenhamLine(robotX, robotY, (int)round(endPoint.x), (int)round(endPoint.y), steps);
    
  }
}


int main(int argc, char **argv) {
  /* necessary to initialize webots stuff */
  wb_robot_init();
  
  
  //Set motors
  WbDeviceTag left_wheel = wb_robot_get_device("left wheel");
  WbDeviceTag right_wheel = wb_robot_get_device("right wheel");
  
  // Set motor to infinity mode and velocity to zero
  wb_motor_set_position(left_wheel, INFINITY);
  wb_motor_set_position(right_wheel, INFINITY);
  wb_motor_set_velocity(left_wheel, 0.0);
  wb_motor_set_velocity(right_wheel, 0.0);
  
  WbDeviceTag gps = wb_robot_get_device("gps");
  wb_gps_enable(gps, TIME_STEP);
  
  WbDeviceTag imu = wb_robot_get_device("inertial unit");
  wb_inertial_unit_enable(imu, TIME_STEP);

  char sensor_name[5] = "";
  double sensor_value;
  int i,j;
  
  for (i = 0; i < MAX_SENSOR_NUMBER; ++i) {
    sprintf(sensor_name, "so%d", i);
    sensors[i].device_tag = wb_robot_get_device(sensor_name);
    wb_distance_sensor_enable(sensors[i].device_tag, TIME_STEP);
  }
    
  double speed[2] = {0.0, 0.0};
  double wheel_weight_total[2] = {0.0, 0.0};
  double distance, speed_modifier, randomValue;
  
  double cell_size_x = 9.5 / MATRIX_SIZE;
  double cell_size_y = 9.5 / MATRIX_SIZE;
  int matrix_x,matrix_y, robotX, robotY;
  double robotTheta;
  
  
  // by default, the robot goes forward
  State state = FORWARD;
  
  // Seed the random number generator with the current time
  srand(time(NULL));
  
  //Initialize MAP with all gray
  for(i=0; i<MATRIX_SIZE+2; i++){
    for(j=0; j<MATRIX_SIZE+2; j++){
      mapM[i][j] = GRAY;
    }
  }
  
  FILE *file = fopen("mapas.csv", "w");
  int index = 0;
  /* main loop */
  while (wb_robot_step(TIME_STEP) != -1) {
    // initialize speed and wheel_weight_total arrays at the beginning of the loop
    memset(speed, 0, sizeof(double) * 2);
    memset(wheel_weight_total, 0, sizeof(double) * 2);
    
    const double *gps_values = wb_gps_get_values(gps);
    //printf("GPS position: %.3f %.3f %.3f\n", gps_values[0], gps_values[1], gps_values[2]);
    matrix_x = (MATRIX_SIZE) - (int)((gps_values[0] - (-4.65)) / cell_size_x);
    matrix_y = (MATRIX_SIZE) - (int)((gps_values[1] - (-4.65)) / cell_size_y);
    matrix_x = (matrix_x < 0) ? 0 : ((matrix_x > MATRIX_SIZE) ? MATRIX_SIZE : matrix_x);
    matrix_y = (matrix_y < 0) ? 0 : ((matrix_y > MATRIX_SIZE) ? MATRIX_SIZE : matrix_y);
    robotX = matrix_x;
    robotY = matrix_y;
    //printf("GPS position: %d %d\n", matrix_x,matrix_y);

    const double *values = wb_inertial_unit_get_roll_pitch_yaw(imu);
    robotTheta = values[2] * (180.0 / M_PI); // convert to degrees
    //printf("Angle: %f\n", robotTheta);
    
    double curr_sensor_read[MAX_SENSOR_NUMBER];
    
    for (i = 0; i < MAX_SENSOR_NUMBER; ++i) {
      sensor_value = wb_distance_sensor_get_value(sensors[i].device_tag);
      //printf("%d: %.2f  ", i, (1024-sensor_value));
      curr_sensor_read[i] = (1024-sensor_value);
      // if the sensor doesn't see anything, we don't use it for this round
      if (sensor_value == 0.0)
        speed_modifier = 0.0;
      else {
        // computes the actual distance to the obstacle, given the value returned by the sensor
        distance = 5.0 * (1.0 - (sensor_value / MAX_SENSOR_VALUE));  // lookup table inverse.

        // if the obstacle is close enough, we may want to turn
        // here we compute how much this sensor will influence the direction of the robot
        if (distance < MIN_DISTANCE)
          speed_modifier = 1 - (distance / MIN_DISTANCE);
        else
          speed_modifier = 0.0;
      }
      // add the modifier for both wheels
      for (j = 0; j < 2; ++j)
        wheel_weight_total[j] += sensors[i].wheel_weight[j] * speed_modifier;
    }//printf("\n\n");
   
    
    //printf("(%d, %d) (%d, %d)\n",currentPos.x, currentPos.y, robotX, robotY);
    if(state == FORWARD /*currentPos.x != robotX || currentPos.y != robotY*/){
      currentPos.x = robotX;
      currentPos.y = robotY;  
      //printf("%d\n", currentPos.x==robotX);    
      occupancyGridMapping(robotX, robotY, robotTheta, curr_sensor_read);
    }
    
    /*for (int i = 0; i < (MATRIX_SIZE+2)/2; i++) {
      for (int j = 0; j < MATRIX_SIZE+2; j++) {
        printf("%7.2f ", mapM[i][j]);
      }
      printf("\n");
    }printf("\n");*/
    
    randomValue = (double)rand() / RAND_MAX;
    //printf("%f\n", randomValue);
    // state machine to handle the direction of the robot
    switch (state) {
      // when the robot is going forward, it will start turning in either direction when an obstacle is close enough
      case FORWARD:
        if (wheel_weight_total[0] > WHEEL_WEIGHT_THRESHOLD) {
          speed[0] = (randomValue + 0.4) * MAX_SPEED;
          speed[1] = -randomValue * MAX_SPEED;
          state = LEFT;
        } else if (wheel_weight_total[1] > WHEEL_WEIGHT_THRESHOLD) {
          speed[0] = -randomValue * MAX_SPEED;
          speed[1] = (randomValue + 0.4) * MAX_SPEED;
          state = RIGHT;
        } else {
          speed[0] = MAX_SPEED;
          speed[1] = MAX_SPEED;
        }
        break;
      // when the robot has started turning, it will go on in the same direction until no more obstacle are in sight
      // this will prevent the robot from being caught in a loop going left, then right, then left, and so on.
      case LEFT:
        if (wheel_weight_total[0] > WHEEL_WEIGHT_THRESHOLD || wheel_weight_total[1] > WHEEL_WEIGHT_THRESHOLD) {
          speed[0] = (randomValue + 0.4) * MAX_SPEED;
          speed[1] = -randomValue * MAX_SPEED;
        } else {
          speed[0] = MAX_SPEED;
          speed[1] = MAX_SPEED;
          state = FORWARD;
        }
        break;
      case RIGHT:
        if (wheel_weight_total[0] > WHEEL_WEIGHT_THRESHOLD || wheel_weight_total[1] > WHEEL_WEIGHT_THRESHOLD) {
          speed[0] = -randomValue * MAX_SPEED;
          speed[1] = (randomValue + 0.4) * MAX_SPEED;
        } else {
          speed[0] = MAX_SPEED;
          speed[1] = MAX_SPEED;
          state = FORWARD;
        }
        break;
    }
    // sets the motor speeds
    wb_motor_set_velocity(left_wheel, speed[0]);
    wb_motor_set_velocity(right_wheel, speed[1]);
    
    
    // Save matrix
    fprintf(file, "%d %d ", robotX, robotY);
    for (int i = 0; i < MATRIX_SIZE+2; i++) {
      for (int j = 0; j < MATRIX_SIZE+2; j++) {
        fprintf(file, "%f", mapM[i][j]);
        if(i==MATRIX_SIZE+1 && j==MATRIX_SIZE+1){
          fprintf(file,"\n");
        }
        else{
          fprintf(file," ");
        }
        //if (j < MATRIX_SIZE) {
        //  fprintf(file, ",");
        //}
      }
    }
    //if(index++==2)
      //break;
     index++;
    //mapM[0][0] += 0.9;
  };
  fclose(file);

  /* Enter your cleanup code here */

  /* This is necessary to cleanup webots resources */
  wb_robot_cleanup();

  return 0;
}

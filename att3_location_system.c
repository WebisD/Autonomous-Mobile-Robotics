/*
 * File:          att3_location_system.c
 * Date:
 * Description:
 * Author:
 * Modifications:
 */

/*
 * You may need to add include files like <webots/distance_sensor.h> or
 * <webots/motor.h>, etc.
 */
#include <webots/robot.h>
#include <webots/motor.h>
#include <webots/lidar.h>
#include <stdio.h>
#include <math.h>
/*
 * You may want to add macros here.
 */
#define TIME_STEP 64
#define MAX_SPEED 3.14
#define SPEED 2
#define MIN_SPEED -3.14
/*
 * This is the main program.
 * The arguments of the main function can be specified by the
 * "controllerArgs" field of the Robot node
 */
 
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

// Function to calculate the normal distribution value
double normalDistribution(int x, double mean, double stddev) {
    return (1.0 / (stddev * sqrt(2.0 * M_PI))) * exp(-((x - mean) * (x - mean)) / (2.0 * stddev * stddev));
}

void printMap(float arr[], int size) {
  printf("Mapa> ");
  for (int i = 0; i < size; i++) {
    printf("%.4f ", arr[i]);
  }
  printf("\n");
}

int main(int argc, char **argv) {
  /* necessary to initialize webots stuff */
  wb_robot_init();
  
  WbDeviceTag right_motor_front = wb_robot_get_device("right wheel motor");
  WbDeviceTag left_motor_front = wb_robot_get_device("left wheel motor");
  // Set motor to infinity mode
  wb_motor_set_position(right_motor_front, INFINITY);
  wb_motor_set_position(left_motor_front, INFINITY);
  
  wb_motor_set_velocity(left_motor_front, SPEED);
  wb_motor_set_velocity(right_motor_front, SPEED);
  
  //LIDAR
  WbDeviceTag lidar = wb_robot_get_device("LDS-01");
  wb_lidar_enable(lidar, TIME_STEP);
  wb_lidar_enable_point_cloud(lidar);
  
  
  float x_atual, x_anterior, delta_t, v, x_atual_pred, qt, zt;
  float cov_atual, cov_atual_pred;
  float cov_anterior = 0.5;
  float rt = 1;
  float mapa[18] = {0};
  int index = 0;
  
  x_anterior = 0;
  delta_t = 7.4; //seconds
  v = 0.135;
  qt = 0.5; // meters based on LDS-01 Docummentation https://emanual.robotis.com/docs/en/platform/turtlebot3/appendix_lds_01/
  zt = 0;
  
  float known_map[3] = {3,7,14};
  
  for (int i = 0; i < 18; i++) {
      mapa[i] = normalDistribution(i, x_anterior, cov_anterior);
  }
  printMap(mapa, 18);
  printf("xt = %f\n", x_anterior);
  printf("cov = %f\n", cov_anterior);
  
  FILE *file = fopen("teste.txt", "w");
  if (file == NULL) {
      fprintf(stderr, "Error opening the file.\n");
      return 1; // Return an error code
  }
  // Save the floats to the file separated by a space
  fprintf(file, "%f %f\n", x_anterior, cov_anterior);
  
  /* main loop
   * Perform simulation steps of TIME_STEP milliseconds
   * and leave the loop when the simulation is over
   */
  int a=0;
  while (wb_robot_step(TIME_STEP) != -1) {
    delay(7400); // Delay seconds between sensors reads
    
    printf("============= Pos: %d =============\n", ++a);
    const float *range_image = wb_lidar_get_range_image(lidar);
    qt+=0.3;
    bool pos72 = false, pos108 = false;
    
    for(int ind=0; ind<110; ind++){
      if(ind==73 && isinf(*(range_image+ind)))
        pos72 = true;
      if(ind==107 && isinf(*(range_image+ind)))
        pos108 = true;
    }
    
    if(pos72 && pos108){
      zt = known_map[index++]; // sensor reads door
      qt = 0.5;
    }
    else{
      zt = x_anterior + (delta_t * v); // Sensor reads wall
    }
   
    // PREDICTION
    x_atual_pred = x_anterior + (delta_t * v);
    cov_atual_pred = cov_anterior + rt;
    
    // CORRECTION
    x_atual = ((x_atual_pred * qt) + (zt * cov_atual_pred)) / (cov_atual_pred+qt);
    cov_atual = (cov_atual_pred*qt)/(cov_atual_pred + qt);

    for (int i = 0; i < 18; i++) {
        mapa[i] = normalDistribution(i, x_atual, cov_atual);
    }
    printMap(mapa, 18);
    printf("bel_xt = %f\t xt = %f\n", x_atual_pred, x_atual);
    printf("bel_cov = %f\t cov = %f\n", cov_atual_pred, cov_atual);
    
    fprintf(file, "%f %f\n", x_atual, cov_atual);
    
    x_anterior = x_atual;
    cov_anterior = cov_atual;
    
    if(a==18) // end of the map
      break;
    
  };
  
  // Close the file
  fclose(file);

  /* This is necessary to cleanup webots resources */
  wb_robot_cleanup();

  return 0;
}

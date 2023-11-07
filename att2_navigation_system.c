/*
 * File:          att2_navigation_system.c
 * Date:          05/11/23
 * Description:   Atividade 2
 * Author:        WebisD
 */


#include <webots/robot.h>
#include <webots/motor.h>
#include <stdio.h>
#include <math.h>

#define TIME_STEP 64
#define MAX_SPEED 3.14
#define SPEED 2
#define MIN_SPEED -3.14
#define DELAY 7300
#define DELAY_DIAG 15000
#define TIME_STEP 64

int obstacles[20][20] = {
        {0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 1},
        {0, 0, 1, 0, 1, 1, 0, 0, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0, 1, 1},
        {0, 0, 0, 0, 1, 1, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0},
        {0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0},
        {0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0},
        {0, 1, 0, 0, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 1, 0, 0},
        {0, 0, 0, 0, 0, 1, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0},
        {0, 0, 0, 0, 0, 1, 1, 0, 0, 0, 0, 0, 1, 1, 0, 0, 0, 0, 0, 0},
        {0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 0, 1, 1, 0, 0, 0, 0, 0, 0},
        {1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 1, 0},
        {0, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 1, 0},
        {0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0},
        {0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 1, 0},
        {0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 1, 0, 0, 0, 0, 0, 0, 0, 0},
        {0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 1, 0, 0, 0, 0, 0, 0, 0, 0},
        {1, 1, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0},
        {1, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0},
        {0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0},
        {0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 0, 0, 1, 1, 0, 0, 0, 0, 0, 0},
        {0, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 1, 1, 0, 0, 0, 0, 0, 0}
    };

double pot_fields[20][20]; // Campos potenciais
int targetx=0, targety=0;  // Final target
int robox=19, roboy=19;    // Initial Target

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

double euclideanDistance(int x1, int y1, int x2, int y2) {
    return sqrt((x2 - x1) * (x2 - x1) + (y2 - y1) * (y2 - y1));
}

void fill_potencial_fields(){
  //Fill with 0
  for(int i=0; i<20; i++){
    for(int j=0; j<20; j++){
      pot_fields[i][j] = 0;
    }
  }
  
  //U(q) = Uatt(q) + Urep(q)
  double Katt = 1, Krep = 5;
  double Uatt, Urep;
  double p0 = 1;
  
  for(int i=0; i<20; i++){
    for(int j=0; j<20; j++){
    
      //potencial atrativo
      Uatt = (0.5) * Katt * euclideanDistance(targetx, targety, i,j);
      pot_fields[i][j] += Uatt;
      
      //potencial repulsivo
      if(obstacles[i][j]==1){
      
         double pq = euclideanDistance(robox, roboy, i,j);
         Urep = (0.5) * Krep * (((1.0/pq) - (1.0/p0)) * ((1.0/pq) - (1.0/p0)));

         // 3x3 influence
         for(int k=-1; k<2; k++){
           for(int l=-1; l<2; l++){
             if(i+k >= 0 && i+k < 20 && j+l >= 0 && j+l < 20){
               pot_fields[i+k][j+l] += Urep;
             }
           }
         }
      }
      else{
        Urep = 0.0;
      }
    }
  }
}


WbDeviceTag right_motor_front, left_motor_front, right_motor_back, left_motor_back;

// Robot movements
void go_front();
void go_back();
void go_left();
void go_right();
void go_diag1_front();
void go_diag2_front();
void go_diag1_back();
void go_diag2_back();
void dont_go();

int main(int argc, char **argv) {
  fill_potencial_fields();
  /*for (int i = 0; i < 20; i++) {
    for (int j = 0; j < 20; j++) {
        printf("%4.2f ", pot_fields[i][j]);
    }
    printf("\n");
  }*/
    
  /* necessary to initialize webots stuff */
  wb_robot_init();
  
  //Set motors
  right_motor_front = wb_robot_get_device("wheel1");
  left_motor_front = wb_robot_get_device("wheel2");
  right_motor_back = wb_robot_get_device("wheel3");
  left_motor_back = wb_robot_get_device("wheel4");
  // Set motor to infinity mode
  wb_motor_set_position(right_motor_front, INFINITY);
  wb_motor_set_position(left_motor_front, INFINITY);
  wb_motor_set_position(right_motor_back, INFINITY);
  wb_motor_set_position(left_motor_back, INFINITY);

  int current_x=robox, current_y=roboy;
  
  while (wb_robot_step(TIME_STEP) != -1) {
    int min_x, min_y;
    double min_grad = INFINITY;
    
    // Get minimum grad path based on current cell
    for(int i=-1; i<2; i++){
      for(int j=-1; j<2; j++){
        if(current_x+i >= 0 && current_x+i < 20 && current_y+j >= 0 && current_y+j < 20){
          if(pot_fields[current_x+i][current_y+j] < min_grad){
            min_grad = pot_fields[current_x+i][current_y+j];
            min_x = current_x+i;
            min_y = current_y+j;
          }
        }
      }
    }
    printf("====\n");
    printf("MIN: x:%d y:%d - value: %f\n", min_x, min_y, pot_fields[min_x][min_y]);
    
    // LEFT
    if(min_x==current_x && min_y==current_y-1){
      printf("LEFT\n");
      go_left();
      delay(DELAY);
    }
    // RIGHT
    else if(min_x==current_x && min_y==current_y+1){
      printf("RIGHT\n");
      go_right();
      delay(DELAY);
    }
    // FRONT
    else if(min_x==current_x-1 && min_y==current_y){
      printf("FRONT\n");
      go_front();
      delay(DELAY);
    }
    // BACK
    else if(min_x==current_x+1 && min_y==current_y){
      printf("BACK\n");
      go_back();
      delay(DELAY);
    }
    // Diagonal 1 FRONT
    else if(min_x==current_x-1 && min_y==current_y+1){
      printf("DIAG 1 FRONT\n");
      go_diag1_front();
      delay(DELAY_DIAG);
    }
    // Diagonal 2 FRONT
    else if(min_x==current_x-1 && min_y==current_y-1){
      printf("DIAG 2 FRONT\n");
      go_diag2_front();
      delay(DELAY_DIAG);
    }
    // Diagonal 1 BACK
    else if(min_x==current_x+1 && min_y==current_y-1){
      printf("DIAG 1 BACK\n");
      go_diag1_back();
      delay(DELAY_DIAG);
    }
    // Diagonal 2 BACK
    else if(min_x==current_x+1 && min_y==current_y+1){
      printf("DIAG 2 BACK\n");
      go_diag2_back();
      delay(DELAY_DIAG);
    }
    else{
      dont_go();
    }
    
    current_x = min_x;
    current_y = min_y;
  };

  /* This is necessary to cleanup webots resources */
  wb_robot_cleanup();

  return 0;
}

void dont_go(){
  wb_motor_set_velocity(right_motor_front, 0);
  wb_motor_set_velocity(left_motor_front, 0);
  wb_motor_set_velocity(right_motor_back, 0);
  wb_motor_set_velocity(left_motor_back, 0);
}

void go_front(){
  wb_motor_set_velocity(left_motor_front, SPEED);
  wb_motor_set_velocity(right_motor_front, SPEED);
  wb_motor_set_velocity(left_motor_back, SPEED);
  wb_motor_set_velocity(right_motor_back, SPEED);
}

void go_back(){
  wb_motor_set_velocity(left_motor_front, -SPEED);
  wb_motor_set_velocity(right_motor_front, -SPEED);
  wb_motor_set_velocity(left_motor_back, -SPEED);
  wb_motor_set_velocity(right_motor_back, -SPEED);
}

void go_right(){
  wb_motor_set_velocity(left_motor_front, SPEED);
  wb_motor_set_velocity(right_motor_front, -SPEED);
  wb_motor_set_velocity(left_motor_back, -SPEED);
  wb_motor_set_velocity(right_motor_back, SPEED);
}

void go_left(){
  wb_motor_set_velocity(left_motor_front, -SPEED);
  wb_motor_set_velocity(right_motor_front, SPEED);
  wb_motor_set_velocity(left_motor_back, SPEED);
  wb_motor_set_velocity(right_motor_back, -SPEED);
}

void go_diag1_front(){
  wb_motor_set_velocity(left_motor_front, SPEED);
  wb_motor_set_velocity(right_motor_front, 0);
  wb_motor_set_velocity(left_motor_back, 0);
  wb_motor_set_velocity(right_motor_back, SPEED);
}

void go_diag2_front(){
  wb_motor_set_velocity(left_motor_front, 0);
  wb_motor_set_velocity(right_motor_front, SPEED);
  wb_motor_set_velocity(left_motor_back, SPEED);
  wb_motor_set_velocity(right_motor_back, 0);
}

void go_diag1_back(){
  wb_motor_set_velocity(left_motor_front, -SPEED);
  wb_motor_set_velocity(right_motor_front, 0);
  wb_motor_set_velocity(left_motor_back, 0);
  wb_motor_set_velocity(right_motor_back, -SPEED);
}

void go_diag2_back(){
  wb_motor_set_velocity(left_motor_front, 0);
  wb_motor_set_velocity(right_motor_front, -SPEED);
  wb_motor_set_velocity(left_motor_back, -SPEED);
  wb_motor_set_velocity(right_motor_back, 0);
}
/*
 * File:          crazybot_Quintal_Nora.c
 * Date:
 * Description:
 * Author:
 * Modifications:
 */

/*
 * You may need to add include files like <webots/distance_sensor.h> or
 * <webots/differential_wheels.h>, etc.
 */
#include <webots/robot.h>
#include <webots/motor.h>
#include <webots/keyboard.h>
#include <webots/distance_sensor.h>
#include <webots/position_sensor.h>

#include <stdio.h>
#include <stdlib.h>
#include <math.h>

/*
 * You may want to add macros here.
 */
#define TIME_STEP 64
#define PI 3.1416
#define Resolution 65535

/*
 * This is the main program.
 * The arguments of the main function can be specified by the
 * "controllerArgs" field of the Robot node
 */
int main(int argc, char **argv)
{
  /* necessary to initialize webots stuff */
  wb_robot_init();
  double vel=5;
  double dis_sen1;
  double med_1;
  double med_2;
  int g_d=0;
  int g_i=0;
  double dis_sen2;
  double pos_sensor1;
  double pos_sensor2;
  double pos_sensor3;
  int New_Pos;
  

  /*
   * You should declare here WbDeviceTag variables for storing
   * robot devices like this:
   *  WbDeviceTag my_sensor = wb_robot_get_device("my_sensor");
   *  WbDeviceTag my_actuator = wb_robot_get_device("my_actuator");
   */
   //llantas
  WbDeviceTag wheel_right = wb_robot_get_device("wheel1");
  WbDeviceTag wheel_left = wb_robot_get_device("wheel2");
  WbDeviceTag wheel_back = wb_robot_get_device("wheel3");
 
  WbDeviceTag Dist_1 = wb_robot_get_device("D_sensor1");
  wb_distance_sensor_enable(Dist_1,TIME_STEP);  
  
  WbDeviceTag Dist_2= wb_robot_get_device("D_sensor2");
  wb_distance_sensor_enable(Dist_2,TIME_STEP);  
  
  //this is the position sensor device, darle nombre a la variable
  WbDeviceTag Encoder_1 = wb_robot_get_device("encoder1");
  WbDeviceTag Encoder_2 = wb_robot_get_device("encoder2");
  WbDeviceTag Encoder_3 = wb_robot_get_device("encoder3");
  
  
  
  //habilitar el sensor
  wb_position_sensor_enable(Encoder_1, TIME_STEP);
  wb_position_sensor_enable(Encoder_2, TIME_STEP);
  wb_position_sensor_enable(Encoder_3, TIME_STEP);
  
  
  
   wb_motor_set_position(wheel_right, INFINITY);   
   wb_motor_set_position(wheel_left, INFINITY);
   wb_motor_set_position(wheel_back, INFINITY);
  /* main loop
   * Perform simulation steps of TIME_STEP milliseconds
   * and leave the loop when the simulation is over
   */
  while (wb_robot_step(TIME_STEP) != -1) {
      
      /*
     * Read the sensors :
     * Enter here functions to read sensor data, like:
     *  double val = wb_distance_sensor_get_value(my_sensor);
     */
     dis_sen1 = wb_distance_sensor_get_value(Dist_1);
     dis_sen2 = wb_distance_sensor_get_value(Dist_2);
     
     
     
     
     med_1= (dis_sen1)*.2/Resolution;//measure of distance sensor 1
     med_2=(dis_sen2)*.2/Resolution;//measure of distance sensor 2
     
     pos_sensor1 =wb_position_sensor_get_value(Encoder_1);
     pos_sensor2 =wb_position_sensor_get_value(Encoder_2);
     pos_sensor3 =wb_position_sensor_get_value(Encoder_3);
     
    
     //avanzar
     wb_motor_set_velocity(wheel_right, -vel);   
     wb_motor_set_velocity(wheel_left, vel);
     wb_motor_set_velocity(wheel_back, 0);
     
     ////////////////////////////////
     if (med_2<=0.17 && med_2> med_1 && g_d==0){
     
     New_Pos =pos_sensor1- PI;
     g_d=1;     
    
     }
     if (g_d==1) {
     
       if (New_Pos > pos_sensor1){
       wb_motor_set_velocity(wheel_right, 0);   
       wb_motor_set_velocity(wheel_left, 0);
       wb_motor_set_velocity(wheel_back, 0);
       }
    }
    else {
          g_d=0;
         }    
     
     if (med_1<=0.17 && med_1 > med_2 && g_i==0){
       New_Pos =pos_sensor1- PI;
       g_i=1;     }
    
     if (g_i==1) {
     
       if (New_Pos > pos_sensor1){
       wb_motor_set_velocity(wheel_right, vel);   
       wb_motor_set_velocity(wheel_left, 0);
       wb_motor_set_velocity(wheel_back,-vel );
       }
       }
     else{g_i=0;}
    
     
     
     
     
     
     
  };

  /* Enter your cleanup code here */

  /* This is necessary to cleanup webots resources */
  
  
  wb_robot_cleanup();

  return 0;
}
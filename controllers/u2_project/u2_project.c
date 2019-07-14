/*
 * File: u2_exam_Pieiro_Lazaro.c
 * Date: 9 de julio del 2019      
 * Description:
 * Author: Lazaro Piñeiro Alcocer
 * Modifications:
 */

#include <webots/robot.h>
#include <webots/motor.h>
#include <webots/distance_sensor.h>
#include <webots/position_sensor.h>
#include <webots/keyboard.h>

#include <stdio.h>

/*
 * You may want to add macros here.
 */
#define TIME_STEP 64
#define PI 3.1416

/*variables globales*/
double b=0;
double b1=0;
double b2=0;
double dl,dr;
int veces=0;
int veces2=0;
/////////variables para el encoder///
double pos_final, pos_final1, pos_final2;
double a, a1, a2;
double rpm, rpm1, rpm2;

/////variables para la velocidad lineal
float rad=0.06; //radio de la llanta
double vel, vel1,vel2;//velocidad lineal por llanta
double vel_rob; //velocidad lineal del robot

int main(int argc, char **argv)
{
  wb_robot_init(); 

  WbDeviceTag wheel1 = wb_robot_get_device("motor1");
  WbDeviceTag wheel2 = wb_robot_get_device("motor2");
  WbDeviceTag wheel3 = wb_robot_get_device("motor3");
   
   
  /////////position sensor//////////
  WbDeviceTag ps1 = wb_robot_get_device("pos1");
  WbDeviceTag ps2 = wb_robot_get_device("pos2");
  WbDeviceTag ps3 = wb_robot_get_device("pos3");
  wb_position_sensor_enable(ps1, TIME_STEP);
  wb_position_sensor_enable(ps2, TIME_STEP);
  wb_position_sensor_enable(ps3, TIME_STEP);
   
  ///////////distance sensor/////////
  WbDeviceTag ds_l = wb_robot_get_device("distance_sensor");
  WbDeviceTag ds_r = wb_robot_get_device("distance_sensor2");   
  wb_distance_sensor_enable(ds_l, TIME_STEP);   
  wb_distance_sensor_enable(ds_r, TIME_STEP); 
  
  while (wb_robot_step(TIME_STEP) != -1) {
  
  dl=(wb_distance_sensor_get_value(ds_l)*0.2)/65535;
  dr=(wb_distance_sensor_get_value(ds_r)*0.2)/65535;
  
  ////rueda #1 //////////////
  a = wb_position_sensor_get_value(ps1);
  pos_final = ((a - b))/0.064;
  rpm= (pos_final*60)/(2*PI);
  b = a;
          
  //////rueda #2////////////
  a1 = wb_position_sensor_get_value(ps2);
  pos_final1 = ((a1 - b1))/0.064;
  rpm1= (pos_final1*60)/(2*PI);
  b1 = a1;
          
  //////rueda #3////////////
  a2 = wb_position_sensor_get_value(ps3);
  pos_final2 = ((a2 - b2))/0.064;
  rpm2 = (pos_final2*60)/(2*PI);
  b2 = a2;
          
  /////////velocidad lineal del robot///////
  vel=pos_final*rad;
  vel1=pos_final1*rad;
  vel2=pos_final2*rad;
  
    if (pos_final<0) {
      vel=vel*-1;
    }
    
    if (pos_final1<0) {
      vel1=vel1*-1;
    }
    
    if (pos_final2<0) {
      vel2=vel2*-1;
    }
    
  vel_rob=(vel+vel1+vel2)/3;
  
  /////////////movimiento del motor////////
  wb_motor_set_position(wheel1, INFINITY);
  wb_motor_set_velocity(wheel1, 0);
  wb_motor_set_position(wheel2, INFINITY);
  wb_motor_set_velocity(wheel2, -1);
  wb_motor_set_position(wheel3, INFINITY);
  wb_motor_set_velocity(wheel3, 1);
        
    if (dl<=0.17 && dl<dr) {
      veces++;
    }
    if (veces>=1 && veces<=58){
      wb_motor_set_position(wheel1, INFINITY);
      wb_motor_set_velocity(wheel1, 1);
      wb_motor_set_position(wheel2, INFINITY);
      wb_motor_set_velocity(wheel2, 1);
      wb_motor_set_position(wheel3, INFINITY);
      wb_motor_set_velocity(wheel3, 1);
      veces++;
    }
    
    else {
      veces=0;
    }
       
    if (dr<0.17 && dr<dl) {
      veces2++;
    }
     
    if (veces2>=1 && veces2<=58){
      wb_motor_set_position(wheel1, INFINITY);
      wb_motor_set_velocity(wheel1, -1);
      wb_motor_set_position(wheel2, INFINITY);
      wb_motor_set_velocity(wheel2, -1);
      wb_motor_set_position(wheel3, INFINITY);
      wb_motor_set_velocity(wheel3, -1);
      veces2++;
    }
     
    else {
      veces2=0;
    }
    
  printf("RPM_1= %f RPM\tRPM_2= %f RPM\tRPM_3= %f RPM\tlinear velocity= %f m/s\tds_l=%f\tds_r=%f\t%i\n",rpm,rpm1,rpm2,vel_rob,dl,dr,veces);
    
  };

  wb_robot_cleanup();

  return 0;
}

/*
 * File: u3_project_Pieiro_Lazaro.c
 * Date: 16 de julio del 2019
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

 #include <promedio.h>
 #include <velocidad.h>

 #define TIME_STEP 64
 #define PI 3.1416

 enum {
 AUTOMATIC,
 MANUAL
 };
/*variables globales*/
 double b, b1, b2=0;
 double dl,dr;
 int veces, veces2=0;
 short int robot_state;
 int paro;

/////////variables para el encoder///
 double pos_final, pos_final1, pos_final2;
 double a, a1, a2;
 double rpm, rpm1, rpm2;

/////variables para la velocidad lineal
 float rad=0.06; //radio de la llanta
 double vel, vel1,vel2;//velocidad lineal por llanta
 double vel_rob; //velocidad lineal del robot

 void stopRobot(WbDeviceTag wheel1,WbDeviceTag wheel2,WbDeviceTag wheel3) {
   wb_motor_set_velocity(wheel1, 0);
   wb_motor_set_velocity(wheel2, 0);
   wb_motor_set_velocity(wheel3, 0);
 }

 void forwardRobot(WbDeviceTag wheel1,WbDeviceTag wheel2,WbDeviceTag wheel3) {
   wb_motor_set_velocity(wheel1, 0);
   wb_motor_set_velocity(wheel2, -5);
   wb_motor_set_velocity(wheel3, 5);
 }

 void backwardRobot(WbDeviceTag wheel1,WbDeviceTag wheel2,WbDeviceTag wheel3) {
   wb_motor_set_velocity(wheel1, 0);
   wb_motor_set_velocity(wheel2, 5);
   wb_motor_set_velocity(wheel3, -5);
 }

 void leftRobot(WbDeviceTag wheel1,WbDeviceTag wheel2,WbDeviceTag wheel3) {
   wb_motor_set_velocity(wheel1, 8);
   wb_motor_set_velocity(wheel2, -4);
   wb_motor_set_velocity(wheel3, -4);
 }

 void rightRobot(WbDeviceTag wheel1,WbDeviceTag wheel2,WbDeviceTag wheel3) {
   wb_motor_set_velocity(wheel1, -8);
   wb_motor_set_velocity(wheel2, 4);
   wb_motor_set_velocity(wheel3, 4);
 }

 void turnRightRobot(WbDeviceTag wheel1,WbDeviceTag wheel2,WbDeviceTag wheel3) {
   wb_motor_set_velocity(wheel1, 0.7853);
   wb_motor_set_velocity(wheel2, 0.7853);
   wb_motor_set_velocity(wheel3, 0.7853);
 }

 void turnLeftRobot(WbDeviceTag wheel1,WbDeviceTag wheel2,WbDeviceTag wheel3) {
   wb_motor_set_velocity(wheel1, -0.7853);
   wb_motor_set_velocity(wheel2, -0.7853);
   wb_motor_set_velocity(wheel3, -0.7853);
 }

 void autonomousMode (WbDeviceTag wheel1,WbDeviceTag wheel2,WbDeviceTag wheel3,
                      WbDeviceTag ds_l, WbDeviceTag ds_r) {

  dl=(wb_distance_sensor_get_value(ds_l)*0.2)/65535;
  dr=(wb_distance_sensor_get_value(ds_r)*0.2)/65535;

  wb_motor_set_position(wheel1, INFINITY);
  wb_motor_set_velocity(wheel1, 0);
  wb_motor_set_position(wheel2, INFINITY);
  wb_motor_set_velocity(wheel2, -6.66);
  wb_motor_set_position(wheel3, INFINITY);
  wb_motor_set_velocity(wheel3, 6.66);

    if (dl<=0.17 && dl<dr) {
      veces++;
    }
    if (veces>=1 && veces<=58) {
    turnRightRobot(wheel1, wheel2, wheel3);
      veces++;
    }
    else {
      veces=0;
    }
    if (dr<0.17 && dr<dl) {
      veces2++;
    }
    if (veces2>=1 && veces2<=58) {
      turnLeftRobot(wheel1, wheel2, wheel3);
      veces2++;
    }
    else {
      veces2=0;
    }
 }

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

  wb_keyboard_enable(TIME_STEP);

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
  vel=velocidad(vel,pos_final,rad);
  vel1=velocidad(vel1,pos_final1,rad);
  vel2=velocidad(vel2,pos_final2,rad);

  vel_rob=prom(vel,vel1,vel2);

  /////////////movimiento del motor////////
  int key=wb_keyboard_get_key();

    if (key == 'G' || robot_state == AUTOMATIC) {
      autonomousMode(wheel1,wheel2,wheel3,ds_l,ds_r);
      robot_state = AUTOMATIC;
    }

    if (key == 'W') {
      robot_state=MANUAL;
      stopRobot(wheel1,wheel2,wheel3);
     }
    if (key == WB_KEYBOARD_LEFT && robot_state == MANUAL) {
      paro=key;
      leftRobot(wheel1,wheel2,wheel3);
    }

    if (paro != key && robot_state == MANUAL) {
      stopRobot(wheel1,wheel2,wheel3);
    }

    if (key==WB_KEYBOARD_UP && robot_state == MANUAL) {
      forwardRobot(wheel1,wheel2,wheel3);
    }

    if(key==WB_KEYBOARD_DOWN && robot_state == MANUAL) {
      backwardRobot(wheel1,wheel2,wheel3);
    }

    if(key==WB_KEYBOARD_RIGHT && robot_state == MANUAL) {
      rightRobot(wheel1,wheel2,wheel3);
    }

    if(key=='A' && robot_state == MANUAL) {
      turnLeftRobot(wheel1,wheel2,wheel3);
    }

    if(key=='S' && robot_state == MANUAL) {
      turnRightRobot(wheel1,wheel2,wheel3);
    }

  printf("MODO: %i\tRPM_1= %f RPM\tRPM_2= %f RPM\tRPM_3= %f RPM\
  linear velocity= %f m/s\tds_l=%f\tds_r=%f\t%i\n"
  ,robot_state,rpm,rpm1,rpm2,vel_rob,dl,dr,veces);

  };

  wb_robot_cleanup();

  return 0;
}

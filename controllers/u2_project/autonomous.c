 #include <webots/motor.h>
 #include <webots/distance_sensor.h>

 double dl,dr=0;
 int veces=0;
 int veces2=0;

void autonomousMode (WbDeviceTag wheel1,WbDeviceTag wheel2,WbDeviceTag wheel3) {


 wb_motor_set_position(wheel1, INFINITY);
  wb_motor_set_velocity(wheel1, 0);
  wb_motor_set_position(wheel2, INFINITY);
  wb_motor_set_velocity(wheel2, -1);
  wb_motor_set_position(wheel3, INFINITY);
  wb_motor_set_velocity(wheel3, 1);
        
    if (dl<=0.17 && dl<dr) {
      veces++;
    }
    if (veces>=1 && veces<=58) {
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
     
    if (veces2>=1 && veces2<=58) { 
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
  
 }
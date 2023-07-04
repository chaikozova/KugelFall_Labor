#include <Servo.h>
#include <math.h>
#include "KugelFall_Methods.h"


void setup() {
    /*
    Setup the hardware when booting the Arduino.
    */

  setupHardware();
}

void loop() {

  /*
    Infinite loop will be run after booting the Arduino. Use polling on the
    trigger pin and begin a pattern when the button gets pressed.
  */

    algorithm();
}

/*
void abwurf_calc(){
  t_u = now_time - old_time;
  pred = time_prediction(t_u);
  if (pred < fall_time){
    abwurf_time_min = round(pred + time_prediction(pred) + now_time - fall_time); // round to get int
    abwurf_time_max = round((abwurf_time_min *1.0433));    // add for hole size
    abwurf_time_min = abwurf_time_min - (t_u/12)-5;
  }else{
    abwurf_time_min = round(pred + now_time - fall_time); // round to get int
    abwurf_time_max = round((abwurf_time_min *1.0433));    // add for hole size
    abwurf_time_min = abwurf_time_min - (t_u/12)-5;    
  }
}
*/

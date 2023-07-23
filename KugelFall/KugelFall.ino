#include <Servo.h>
#include "KugelFall_Methods.h"


void setup() {
    /*
    Setup the hardware when booting the Arduino.
    */

  setupHardware();
}

void loop() {

  /*
    Infinite loop will be run after booting the Arduino.
  */

    algorithm();
}

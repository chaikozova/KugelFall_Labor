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
    Infinite loop will be run after booting the Arduino. ?Use polling on the
    trigger pin and begin a pattern when the button gets pressed.?
  */

    algorithm();
}

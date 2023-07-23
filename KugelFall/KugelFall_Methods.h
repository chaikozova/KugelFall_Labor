// Define physical pins as constant variables
const int photoSensorPin = 2;
const int hallSensorPin = 3;
const int triggerPin = 4;
const int switchPin = 5;
const int blackboxLEDGelb = 7;
const int servoPin = 9;
const int button1Pin = 10;
const int button2Pin = 11;
const int LED1 = 12;
const int LED2 = 13;


// Initializing Servo object
Servo servo;

// Variables for calculating the right throwing time
const float mu = -0.0019124;  // Model Parameters for Speed calculations
const int fall_time = 377;    // Falling time of the ball
float hole_factor = 1.04;     // Used to calculate the upper boundary for release
float c;                      // Delta coefficient for dropping time correction, depends on rotation speed


// Long variables to save time steps
long int now_time = 0, new_time, old_time, abwurf_time_min, abwurf_time_max;
long int t_u;
long int now_time_loop = 0;
long int loop_end = 0;
long int pred = 0;
long int t_error = 0;
long photoTimeNew, photoTimeOld;
int d_t = 0;



// Control variables
bool abwurfState = false;      // Check if it's allowed to throw the ball
bool hallStateFalling = true;  // Check for the hole position, true if the hole is located in the upper part of the disc
bool hallStateRISING = false;  // Check for the hole position, true if the hole is located in the lower part of the disc
int counter = 0;               // Helper variable to make sure that we wait a few rounds before throwing a ball
int speed_state = 0;           // Speed state: 3=fast, 2=middle, 1=slow, -1=Error
float delta_error = 0.2;       // Percentage the prediction can differ from the real time
bool trigger_state = false;    // Check if the trigger Button was pressed
bool errorState;               // Check if there is an error state.
bool photoState;               // Check if the sensor available for use



float map_float(double value, double fromLow, double fromHigh, float toLow, float toHigh) {
  /*
  Re-maps a number from one range to another. A value of `fromLow` would get mapped to `toLow`,
  a value of `fromHigh` would get mapped to `toHigh`, and values in-between are mapped proportionally.
  The function assumes inclusive lower boundaries and exclusive upper boundaries for the ranges.

  Parameters:
  -----------
  value: The number to map.
  fromLow: The lower bound of the value's current range (inclusive).
  fromHigh: The upper bound of the value's current range (exclusive).
  toLow: The lower bound of the value's target range (inclusive).
  toHigh: The upper bound of the value's target range (exclusive).

  Returns:
  ---------
  return: The remapped value as a float.
  */

  return (value - fromLow) * (toHigh - toLow) / (fromHigh - fromLow) + toLow;
}


float t_u_corr(int t_u) {
  /*
  brauchen wir das?
  */
  return (-(1 / 4984) * t_u - (3623 / 14952));
}


float time_prediction(long int t_u) {
  /*
  Calculates and returns the predicted time for the next rotation based on the current rotation time.

  Parameters:
  ------------
  t_u: The current rotation time in milliseconds.

  Returns:
  ---------
  return: The predicted rotation time for the next turn in milliseconds.

  Calculation:
  ------------
  The prediction is based on a mathematical model using the current rotation time (t_u) and model parameters.
  The formula used is (-360 * t_u) / ((t_u * mu) - 360), where mu is a model parameter.
  */

  return ((-360 * t_u) / ((t_u * mu) - 360));
}

void isTriggered() {
  /*
    Checks if the trigger button on the handheld control is being pressed.
  */
  if (trigger_state == false){
      trigger_state = (digitalRead(triggerPin));
  }
  
}


bool hallFALLING() {
  /*
  Uses the Hall Sensor to determine the position of the hole on the disc.
  Updates the boolean variable `hallStateFalling`:
    - Sets it to true if the hole is in the upper part of the disc.
    - Sets it to false if the hole is in the lower part of the disc.

  Returns:
  -----------
  return: True if the hole is in the lower part of the disc, false otherwiese.

  Hall Sensor:
  ------------
  The Hall Sensor detects the magnetic field and is used to identify the position of the hole on the rotating disc.

  Note:
  ------
  The `hallStateFalling` variable is initially set to true. Make sure to set its initial value appropriately before using this function.

  */

  if (digitalRead(hallSensorPin) == LOW && hallStateFalling) {
    hallStateFalling = false;
    return true;
  }

  if (digitalRead(hallSensorPin) == HIGH) {
    hallStateFalling = true;
  }

  return false;
}


bool hallRISING() {
  /*
  Uses the Hall Sensor to determine the position of the hole on the disc.
  Updates the boolean variable `hallStateRISING`:
    - Sets it to true if the hole is in the lower part of the disc.
    - Sets it to false if the hole is in the upper part of the disc.

  Returns:
  ----------
  return: True if the hole is in the upper part of the disc, false otherwiese.

  Hall Sensor:
  ------------
  The Hall Sensor detects the magnetic field and is used to identify the position of the hole on the rotating disc.

  Note:
  ------
  The `hallStateRISING` variable is initially set to false. Make sure to set its initial value appropriately before using this function.
 */

  if (digitalRead(hallSensorPin) == HIGH && hallStateRISING) {
    hallStateRISING = false;
    return true;
  }

  if (digitalRead(hallSensorPin) == LOW) {
    hallStateRISING = true;
  }

  return false;
}


bool photoChange() {

  if (digitalRead(photoSensorPin) == HIGH && !photoState) {
    photoState = true;
    return true;
  }

  if (digitalRead(photoSensorPin) == LOW && photoState) {
    photoState = false;
    return true;
  }

  return false;
}


void release() {
  /*
  Opens and closes the mechanism to release a ball using a servo drive.

  Operation:
  -----------
  1. Sets the servo drive to 65° to open the mechanism and release a ball.
  2. Waits for 100 ms to allow the ball to be released.
  3. Sets the servo drive back to 35° to close the mechanism after throwing a ball.

  Note:
  -------------
  Adjust the servo positions (65° and 35°) based on the specific requirements of your mechanism.
 */

  servo.write(65);
  delay(100);
  servo.write(35);
}


bool check_system(long d_t, float pos, int min, int max) {
  /*
  This function checks if the difference between the predicted time and the actual rotation time is within an acceptable range. 
  The acceptable range is up to 1/24th of a rotation. 
  If the difference is outside this range, it means that there are external disturbances, such as forced acceleration or deceleration.
  The function also checks if the rotation is not too fast or too slow.

  Parameters:
  ------------

  d_t: The current rotation time, in units of 1/12 of a rotation. This is detected and calculated with the help of a photosensor.
  pos: The current position, in units of 1/12 of a rotation. This is used to calculate the predicted time for the current position.
  min: The minimum allowed time for 1/12 of a rotation. This is used to check if the rotation is not too slow.
  max: The maximum allowed time for 1/12 of a rotation. This is used to check if the rotation is not too fast.

  Returns:
  ----------
  True if the difference between the actual and predicted times is within the acceptable range 
          and the rotation is not too fast or too slow.
  False otherwise.
  */
  
  if (abs(d_t - (pred * pos)) > delta_error * (pred * pos) || (d_t < min) || (d_t > max)) {  //

    abwurfState = false;
    speed_state = -1;
    return false;
  }

  return true;
}


//show speed and errors
void show_speed_state(int state) {
  /*
  Displays the rotation mode using LEDs.

  Rotation Modes:
  ---------------
  1. Slow rotation: Both LEDs are on.
  2. Middle rotation: Yellow LED is on.
  3. Fast rotation: Green LED is on.
  -1. Error state: Both LEDs are off.

  Parameters:
  -----------
  state: An integer indicating the rotation mode state.
         Possible values:
         - 1: Slow rotation mode.
         - 2: Middle rotation mode.
         - 3: Fast rotation mode.
         - -1: Error state.
  */

  //fast - green is on
  if (state == 3) {
    digitalWrite(LED1, HIGH);
    digitalWrite(LED2, LOW);
  }

  //middle - yellow is on
  if (state == 2) {
    digitalWrite(LED1, LOW);
    digitalWrite(LED2, HIGH);
  }

  //slow - both LEDs are on
  if (state == 1) {
    digitalWrite(LED1, HIGH);
    digitalWrite(LED2, HIGH);
  }

  //error state - both - LEDs are off
  if (state == -1) {
    digitalWrite(LED1, LOW);
    digitalWrite(LED2, LOW);
  }
}


int abwurf_calc() {
  /*
  Calculates the time parameters required for releasing a ball based on the time difference between the current time and the previous time.

  Returns:
  --------
  The function returns an integer that indicates the rotation mode. The possible values are:

    - 3: Fast rotation mode.
    - 2: Middle rotation mode.
    - 1: Slow rotation mode.

  Global Variables:
  ----------------
  t_u: Time difference between the current time and the previous time. (current rotation time)
  now_time: Current time.
  old_time: Previous time.
  pred: Predicted time for the next rotation.
  t_error: The difference between prediction time and the actual rotation time
  fall_time: Time required for the ball to reach the hole after it is released.
  abwurf_time_min: Minimum time for ball release.
  abwurf_time_max: Maximum time for ball release.
  c: Delta coefficient for dropping time correction.


  Explanation:
  ------------
  This function calculates the time parameters required for releasing a ball based on the time difference (t_u) between the current time and the previous time.

  The function works in three modes:

    - Fast rotation mode: (rotation time less than 450 ms)
        - The minimum time for ball release (abwurf_time_min) is calculated by adding 
                    the predicted time for the next rotation, 
                    the predicted time for the rotation after next 
                    and the time required for the ball to reach the hole after it is released 
                    and difference between prediction time and current rotation time x2 (for both prediction times). 
        - The maximum time for ball release (abwurf_time_max) is calculated by adding 30 ms.
    
    - Slow rotation mode: (rotation time is greater than 2000 ms)
        - The function calculates abwurf_time_min (the minimum time for ball release) 
                    by adding current rotation time (t_u) 
                    to the time difference between the current time and the ball falling time (now_time - fall_time)
                    and difference between prediction time and current rotation time minus 45ms (loop time). 
        - The correction coefficient (c) is calculated by mapping t_u from 2000 to 5000 to the range -0.01 to 0.07 
                    using the map_float function. 
        - abwurf_time_min is adjusted by adding t_u multiplied by c. 
        - abwurf_time_max is calculated by adding 30 ms.

    - Middle rotation mode: (rotation time between 450 ms and 2000 ms)
        - The function calculates abwurf_time_min (the minimum time for ball release)
                    by adding the predicted time for the next rotation (pred)
                    to the time difference between the current time and the ball falling time (now_time - fall_time)
                    and difference between prediction time and current rotation time. 
        - The delta coefficient (c) is calculated by mapping t_u from 450 to 2000 to the range -0.19 to 0.039 
                    using the map_float function. 
        - abwurf_time_min is adjusted by adding t_u multiplied by c.
        - abwurf_time_max is calculated  by adding 30 ms.
  
  */


  t_u = now_time - old_time;
  pred = time_prediction(t_u);
  t_error = t_u - pred;

  //fast
  if (t_u < 450) {
    abwurf_time_min = round(pred + time_prediction(pred) + now_time - fall_time + 2.1 * t_error); //2.1 because 2*error is not enough calc pred twice --> error*2
    
    abwurf_time_min = abwurf_time_min - round((3.0 * t_u / 12.0));
    abwurf_time_max = round(abwurf_time_min + 30);
    pred = pred + t_error;
    return 3;
  }

  //slow
  if (t_u > 2000) {
    abwurf_time_min = round(t_u + now_time - fall_time - 45 + t_error);
    //c = map_float(t_u, 2000, 5000, -0.01, 0.07);
    //abwurf_time_min = abwurf_time_min + round(t_u * c);
    abwurf_time_max = round(abwurf_time_min + 30);
    pred = pred + t_error;
    return 1;
  }

  //middle
  else {
    abwurf_time_min = round(pred + now_time - fall_time + t_error);
    c = map_float(t_u, 450, 2000, -0.19, 0.039);
    abwurf_time_min = abwurf_time_min + round(c * t_u + 1.0/48.0 * t_u);
    abwurf_time_max = round(abwurf_time_min + 30);
    pred = t_error + pred;
    return 2;
  }
}


void algorithm() {
  /*

  Performs the algorithmic operations for the ball throwing mechanism.
  This function is responsible for controlling the timing and behavior of the system.

  Args:
    now_time_loop: The current time in milliseconds.
    photoTimeOld: The previous value of photoTimeNew.
    photoTimeNew: The current value of photoTimeNew.
    errorState: The error state of the system.
    abwurfState: The ball release state.
    isTriggered: The state of the trigger button.
    speed_state: The rotation speed state.
    counter: The counter for skipping calculations.
    loop_end: The end time of the loop.

  Returns:
  ---------
    None.

  */

  now_time_loop = millis();

  // Check for sudden changes in the behavior of the system.
  if (photoChange()) {
    photoTimeOld = photoTimeNew;    // Store the previous photoTimeNew value
    photoTimeNew = now_time_loop;   // Update photoTimeNew with the current time
    // Evaluate system errors and set errorState
    errorState = check_system((photoTimeNew - photoTimeOld), (1.0 / 12.0), (333 / 12), (5000 / 12));
  }

  // Check if it's time for ball release and if conditions are met
  if (now_time_loop > abwurf_time_min && now_time_loop < abwurf_time_max && abwurfState && trigger_state && errorState) {

    release();              // Open the mechanism to release the ball
    abwurfState = false;    // Update the ball release state
    trigger_state = false;  // Update the trigger state
  }

  // Check if the hole is in the falling position
  if (hallFALLING()) {
    old_time = now_time;
    now_time = millis();
    speed_state = abwurf_calc();
    counter++;
    
    // skip some calculations due to servo-sensor interference
    if (counter % speed_state == 0) {
      abwurfState = true;
    }
  }

  /* Show the rotation speed state.
  Fast rotation mode - Green LED is on, 
  Middle rotation mode - Yellow LED is on, 
  Slow rotation mode - both LEDs are on
  Error state - both LEDs are off

  */
  show_speed_state(speed_state);
  isTriggered();
  loop_end = micros();

}


void setupHardware() {

  /*
  Initializes the hardware by initializing the serial port, declaring the all
  the pins, initializing the servo drive and setting it to 35° to close the mechanism
  */

  Serial.begin(9600);

  pinMode(photoSensorPin, INPUT);
  pinMode(hallSensorPin, INPUT);
  pinMode(triggerPin, INPUT);
  pinMode(switchPin, INPUT);
  pinMode(blackboxLEDGelb, INPUT);
  pinMode(button1Pin, INPUT);
  pinMode(button2Pin, INPUT);
  pinMode(LED1, OUTPUT);
  pinMode(LED2, OUTPUT);

  servo.attach(servoPin);
  servo.write(35);
}

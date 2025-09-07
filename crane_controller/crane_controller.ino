/*************************************************** 
 This is a controller for Crane. The code receives 
 instructions from a remote computer and moves the 
 crane to a desired position by controlling 3 servo 
 motors
 ****************************************************/

#include <Wire.h>
#include <Adafruit_PWMServoDriver.h>

// called this way, it uses the default address 0x40
Adafruit_PWMServoDriver pwm = Adafruit_PWMServoDriver();

#define SERVO_FREQ 50 // Analog servos run at ~50 Hz updates

#define CRADLE_MOTOR 0 // Servo motor that controls the cradle movement
#define ANGLE_MOTOR 1 // Servo motor that controls the crane's angle
#define HOOK_MOTOR 4 // Servo motor that moves the hook up and down

#define DISTANCE_SENSOR_TRIGGER_PIN 9
#define DISTANCE_SENSOR_ECHO_PIN 10

// Min and Max cradle distances supported in centimeters
#define MIN_DISTANCE 5
#define MAX_DISTANCE 35

// Min and Max angles supported in degrees
#define MIN_ANGLE 0
#define MAX_ANGLE 180

typedef enum {up, down} HookPosition;

typedef struct {
  uint16_t angle;
  float distance;
  HookPosition hook_position;
} CraneState;

CraneState crane_state;
uint8_t counter = 0;

void move_hook (uint16_t value, uint16_t time_ms) {
  pwm.setPWM(HOOK_MOTOR,0,0);
  delay(100);
  pwm.setPWM(HOOK_MOTOR, 0, value);
  delay(time_ms);
  pwm.setPWM(HOOK_MOTOR,0,0);
  delay(100);
  // Sometimes write 0 fails causing the servo to move continuously
  // write 0 again to prevent such failures
  pwm.setPWM(HOOK_MOTOR,0,0);
}

void set_hook_position(HookPosition target) {
  if (crane_state.hook_position == target){
    // Nothing to change
    return;
  }

  if (target == up) {
    move_hook(180, 3000);
    crane_state.hook_position = up;
  } else if (target == down) {
    move_hook(360, 3000);
    crane_state.hook_position = down;
  }
  return;
}

float get_cradle_distance() {
  float duration, distance;
  digitalWrite(DISTANCE_SENSOR_TRIGGER_PIN, LOW);  
	delayMicroseconds(2);  
	digitalWrite(DISTANCE_SENSOR_TRIGGER_PIN, HIGH);  
	delayMicroseconds(10);  
	digitalWrite(DISTANCE_SENSOR_TRIGGER_PIN, LOW);  
  duration = pulseIn(DISTANCE_SENSOR_ECHO_PIN, HIGH);  
  distance = (duration*.0343)/2;
  Serial.println("returning cradle distance: " + String(distance));   
	return distance;
}

bool set_cradle_distance(float target) {
  if ((target > MAX_DISTANCE) || (target < MIN_DISTANCE)) {
    Serial.println("Error: Attempt to set out of range distance: ");
    return false;
  }

  Serial.println("setting distance" + String(target));
  
  if (crane_state.distance < target) {
    // Move the cradle out till the distance reaches target
    while (crane_state.distance <= target) {
      pwm.setPWM(CRADLE_MOTOR,0,350);
      delay(500);
      crane_state.distance = get_cradle_distance();
    }
    // Stop the cradle motor
    pwm.setPWM(CRADLE_MOTOR,0,0);
    delay(100);
    return true;
  } else if (crane_state.distance > target) {
    // Move the cradle in till the distance reaches target
    while (crane_state.distance >= target) {
      pwm.setPWM(CRADLE_MOTOR,0,180);
      delay(500);
      crane_state.distance = get_cradle_distance();
    }
    // Stop the cradle motor
    pwm.setPWM(CRADLE_MOTOR,0,0);
    delay(100);
    return true;
  }

  // Stop the cradle motor
  pwm.setPWM(CRADLE_MOTOR,0,0);
  delay(100);
  return true;
}

uint16_t convert_angle_to_servopos(uint16_t angle) {
  return angle*2 + 90;
}

uint16_t convert_servopos_to_angle(uint16_t servopos) {
  return (servopos - 90)/2;
}

bool set_angle(uint16_t target_angle){
  Serial.println("Set angle to: " + String(target_angle));
  uint16_t current_servopos, target_servopos;
  if ((target_angle > MAX_ANGLE) || (target_angle < MIN_ANGLE)) {
    Serial.println("Error: Attempt to set out of range angle: ");
    //return false;
  }
  
  current_servopos = convert_angle_to_servopos(crane_state.angle);
  target_servopos = convert_angle_to_servopos(target_angle);

  if (current_servopos < target_servopos) {
    for (uint16_t i = current_servopos; i <= target_servopos; i++) {
      Serial.println("setting angle motor to " + String(i));
      pwm.setPWM(ANGLE_MOTOR,0,i);
      delay(50);
    }
  }

  if (current_servopos > target_servopos) {
    for (uint16_t i = current_servopos; i >= target_servopos; i--) {
      Serial.println("setting angle motor to " + String(i));
      pwm.setPWM(ANGLE_MOTOR,0,i);
      delay(50);
    }
  }
  crane_state.angle = target_angle;
  return true;
}

void reset_crane() {
  bool bool_sink;
  pwm.setPWM(CRADLE_MOTOR, 0, 0);
  delay(100);
  
  crane_state.angle = 0;
  crane_state.distance = get_cradle_distance();
  crane_state.hook_position = up;

  Serial.println("Resetting angle to 0");
  pwm.setPWM(ANGLE_MOTOR,0,90);
  //bool_sink = set_angle(0);
  //bool_sink = set_cradle_distance(5);
  set_hook_position(up);
}

void setup() {
  // Initialize the pins for the untrasonic distance sensor
  pinMode(DISTANCE_SENSOR_TRIGGER_PIN, OUTPUT);  
	pinMode(DISTANCE_SENSOR_ECHO_PIN, INPUT);  

  // Setup baud rate for serial communication
  Serial.begin(9600);

  // Setup frequency for servos
  pwm.begin();
  pwm.setOscillatorFrequency(27000000);
  pwm.setPWMFreq(SERVO_FREQ);  // Analog servos run at ~50 Hz updates
  delay(10);

  reset_crane();
}


void loop() {
  if (counter < 1) {
    delay(5000);
    bool sink;
    Serial.println("Testing Crane");
    delay(5000);
    Serial.println("setting distance to 35");
    //sink = set_cradle_distance(35);
    delay(10000);
    Serial.println("setting distance to 5");
    //sink = set_cradle_distance(5);
    delay (5000);
    Serial.println("setting angle to 90");
    sink = set_angle(90);
    delay(5000);
    Serial.println("Moving hook down");
    //set_hook_position(down);
    Serial.println("setting angle to 180");
    sink = set_angle(230);
    delay(5000);
    Serial.println("Moving hook up");
    //set_hook_position(up);
    delay(5000);
    Serial.println("setting angle to 0");
    sink = set_angle(0);

    counter++;
  }
  delay(1000);
}

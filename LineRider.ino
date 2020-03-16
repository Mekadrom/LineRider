#include <Servo.h>
#include <A4990MotorShield.h>
#include <QTRSensors.h>

/* constants */

// motor speed offsets (to account for any slight difference in speed when going at max speed)
#define LEFT_MOTOR_STATIC_OFFSET        0
#define RIGHT_MOTOR_STATIC_OFFSET       0

#define FLIP_L                          1
#define FLIP_R                          1

#define SETPOINT                     2500//3500

// pid constants (200 max speed)

#define GOAL_SPEED                     75
#define MAX_SPEED                     125

#define Kp                        0.100000
#define Ki                        0.000000
#define Kd                        0.000000

// motor speed offsets and multipliers (to account for any slight difference in speed when going at max speed or wiring flips)

#define SENSOR_COUNT                  6     // number of sensors used

#define EMITTER_PIN                   2 // sensor emitter control pin

// sensor input pins
#define SENSOR_PIN_1                 A0
#define SENSOR_PIN_2                 A1
#define SENSOR_PIN_3                 A2
#define SENSOR_PIN_4                 A3
#define SENSOR_PIN_5                 A4
#define SENSOR_PIN_6                 A5
#define SENSOR_PIN_7                  4
#define SENSOR_PIN_8                  5

#define FAN_CONTROL_PIN               3 // ducted fan speed control pin (pwm)

// constant array for storing sensor pins for constructor of sensor library
const uint8_t sensorPins[SENSOR_COUNT] = { 
                                           SENSOR_PIN_1,
                                           SENSOR_PIN_2,
                                           SENSOR_PIN_3,
                                           SENSOR_PIN_4,
                                           SENSOR_PIN_5,
                                           SENSOR_PIN_6
//                                           SENSOR_PIN_7,
//                                           SENSOR_PIN_8 
                                         };

// array for storing input values from sensor
uint16_t sensorValues[SENSOR_COUNT];

QTRSensors sensor;

// the library for controlling the motors
A4990MotorShield motors;

int lastError;

// the ducted fan control is the same as a servo's, se we use the servo library
Servo fan;

void setup() {
  Serial.begin(9600);
  
  setupPinModes();

  doCallibration();
  
  motors.setSpeeds(0, 0);
  delay(2000); // wait two seconds for human to center manually

//  setFan(180);
}

void setupPinModes() {
  Serial.println("Setting pin modes...");
  pinMode(2, OUTPUT);
  pinMode(3, OUTPUT);
  pinMode(6, INPUT);
  pinMode(7, OUTPUT);
  pinMode(8, OUTPUT);
  pinMode(9, OUTPUT);
  pinMode(10, OUTPUT);
}

void doCallibration() {
  digitalWrite(LED_BUILTIN, HIGH);
  setupSensor();
//  callibrateFan();
  digitalWrite(LED_BUILTIN, LOW);
}

void setupSensor() {
  sensor.setTypeRC();
  sensor.setSensorPins(sensorPins, SENSOR_COUNT);
  sensor.setEmitterPin(EMITTER_PIN);
  
  Serial.println("Callibrating sensors...");

  for (uint16_t i = 0; i < 200; i++) {
    sensor.calibrate();
  }
  motors.setSpeeds(0, 0);
  digitalWrite(LED_BUILTIN, LOW); // turn off Arduino's LED to indicate we are through with calibration

  delay(1000);
}

void callibrateFan() {
  fan.attach(FAN_CONTROL_PIN);

  Serial.println("Callibrating fan...");

  Serial.println("Setting fan high");
  setFan(180); // controller wants max fan speed first
  delay(2000); // should wait enough for beeps

  Serial.println("Setting fan low");
  setFan(0); // next, controller wants minimum speed
  delay(2000); // wait for beeps again

  Serial.println("Revving fan");
  setFan(50); // rev fan to test it
  delay(1000); // wait a little bit for fan to rev

  Serial.println("Fan callibration complete.");
  fan.write(0); // let fan rest for further callibration
}

 void loop() {
  // 3500 is offset for line sensor input
  int error = sensor.readLineBlack(sensorValues) - SETPOINT;

  double output = (Kp * (double)error) + (Kd * ((double)error - (double)lastError));

  lastError = error;

//  Serial.println(output);
  
//  setMotorsDiff(vel, output);
//  setMotorsDiff(GOAL_SPEED, output);
  setMotorsD(GOAL_SPEED, output);
}

void setMotorsD(const int goalSpeed, int diff) {
  int leftSpeed = goalSpeed;
  int rightSpeed = goalSpeed;
  
  if(diff < 0) {
    rightSpeed = goalSpeed + abs(diff);
  } else if(diff > 0) {
    leftSpeed = goalSpeed + abs(diff);
  }

  if(rightSpeed > MAX_SPEED) {
    leftSpeed -= (MAX_SPEED - rightSpeed);
    rightSpeed = MAX_SPEED;
  }
  if(leftSpeed > MAX_SPEED) {
    rightSpeed -= (MAX_SPEED - leftSpeed);
    leftSpeed = MAX_SPEED;
  }
  
//  if(rightSpeed < 0){
//    leftSpeed += rightSpeed;
//    if(leftSpeed > MAX_SPEED) leftSpeed = MAX_SPEED;
//    rightSpeed = 0;
//  }
//  if(leftSpeed < 0){
//    rightSpeed += leftSpeed;
//    if(rightSpeed > MAX_SPEED) rightSpeed = MAX_SPEED;
//    leftSpeed = 0;
//  }

//  motors.setSpeeds(rightSpeed * FLIP_R, leftSpeed * FLIP_L); // might need to switch these; do tests to confirm placement
  motors.setSpeeds(leftSpeed * FLIP_L ,rightSpeed * FLIP_R); // might need to switch these; do tests to confirm placement
}

void setMotorsDiff(const int goalSpeed, int diff) {
  diff *= 2.0;
  
  int rightSpeed = goalSpeed;// - (diff / 2.0);
  int leftSpeed = goalSpeed + (diff / 2.0);
  
//  if(diff > 0) {
//    rightSpeed = GOAL_SPEED;
//    leftSpeed = rightSpeed - diff;
//  } else if(diff < 0) {
//    leftSpeed = GOAL_SPEED;
//    rightSpeed = leftSpeed - diff;
//  } else {
//    leftSpeed = GOAL_SPEED;
//    rightSpeed = GOAL_SPEED;
//  }

  if(rightSpeed > MAX_SPEED) {
    leftSpeed -= (MAX_SPEED - rightSpeed);
    rightSpeed = MAX_SPEED;
  }
  if(leftSpeed > MAX_SPEED) {
    rightSpeed -= (MAX_SPEED - leftSpeed);
    leftSpeed = MAX_SPEED;
  }
  
  if(rightSpeed < 0) rightSpeed = 0;
  if(leftSpeed < 0) leftSpeed = 0;

//  motors.setSpeeds(rightSpeed * FLIP_R, leftSpeed * FLIP_L); // might need to switch these; do tests to confirm placement
  motors.setSpeeds(leftSpeed * FLIP_L ,rightSpeed * FLIP_R); // might need to switch these; do tests to confirm placement
}

void setFan(const int fanSpeed) {
  analogWrite(FAN_CONTROL_PIN, fanSpeed);
}

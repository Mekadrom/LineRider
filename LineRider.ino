#include <Servo.h>
#include <PID_v1.h>
#include <A4990MotorShield.h>
#include <QTRSensors.h>

/* constants */

// motor speed offsets (to account for any slight difference in speed when going at max speed)
#define LEFT_MOTOR_STATIC_OFFSET      0
#define RIGHT_MOTOR_STATIC_OFFSET     0

// pid constants
#define Kp                            0
#define Ki                            0
#define Kd                            0

// motor speed constants (ADJUST AS NEEDED)
#define GOAL_SPEED                   200
#define MAX_SPEED                   400


#define SENSOR_COUNT                  8 // # of sensors on sensor board
#define CALLIBRATE_TIME           10000 // target time length of callibration of sensors

// motor speed offsets and multipliers (to account for any slight difference in speed when going at max speed or wiring flips)

#define SENSOR_COUNT     8     // number of sensors used

#define EMITTER_PIN      2 // sensor emitter control pin

// sensor input pins
#define SENSOR_PIN_1    A0
#define SENSOR_PIN_2    A1
#define SENSOR_PIN_3    A2
#define SENSOR_PIN_4    A3
#define SENSOR_PIN_5    A4
#define SENSOR_PIN_6    A5
#define SENSOR_PIN_7    A6
#define SENSOR_PIN_8    A7

#define FAN_CONTROL_PIN  3 // ducted fan speed control pin (pwm)

// constant array for storing sensor pins for constructor of sensor library
const uint8_t sensorPins[SENSOR_COUNT] = { 
                                           SENSOR_PIN_1,
                                           SENSOR_PIN_2,
                                           SENSOR_PIN_3,
                                           SENSOR_PIN_4,
                                           SENSOR_PIN_5,
                                           SENSOR_PIN_6,
                                           SENSOR_PIN_7,
                                           SENSOR_PIN_8 
                                         };

// array for storing input values from sensor
uint16_t sensorValues[SENSOR_COUNT];

QTRSensors sensor;

// the library for controlling the motors
A4990MotorShield motors;

double setpoint;
double input;
double output;

// the library for the pid controller       P   I   D
PID controller(&input, &output, &setpoint, Kp, Ki, Kd, DIRECT);

// the ducted fan control is the same as a servo's, se we use the servo library
Servo fan;

void setup() {
  Serial.begin(9600);
  
  setupPinModes();
  
  sensor.setTypeAnalog();
  sensor.setSensorPins(sensorPins, SENSOR_COUNT);
  sensor.setEmitterPin(EMITTER_PIN);

  doCallibration();
  
  motors.setSpeeds(0, 0);
  delay(2000); // wait two seconds for human to center manually

  setFan(180);
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
  callibrateFan();
  callibrateSensors();
  digitalWrite(LED_BUILTIN, LOW);
}

void callibrateFan() {
  Serial.println("Callibrating fan...");
  
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

void callibrateSensors() {
  Serial.println("Callibrating sensors...");

  int totalI = 100; // CALLIBRATE_TIME / 40;
  
  for(int i = 0; i < totalI; i++) {
    if(i < totalI * 0.25 || i >= totalI * 0.75) {
     turn(1);
    } else {
     turn(-1);
    }
    sensor.calibrate();
  }
  delay(20);
}

void turn(const int amount) {
  motors.setSpeeds(amount / 2, -(amount / 2));
  delayMicroseconds(20000);
}

void loop() {
  // 3500 is offset for line sensor input
  input = sensor.readLineBlack(sensorValues) - 3500;

  controller.Compute();

  setMotorsDiff(GOAL_SPEED, output);
}

void setMotorsDiff(const int goalSpeed, const int diff) {
  int rightSpeed = goalSpeed + diff;
  int leftSpeed = goalSpeed - diff;

  if(rightSpeed > MAX_SPEED) rightSpeed = MAX_SPEED;
  if(leftSpeed > MAX_SPEED) leftSpeed = MAX_SPEED;
  
  if(rightSpeed < 0) rightSpeed = 0;
  if(leftSpeed < 0) leftSpeed = 0;

  motors.setSpeeds(rightSpeed, leftSpeed); // might need to switch these; do tests to confirm placement
}

void setFan(const int fanSpeed) {
  analogWrite(FAN_CONTROL_PIN, fanSpeed);
}

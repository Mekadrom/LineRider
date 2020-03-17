#include <Servo.h>
#include <A4990MotorShield.h>
#include <QTRSensors.h>

/* constants */

#define SETPOINT                     2500//3500

// pid constants

#define GOAL_SPEED                    175

#define Kp                        0.06900
#define Ki                        0.00000
#define Kd                        2.00000

#define SENSOR_COUNT                    6     // number of sensors used

#define EMITTER_PIN                     2 // sensor emitter control pin

// sensor input pins
#define SENSOR_PIN_1                   A0
#define SENSOR_PIN_2                   A1
#define SENSOR_PIN_3                   A2
#define SENSOR_PIN_4                   A3
#define SENSOR_PIN_5                   A4
#define SENSOR_PIN_6                   A5

#define FAN_CONTROL_PIN               3 // ducted fan speed control pin (pwm)

// constant array for storing sensor pins for constructor of sensor library
const uint8_t sensorPins[SENSOR_COUNT] = { 
                                           SENSOR_PIN_1,
                                           SENSOR_PIN_2,
                                           SENSOR_PIN_3,
                                           SENSOR_PIN_4,
                                           SENSOR_PIN_5,
                                           SENSOR_PIN_6
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
  setupPinModes();

  calibration();

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

void calibration() {
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
  int error = sensor.readLineBlack(sensorValues) - SETPOINT;

  double adjustment = (Kp * (double)error) + (Kd * ((double)error - (double)lastError));

  lastError = error;

  int leftSpeed = constrain(GOAL_SPEED + adjustment, 0, GOAL_SPEED);
  int rightSpeed = constrain(GOAL_SPEED - adjustment, 0, GOAL_SPEED);

  motors.setSpeeds(leftSpeed, rightSpeed);
}

void setFan(const int fanSpeed) {
  analogWrite(FAN_CONTROL_PIN, fanSpeed);
}

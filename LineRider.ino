#include <PID_v1.h>
#include <DRV8835MotorShield.h>
#include <QTRSensors.h>
#include <Servo.h>

/* state constants */

// turning in place constants (used in callibration)
#define CCW                          -1
#define CW                            1

// backward/forward constants for motors
#define BWD                          -1
#define FWD                           1

// motor assignment constants
#define LEFT_MOTOR                   -1
#define RIGHT_MOTOR                   1

// motor speed constants
#define MIN_SPEED                     0
#define MAX_SPEED                   400


/* constants */

#define SENSOR_COUNT                  8 // # of sensors on sensor board
#define CALLIBRATE_TIME           10000 // target time length of callibration of sensors

// motor speed offsets and multipliers (to account for any slight difference in speed when going at max speed or wiring flips)
#define LEFT_MOTOR_STATIC_OFFSET      0
#define RIGHT_MOTOR_STATIC_OFFSET     0

#define LEFT_MOTOR_STATIC_MULTIPLIER  1
#define RIGHT_MOTOR_STATIC_MULTIPLIER 1


/* pins */

// motor driver control pins
#define MOTOR_DRIVER_CTRL_PIN_1       7
#define MOTOR_DRIVER_CTRL_PIN_2       8
#define MOTOR_DRIVER_CTRL_PIN_3       9
#define MOTOR_DRIVER_CTRL_PIN_4      10

#define SENSOR_EMITTER_PIN            2 // sensor emitter control pin

// sensor input pins
#define SENSOR_PIN_1                  3
#define SENSOR_PIN_2                  4
#define SENSOR_PIN_3                  5
#define SENSOR_PIN_4                  6
#define SENSOR_PIN_5                 A0
#define SENSOR_PIN_6                 A1
#define SENSOR_PIN_7                 A2
#define SENSOR_PIN_8                 A3

#define FAN_CONTROL_PIN              11 // ducted fan speed control pin

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

// the ducted fan control is the same as a servo's, se we use the servo library
Servo fan;

// the library for obtaining line-sensor data
QTRSensors sensor;

// the library for controlling the motors
DRV8835MotorShield motors;

double setpoint;
double input;
double output;

// the library for the pid controller      P  I  D
PID controller(&input, &output, &setpoint, 1, 1, 1, DIRECT);

void setup() {
  Serial.begin(9600);
  
  doPinModes();
  setupSensor();
  delay(500);

  prelim();
  setupPID();
  delay(1000);
}

void doPinModes() {
  // in order of use, sets pin modes
  pinMode(LED_BUILTIN, OUTPUT);
  
  pinMode(MOTOR_DRIVER_CTRL_PIN_1, OUTPUT);
  pinMode(MOTOR_DRIVER_CTRL_PIN_2, OUTPUT);
  pinMode(MOTOR_DRIVER_CTRL_PIN_3, OUTPUT);
  pinMode(MOTOR_DRIVER_CTRL_PIN_4, OUTPUT);

  pinMode(SENSOR_EMITTER_PIN, OUTPUT);
  
  for(uint8_t i = 0; i < SENSOR_COUNT; pinMode(sensorPins[i++], INPUT)); // in one line, sets all sensorPins to input
  
  pinMode(FAN_CONTROL_PIN, OUTPUT);
}

void setupSensor() {
  sensor.setTypeRC();
  
  sensor.setSensorPins(sensorPins, SENSOR_COUNT);
  
  sensor.setEmitterPin(SENSOR_EMITTER_PIN);
}

void prelim() {
  digitalWrite(LED_BUILTIN, HIGH);
  callibrateSensor();
  setupFan();
  digitalWrite(LED_BUILTIN, LOW);
}

void callibrateSensor() {
  int angleTracker = 0;
  int angleSpeed = 1;
  bool turnCW = true;

  // one call to callibrate() takes ~25 ms to complete
  for(uint16_t i = 0; i < CALLIBRATE_TIME / 25; i++) {
    sensor.calibrate();
    
    if(turnCW) {
      turnInPlace(255);
      angleTracker += angleSpeed;
      
      if(angleTracker >= 50) {
        turnCW = false;
      }
    } else {
      turnInPlace(-255);
      angleTracker -= angleSpeed;
      
      if(angleTracker <= -50) {
        turnCW = true;
      }
    }
  }

  // should roughly turn the bot back towards where it was placed
  while(angleTracker != 0) {
    if(angleTracker > 0) {
      turnInPlace(-31);
      angleTracker -= angleSpeed;
    } else {
      turnInPlace(31);
      angleTracker += angleSpeed;
    }
  }
  
//  findLine(angleTracker / abs(angleTracker));
}

/**
 * Not used currently; not finished code
 */
void findLine(const int initialTurnDir) {
  // todo: have bot turn until sensor values indicate it is centered on line
  double initSetpoint = 5000;
  double initInput = sensor.readLineBlack(sensorValues);
  double initOutput;

  PID initPID(&initInput, &initOutput, &initSetpoint, 0, 0, 0, DIRECT);
  
  initPID.SetMode(AUTOMATIC);

  if(initialTurnDir == CW) {
    
  }
}

void setupFan() {
  // todo: write code that sets fan high, waits, then sets it low, then waits and then revs to test before setting low
  fan.attach(FAN_CONTROL_PIN);
  
  setFan(180); // controller wants max fan speed first
  delay(2000); // should wait enough for beeps
  
  setFan(0); // next, controller wants minimum speed
  delay(2000); // wait for beeps again
  
  setFan(50); // rev fan to test it
  delay(1000); // wait a little bit for fan to rev
  
  fan.write(0); // let fan rest for further callibration
}

void setupPID() {
  setpoint = 5000;
  input = sensor.readLineBlack(sensorValues);
  
  controller.SetOutputLimits(MIN_SPEED, MAX_SPEED);
  controller.SetMode(AUTOMATIC);
}

void loop() {
  setFan(180);
  control();
}

void control() {
  input = sensor.readLineBlack(sensorValues);
  
  controller.Compute();
  
  setMotorsDiff(output, MAX_SPEED);
}

void setMotorsDiff(const int diff, int vel) {
  if(diff > 0) { // positive diff means right should be slower than left; turning right/clockwise
    if(vel - abs(diff) < MIN_SPEED) {
      setMotor(LEFT_MOTOR, MIN_SPEED + abs(diff));
      setMotor(RIGHT_MOTOR, MIN_SPEED);
    } else {
      setMotor(LEFT_MOTOR, vel);
      setMotor(RIGHT_MOTOR, vel - abs(diff));
    }
  } else if(diff < 0) {
    if(vel - abs(diff) < MIN_SPEED) {
      setMotor(LEFT_MOTOR, MIN_SPEED);
      setMotor(RIGHT_MOTOR, MIN_SPEED + abs(diff));
    } else {
      setMotor(LEFT_MOTOR, vel - abs(diff));
      setMotor(RIGHT_MOTOR, vel);
    }
  } else {
    setMotor(LEFT_MOTOR, vel);
    setMotor(RIGHT_MOTOR, vel);
  }
}

void turnInPlace(const double vel) {
    setMotor(LEFT_MOTOR, vel / 2);
    setMotor(RIGHT_MOTOR, -vel / 2);
}

void setMotor(const int motor, int vel) {
  if(motor == LEFT_MOTOR) {
    vel *= LEFT_MOTOR_STATIC_MULTIPLIER;
    vel += LEFT_MOTOR_STATIC_OFFSET;
    
    motors.setM1Speed(vel > MAX_SPEED ? MAX_SPEED : vel < MIN_SPEED ? MIN_SPEED : vel);
  } else if(motor == RIGHT_MOTOR) {
    vel *= RIGHT_MOTOR_STATIC_MULTIPLIER;
    vel += RIGHT_MOTOR_STATIC_OFFSET;
    
    motors.setM2Speed(vel > MAX_SPEED ? MAX_SPEED : vel < MIN_SPEED ? MIN_SPEED : vel);
  } else {
    Serial.print("[Error] (setMotor) invalid motor id: ");
    Serial.println(motor);
  }
}

void setFan(const int vel) {
  fan.writeMicroseconds(map(abs(vel), 0, 180, 1000, 2000));
}

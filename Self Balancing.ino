#include <Wire.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BNO055.h>
#include <Servo.h>

// BNO055 Setup
Adafruit_BNO055 bno = Adafruit_BNO055(55);

// Servo Setup
Servo yawServo;   // Controls yaw (Z-axis)
Servo pitchServo; // Controls pitch (Y-axis)
const int yawPin = 9;
const int pitchPin = 10;

// Angle Mapping (Adjust these values)
const float YAW_MIN = -90;   // Minimum yaw angle (degrees)
const float YAW_MAX = +90;   // Maximum yaw angle (degrees)
const float PITCH_MIN = -60; // Minimum pitch angle (degrees)
const float PITCH_MAX = +60; // Maximum pitch angle (degrees)

void setup() {
  Serial.begin(115200);
  
  // Initialize BNO055
  if (!bno.begin()) {
    Serial.println("BNO055 not detected! Check wiring.");
    while (1);
  }
  delay(1000);
  bno.setExtCrystalUse(true);

  // Attach servos
  yawServo.attach(yawPin);
  pitchServo.attach(pitchPin);
  yawServo.write(90); // Center position
  pitchServo.write(90);
  delay(1000);
}

void loop() {
  // Get yaw (Z-axis) and pitch (Y-axis)
  imu::Vector<3> euler = bno.getVector(Adafruit_BNO055::VECTOR_EULER);
  float yaw = euler.z();
  float pitch = euler.y();

  // Handle angle wrap-around (e.g., 359° → -1°)
  if (yaw > 180) yaw -= 360;
  if (pitch > 180) pitch -= 360;

  // Map angles to servo positions (opposite direction)
  int yawServoPos = map(-yaw, YAW_MIN, YAW_MAX, 0, 180);   // Reverse yaw input
  int pitchServoPos = map(-pitch, PITCH_MIN, PITCH_MAX, 0, 180); // Reverse pitch input

  // Constrain to servo limits
  yawServoPos = constrain(yawServoPos, 0, 180);
  pitchServoPos = constrain(pitchServoPos, 0, 180);

  // Update servos
  yawServo.write(yawServoPos);
  pitchServo.write(pitchServoPos);

  // Debug output
  Serial.print("Yaw: ");
  Serial.print(yaw);
  Serial.print("° → Servo: ");
  Serial.print(yawServoPos);
  Serial.print("\tPitch: ");
  Serial.print(pitch);
  Serial.print("° → Servo: ");
  Serial.println(pitchServoPos);

  delay(50); // Reduce servo jitter
}

/*
IMPLEMENTASI FILTER MAHONY PADA MPU 6050 MENGGUNAKAN ESP32 

TUGAS ADVANCED ROBOTICS 2024 
OLEH :
1.	Ahmad Fattah Wardoyo		22/493136/PA/21166
2.	Romdlon Musyaffa Akbar	22/500294/PA/21548
3.	Wisnu Aryo Jatmiko		  22/482380/PA/21036

*/

#include "Wire.h"

// AD0 low = 0x68 (default for Sparkfun module)
// AD0 high = 0x69
int MPU_addr = 0x68;

// These are the previously determined offsets and scale factors for accelerometer and gyro for a particular example of an MPU-6050. They are not correct for other examples.
float A_cal[6] = {265.0, -80.0, -700.0, 0.994, 1.000, 1.014}; // 0..2 offset xyz, 3..5 scale xyz
// For an uncalibrated accelerometer, use the following line instead:
// float A_cal[6] = {0.0, 0.0, 0.0, 1.000, 1.000, 1.000}; // 0..2 offset xyz, 3..5 scale xyz

float G_off[3] = { -499.5, -17.7, -82.0}; // raw offsets, determined for gyro at rest
#define gscale ((250./32768.0)*(PI/180.0))  // gyro default 250 LSB per d/s -> rad/s

// GLOBALLY DECLARED, required for Mahony filter
float q[4] = {1.0, 0.0, 0.0, 0.0}; // quaternion vector

// Free parameters in the Mahony filter and fusion scheme,
// Kp for proportional feedback, Ki for integral
float Kp = 30.0;
float Ki = 0.0;

unsigned long now_ms, last_ms = 0; // millis() timers
unsigned long print_ms = 20; // print angles every "print_ms" milliseconds
float yaw, pitch, roll; // Euler angle output

// LED pins
const int rollLedPin = 13;
const int pitchLedPin = 27;
const int yawLedPin = 33;

// Smooth transition variables
int currentRollBrightness = 0;
int currentPitchBrightness = 0;
int currentYawBrightness = 0;
const int transitionSpeed = 5; // Increase for slower transition

void setup() {
  Wire.begin();
  Serial.begin(115200);
  
  // Initialize sensor
  Wire.beginTransmission(MPU_addr);
  Wire.write(0x6B);  // PWR_MGMT_1 register
  Wire.write(0);     // set to zero (wakes up the MPU-6050)
  Wire.endTransmission(true);

  // Initialize LED pins
  pinMode(rollLedPin, OUTPUT);
  pinMode(pitchLedPin, OUTPUT);
  pinMode(yawLedPin, OUTPUT);
}

void loop() {
  static float deltat = 0;  // loop time in seconds
  static unsigned long now = 0, last = 0; // micros() timers

  // Raw data
  int16_t ax, ay, az;
  int16_t gx, gy, gz;
  int16_t Tmp; // temperature

  // Scaled data as vector
  float Axyz[3];
  float Gxyz[3];

  Wire.beginTransmission(MPU_addr);
  Wire.write(0x3B);  // starting with register 0x3B (ACCEL_XOUT_H)
  Wire.endTransmission(false);
  Wire.requestFrom(MPU_addr, 14); // request a total of 14 registers
  int t = Wire.read() << 8;
  ax = t | Wire.read(); // 0x3B (ACCEL_XOUT_H) & 0x3C (ACCEL_XOUT_L)
  t = Wire.read() << 8;
  ay = t | Wire.read(); // 0x3D (ACCEL_YOUT_H) & 0x3E (ACCEL_XOUT_L)
  t = Wire.read() << 8;
  az = t | Wire.read(); // 0x3F (ACCEL_ZOUT_H) & 0x40 (ACCEL_XOUT_L)
  t = Wire.read() << 8;
  Tmp = t | Wire.read(); // 0x41 (TEMP_OUT_H) & 0x42 (TEMP_OUT_L)
  t = Wire.read() << 8;
  gx = t | Wire.read(); // 0x43 (GYRO_XOUT_H) & 0x44 (GYRO_XOUT_L)
  t = Wire.read() << 8;
  gy = t | Wire.read(); // 0x45 (GYRO_YOUT_H) & 0x46 (GYRO_XOUT_L)
  t = Wire.read() << 8;
  gz = t | Wire.read(); // 0x47 (GYRO_ZOUT_H) & 0x48 (GYRO_XOUT_L)

  Axyz[0] = (float) ax;
  Axyz[1] = (float) ay;
  Axyz[2] = (float) az;

  // Apply offsets and scale factors from Magneto
  for (int i = 0; i < 3; i++) Axyz[i] = (Axyz[i] - A_cal[i]) * A_cal[i + 3];

  Gxyz[0] = ((float) gx - G_off[0]) * gscale; // 250 LSB(d/s) default to radians/s
  Gxyz[1] = ((float) gy - G_off[1]) * gscale;
  Gxyz[2] = ((float) gz - G_off[2]) * gscale;

  now = micros();
  deltat = (now - last) * 1.0e-6; // seconds since last update
  last = now;

  Mahony_update(Axyz[0], Axyz[1], Axyz[2], Gxyz[0], Gxyz[1], Gxyz[2], deltat);

  // Compute Tait-Bryan angles. Strictly valid only for approximately level movement
  roll  = atan2((q[0] * q[1] + q[2] * q[3]), 0.5 - (q[1] * q[1] + q[2] * q[2]));
  pitch = asin(2.0 * (q[0] * q[2] - q[1] * q[3]));
  yaw   = -atan2((q[1] * q[2] + q[0] * q[3]), 0.5 - (q[2] * q[2] + q[3] * q[3]));

  // Convert to degrees
  yaw   *= 180.0 / PI;
  if (yaw < 0) yaw += 360.0; // compass circle
  pitch *= 180.0 / PI;
  roll *= 180.0 / PI;

  now_ms = millis(); // time to print?
  if (now_ms - last_ms >= print_ms) {
    last_ms = now_ms;

    // Print roll, pitch, and yaw in a format suitable for the Serial Plotter
    Serial.print(roll, 0);
    Serial.print(",");
    Serial.print(pitch, 0);
    Serial.print(",");
    Serial.println(yaw, 0);

    // Update LED brightness based on angles with threshold and smooth transition
    updateLEDBrightness(rollLedPin, &currentRollBrightness, roll, 5, 180);
    updateLEDBrightness(pitchLedPin, &currentPitchBrightness, pitch, 5, 180);
    updateLEDBrightness(yawLedPin, &currentYawBrightness, yaw, 5, 360);
  }
}

void updateLEDBrightness(int ledPin, int* currentBrightness, float angle, int threshold, int maxAngle) {
  int targetBrightness = 0;
  if (abs(angle) > threshold) {
    targetBrightness = map(abs(angle), threshold, maxAngle, 0, 255);
  }
  
  // Smooth transition
  if (*currentBrightness < targetBrightness) {
    *currentBrightness = min(*currentBrightness + transitionSpeed, targetBrightness);
  } else if (*currentBrightness > targetBrightness) {
    *currentBrightness = max(*currentBrightness - transitionSpeed, targetBrightness);
  }

  analogWrite(ledPin, *currentBrightness);
}

void Mahony_update(float ax, float ay, float az, float gx, float gy, float gz, float deltat) {
  float recipNorm;
  float vx, vy, vz;
  float ex, ey, ez;  // error terms
  float qa, qb, qc;
  static float ix = 0.0, iy = 0.0, iz = 0.0;  // integral feedback terms
  float tmp;

  // Compute feedback only if accelerometer measurement valid (avoids NaN in accelerometer normalization)
  tmp = ax * ax + ay * ay + az * az;
  if (tmp > 0.0) {
    // Normalize accelerometer (assumed to measure the direction of gravity in body frame)
    recipNorm = 1.0 / sqrt(tmp);
    ax *= recipNorm;
    ay *= recipNorm;
    az *= recipNorm;

    // Estimated direction of gravity in the body frame (factor of two divided out)
    vx = q[1] * q[3] - q[0] * q[2];  // to normalize these terms, multiply each by 2.0
    vy = q[0] * q[1] + q[2] * q[3];
    vz = q[0] * q[0] - 0.5f + q[3] * q[3];

    // Error is cross product between estimated and measured direction of gravity in body frame
    // (half the actual magnitude)
    ex = (ay * vz - az * vy);
    ey = (az * vx - ax * vz);
    ez = (ax * vy - ay * vx);

    // Compute and apply to gyro term the integral feedback, if enabled
    if (Ki > 0.0f) {
      ix += Ki * ex * deltat;  // integral error scaled by Ki
      iy += Ki * ey * deltat;
      iz += Ki * ez * deltat;
      gx += ix;  // apply integral feedback
      gy += iy;
      gz += iz;
    }

    // Apply proportional feedback to gyro term
    gx += Kp * ex;
    gy += Kp * ey;
    gz += Kp * ez;
  }

  // Integrate rate of change of quaternion, q cross gyro term
  deltat = 0.5 * deltat;
  gx *= deltat;   // pre-multiply common factors
  gy *= deltat;
  gz *= deltat;
  qa = q[0];
  qb = q[1];
  qc = q[2];
  q[0] += (-qb * gx - qc * gy - q[3] * gz);
  q[1] += (qa * gx + qc * gz - q[3] * gy);
  q[2] += (qa * gy - qb * gz + q[3] * gx);
  q[3] += (qa * gz + qb * gy - qc * gx);

  // Renormalize quaternion
  recipNorm = 1.0 / sqrt(q[0] * q[0] + q[1] * q[1] + q[2] * q[2] + q[3] * q[3]);
  q[0] = q[0] * recipNorm;
  q[1] = q[1] * recipNorm;
  q[2] = q[2] * recipNorm;
  q[3] = q[3] * recipNorm;
}

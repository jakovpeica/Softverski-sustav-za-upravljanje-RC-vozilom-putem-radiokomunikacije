#include <Wire.h>
#include <MPU6050.h>

MPU6050 mpu;

float pitch = 0.0;
float yaw   = 0.0;

unsigned long lastTime = 0;

const float GYRO_SCALE = 131.0;
const float FILTER = 0.95;

void setup() {
  Serial.begin(115200);
  Wire.begin();
  mpu.initialize();

  lastTime = millis();
}

void loop() {
  int16_t ax, ay, az;
  int16_t gx, gy, gz;

  mpu.getMotion6(&ax, &ay, &az, &gx, &gy, &gz);

  unsigned long now = millis();
  float dt = (now - lastTime) / 1000.0;
  lastTime = now;

  if (dt <= 0 || dt > 0.1) return;

  float gyroYaw   = gz / GYRO_SCALE;
  float gyroPitch = gy / GYRO_SCALE;

  yaw   += gyroYaw * dt;
  pitch += gyroPitch * dt;

  float denom = sqrt((float)ay * ay + (float)az * az);
  if (denom > 0.0001) {
    float accPitch = atan2(ax, denom) * 57.2958;
    pitch = pitch * FILTER + accPitch * (1.0 - FILTER);
  }

  pitch = constrain(pitch, -45, 45);

  if (yaw > 180) yaw -= 360;
  if (yaw < -180) yaw += 360;

  Serial.print("YAW:");
  Serial.print(yaw, 1);
  Serial.print(",PITCH:");
  Serial.println(pitch, 1);

  delay(10);
}

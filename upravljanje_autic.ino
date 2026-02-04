#include <Arduino.h>
#include <Servo.h>

#define CRSF_ADDRESS 0xC8
#define CRSF_FRAMETYPE_RC_CHANNELS 0x16

uint8_t buffer[64];
uint8_t buf_pos = 0;
uint16_t channels[16];

Servo steerServo;
Servo escMotor;
Servo yawServo;
Servo pitchServo;

unsigned long lastPacketTime = 0;
unsigned long lastWriteTime = 0;
const unsigned long WRITE_INTERVAL_MS = 20;


const int ESC_MIN = 1000;
const int ESC_MAX = 1600;
const int SERVO_MIN = 1000;
const int SERVO_MAX = 2000;


int targetSteer = 1500;
int targetThrottle = ESC_MIN;
int targetYaw = 1500;
int targetPitch = 1500;

float steerOut = 1500.0;
float throttleOut = ESC_MIN;
float yawOut = 1500.0;
float pitchOut = 1500.0;

bool gotFirstFrame = false;

int mapCRSFtoPWM(int v, int outMin, int outMax) {
  if (v < 172) v = 172;
  if (v > 1811) v = 1811;
  long m = map(v, 172, 1811, outMin, outMax);
  return constrain((int)m, outMin, outMax);
}

void decodeCRSFChannels(uint8_t *payload) {
  uint32_t bitbuf = 0;
  uint8_t bits = 0;
  uint8_t ch_index = 0;

  for (int i = 0; i < 22; i++) {
    bitbuf |= ((uint32_t)payload[i]) << bits;
    bits += 8;
    while (bits >= 11 && ch_index < 16) {
      channels[ch_index] = bitbuf & 0x7FF;
      bitbuf >>= 11;
      bits -= 11;
      ch_index++;
    }
  }
}

void setup() {
  Serial.begin(115200);

  steerServo.attach(9);
  escMotor.attach(10);
  yawServo.attach(6);
  pitchServo.attach(5);

  steerServo.writeMicroseconds(1500);
  yawServo.writeMicroseconds(1500);
  pitchServo.writeMicroseconds(1500);

  escMotor.writeMicroseconds(ESC_MIN);
  delay(3000);

  lastWriteTime = millis();
  lastPacketTime = millis();

  Serial.println("Arduino CRSF receiver ready. ESC armed (min).");
}

void loop() {
  while (Serial.available()) {
    uint8_t b = Serial.read();
    buffer[buf_pos++] = b;
    if (buf_pos >= sizeof(buffer)) buf_pos = 0;

    if (buf_pos >= 3 && buffer[0] == CRSF_ADDRESS) {
      uint8_t length = buffer[1];
      uint8_t total = 2 + length + 1;
      if (buf_pos >= total) {
        uint8_t frametype = buffer[2];
        uint8_t *payload = &buffer[3];
        if (frametype == CRSF_FRAMETYPE_RC_CHANNELS) {
          decodeCRSFChannels(payload);
          lastPacketTime = millis();
          gotFirstFrame = true;

          int s = channels[0];
          int t = channels[1];
          int y = channels[2];
          int p = channels[3];

          targetSteer = mapCRSFtoPWM(s, SERVO_MIN, SERVO_MAX);
          targetThrottle = mapCRSFtoPWM(t, ESC_MIN, ESC_MAX);
          targetYaw = mapCRSFtoPWM(y, SERVO_MIN, SERVO_MAX);
          targetPitch = mapCRSFtoPWM(p, SERVO_MIN, SERVO_MAX);

        }

        memmove(buffer, buffer + total, buf_pos - total);
        buf_pos -= total;
      }
    } else if (buf_pos >= 3 && buffer[0] != CRSF_ADDRESS) {
      // shift away bad leading byte(s)
      memmove(buffer, buffer + 1, buf_pos - 1);
      buf_pos--;
    }
  }

  unsigned long now = millis();

  if (now - lastPacketTime > 500) {
    targetSteer = 1500;
    targetYaw = 1500;
    targetPitch = 1500;
    targetThrottle = ESC_MIN;
  }

  if (now - lastWriteTime >= WRITE_INTERVAL_MS) {
    lastWriteTime = now;

    steerOut += (targetSteer - steerOut) * 0.35;

    yawOut += (targetYaw - yawOut) * 0.28;
    pitchOut += (targetPitch - pitchOut) * 0.28;

    if (targetThrottle < throttleOut) {
      throttleOut += (targetThrottle - throttleOut) * 0.65;
    } else {
      throttleOut += (targetThrottle - throttleOut) * 0.18;
    }

    if (throttleOut < ESC_MIN + 4) throttleOut = ESC_MIN;

    steerServo.writeMicroseconds((int)round(steerOut));
    escMotor.writeMicroseconds((int)round(throttleOut));
    yawServo.writeMicroseconds((int)round(yawOut));
    pitchServo.writeMicroseconds((int)round(pitchOut));
  }
}

// RECEIVER: ESP32 (30-pin expansion shield) + L9110 + steering servo
// Needs Arduino-ESP32 core 3.x and the ESP32Servo library.
// Transmitter sketch is unchanged.

#include <WiFi.h>
#include <esp_now.h>
#include <esp_wifi.h>
#include <ESP32Servo.h>

// ===================== PINS (edit if your wiring differs) =====================
const int L_FWD = 25;     // L9110 A-IA  (left motor)
const int L_REV = 26;     // L9110 A-IB
const int R_FWD = 32;     // L9110 B-IA  (right motor)
const int R_REV = 33;     // L9110 B-IB
const int SERVO_PIN = 18; // servo signal

// ===================== TUNING =====================
const int  PWM_FREQ   = 5000;   // Hz, motor PWM
const int  MAX_DUTY   = 255;    // lower to ~170 if the N30s are 6V and you run ~9V
const bool INVERT_LEFT  = false; // flip if a wheel spins backwards
const bool INVERT_RIGHT = false;
const bool DIFF_STEER   = true;  // also slow the inner wheel when turning

const int SERVO_MIN    = 40;    // left limit (deg), keep inside what the linkage allows
const int SERVO_MAX    = 140;   // right limit (deg); swap MIN/MAX to reverse direction

const int RAMP_STEP    = 10;    // max duty increase per 10 ms (soft start, eases battery load)
const unsigned long FAILSAFE_MS = 500; // stop if no packet for this long

// Must match the transmitter's target address
uint8_t myMac[] = {0x12, 0x34, 0x56, 0x78, 0x9A, 0xBC};

// ===================== PACKET (identical to transmitter) =====================
typedef struct struct_message {
  int steeringAngle; // -360..360
  int throttle;      // 0..100
  int brake;         // 0..100
  int driveMode;     // 1..5
} struct_message;

volatile struct_message rxData = {0, 0, 0, 3};
volatile unsigned long lastRecv = 0;

Servo steeringServo;
int curL = 0, curR = 0;

void OnDataRecv(const esp_now_recv_info_t *info, const uint8_t *data, int len) {
  if (len != sizeof(struct_message)) return;
  memcpy((void *)&rxData, data, sizeof(rxData));
  lastRecv = millis();
}

// speed: -255..255 (negative = reverse)
void driveMotor(int fwdPin, int revPin, int speed, bool invert) {
  if (invert) speed = -speed;
  speed = constrain(speed, -255, 255);
  if (speed > 0)      { ledcWrite(fwdPin, speed);  ledcWrite(revPin, 0); }
  else if (speed < 0) { ledcWrite(fwdPin, 0);      ledcWrite(revPin, -speed); }
  else                { ledcWrite(fwdPin, 0);      ledcWrite(revPin, 0); }
}

// L9110: both inputs HIGH = brake
void brakeMotor(int fwdPin, int revPin) {
  ledcWrite(fwdPin, 255);
  ledcWrite(revPin, 255);
}

void setup() {
  Serial.begin(115200);

  ledcAttach(L_FWD, PWM_FREQ, 8);
  ledcAttach(L_REV, PWM_FREQ, 8);
  ledcAttach(R_FWD, PWM_FREQ, 8);
  ledcAttach(R_REV, PWM_FREQ, 8);
  driveMotor(L_FWD, L_REV, 0, false);
  driveMotor(R_FWD, R_REV, 0, false);

  ESP32PWM::allocateTimer(3);
  steeringServo.setPeriodHertz(50);
  steeringServo.attach(SERVO_PIN, 500, 2400);
  steeringServo.write((SERVO_MIN + SERVO_MAX) / 2);

  WiFi.mode(WIFI_STA);
  esp_wifi_set_mac(WIFI_IF_STA, myMac);   // so the transmitter's fixed address reaches this board

  if (esp_now_init() != ESP_OK) {
    Serial.println("ESP-NOW init failed");
    return;
  }
  esp_now_register_recv_cb(OnDataRecv);
  Serial.print("Ready. MAC: ");
  Serial.println(WiFi.macAddress());
}

void loop() {
  static unsigned long lastRun = 0, lastPrint = 0;
  if (millis() - lastRun < 10) return;
  lastRun = millis();

  struct_message d;
  memcpy(&d, (const void *)&rxData, sizeof(d));

  // Failsafe: no recent packet -> stop and center
  if (millis() - lastRecv > FAILSAFE_MS) {
    d.steeringAngle = 0;
    d.throttle = 0;
    d.brake = 0;
  }

  // ---- Steering servo ----
  int servoPos = map(constrain(d.steeringAngle, -360, 360), -360, 360, SERVO_MIN, SERVO_MAX);
  steeringServo.write(servoPos);

  // ---- Base speed from throttle ----
  int baseSpeed = map(d.throttle, 0, 100, 0, MAX_DUTY);
  if (d.throttle <= 5) baseSpeed = 0;

  // ---- Progressive braking ----
  float brakeMult = 1.0;
  if (d.brake > 10) {
    if (d.brake <= 50)      brakeMult = map(d.brake, 10, 50, 100, 30) / 100.0;
    else if (d.brake < 80)  brakeMult = map(d.brake, 50, 80, 30, 0) / 100.0;
    else                    brakeMult = 0.0;
  }
  int target = baseSpeed * brakeMult;

  // ---- Differential blend ----
  int tL = target, tR = target;
  if (DIFF_STEER) {
    if (d.steeringAngle > 15) {
      tR = target - map(constrain(d.steeringAngle, 0, 360), 0, 360, 0, target);
    } else if (d.steeringAngle < -15) {
      tL = target - map(constrain(-d.steeringAngle, 0, 360), 0, 360, 0, target);
    }
  }

  // ---- Hard lock ----
  if (d.brake >= 80 && baseSpeed == 0) {
    curL = curR = 0;
    brakeMotor(L_FWD, L_REV);
    brakeMotor(R_FWD, R_REV);
  } else {
    if (target <= 5) { tL = 0; tR = 0; }
    // soft start up, instant slow-down
    curL = (tL > curL) ? min(tL, curL + RAMP_STEP) : tL;
    curR = (tR > curR) ? min(tR, curR + RAMP_STEP) : tR;
    driveMotor(L_FWD, L_REV, curL, INVERT_LEFT);
    driveMotor(R_FWD, R_REV, curR, INVERT_RIGHT);
  }

  if (millis() - lastPrint > 200) {
    lastPrint = millis();
    Serial.printf("STR:%d THR:%d BRK:%d | L:%d R:%d | servo:%d\n",
                  d.steeringAngle, d.throttle, d.brake, curL, curR, servoPos);
  }
}

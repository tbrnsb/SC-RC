// TRANSMITTER v2: same pins, same packet, same target address as before.
// Changes: pedal averaging + smoothing, calibration, deadzone, 20 ms send rate.
// The receiver sketch does not need to change.

#include <esp_now.h>
#include <WiFi.h>

#define THROTTLE_PIN 33
#define BRAKE_PIN    34
#define ENCODER_CLK  21
#define ENCODER_DT   22

// ===================== CALIBRATION =====================
// 1) Flash with the defaults, open Serial Monitor (115200).
// 2) Release both pedals and note the RAW numbers -> *_RAW_MIN (add a small margin, ~+100).
// 3) Press each pedal fully and note RAW -> *_RAW_MAX (subtract a small margin, ~-100).
const int THROTTLE_RAW_MIN = 0;
const int THROTTLE_RAW_MAX = 4095;
const int BRAKE_RAW_MIN    = 0;
const int BRAKE_RAW_MAX    = 4095;

const int DEADZONE_PCT      = 6;    // anything below this reads as 0
const int SEND_INTERVAL_MS  = 20;   // 50 packets per second
const float SMOOTHING       = 0.30; // 0..1, higher = faster but noisier

// ===================== PACKET (must match receiver) =====================
typedef struct struct_message {
  int steeringAngle; // -360..360
  int throttle;      // 0..100
  int brake;         // 0..100
  int driveMode;     // 1..5
} struct_message;

uint8_t broadcastAddress[] = {0x12, 0x34, 0x56, 0x78, 0x9A, 0xBC};

struct_message txData;
esp_now_peer_info_t peerInfo;

volatile int encoderTicks = 0;
int lastClkState;

void IRAM_ATTR readEncoder() {
  int clkState = digitalRead(ENCODER_CLK);
  int dtState  = digitalRead(ENCODER_DT);

  if (clkState != lastClkState) {
    if (dtState != clkState) encoderTicks++;
    else                     encoderTicks--;

    txData.steeringAngle = encoderTicks * 18;

    if (txData.steeringAngle > 360)  { txData.steeringAngle = 360;  encoderTicks = 20; }
    if (txData.steeringAngle < -360) { txData.steeringAngle = -360; encoderTicks = -20; }
  }
  lastClkState = clkState;
}

void OnDataSent(const uint8_t *mac_addr, esp_now_send_status_t status) {
  static unsigned long lastMsg = 0;
  if (status != ESP_NOW_SEND_SUCCESS && millis() - lastMsg > 1000) {
    lastMsg = millis();
    Serial.println("Delivery FAILED (car offline?)");
  }
}

// Average 16 ADC samples to knock down noise
int readRaw(int pin) {
  long sum = 0;
  for (int i = 0; i < 16; i++) sum += analogRead(pin);
  return sum / 16;
}

int rawToPercent(int raw, int rawMin, int rawMax) {
  if (rawMax <= rawMin) return 0;
  int p = (int)(((long)(raw - rawMin) * 100) / (rawMax - rawMin));
  p = constrain(p, 0, 100);
  if (p < DEADZONE_PCT) p = 0;
  if (p > 97) p = 100;
  return p;
}

void setup() {
  Serial.begin(115200);

  pinMode(THROTTLE_PIN, INPUT);
  pinMode(BRAKE_PIN, INPUT);
  pinMode(ENCODER_CLK, INPUT_PULLUP);
  pinMode(ENCODER_DT, INPUT_PULLUP);

  lastClkState = digitalRead(ENCODER_CLK);
  attachInterrupt(digitalPinToInterrupt(ENCODER_CLK), readEncoder, CHANGE);

  WiFi.mode(WIFI_STA);
  if (esp_now_init() != ESP_OK) {
    Serial.println("Error initializing ESP-NOW");
    return;
  }
  esp_now_register_send_cb((esp_now_send_cb_t)OnDataSent);

  memcpy(peerInfo.peer_addr, broadcastAddress, 6);
  peerInfo.channel = 0;
  peerInfo.encrypt = false;
  if (esp_now_add_peer(&peerInfo) != ESP_OK) {
    Serial.println("Failed to add peer car");
    return;
  }

  txData.steeringAngle = 0;
  txData.throttle = 0;
  txData.brake = 0;
  txData.driveMode = 3;
}

void loop() {
  static unsigned long lastSend = 0, lastPrint = 0;
  static float thrSm = 0, brkSm = 0;

  if (millis() - lastSend < SEND_INTERVAL_MS) return;
  lastSend = millis();

  int thrRaw = readRaw(THROTTLE_PIN);
  int brkRaw = readRaw(BRAKE_PIN);

  int thr = rawToPercent(thrRaw, THROTTLE_RAW_MIN, THROTTLE_RAW_MAX);
  int brk = rawToPercent(brkRaw, BRAKE_RAW_MIN, BRAKE_RAW_MAX);

  thrSm += SMOOTHING * (thr - thrSm);
  brkSm += SMOOTHING * (brk - brkSm);

  txData.throttle = (thrSm < 1.0) ? 0 : (int)(thrSm + 0.5);
  txData.brake    = (brkSm < 1.0) ? 0 : (int)(brkSm + 0.5);
  txData.driveMode = 3;

  esp_now_send(broadcastAddress, (uint8_t *)&txData, sizeof(txData));

  if (millis() - lastPrint > 250) {
    lastPrint = millis();
    Serial.printf("STEER:%d | THR raw:%d -> %d%% | BRK raw:%d -> %d%%\n",
                  txData.steeringAngle, thrRaw, txData.throttle, brkRaw, txData.brake);
  }
}

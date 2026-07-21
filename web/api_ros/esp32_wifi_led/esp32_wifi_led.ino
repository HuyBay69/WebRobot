/*
 * esp32_wifi_led.ino — Điều khiển xe ESP32 thực nghiệm: Wi-Fi + đèn báo trạng
 * thái + nhận lệnh từ Web (Chiều 1) + báo tiến độ (Chiều 2) + thực thi chuỗi
 * lệnh hình học (đi thẳng có bù trôi MPU / quay 90° tại chỗ).
 *
 * ĐÈN TRẠNG THÁI:
 *   - Đang tìm/kết nối Wi-Fi  → nhấp nháy XANH chậm (chu kỳ 500ms)
 *   - Đã kết nối thành công   → XANH SÁNG HẲN (không nháy nữa)
 *   - Mất kết nối giữa chừng  → tự động quay lại nháy chậm + thử kết nối lại
 *
 * THUẬT TOÁN DI CHUYỂN:
 *   - Đi thẳng: bộ điều khiển P đơn thuần bù trôi yaw.
 *   - Quay: Cắt điện sớm trước khi đạt 90 độ để bù quán tính (tránh quay quá đà).
 *
 * THƯ VIỆN CẦN CÀI (Arduino IDE → Library Manager):
 *   - ArduinoJson (bởi Benoit Blanchon), bản 7.x
 */

#include <WiFi.h>
#include <WebServer.h>
#include <HTTPClient.h>
#include <ArduinoJson.h>
#include <Wire.h>
#include <esp_system.h>

// ── Cấu hình Wi-Fi ────────────────────────────────────────────────────────
const char* WIFI_SSID     = "ESP32_CONTROL";
const char* WIFI_PASSWORD = "12345678";

// ── Cấu hình server Flask (Ubuntu) ────────────────────────────────────────
const char* FLASK_HOST = "10.42.0.1";  // IP hotspot Ubuntu (interface wlp4s0)
const int   FLASK_PORT = 5000;

// ── IP TĨNH cho chính ESP32 ────────────────────────────────────────────────
IPAddress ESP32_STATIC_IP(10, 42, 0, 205);
IPAddress ESP32_GATEWAY(10, 42, 0, 1);
IPAddress ESP32_SUBNET(255, 255, 255, 0);

// ── Đèn báo trạng thái Wi-Fi ──────────────────────────────────────────────
const int LED_PIN = 2;
const unsigned long BLINK_INTERVAL_MS = 500;

unsigned long lastBlinkTime = 0;
bool ledState = false;

WebServer server(80);

// ═══════════════════════════════════════════════════════════════════════════
// L298N — điều khiển động cơ
// ═══════════════════════════════════════════════════════════════════════════
const int ENA = 25, IN1 = 26, IN2 = 27;  // Kênh A — cặp bánh TRÁI
const int ENB = 32, IN3 = 33, IN4 = 14;  // Kênh B — cặp bánh PHẢI
const bool LEFT_REVERSED  = true;
const bool RIGHT_REVERSED = false;
const int  BASE_PWM = 200;      // 0-255 — dùng khi QUAY (turn)
const int  STRAIGHT_PWM = 200;  // 0-255 — dùng khi ĐI THẲNG (straight)

const int PWM_FREQ = 20000;  // 20kHz
const int PWM_RESOLUTION = 8;  // 0-255

void setMotorLeft(int pwm) {
  bool forward = pwm >= 0;
  if (LEFT_REVERSED) forward = !forward;
  int speed = constrain(abs(pwm), 0, 255);
  digitalWrite(IN1, forward ? HIGH : LOW);
  digitalWrite(IN2, forward ? LOW  : HIGH);
  ledcWrite(ENA, speed);
}

void setMotorRight(int pwm) {
  bool forward = pwm >= 0;
  if (RIGHT_REVERSED) forward = !forward;
  int speed = constrain(abs(pwm), 0, 255);
  digitalWrite(IN3, forward ? HIGH : LOW);
  digitalWrite(IN4, forward ? LOW  : HIGH);
  ledcWrite(ENB, speed);
}

void setMotors(int leftPwm, int rightPwm) {
  setMotorLeft(leftPwm);
  setMotorRight(rightPwm);
}

void stopMotors() {
  setMotors(0, 0);
}

void motorsInit() {
  pinMode(IN1, OUTPUT);
  pinMode(IN2, OUTPUT);
  pinMode(IN3, OUTPUT);
  pinMode(IN4, OUTPUT);

  ledcAttach(ENA, PWM_FREQ, PWM_RESOLUTION);
  ledcAttach(ENB, PWM_FREQ, PWM_RESOLUTION);

  stopMotors();
}

// ═══════════════════════════════════════════════════════════════════════════
// MPU6050 — đọc thanh ghi trực tiếp qua Wire
// ═══════════════════════════════════════════════════════════════════════════
const int MPU_SDA = 23, MPU_SCL = 22;
const uint8_t MPU_ADDR = 0x68;

const uint8_t MPU_REG_PWR_MGMT_1  = 0x6B;
const uint8_t MPU_REG_GYRO_CONFIG = 0x1B;
const uint8_t MPU_REG_ACCEL_CONFIG = 0x1C;
const uint8_t MPU_REG_GYRO_ZOUT_H = 0x47;
const uint8_t MPU_REG_WHO_AM_I    = 0x75;

const float GYRO_SCALE = 65.5;  // LSB/(độ/s) ứng với dải đo ±500°/s

void mpuWriteRegister(uint8_t reg, uint8_t value) {
  Wire.beginTransmission(MPU_ADDR);
  Wire.write(reg);
  Wire.write(value);
  Wire.endTransmission();
}

uint8_t mpuReadByte(uint8_t reg) {
  Wire.beginTransmission(MPU_ADDR);
  Wire.write(reg);
  Wire.endTransmission(false);
  Wire.requestFrom((int)MPU_ADDR, 1, (int)true);
  return Wire.read();
}

int16_t mpuReadWord(uint8_t reg) {
  Wire.beginTransmission(MPU_ADDR);
  Wire.write(reg);
  Wire.endTransmission(false);
  Wire.requestFrom((int)MPU_ADDR, 2, (int)true);
  int16_t high = Wire.read();
  int16_t low  = Wire.read();
  return (high << 8) | low;
}

void mpuInit() {
  Wire.begin(MPU_SDA, MPU_SCL);

  mpuWriteRegister(MPU_REG_PWR_MGMT_1, 0x00);   // đánh thức MPU
  mpuWriteRegister(MPU_REG_GYRO_CONFIG, 0x08);  // FS_SEL=1  -> ±500 deg/s
  mpuWriteRegister(MPU_REG_ACCEL_CONFIG, 0x08); // AFS_SEL=1 -> ±4g
  delay(100);

  uint8_t who = mpuReadByte(MPU_REG_WHO_AM_I);
  Serial.print("MPU WHO_AM_I = 0x");
  Serial.println(who, HEX);
}

// ── Hiệu chỉnh bias gyro Z ────────────────────────────────────────────────
float gyroBiasZ = 0.0;  // độ/s

void calibrateGyro() {
  Serial.println("Đang hiệu chỉnh MPU — giữ xe đứng yên...");
  delay(3000);

  const int SAMPLES = 300;
  long sum = 0;
  for (int i = 0; i < SAMPLES; i++) {
    sum += mpuReadWord(MPU_REG_GYRO_ZOUT_H);
    delay(5);
  }
  gyroBiasZ = (float)sum / SAMPLES / GYRO_SCALE;

  Serial.print("Hiệu chỉnh xong — gyro_bias_z = ");
  Serial.print(gyroBiasZ);
  Serial.println(" độ/s");
}

// ── Tích luỹ yaw liên tục ──────────────────────────────────────────────────
float yaw = 0.0;  // độ, tích luỹ
unsigned long lastYawUpdateUs = 0;

void resetYawClock() {
  lastYawUpdateUs = micros();
}

void updateYaw() {
  unsigned long nowUs = micros();
  float dt = (nowUs - lastYawUpdateUs) / 1000000.0;
  lastYawUpdateUs = nowUs;

  float gyroZ_dps = (mpuReadWord(MPU_REG_GYRO_ZOUT_H) / GYRO_SCALE) - gyroBiasZ;
  yaw += gyroZ_dps * dt;
}

// ═══════════════════════════════════════════════════════════════════════════
// Đèn báo trạng thái Wi-Fi
// ═══════════════════════════════════════════════════════════════════════════
void updateStatusLed() {
  if (WiFi.status() == WL_CONNECTED) {
    digitalWrite(LED_PIN, HIGH);
    return;
  }
  unsigned long now = millis();
  if (now - lastBlinkTime >= BLINK_INTERVAL_MS) {
    lastBlinkTime = now;
    ledState = !ledState;
    digitalWrite(LED_PIN, ledState ? HIGH : LOW);
  }
}

void connectWifi() {
  WiFi.mode(WIFI_STA);

  if (!WiFi.config(ESP32_STATIC_IP, ESP32_GATEWAY, ESP32_SUBNET)) {
    Serial.println("Cảnh báo: đặt IP tĩnh thất bại.");
  }

  WiFi.begin(WIFI_SSID, WIFI_PASSWORD);

  Serial.print("Đang kết nối Wi-Fi");
  while (WiFi.status() != WL_CONNECTED) {
    updateStatusLed();
    delay(50);
    Serial.print(".");
  }

  digitalWrite(LED_PIN, HIGH);
  Serial.println();
  Serial.print("Đã kết nối Wi-Fi! Địa chỉ IP của ESP32: ");
  Serial.println(WiFi.localIP());
}

// ═══════════════════════════════════════════════════════════════════════════
// Chiều 2 (ESP32 → Web): báo tiến độ
// ═══════════════════════════════════════════════════════════════════════════
unsigned long packetIndex = 0;

String commonTelemetryJson() {
  return String("\"packet_index\":") + String(packetIndex) +
         ",\"esp32_millis\":" + String(millis()) +
         ",\"ip\":\"" + WiFi.localIP().toString() + "\"" +
         ",\"rssi\":" + String(WiFi.RSSI());
}

void reportProgress(int stepIndex, const char* status) {
  if (WiFi.status() != WL_CONNECTED) return;
  packetIndex++;

  HTTPClient http;
  String url = String("http://") + FLASK_HOST + ":" + String(FLASK_PORT) + "/api/esp32/progress";
  http.begin(url);
  http.addHeader("Content-Type", "application/json");

  String payload = String("{\"step_index\":") + String(stepIndex) +
                     ",\"status\":\"" + status + "\"," + commonTelemetryJson() + "}";
  int httpCode = http.POST(payload);

  if (httpCode <= 0) {
    Serial.print("Lỗi gửi tiến độ về Web: ");
    Serial.println(http.errorToString(httpCode));
  }
  http.end();
}

void reportFinish() {
  if (WiFi.status() != WL_CONNECTED) return;
  packetIndex++;

  HTTPClient http;
  String url = String("http://") + FLASK_HOST + ":" + String(FLASK_PORT) + "/api/esp32/progress";
  http.begin(url);
  http.addHeader("Content-Type", "application/json");

  String payload = String("{\"status\":\"finish\",\"total_packets_sent\":") + String(packetIndex) +
                     "," + commonTelemetryJson() + "}";
  int httpCode = http.POST(payload);

  if (httpCode <= 0) {
    Serial.print("Lỗi gửi báo cáo hoàn thành: ");
    Serial.println(http.errorToString(httpCode));
  } else {
    Serial.println("Đã gửi báo cáo hoàn thành hành trình.");
  }
  http.end();
}

const unsigned long HEARTBEAT_INTERVAL_MS = 2000;
unsigned long lastHeartbeatMs = 0;

void sendHeartbeatIfDue() {
  if (WiFi.status() != WL_CONNECTED) return;
  unsigned long now = millis();
  if (now - lastHeartbeatMs < HEARTBEAT_INTERVAL_MS) return;
  lastHeartbeatMs = now;
  packetIndex++;

  HTTPClient http;
  String url = String("http://") + FLASK_HOST + ":" + String(FLASK_PORT) + "/api/esp32/heartbeat";
  http.begin(url);
  http.addHeader("Content-Type", "application/json");

  String payload = String("{") + commonTelemetryJson() + "}";
  int httpCode = http.POST(payload);

  static bool firstHeartbeatLogged = false;
  if (httpCode == 200 && !firstHeartbeatLogged) {
    firstHeartbeatLogged = true;
    Serial.println("✓ Heartbeat đầu tiên đã gửi thành công tới Flask.");
  }

  if (httpCode != 200) {
    Serial.print("Heartbeat lỗi: ");
    Serial.println(httpCode > 0 ? http.getString() : http.errorToString(httpCode));
  }
  http.end();
}

// ═══════════════════════════════════════════════════════════════════════════
// Hàng đợi lệnh + máy trạng thái thực thi
// ═══════════════════════════════════════════════════════════════════════════
#define MAX_COMMANDS 64

struct Command {
  String type;    // "straight" | "turn"
  long timeMs;
  int  turnAngle; // độ, có dấu — dương = trái, âm = phải
};

Command commandQueue[MAX_COMMANDS];
int commandCount = 0;
int currentCommandIndex = -1;

enum ExecState { EXEC_IDLE, EXEC_PAUSE_BEFORE, EXEC_RUNNING };
ExecState execState = EXEC_IDLE;

unsigned long commandStartMs = 0;
float straightTargetYaw = 0.0;  // yaw tham chiếu cho lệnh "straight"
float turnStartYaw = 0.0;       // yaw lúc bắt đầu lệnh "turn"

const unsigned long INTER_COMMAND_PAUSE_MS = 400;
float KP_STRAIGHT = -5.0;

void finishCurrentCommand() {
  reportProgress(currentCommandIndex, "done");
  execState = EXEC_PAUSE_BEFORE;
  commandStartMs = millis();
}

void advanceCommandQueue() {
  if (currentCommandIndex + 1 >= commandCount) {
    execState = EXEC_IDLE;
    Serial.println("Đã chạy xong toàn bộ chuỗi lệnh.");
    return;
  }

  currentCommandIndex++;
  Command &cmd = commandQueue[currentCommandIndex];

  Serial.print("Bắt đầu lệnh ");
  Serial.print(currentCommandIndex);
  Serial.print(": ");
  Serial.println(cmd.type);

  commandStartMs = millis();
  resetYawClock();

  if (cmd.type == "finish") {
    execState = EXEC_RUNNING;
    return;
  }

  reportProgress(currentCommandIndex, "running");

  if (cmd.type == "straight") {
    straightTargetYaw = yaw;
  } else if (cmd.type == "turn") {
    turnStartYaw = yaw;
  }
  execState = EXEC_RUNNING;
}

const bool ENABLE_STRAIGHT_CORRECTION = false;

void runStraightStep() {
  updateYaw();

  if (!ENABLE_STRAIGHT_CORRECTION) {
    setMotors(STRAIGHT_PWM, STRAIGHT_PWM);
    return;
  }

  float error = yaw - straightTargetYaw;
  int correction = (int)(KP_STRAIGHT * error);

  int leftPwm  = constrain(STRAIGHT_PWM + correction, -255, 255);
  int rightPwm = constrain(STRAIGHT_PWM - correction, -255, 255);
  setMotors(leftPwm, rightPwm);
}

// ═══════════════════════════════════════════════════════════════════════════
// THUẬT TOÁN QUAY: Cắt điện sớm để bù quán tính (trở về cách cũ)
// ═══════════════════════════════════════════════════════════════════════════

// Bù quán tính: Để giảm thời gian xoay (tránh quay lố), chúng ta ngắt điện 
// TRƯỚC KHI đạt 90 độ. Đã tăng từ 8.0 lên 15.0 để xe dừng sớm hơn nữa.
// Nếu xe vẫn quay QUÁ 90 độ -> Tăng số này lên (20.0, 25.0...)
// Nếu xe quay CHƯA TỚI 90 độ -> Giảm số này xuống (10.0, 5.0...)
float TURN_OVERSHOOT_COMPENSATION_DEG = 30.0;

void runTurnStep() {
  updateYaw();

  Command &cmd = commandQueue[currentCommandIndex];
  
  float turned = yaw - turnStartYaw;
  float effectiveTarget = fabs((float)cmd.turnAngle) - TURN_OVERSHOOT_COMPENSATION_DEG;
  
  // Phòng trường hợp góc bù lớn hơn cả góc đích
  if (effectiveTarget < 0) effectiveTarget = 0; 

  // Nếu góc đã quay vượt quá hoặc bằng mức cắt điện -> dừng động cơ
  if (fabs(turned) >= effectiveTarget) {
    stopMotors();
    finishCurrentCommand();
    return;
  }

  // Cấp xung cố định (BASE_PWM)
  int pwm = (cmd.turnAngle > 0) ? BASE_PWM : -BASE_PWM;
  setMotors(-pwm, pwm);
}

void handleExecutionLoop() {
  if (execState == EXEC_IDLE) return;

  if (execState == EXEC_PAUSE_BEFORE) {
    stopMotors();
    if (millis() - commandStartMs >= INTER_COMMAND_PAUSE_MS) {
      advanceCommandQueue();
    }
    return;
  }

  // EXEC_RUNNING
  Command &cmd = commandQueue[currentCommandIndex];
  if (cmd.type == "straight") {
    if ((long)(millis() - commandStartMs) >= cmd.timeMs) {
      stopMotors();
      finishCurrentCommand();
    } else {
      runStraightStep();
    }
  } else if (cmd.type == "turn") {
    runTurnStep();
  } else if (cmd.type == "finish") {
    stopMotors();
    reportFinish();
    execState = EXEC_IDLE;
    Serial.println("Đã hoàn thành toàn bộ hành trình (finish).");
  } else {
    Serial.print("Lệnh không rõ, bỏ qua: ");
    Serial.println(cmd.type);
    finishCurrentCommand();
  }
}

// ═══════════════════════════════════════════════════════════════════════════
// Chiều 1 (Web → ESP32): nhận chuỗi lệnh hình học
// ═══════════════════════════════════════════════════════════════════════════
void handleCommands() {
  if (server.method() != HTTP_POST) {
    server.send(405, "application/json", "{\"error\":\"method not allowed\"}");
    return;
  }

  String body = server.arg("plain");
  Serial.println("Nhận chuỗi lệnh mới:");
  Serial.println(body);

  server.send(200, "application/json", "{\"ok\":true}");

  JsonDocument doc; 
  DeserializationError err = deserializeJson(doc, body);
  if (err) {
    Serial.print("Lỗi parse JSON: ");
    Serial.println(err.c_str());
    return;
  }

  JsonArray arr = doc.as<JsonArray>();
  commandCount = 0;
  for (JsonObject obj : arr) {
    if (commandCount >= MAX_COMMANDS) {
      Serial.println("Cảnh báo: vượt quá MAX_COMMANDS, các lệnh sau bị bỏ qua.");
      break;
    }
    commandQueue[commandCount].type      = obj["command"].as<String>();
    commandQueue[commandCount].timeMs    = obj["time_ms"]    | 0;
    commandQueue[commandCount].turnAngle = obj["turn_angle"] | 0;
    commandCount++;
  }

  Serial.print("Đã nhận ");
  Serial.print(commandCount);
  Serial.println(" lệnh, bắt đầu chạy...");

  currentCommandIndex = -1;
  yaw = 0.0;  // mốc hướng ban đầu cho cả hành trình mới
  advanceCommandQueue();
}

void printResetReason() {
  esp_reset_reason_t reason = esp_reset_reason();
  Serial.print("Lý do khởi động/reset lần trước: ");
  switch (reason) {
    case ESP_RST_POWERON:  Serial.println("Bật nguồn bình thường (POWERON)"); break;
    case ESP_RST_BROWNOUT: Serial.println("*** BROWNOUT — NGUỒN ĐIỆN KHÔNG ĐỦ/KHÔNG ỔN ĐỊNH! ***"); break;
    case ESP_RST_PANIC:    Serial.println("PANIC — lỗi phần mềm (crash), không phải nguồn điện"); break;
    case ESP_RST_TASK_WDT: Serial.println("TASK WATCHDOG TIMEOUT — 1 tác vụ bị treo quá lâu"); break;
    case ESP_RST_INT_WDT:  Serial.println("INTERRUPT WATCHDOG TIMEOUT"); break;
    case ESP_RST_SW:       Serial.println("Reset bằng phần mềm (esp_restart)"); break;
    default:
      Serial.print("Khác, mã số = ");
      Serial.println((int)reason);
  }
}

// ═══════════════════════════════════════════════════════════════════════════
void setup() {
  Serial.begin(115200);
  delay(200); 
  printResetReason();

  pinMode(LED_PIN, OUTPUT);
  digitalWrite(LED_PIN, LOW);

  motorsInit();
  mpuInit();
  calibrateGyro();

  connectWifi();

  server.on("/commands", handleCommands);
  server.begin();
  Serial.println("HTTP server đã sẵn sàng nhận lệnh ở cổng 80.");
}

void loop() {
  server.handleClient();
  updateStatusLed();
  handleExecutionLoop();
  sendHeartbeatIfDue();

  if (WiFi.status() != WL_CONNECTED) {
    static unsigned long lastRetry = 0;
    if (millis() - lastRetry > 5000) {
      lastRetry = millis();
      Serial.println("Mất kết nối Wi-Fi, đang thử kết nối lại...");
      WiFi.disconnect();
      WiFi.begin(WIFI_SSID, WIFI_PASSWORD);
    }
  }
}

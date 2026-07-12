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
 * THUẬT TOÁN DI CHUYỂN (đã chốt với người dùng trước khi viết):
 *   - Đi thẳng: bộ điều khiển P đơn thuần bù trôi yaw (không PID — ESP32
 *     không có encoder bánh xe nên I/D không có nhiều ý nghĩa, P đơn giản dễ
 *     tinh chỉnh thực nghiệm là đủ).
 *   - Quay 90°: quay TẠI CHỖ (pivot) — 2 bánh quay NGƯỢC chiều nhau, KHÔNG
 *     giảm tốc khi gần đạt góc đích (giữ nguyên BASE_PWM xuyên suốt, dừng
 *     ngay khi đủ góc).
 *
 * THƯ VIỆN CẦN CÀI (Arduino IDE → Library Manager):
 *   - ArduinoJson (bởi Benoit Blanchon), bản 7.x (code dùng JsonDocument
 *     không cố định kích thước — nếu bạn đang có ArduinoJson 6.x, đổi
 *     `JsonDocument doc;` thành `DynamicJsonDocument doc(4096);`)
 *   (WiFi.h, WebServer.h, HTTPClient.h, Wire.h đã có sẵn trong ESP32 core)
 *
 * CẦN BẠN ĐIỀN/CHỈNH TRƯỚC KHI NẠP:
 *   1. WIFI_SSID / WIFI_PASSWORD — đã điền theo bạn báo (ESP32_CONTROL / 12345678).
 *   2. LED_PIN — mặc định GPIO2, đổi lại nếu board bạn dùng chân khác.
 *   3. KP_STRAIGHT — cần TINH CHỈNH THỰC NGHIỆM trên xe thật (xem ghi chú ở
 *      hằng số bên dưới). Nếu sau khi chạy thử thấy xe lệch NGÀY CÀNG XA
 *      (không phải sửa lại), đảo dấu hằng số này trước, rồi mới tăng/giảm độ lớn.
 *   4. Có thể cần đảo dấu ở runTurnStep() nếu chiều quay trái/phải bị ngược so
 *      với turn_angle — xem ghi chú ngay tại hàm đó.
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
// L298N — điều khiển động cơ (theo Promt.txt)
// ═══════════════════════════════════════════════════════════════════════════
const int ENA = 25, IN1 = 26, IN2 = 27;  // Kênh A — cặp bánh TRÁI
const int ENB = 32, IN3 = 33, IN4 = 14;  // Kênh B — cặp bánh PHẢI
const bool LEFT_REVERSED  = true;
const bool RIGHT_REVERSED = false;
const int  BASE_PWM = 200;      // 0-255 — dùng khi QUAY (turn), giữ mức gốc
const int  STRAIGHT_PWM = 180;  // 0-255 — dùng riêng khi ĐI THẲNG (straight), thấp hơn BASE_PWM 1 chút để chạy chậm/an toàn hơn lúc thẳng đường

// PWM (LEDC) — dùng API core 3.x: ledcAttach(chân, tần_số, độ_phân_giải) rồi
// ledcWrite(chân, giá_trị) — KHÔNG còn ledcSetup()/ledcAttachPin()/số kênh
// riêng như core 2.x cũ (core mới tự quản lý kênh nội bộ theo chân GPIO).
// Nếu bạn dùng core 2.x cũ hơn và gặp lỗi ngược lại ("ledcAttach was not
// declared"), đổi 2 dòng ledcAttach() trong motorsInit() thành:
//   ledcSetup(0, PWM_FREQ, PWM_RESOLUTION); ledcAttachPin(ENA, 0);
//   ledcSetup(1, PWM_FREQ, PWM_RESOLUTION); ledcAttachPin(ENB, 1);
// và đổi ledcWrite(ENA,...)/ledcWrite(ENB,...) thành ledcWrite(0,...)/ledcWrite(1,...).
const int PWM_FREQ = 20000;  // 20kHz — TRÊN ngưỡng tai người nghe được (đã tăng từ 5000, vốn gây tiếng rít "i i i" vì nằm giữa dải nghe rõ nhất của tai). L298N vẫn hoạt động tốt ở tần số này.
const int PWM_RESOLUTION = 8;  // 0-255

// pwm: -255..255 — âm = lùi, dương = tiến (trước khi áp dụng cờ *_REVERSED)
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
// MPU6050 — đọc thanh ghi trực tiếp qua Wire (không dùng thư viện Adafruit,
// theo đúng yêu cầu Promt.txt)
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

  mpuWriteRegister(MPU_REG_PWR_MGMT_1, 0x00);   // đánh thức MPU (thoát sleep)
  mpuWriteRegister(MPU_REG_GYRO_CONFIG, 0x08);  // FS_SEL=1  -> ±500 deg/s
  mpuWriteRegister(MPU_REG_ACCEL_CONFIG, 0x08); // AFS_SEL=1 -> ±4g
  delay(100);

  uint8_t who = mpuReadByte(MPU_REG_WHO_AM_I);
  Serial.print("MPU WHO_AM_I = 0x");
  Serial.println(who, HEX);
  if (who != 0x70) {
    Serial.println("Cảnh báo: WHO_AM_I khác 0x70 như Promt.txt ghi — kiểm tra lại dây nối/địa chỉ I2C.");
  }
}

// ── Hiệu chỉnh bias gyro Z — giữ xe đứng yên 3s, lấy trung bình 300 mẫu ────
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

// ── Tích luỹ yaw liên tục — gọi mỗi vòng loop() lúc đang chạy 1 lệnh ───────
float yaw = 0.0;  // độ, tích luỹ từ lúc handleCommands() nhận lệnh mới
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
    Serial.println("Cảnh báo: đặt IP tĩnh thất bại — sẽ dùng DHCP như cũ.");
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
// Chiều 2 (ESP32 → Web): báo tiến độ từng bước + heartbeat định kỳ + báo cáo
// hoàn thành. Mọi gói gửi đi đều đánh số packetIndex tăng dần, kèm millis()
// và RSSI — Flask dùng để tính tỉ lệ mất gói + độ trễ + hiển thị chất lượng
// Wi-Fi (xem _record_packet() trong esp32_control.py).
// ═══════════════════════════════════════════════════════════════════════════
unsigned long packetIndex = 0;  // tăng dần ở MỌI gói gửi đi (heartbeat lẫn báo cáo tiến độ) — dùng phát hiện gói bị rớt

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

// Báo cáo KẾT THÚC hành trình — gửi khi thực thi tới lệnh command="finish"
// (luôn ở cuối chuỗi lệnh, do trajectory_converter.py tự thêm). Kèm tổng số
// gói đã gửi (packetIndex tính TỚI THỜI ĐIỂM NÀY) để Flask so với số gói THỰC
// SỰ nhận được, tính ra tỉ lệ mất gói trên đường truyền Wi-Fi.
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

// Heartbeat định kỳ — báo còn sống + chất lượng Wi-Fi ngay cả khi ĐANG RẢNH
// (chưa nhận chuỗi lệnh nào), để panel kết nối trên web luôn có dữ liệu mới.
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

  // Chỉ in ra khi CÓ LỖI — tránh spam Serial mỗi 2 giây lúc bình thường, nhưng
  // vẫn thấy được ngay nếu heartbeat không tới được Flask (vd route chưa tồn
  // tại vì Flask chưa nạp lại code mới — httpCode=404 — hoặc sai IP/cổng).
  if (httpCode != 200) {
    Serial.print("Heartbeat lỗi (http code=");
    Serial.print(httpCode);
    Serial.print("): ");
    Serial.println(httpCode > 0 ? http.getString() : http.errorToString(httpCode));
  }
  http.end();
}

// ═══════════════════════════════════════════════════════════════════════════
// Hàng đợi lệnh + máy trạng thái thực thi (state machine, không dùng delay())
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
float straightTargetYaw = 0.0;  // yaw tham chiếu (giữ nguyên hướng) cho lệnh "straight" hiện tại
float turnStartYaw = 0.0;       // yaw lúc bắt đầu lệnh "turn" hiện tại, để tính đã quay được bao nhiêu

// Dừng ngắn giữa các lệnh — khớp mô tả "đi thẳng N giây → dừng → quay → dừng..."
const unsigned long INTER_COMMAND_PAUSE_MS = 400;

// Hệ số P bù trôi khi đi thẳng — ĐƠN VỊ: (chênh lệch PWM) / (độ lệch yaw).
// TINH CHỈNH THỰC NGHIỆM: bắt đầu với giá trị nhỏ (2-4), tăng dần nếu xe vẫn
// lệch rõ; nếu xe lệch NGÀY CÀNG XA thay vì tự sửa lại — nghĩa là dấu bù đang
// SAI CHIỀU — đảo dấu KP_STRAIGHT (đổi thành số âm) rồi thử lại.
float KP_STRAIGHT = 3.0;

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

  // "finish" không phải bước di chuyển — không gọi reportProgress() theo
  // step_index thường, để handleExecutionLoop() xử lý riêng (gửi báo cáo
  // tổng kết qua reportFinish() ngay dưới).
  if (cmd.type == "finish") {
    execState = EXEC_RUNNING;
    return;
  }

  reportProgress(currentCommandIndex, "running");

  if (cmd.type == "straight") {
    straightTargetYaw = yaw;  // giữ nguyên hướng hiện tại trong suốt đoạn thẳng này
  } else if (cmd.type == "turn") {
    turnStartYaw = yaw;
  }
  execState = EXEC_RUNNING;
}

// ── Đi thẳng: P-controller bù trôi yaw quanh straightTargetYaw ─────────────
void runStraightStep() {
  updateYaw();

  float error = yaw - straightTargetYaw;   // >0: đã lệch trái so với hướng ban đầu
  int correction = (int)(KP_STRAIGHT * error);

  // Lệch trái (error>0) -> tăng PWM bánh trái, giảm PWM bánh phải -> xe tự
  // xoay bù về phải. Nếu thực tế thấy NGƯỢC LẠI (xe lệch thêm), đảo dấu
  // KP_STRAIGHT ở khai báo hằng số phía trên thay vì sửa ở đây.
  int leftPwm  = constrain(STRAIGHT_PWM + correction, -255, 255);
  int rightPwm = constrain(STRAIGHT_PWM - correction, -255, 255);
  setMotors(leftPwm, rightPwm);
}

// ── Quay 90° tại chỗ: PWM cố định BASE_PWM, không giảm tốc, dừng ngay khi đủ góc ──
void runTurnStep() {
  updateYaw();

  Command &cmd = commandQueue[currentCommandIndex];
  float turned = yaw - turnStartYaw;

  if (fabs(turned) >= fabs((float)cmd.turnAngle)) {
    stopMotors();
    finishCurrentCommand();
    return;
  }

  // turn_angle > 0 = quay trái. Quy ước: quay trái = bánh trái LÙI, bánh phải
  // TIẾN (xe xoay ngược chiều kim đồng hồ nhìn từ trên xuống). Nếu thực tế xe
  // quay NGƯỢC HƯỚNG so với turn_angle yêu cầu, đảo dấu ở dòng "pwm = ..." bên
  // dưới (đổi (cmd.turnAngle > 0) thành (cmd.turnAngle < 0)).
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
// Body JSON: [ {"index":0,"command":"straight","time_ms":2000,"turn_angle":0}, ... ]
// ═══════════════════════════════════════════════════════════════════════════
void handleCommands() {
  if (server.method() != HTTP_POST) {
    server.send(405, "application/json", "{\"error\":\"method not allowed\"}");
    return;
  }

  String body = server.arg("plain");
  Serial.println("Nhận chuỗi lệnh mới:");
  Serial.println(body);

  // Trả lời ngay cho Web biết đã nhận — không chờ chạy xong cả hành trình
  // mới trả response (đúng thiết kế Chiều 1: Web gửi 1 lần, không chờ).
  server.send(200, "application/json", "{\"ok\":true}");

  JsonDocument doc;  // ArduinoJson v7 — nếu bạn dùng v6, đổi thành DynamicJsonDocument doc(4096);
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

// ── Chẩn đoán: in ra lý do lần reset/khởi động gần nhất — gọi ĐẦU TIÊN trong
// setup(), trước cả Serial.begin() thật sự ổn định, để biết chắc có phải
// brownout (sụt áp) hay không thay vì đoán qua log boot lộn xộn.
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
  delay(200);  // cho Serial ổn định trước khi in dòng chẩn đoán đầu tiên
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

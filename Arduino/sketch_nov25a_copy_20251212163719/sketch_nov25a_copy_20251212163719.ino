#include <WiFi.h>
#include <WiFiManager.h>      // WiFi cấu hình qua AP
#include <PubSubClient.h>     // MQTT client
#include <DHT.h>              // DHT11
#include <Wire.h>             // I2C
#include <BH1750.h>           // BH1750 Light Sensor
#include <ESP32Servo.h>       // Servo

// ================== CẤU HÌNH CHÂN ==================
#define SOIL_PIN 34           // Cảm biến độ ẩm đất (analog)
#define DHTPIN   4            // Chân DATA của DHT11
#define DHTTYPE  DHT11
#define RELAY_PIN 27          // IN relay điều khiển bơm
#define LED_PIN 16            // PWM LED (MOSFET AOD4184)
#define SERVO_PIN 19          // Servo mái che
#define RAIN_PIN  23          // Cảm biến mưa D0

const bool RELAY_ACTIVE_LOW = true;
const bool RAIN_ACTIVE_LOW  = true;  // nếu ngược thì đổi false

DHT dht(DHTPIN, DHTTYPE);
BH1750 lightMeter;            // BH1750 dùng I2C (SDA=21, SCL=22 mặc định)
Servo roofServo;              // Servo mái che

// ================== BIẾN LƯU TRẠNG THÁI CẢM BIẾN ==================
int   soilRaw = 0;            // Chỉ giữ RAW
float dhtTemp = 0.0;          // Chỉ giữ NHIỆT ĐỘ
float lightLux = 0.0;         // Cường độ ánh sáng (lux)
bool  bh1750Connected = false; // Trạng thái kết nối BH1750

// ================== TRẠNG THÁI BƠM ==================
bool pumpOn = false;
unsigned long pumpEndMs = 0;

// ================== CẤU HÌNH PWM LED ==================
#define PWM_CHANNEL 0
#define PWM_FREQ 5000
#define PWM_RESOLUTION 8
int ledBrightness = 0;        // Độ sáng hiện tại (0-255)

// ================== CẤU HÌNH MQTT ==================
WiFiClient espClient;
PubSubClient mqttClient(espClient);

// ĐIỀN IP BROKER CỦA BẠN Ở ĐÂY
const char* MQTT_SERVER = "10.101.217.152";
const int   MQTT_PORT      = 1883;
const char* MQTT_CLIENT_ID = "esp32-garden-1";

// Các topic MQTT
const char* TOPIC_SOIL_RAW   = "garden/soil/raw";
const char* TOPIC_DHT_TEMP   = "garden/dht/temp";
const char* TOPIC_LIGHT_LUX  = "garden/light/lux";

// Topic trạng thái bơm
const char* TOPIC_PUMP_STATE = "garden/pump/state";

// Topic điều khiển máy bơm (nhận lệnh từ Node-RED)
const char* TOPIC_PUMP_CONTROL = "garden/pump/control";

// Topic điều khiển độ sáng LED (nhận từ Node-RED)
const char* TOPIC_LED_BRIGHTNESS = "garden/led/brightness";

// Topic trạng thái mái che servo
const char* TOPIC_ROOF_STATE = "garden/roof/state";

// ================== TIMER ==================
unsigned long lastSensorRead = 0;
const unsigned long SENSOR_INTERVAL_MS = 5000; // 5 giây đọc & gửi 1 lần

unsigned long lastBH1750Check = 0;
const unsigned long BH1750_CHECK_INTERVAL_MS = 10000; // 10 giây kiểm tra 1 lần

unsigned long lastRainCheck = 0;
const unsigned long RAIN_CHECK_INTERVAL_MS = 200; // 200ms kiểm tra mưa

// ================== HÀM HỖ TRỢ RELAY & RAIN ==================
void relayWrite(bool on) {
  if (RELAY_ACTIVE_LOW) {
    digitalWrite(RELAY_PIN, on ? LOW : HIGH);
  } else {
    digitalWrite(RELAY_PIN, on ? HIGH : LOW);
  }
  pumpOn = on;
}

bool isRaining() {
  int v = digitalRead(RAIN_PIN);
  return RAIN_ACTIVE_LOW ? (v == LOW) : (v == HIGH);
}

void pumpStart30s() {
  pumpOn = true;
  pumpEndMs = millis() + 30000UL;
  relayWrite(true);
  mqttClient.publish(TOPIC_PUMP_STATE, "ON_30S");
  Serial.println("[PUMP] ON for 30 seconds");
}

void pumpStop() {
  pumpOn = false;
  relayWrite(false);
  mqttClient.publish(TOPIC_PUMP_STATE, "OFF");
  Serial.println("[PUMP] OFF");
}

void handlePumpTimeout() {
  if (!pumpOn) return;
  if ((long)(millis() - pumpEndMs) >= 0) {
    pumpStop();
  }
}

// ================== SERVO QUAY LIÊN TỤC: 20 VÒNG ==================
const int SERVO_STOP_US = 1500;
const int SERVO_CW_US   = 1700;  // quay chiều A
const int SERVO_CCW_US  = 1300;  // quay chiều ngược

const unsigned long REV_TIME_MS = 1000; // thời gian 1 vòng (ms)
const int TURNS = 20;
const unsigned long SERVO_RUN_MS = (unsigned long)TURNS * REV_TIME_MS;

bool lastRainState = false;
bool servoBusy = false;
int  servoDir = 0; // +1 CW, -1 CCW
unsigned long servoEndMs = 0;

void servoStop() {
  roofServo.writeMicroseconds(SERVO_STOP_US);
  mqttClient.publish(TOPIC_ROOF_STATE, "STOP");
}

void servoRunCW() {
  roofServo.writeMicroseconds(SERVO_CW_US);
  mqttClient.publish(TOPIC_ROOF_STATE, "CW");
}

void servoRunCCW() {
  roofServo.writeMicroseconds(SERVO_CCW_US);
  mqttClient.publish(TOPIC_ROOF_STATE, "CCW");
}

void startServoTurns(int dir) {
  if (servoBusy) return;

  servoBusy = true;
  servoDir = dir;
  servoEndMs = millis() + SERVO_RUN_MS;

  if (dir > 0) {
    Serial.println("[SERVO] RAIN -> rotate CW 20 turns then STOP");
    servoRunCW();
  } else {
    Serial.println("[SERVO] NO RAIN -> rotate CCW 20 turns then STOP");
    servoRunCCW();
  }
}

void handleServoTurns() {
  if (!servoBusy) return;
  if ((long)(millis() - servoEndMs) >= 0) {
    servoStop();
    servoBusy = false;
    Serial.println("[SERVO] STOP (done 20 turns)");
  }
}

// ================== HÀM ĐIỀU KHIỂN ĐỘ SÁNG LED ==================
void setLEDBrightness(int brightness) {
  // Giới hạn giá trị 0-255
  if (brightness < 0) brightness = 0;
  if (brightness > 255) brightness = 255;
  
  ledBrightness = brightness;
  
  #if ESP_ARDUINO_VERSION >= ESP_ARDUINO_VERSION_VAL(3, 0, 0)
    ledcWrite(LED_PIN, ledBrightness);
  #else
    ledcWrite(PWM_CHANNEL, ledBrightness);
  #endif
  
  Serial.print("LED Brightness set to: ");
  Serial.print(ledBrightness);
  Serial.print("/255 (");
  Serial.print((ledBrightness * 100) / 255);
  Serial.println("%)");
}

// ================== HÀM KẾT NỐI WI-FI (WIFIMANAGER) ==================
void setupWiFi() {
  WiFiManager wm;
  bool res = wm.autoConnect("ESP32-Garden-Config");

  if (!res) {
    Serial.println("Khong ket noi duoc WiFi. Reset lai ESP32...");
    delay(3000);
    ESP.restart();
  }

  Serial.println("Da ket noi WiFi!");
  Serial.print("IP: ");
  Serial.println(WiFi.localIP());
}

// ================== HÀM CALLBACK MQTT ==================
void mqttCallback(char* topic, byte* payload, unsigned int length) {
  String msg = "";
  for (unsigned int i = 0; i < length; i++) msg += (char)payload[i];

  Serial.print("MQTT message [");
  Serial.print(topic);
  Serial.print("]: ");
  Serial.println(msg);

  // Xử lý điều khiển độ sáng LED
  if (String(topic) == String(TOPIC_LED_BRIGHTNESS)) {
    int brightness = msg.toInt();
    setLEDBrightness(brightness);
    return;
  }
  
  // Xử lý điều khiển máy bơm
  if (String(topic) == String(TOPIC_PUMP_CONTROL)) {
    String m = msg;
    m.toLowerCase();
    
    if (m == "on" || m == "yes" || m == "1") {
      pumpStart30s();
    } else if (m == "off" || m == "no" || m == "0") {
      pumpStop();
    }
    return;
  }
}

// ================== HÀM KẾT NỐI LẠI MQTT NẾU MẤT ==================
void reconnectMQTT() {
  while (!mqttClient.connected()) {
    Serial.print("Dang ket noi MQTT...");
    if (mqttClient.connect(MQTT_CLIENT_ID)) {
      Serial.println("OK");
      mqttClient.subscribe(TOPIC_LED_BRIGHTNESS);
      mqttClient.subscribe(TOPIC_PUMP_CONTROL);
    } else {
      Serial.print("Failed, rc=");
      Serial.print(mqttClient.state());
      Serial.println(" -> Thu lai sau 2 giay");
      delay(2000);
    }
  }
}

// ================== HÀM ĐỌC CẢM BIẾN ĐỘ ẨM ĐẤT (RAW) ==================
void readSoilSensor() {
  soilRaw = analogRead(SOIL_PIN);
}

// ================== HÀM GỬI DỮ LIỆU SOIL RAW LÊN MQTT ==================
void sendSoilMQTT() {
  mqttClient.publish(TOPIC_SOIL_RAW, String(soilRaw).c_str());
}

// ================== HÀM ĐỌC DHT11 (CHỈ NHIỆT ĐỘ) ==================
void readDHT11() {
  float t = dht.readTemperature();
  if (!isnan(t)) dhtTemp = t;
}

// ================== HÀM GỬI DỮ LIỆU NHIỆT ĐỘ LÊN MQTT ==================
void sendDHTMQTT() {
  mqttClient.publish(TOPIC_DHT_TEMP, String(dhtTemp).c_str());
}

// ================== HÀM KIỂM TRA VÀ KẾT NỐI LẠI BH1750 ==================
void reconnectBH1750() {
  if (!bh1750Connected) {
    Serial.print("Dang thu ket noi lai BH1750...");
    Wire.begin();
    if (lightMeter.begin(BH1750::CONTINUOUS_HIGH_RES_MODE)) {
      Serial.println("Thanh cong!");
      bh1750Connected = true;
    } else {
      Serial.println("That bai! Thu lai sau.");
    }
  }
}

// ================== HÀM ĐỌC BH1750 (CƯỜNG ĐỘ ÁNH SÁNG) ==================
void readBH1750() {
  if (bh1750Connected) {
    if (lightMeter.measurementReady()) {
      float lux = lightMeter.readLightLevel();
      if (lux >= 0) {
        lightLux = lux;
      } else {
        Serial.println("Loi doc BH1750! Mat ket noi.");
        bh1750Connected = false;
        lightLux = -1.0;
      }
    }
  } else {
    lightLux = -1.0;
  }
}

// ================== HÀM GỬI DỮ LIỆU ÁNH SÁNG LÊN MQTT ==================
void sendLightMQTT() {
  if (bh1750Connected && lightLux >= 0) {
    mqttClient.publish(TOPIC_LIGHT_LUX, String(lightLux).c_str());
  }
}

// ================== HÀM IN RA SERIAL ĐỂ DEBUG ==================
void printSensorToSerial() {
  Serial.println("===== SENSOR DATA =====");
  Serial.print("Soil raw: "); Serial.println(soilRaw);
  Serial.print("Temp: "); Serial.print(dhtTemp); Serial.println(" C");
  Serial.print("Light: "); Serial.print(lightLux); Serial.println(" lux");
  Serial.print("Raining: "); Serial.println(isRaining() ? "YES" : "NO");
  Serial.print("Pump: "); Serial.println(pumpOn ? "ON" : "OFF");
  Serial.print("ServoBusy: "); Serial.println(servoBusy ? "YES" : "NO");
  Serial.println("=======================\n");
}

// ================== SETUP ==================
void setup() {
  Serial.begin(9600);
  delay(1000);

  Serial.println();
  Serial.println("=== ESP32 GARDEN NODE STARTING ===");

  pinMode(SOIL_PIN, INPUT);
  dht.begin();

  // Rain sensor
  pinMode(RAIN_PIN, INPUT_PULLUP);

  // Relay
  pinMode(RELAY_PIN, OUTPUT);
  relayWrite(false); // tắt bơm lúc khởi động

  // Servo continuous rotation
  roofServo.setPeriodHertz(50);
  roofServo.attach(SERVO_PIN, 500, 2400);
  servoStop();

  // PWM LED
  #if ESP_ARDUINO_VERSION >= ESP_ARDUINO_VERSION_VAL(3, 0, 0)
    ledcAttach(LED_PIN, PWM_FREQ, PWM_RESOLUTION);
  #else
    ledcSetup(PWM_CHANNEL, PWM_FREQ, PWM_RESOLUTION);
    ledcAttachPin(LED_PIN, PWM_CHANNEL);
  #endif
  setLEDBrightness(0);  // Tắt LED lúc khởi động
  Serial.println("LED PWM initialized on GPIO 16");

  // Khởi tạo BH1750
  Wire.begin();
  if (lightMeter.begin(BH1750::CONTINUOUS_HIGH_RES_MODE)) {
    Serial.println("BH1750 khoi tao thanh cong");
    bh1750Connected = true;
  } else {
    Serial.println("Loi khoi tao BH1750! Se thu ket noi lai...");
    bh1750Connected = false;
  }

  // 1. Kết nối WiFi qua WiFiManager
  setupWiFi();

  // 2. Cấu hình MQTT
  mqttClient.setServer(MQTT_SERVER, MQTT_PORT);
  mqttClient.setCallback(mqttCallback);

  // 3. Init trạng thái mưa
  lastRainState = isRaining();
  if (lastRainState) startServoTurns(+1);
}

// ================== LOOP ==================
void loop() {
  if (!mqttClient.connected()) reconnectMQTT();
  mqttClient.loop();
  
  // 1) Xử lý servo quay đủ 20 vòng rồi dừng
  handleServoTurns();

  // 2) Xử lý tự động tắt bơm sau 30s
  handlePumpTimeout();

  unsigned long now = millis();

  // 3) Kiểm tra mưa: đổi trạng thái thì chạy servo 20 vòng
  if (now - lastRainCheck >= RAIN_CHECK_INTERVAL_MS) {
    lastRainCheck = now;

    bool rainingNow = isRaining();
    if (rainingNow != lastRainState) {
      lastRainState = rainingNow;

      if (rainingNow) startServoTurns(+1); // mưa -> quay chiều A 20 vòng
      else            startServoTurns(-1); // hết mưa -> quay ngược 20 vòng
    }
  }

  // 4) Kiểm tra và kết nối lại BH1750 nếu mất kết nối
  if (now - lastBH1750Check >= BH1750_CHECK_INTERVAL_MS) {
    lastBH1750Check = now;
    reconnectBH1750();
  }

  // 5) Đọc cảm biến và gửi dữ liệu theo chu kỳ
  if (now - lastSensorRead >= SENSOR_INTERVAL_MS) {
    lastSensorRead = now;

    readSoilSensor();
    readDHT11();
    readBH1750();

    printSensorToSerial();

    sendSoilMQTT();
    sendDHTMQTT();
    sendLightMQTT();
  }
}
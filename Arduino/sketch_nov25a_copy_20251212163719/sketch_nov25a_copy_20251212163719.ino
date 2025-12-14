#include <WiFi.h>
#include <WiFiManager.h>      // WiFi cấu hình qua AP
#include <PubSubClient.h>     // MQTT client
#include <DHT.h>              // DHT11
#include <Wire.h>             // I2C
#include <BH1750.h>           // BH1750 Light Sensor

// ================== CẤU HÌNH CHÂN ==================
#define SOIL_PIN 34           // Cảm biến độ ẩm đất (analog)
#define DHTPIN   4            // Chân DATA của DHT11
#define DHTTYPE  DHT11
#define RELAY_PIN 27          // IN relay điều khiển bơm
#define LED_PIN 16            // PWM LED (MOSFET AOD4184)

// Logic phổ biến của relay 5V: IN LOW = ON (nếu ngược thì đổi false)
const bool RELAY_ACTIVE_LOW = true;

DHT dht(DHTPIN, DHTTYPE);
BH1750 lightMeter;            // BH1750 dùng I2C (SDA=21, SCL=22 mặc định)

// ================== BIẾN LƯU TRẠNG THÁI CẢM BIẾN ==================
int   soilRaw = 0;            // Chỉ giữ RAW
float dhtTemp = 0.0;          // Chỉ giữ NHIỆT ĐỘ
float lightLux = 0.0;         // Cường độ ánh sáng (lux)
bool  bh1750Connected = false; // Trạng thái kết nối BH1750

// ================== TRẠNG THÁI BƠM ==================
bool pumpOn = false;

// ================== CẤU HÌNH PWM LED ==================
#define PWM_CHANNEL 0
#define PWM_FREQ 5000
#define PWM_RESOLUTION 8
int ledBrightness = 0;        // Độ sáng hiện tại (0-255)

// ================== CẤU HÌNH MQTT ==================
WiFiClient espClient;
PubSubClient mqttClient(espClient);

// ĐIỀN IP BROKER CỦA BẠN Ở ĐÂY
const char* MQTT_SERVER = "10.166.190.152";
const int   MQTT_PORT      = 1883;
const char* MQTT_CLIENT_ID = "esp32-garden-1";

// Các topic MQTT
const char* TOPIC_SOIL_RAW   = "garden/soil/raw";
const char* TOPIC_DHT_TEMP   = "garden/dht/temp";
const char* TOPIC_LIGHT_LUX  = "garden/light/lux";

// Topic phục vụ chatbot đơn giản
const char* TOPIC_CHATBOT_CMD   = "garden/chatbot/cmd";
const char* TOPIC_CHATBOT_REPLY = "garden/chatbot/reply";

// Topic trạng thái bơm
const char* TOPIC_PUMP_STATE = "garden/pump/state";

// Topic điều khiển độ sáng LED (nhận từ Node-RED)
const char* TOPIC_LED_BRIGHTNESS = "garden/led/brightness";

// ================== TIMER ==================
unsigned long lastSensorRead = 0;
const unsigned long SENSOR_INTERVAL_MS = 5000; // 5 giây đọc & gửi 1 lần

unsigned long lastBH1750Check = 0;
const unsigned long BH1750_CHECK_INTERVAL_MS = 10000; // 10 giây kiểm tra 1 lần

// ================== HÀM HỖ TRỢ RELAY ==================
void relayWrite(bool on) {
  if (RELAY_ACTIVE_LOW) {
    digitalWrite(RELAY_PIN, on ? LOW : HIGH);
  } else {
    digitalWrite(RELAY_PIN, on ? HIGH : LOW);
  }
  pumpOn = on;
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

// ================== HÀM CALLBACK MQTT (XỬ LÝ CHATBOT CƠ BẢN) ==================
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
  
  if (String(topic) == String(TOPIC_CHATBOT_CMD)) {
    String reply;

    // --- lệnh cũ ---
    if (msg == "soil") {
      reply = "Do am dat RAW: " + String(soilRaw);
    } else if (msg == "temp") {
      reply = "Nhiet do: " + String(dhtTemp) + " C";
    } else if (msg == "light") {
      reply = "Anh sang: " + String(lightLux) + " lux";

    // --- lệnh mới ---
    } else if (msg == "pump_on") {
      relayWrite(true);
      mqttClient.publish(TOPIC_PUMP_STATE, "ON");
      reply = "Pump: ON";
    } else if (msg == "pump_off") {
      relayWrite(false);
      mqttClient.publish(TOPIC_PUMP_STATE, "OFF");
      reply = "Pump: OFF";

    } else if (msg == "all" || msg == "status") {
      reply  = "Soil RAW: " + String(soilRaw);
      reply += ", Temp: " + String(dhtTemp) + " C";
      reply += ", Light: " + String(lightLux) + " lux";
      reply += ", Pump: " + String(pumpOn ? "ON" : "OFF");
    } else {
      reply = "Lenh: soil/temp/light/status/all/pump_on/pump_off";
    }

    mqttClient.publish(TOPIC_CHATBOT_REPLY, reply.c_str());
  }
}

// ================== HÀM KẾT NỐI LẠI MQTT NẾU MẤT ==================
void reconnectMQTT() {
  while (!mqttClient.connected()) {
    Serial.print("Dang ket noi MQTT...");
    if (mqttClient.connect(MQTT_CLIENT_ID)) {
      Serial.println("OK");
      mqttClient.subscribe(TOPIC_CHATBOT_CMD);
      mqttClient.subscribe(TOPIC_LED_BRIGHTNESS);
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
  Serial.print("Pump: "); Serial.println(pumpOn ? "ON" : "OFF");
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

  // Relay
  pinMode(RELAY_PIN, OUTPUT);
  relayWrite(false); // tắt bơm lúc khởi động

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
}

// ================== LOOP ==================
void loop() {
  if (!mqttClient.connected()) reconnectMQTT();
  mqttClient.loop();

  unsigned long now = millis();

  // Kiểm tra và kết nối lại BH1750 nếu mất kết nối
  if (now - lastBH1750Check >= BH1750_CHECK_INTERVAL_MS) {
    lastBH1750Check = now;
    reconnectBH1750();
  }

  // Đọc cảm biến và gửi dữ liệu theo chu kỳ
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
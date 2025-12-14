/*
 * Test MOSFET AOD4184 với PWM
 * Điều khiển độ sáng LED qua chân GPIO 16
 * MOSFET: AOD4184 (P-Channel)
 * 
 * Kết nối:
 * - ESP32 GPIO 16 → Gate của MOSFET (qua điện trở 10kΩ)
 * - Source của MOSFET → VCC (3.3V hoặc 5V)
 * - Drain của MOSFET → LED Anode (+)
 * - LED Cathode (-) → Resistor 220Ω → GND
 */

// Cấu hình chân PWM
#define LED_PIN 16           // Chân điều khiển MOSFET
#define PWM_CHANNEL 0        // Kênh PWM (0-15)
#define PWM_FREQ 5000        // Tần số PWM 5kHz
#define PWM_RESOLUTION 8     // 8-bit resolution (0-255)

void setup() {
  Serial.begin(9600);
  delay(1000);
  
  Serial.println("\n=== Test MOSFET AOD4184 PWM ===");
  
  // Cấu hình PWM - Tương thích ESP32 Core 3.x
  #if ESP_ARDUINO_VERSION >= ESP_ARDUINO_VERSION_VAL(3, 0, 0)
    // ESP32 Core 3.x trở lên - API mới
    ledcAttach(LED_PIN, PWM_FREQ, PWM_RESOLUTION);
  #else
    // ESP32 Core 2.x - API cũ
    ledcSetup(PWM_CHANNEL, PWM_FREQ, PWM_RESOLUTION);
    ledcAttachPin(LED_PIN, PWM_CHANNEL);
  #endif
  
  Serial.println("PWM initialized on GPIO 16");
  Serial.println("Channel: 0, Freq: 5kHz, Resolution: 8-bit");
}

void loop() {
  // Test 1: Tăng dần độ sáng (0 → 255)
  Serial.println("\n--- Fade IN (0 → 255) ---");
  for (int brightness = 0; brightness <= 255; brightness++) {
    #if ESP_ARDUINO_VERSION >= ESP_ARDUINO_VERSION_VAL(3, 0, 0)
      ledcWrite(LED_PIN, brightness);
    #else
      ledcWrite(PWM_CHANNEL, brightness);
    #endif
    Serial.printf("Brightness: %d/255 (%.1f%%)\n", brightness, (brightness/255.0)*100);
    delay(10);
  }
  
  delay(500);
  
  // Test 2: Giảm dần độ sáng (255 → 0)
  Serial.println("\n--- Fade OUT (255 → 0) ---");
  for (int brightness = 255; brightness >= 0; brightness--) {
    #if ESP_ARDUINO_VERSION >= ESP_ARDUINO_VERSION_VAL(3, 0, 0)
      ledcWrite(LED_PIN, brightness);
    #else
      ledcWrite(PWM_CHANNEL, brightness);
    #endif
    Serial.printf("Brightness: %d/255 (%.1f%%)\n", brightness, (brightness/255.0)*100);
    delay(10);
  }
  
  delay(500);
  
  // Test 3: Các mức độ sáng cố định
  Serial.println("\n--- Fixed Levels ---");
  
  int levels[] = {0, 64, 128, 192, 255};
  String labels[] = {"OFF (0%)", "25%", "50%", "75%", "100%"};
  
  for (int i = 0; i < 5; i++) {
    #if ESP_ARDUINO_VERSION >= ESP_ARDUINO_VERSION_VAL(3, 0, 0)
      ledcWrite(LED_PIN, levels[i]);
    #else
      ledcWrite(PWM_CHANNEL, levels[i]);
    #endif
    Serial.printf("%s - PWM: %d\n", labels[i].c_str(), levels[i]);
    delay(1000);
  }
  
  delay(1000);
  
  // Test 4: Nhấp nháy
  Serial.println("\n--- Blinking ---");
  for (int i = 0; i < 5; i++) {
    #if ESP_ARDUINO_VERSION >= ESP_ARDUINO_VERSION_VAL(3, 0, 0)
      ledcWrite(LED_PIN, 255);
    #else
      ledcWrite(PWM_CHANNEL, 255);
    #endif  // Sáng
    Serial.println("ON");
    delay(300);
    
    #if ESP_ARDUINO_VERSION >= ESP_ARDUINO_VERSION_VAL(3, 0, 0)
      ledcWrite(LED_PIN, 0);
    #else
      ledcWrite(PWM_CHANNEL, 0);
    #endif    // Tắt
    Serial.println("OFF");
    delay(300);
  }
  
  delay(2000);
}

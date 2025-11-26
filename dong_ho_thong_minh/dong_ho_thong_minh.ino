#include <Wire.h> 
#include <SPI.h>
#include <U8g2lib.h>
#include <WiFi.h>
#include <time.h>

// ================== I2C ADDRESSES ==================
#define MAX30100_ADDR 0x57
#define ADXL345_ADDR  0x53
#define SHT30_ADDR    0x44

// Nút bấm chuyển chế độ
#define BUTTON_PIN 26   // GPIO26

// ================== WiFi & Thời gian thực ==================
const char* ssid     = "Nha 05 - P401";    
const char* password = "66668888";         

// Việt Nam GMT+7
const long  gmtOffset_sec      = 7 * 3600;
const int   daylightOffset_sec = 0;
// hiệu chỉnh sht30
const float TEMP_OFFSET = -1.5f;   // chỉnh cho phù hợp với thực tế
const float HUM_OFFSET  = 0.0f;    // 
float t;
float rh;

// ================== OLED SH1106 SPI (U8g2) ==================
// ESP32: SCK = 18, MOSI = 23, DC = 17, RST = 16, CS = none (GND)
U8G2_SH1106_128X64_NONAME_F_4W_HW_SPI u8g2(
  U8G2_R0,           // rotation
  U8X8_PIN_NONE,     // CS (không có chân CS)
  17,                // DC  -> GPIO17
  16                 // RST -> GPIO16
);

// ================== MAX30100 REGISTERS ==================
#define MAX30100_FIFO_WR_PTR    0x02
#define MAX30100_OVF_COUNTER    0x03
#define MAX30100_FIFO_RD_PTR    0x04
#define MAX30100_FIFO_DATA      0x05
#define MAX30100_MODE_CONFIG    0x06
#define MAX30100_SPO2_CONFIG    0x07
#define MAX30100_LED_CONFIG     0x09

// ================== ADXL345 REGISTERS ==================
#define ADXL345_POWER_CTL   0x2D
#define ADXL345_DATA_FORMAT 0x31
#define ADXL345_DATAX0      0x32  // X0, X1, Y0, Y1, Z0, Z1

// ================== I2C HELPER ==================
void i2cWrite8(uint8_t addr, uint8_t reg, uint8_t val) {
  Wire.beginTransmission(addr);
  Wire.write(reg);
  Wire.write(val);
  Wire.endTransmission();
}

uint8_t i2cRead8(uint8_t addr, uint8_t reg) {
  Wire.beginTransmission(addr);
  Wire.write(reg);
  Wire.endTransmission(false);
  Wire.requestFrom(addr, (uint8_t)1);
  if (Wire.available()) {
    return Wire.read();
  }
  return 0;
}

void i2cReadBytes(uint8_t addr, uint8_t reg, uint8_t *buf, uint8_t len) {
  Wire.beginTransmission(addr);
  Wire.write(reg);
  Wire.endTransmission(false);
  Wire.requestFrom(addr, len);
  uint8_t i = 0;
  while (Wire.available() && i < len) {
    buf[i++] = Wire.read();
  }
}

// ================== MAX30100 INIT & READ ==================
bool initMAX30100() {
  // Reset FIFO
  i2cWrite8(MAX30100_ADDR, MAX30100_FIFO_WR_PTR, 0x00);
  i2cWrite8(MAX30100_ADDR, MAX30100_OVF_COUNTER, 0x00);
  i2cWrite8(MAX30100_ADDR, MAX30100_FIFO_RD_PTR, 0x00);

  // SpO2 mode
  i2cWrite8(MAX30100_ADDR, MAX30100_MODE_CONFIG, 0x03);

  // SPO2_CONFIG: hi-res, 100 Hz, 1600us
  i2cWrite8(MAX30100_ADDR, MAX30100_SPO2_CONFIG, 0x4F);

  // LED current: RED=IR=0x0F
  i2cWrite8(MAX30100_ADDR, MAX30100_LED_CONFIG, 0xFF);

  delay(10);
  uint8_t mode = i2cRead8(MAX30100_ADDR, MAX30100_MODE_CONFIG);
  return (mode & 0x07) == 0x03;
}

// Đọc 1 sample IR + RED từ FIFO (4 byte)
bool readMAX30100Raw(uint16_t &ir, uint16_t &red) {
  uint8_t buf[4];
  i2cReadBytes(MAX30100_ADDR, MAX30100_FIFO_DATA, buf, 4);

  ir  = ((uint16_t)buf[0] << 8) | buf[1];
  red = ((uint16_t)buf[2] << 8) | buf[3];

  return true;
}

// ================== ADXL345 INIT & READ ==================
bool initADXL345() {
  i2cWrite8(ADXL345_ADDR, ADXL345_DATA_FORMAT, 0x08); // full-res, ±2g
  i2cWrite8(ADXL345_ADDR, ADXL345_POWER_CTL, 0x08);   // measure = 1
  delay(10);
  uint8_t pctl = i2cRead8(ADXL345_ADDR, ADXL345_POWER_CTL);
  return (pctl & 0x08) != 0;
}

void readADXL345(float &ax, float &ay, float &az) {
  uint8_t buf[6];
  i2cReadBytes(ADXL345_ADDR, ADXL345_DATAX0, buf, 6);

  int16_t x = (int16_t)((buf[1] << 8) | buf[0]);
  int16_t y = (int16_t)((buf[3] << 8) | buf[2]);
  int16_t z = (int16_t)((buf[5] << 8) | buf[4]);

  const float scale = 0.0039f;
  ax = x * scale;
  ay = y * scale;
  az = z * scale;
}

// ================== SHT30 READ (thuần thanh ghi) ==================
bool readSHT30(float &temperature, float &humidity) {
  // Gửi lệnh đo single-shot high repeatability: 0x2400
  Wire.beginTransmission(SHT30_ADDR);
  Wire.write(0x24);
  Wire.write(0x00);
  if (Wire.endTransmission() != 0) {
    return false;
  }

  // Đợi sensor đo xong (datasheet: ~15 ms)
  delay(20);

  // Đọc 6 byte: T_MSB, T_LSB, T_CRC, RH_MSB, RH_LSB, RH_CRC
  Wire.requestFrom(SHT30_ADDR, (uint8_t)6);
  if (Wire.available() < 6) {
    return false;
  }

  uint8_t data[6];
  for (int i = 0; i < 6; i++) {
    data[i] = Wire.read();
  }

  // Ghép raw
  uint16_t rawT  = ((uint16_t)data[0] << 8) | data[1];
  uint16_t rawRH = ((uint16_t)data[3] << 8) | data[4];

  // Tính nhiệt độ chuẩn theo datasheet
  float t  = -45.0f + 175.0f * ((float)rawT / 65535.0f);

  // Áp dụng offset nếu bạn muốn điều chỉnh
  t += TEMP_OFFSET;

  // Gán kết quả
  temperature = t;

  // Nếu bạn không dùng RH thì giữ nguyên như cũ:
  humidity = 100.0f * ((float)rawRH / 65535.0f);

  return true;
}


// ================== THUẬT TOÁN HR + SpO2 ==================
const int SAMPLE_INTERVAL_MS = 10;   // ~100 Hz
uint32_t lastSampleTime = 0;

const int BUFFER_SIZE = 200;        // 2 giây dữ liệu
uint16_t irBuffer[BUFFER_SIZE];
uint16_t redBuffer[BUFFER_SIZE];
int bufferIndex = 0;
bool bufferFilled = false;

float heartRate = 0;
bool validHR = false;
bool lastAbove = false;
uint32_t lastPeakTime = 0;

float spo2 = 0;
bool validSpO2 = false;

bool fingerOnSensor = false;
float dcIR_ema = 0;
float acAbsAvg = 0;
const float DC_ALPHA   = 0.95f;
const float NOISE_ALPHA= 0.95f;
const uint32_t MIN_RR_MS     = 400;
const uint32_t MAX_RR_MS     = 1500;
const uint32_t REFRACTORY_MS = 300;
const uint16_t IR_FINGER_ON  = 5000;
const uint16_t IR_FINGER_OFF = 2000;

// RR smoothing
const int HR_AVG_N = 5;
uint16_t rrIntervals[HR_AVG_N];
int rrIndex = 0;
int rrCount = 0;
const float MAX_HR_JUMP = 20.0f;
uint16_t lastIR = 0;

// Globals for SHT30 values
float g_temperature = NAN;
float g_humidity    = NAN;
uint32_t lastSHTRead = 0;
const uint32_t SHT_INTERVAL_MS = 500;   // đọc mỗi 0.5 giây

// ================== BUTTON DEBOUNCE & MODE ==================
// Dùng đúng kiểu Arduino debounce
int buttonState       = HIGH;  // trạng thái đã debounce (HIGH/LOW)
int lastButtonReading = HIGH;  // lần đọc trước (raw)
unsigned long lastDebounceTime = 0;
const unsigned long debounceDelay = 50;  // ms

// Chế độ hoạt động: TRUE = SENSOR, FALSE = CLOCK
bool sensorMode = true;

// ================== FLAGS & STATE ==================
bool haveMAX  = false;
bool haveADXL = false;
bool haveSHT  = false;
bool sensorsInited = false;

uint32_t lastDisplayUpdate = 0;

// ================== KHỞI TẠO WIFI + THỜI GIAN ==================
void initWiFiAndTime() {
  Serial.print("Connecting to WiFi");
  WiFi.begin(ssid, password);
  while (WiFi.status() != WL_CONNECTED) {
    delay(500);
    Serial.print(".");
  }
  Serial.println(" connected!");

  // Cấu hình NTP
  configTime(gmtOffset_sec, daylightOffset_sec, "pool.ntp.org", "time.nist.gov");

  Serial.print("Syncing time");
  struct tm timeinfo;
  while (!getLocalTime(&timeinfo)) {
    Serial.print(".");
    delay(500);
  }
  Serial.println(" done!");
  Serial.printf("Current time: %02d:%02d:%02d\n", timeinfo.tm_hour, timeinfo.tm_min, timeinfo.tm_sec);
}

// ================== HR SAMPLE ==================
void sampleMaxAndUpdateHR() {
  if (millis() - lastSampleTime < SAMPLE_INTERVAL_MS) return;
  lastSampleTime = millis();

  uint16_t ir = 0, red = 0;
  if (!readMAX30100Raw(ir, red)) return;
  lastIR = ir;

  irBuffer[bufferIndex]  = ir;
  redBuffer[bufferIndex] = red;
  bufferIndex++;
  if (bufferIndex >= BUFFER_SIZE) {
    bufferIndex = 0;
    bufferFilled = true;
  }

  int count = bufferFilled ? BUFFER_SIZE : bufferIndex;
  if (count < 20) return;

  // PHÁT HIỆN CÓ / KHÔNG CÓ TAY
  if (!fingerOnSensor) {
    if (ir > IR_FINGER_ON) {
      fingerOnSensor = true;
      validHR      = false;
      heartRate    = 0;
      rrIndex      = 0;
      rrCount      = 0;
      lastPeakTime = 0;
      lastAbove    = false;
      dcIR_ema     = ir;
      acAbsAvg     = 0;
    } else {
      validHR = false;
      return;
    }
  } else {
    if (ir < IR_FINGER_OFF) {
      fingerOnSensor = false;
      validHR      = false;
      heartRate    = 0;
      rrIndex      = 0;
      rrCount      = 0;
      lastPeakTime = 0;
      lastAbove    = false;
      return;
    }
  }

  // LỌC DC & TÍNH AC
  if (dcIR_ema == 0) dcIR_ema = ir;
  else dcIR_ema = DC_ALPHA * dcIR_ema + (1.0f - DC_ALPHA) * ir;

  float ac = ir - dcIR_ema;
  float acAbs = fabs(ac);
  acAbsAvg = NOISE_ALPHA * acAbsAvg + (1.0f - NOISE_ALPHA) * acAbs;

  const float MIN_AC = 25.0f;
  if (acAbsAvg < MIN_AC) {
    validHR = false;
    return;
  }

  float threshold = acAbsAvg * 0.6f;
  const float MIN_THR = 20.0f;
  if (threshold < MIN_THR) threshold = MIN_THR;

  bool above = (ac > threshold);
  uint32_t now = millis();

  if (lastPeakTime != 0 && (now - lastPeakTime) < REFRACTORY_MS) {
    lastAbove = above;
    return;
  }

  if (!lastAbove && above) {
    if (lastPeakTime != 0) {
      uint32_t interval = now - lastPeakTime;
      if (interval > MIN_RR_MS && interval < MAX_RR_MS) {
        rrIntervals[rrIndex] = interval;
        rrIndex = (rrIndex + 1) % HR_AVG_N;
        if (rrCount < HR_AVG_N) rrCount++;

        float sum = 0;
        for (int i = 0; i < rrCount; i++) sum += rrIntervals[i];
        float avgRR = sum / rrCount;
        float newHR = 60000.0f / avgRR;

        if (validHR && fabs(newHR - heartRate) > MAX_HR_JUMP) {
          // ignore spike
        } else {
          if (!validHR) heartRate = newHR;
          else heartRate = 0.90f * heartRate + 0.10f * newHR;
          validHR = true;
        }
      }
    }
    lastPeakTime = now;
  }
  lastAbove = above;
}

void computeSpO2() {
  int count = bufferFilled ? BUFFER_SIZE : bufferIndex;
  if (count < 50) {
    validSpO2 = false;
    return;
  }

  uint32_t sumIR = 0, sumRED = 0;
  for (int i = 0; i < count; i++) {
    sumIR  += irBuffer[i];
    sumRED += redBuffer[i];
  }
  float dcIR  = (float)sumIR / count;
  float dcRED = (float)sumRED / count;

  double sqSumIR  = 0;
  double sqSumRED = 0;
  for (int i = 0; i < count; i++) {
    float xIR  = irBuffer[i]  - dcIR;
    float xRED = redBuffer[i] - dcRED;
    sqSumIR  += xIR * xIR;
    sqSumRED += xRED * xRED;
  }

  float acIR  = sqrt(sqSumIR  / count);
  float acRED = sqrt(sqSumRED / count);

  if (dcIR < 1000 || dcRED < 1000 || acIR < 1 || acRED < 1) {
    validSpO2 = false;
    return;
  }

  float R = (acRED / dcRED) / (acIR / dcIR);
  float s = 110.0f - 25.0f * R;  // xấp xỉ demo

  if (s > 100) s = 100;
  if (s < 50)  s = 50;

  spo2 = s;
  validSpO2 = true;
}

// ================== SETUP ==================
void setup() {
  Serial.begin(115200);
  delay(2000);
  Serial.println();
  Serial.println("ESP32 + MAX30100 + ADXL345 + SHT30 + OLED SH1106 (SPI)");

  Wire.begin(21, 22);
  Wire.setClock(100000);

  pinMode(BUTTON_PIN, INPUT_PULLUP);
  buttonState       = digitalRead(BUTTON_PIN);
  lastButtonReading = buttonState;

  u8g2.begin();
  u8g2.clearBuffer();
  u8g2.setFont(u8g2_font_6x10_tf);
  u8g2.drawStr(0, 12, "Init sensors...");
  u8g2.sendBuffer();

  initWiFiAndTime();
}

// ================== LOOP ==================
void loop() {
  // ---- Debounce + TOGGLE MODE ----
  int reading = digitalRead(BUTTON_PIN);

  if (reading != lastButtonReading) {
    lastDebounceTime = millis();
  }

  if ((millis() - lastDebounceTime) > debounceDelay) {
    if (reading != buttonState) {
      buttonState = reading;

      // Nhấn nút (PULLUP -> LOW) => toggle mode
      if (buttonState == LOW) {
        sensorMode = !sensorMode;
        Serial.print("Mode changed to: ");
        Serial.println(sensorMode ? "SENSOR" : "CLOCK");
      }
    }
  }
  lastButtonReading = reading;

  // 1) Khởi tạo cảm biến 1 lần
  if (!sensorsInited) {
    Serial.println("Initializing sensors...");

    haveMAX  = initMAX30100();
    Serial.print("MAX30100: ");
    Serial.println(haveMAX ? "OK" : "FAIL");

    haveADXL = initADXL345();
    Serial.print("ADXL345: ");
    Serial.println(haveADXL ? "OK" : "FAIL");

    float T, H;
    haveSHT  = readSHT30(T, H);
    Serial.print("SHT30: ");
    Serial.println(haveSHT ? "OK" : "FAIL");

    // Lưu lần đầu đọc được (nếu OK) vào biến global
    if (haveSHT) {
      g_temperature = T;
      g_humidity    = H;
    }

    sensorsInited = true;
    delay(200);
    lastDisplayUpdate = 0;
  }

  // 2) Lấy mẫu HR nếu ở SENSOR MODE
  if (sensorMode && haveMAX) {
    sampleMaxAndUpdateHR();
  }

  // 2b) Đọc SHT30 định kỳ (không phụ thuộc OLED)
  if (haveSHT && (millis() - lastSHTRead > SHT_INTERVAL_MS)) {
    lastSHTRead = millis();
    float T, H;
    if (readSHT30(T, H)) {
      g_temperature = T;
      g_humidity    = H;
    }
  }

  // 3) Cập nhật OLED mỗi giây
  if (millis() - lastDisplayUpdate > 1000) {
    lastDisplayUpdate = millis();

    // === CLOCK MODE ===
    if (!sensorMode) {
      u8g2.clearBuffer();

      struct tm timeinfo;
      if (!getLocalTime(&timeinfo)) {
        u8g2.setFont(u8g2_font_6x10_tf);
        u8g2.drawStr(0, 12, "TIME ERROR");
      } else {
        int hours   = timeinfo.tm_hour;
        int minutes = timeinfo.tm_min;
        int seconds = timeinfo.tm_sec;
        int day     = timeinfo.tm_mday;
        int month   = timeinfo.tm_mon + 1;
        int year    = timeinfo.tm_year + 1900;

        u8g2.setFont(u8g2_font_6x10_tf);
        u8g2.drawStr(0, 10, "DATE/TIME");

        char dateStr[20];
        snprintf(dateStr, sizeof(dateStr), "%02d/%02d/%04d", day, month, year);
        u8g2.drawStr(0, 22, dateStr);

        char timeStr[16];
        snprintf(timeStr, sizeof(timeStr), "%02d:%02d:%02d", hours, minutes, seconds);

        u8g2.setFont(u8g2_font_fub20_tr);
        u8g2.drawStr(10, 55, timeStr);
      }

      u8g2.setFont(u8g2_font_5x8_tf);
      u8g2.drawStr(0, 62, "MODE: CLOCK");

      u8g2.sendBuffer();
      return;
    }

    // === SENSOR MODE ===
    if (haveMAX) {
      computeSpO2();
    }

    float ax = 0, ay = 0, az = 0;
    if (haveADXL) {
      readADXL345(ax, ay, az);
    }

    // Dùng giá trị toàn cục đã được cập nhật định kỳ
    float temp = g_temperature;
    float hum  = g_humidity;

    // Debug Serial
    Serial.print("HR=");
    if (haveMAX && validHR) Serial.print(heartRate, 1); else Serial.print("NaN");
    Serial.print(" bpm, SpO2=");
    if (haveMAX && validSpO2) Serial.print(spo2, 1); else Serial.print("NaN");
    Serial.print("% | T=");
    if (!isnan(temp)) Serial.print(temp, 1); else Serial.print("Err");
    Serial.print("C, RH=");
    if (!isnan(hum)) Serial.print(hum, 1); else Serial.print("Err");
    Serial.print("% | AX=");
    Serial.print(ax, 2);
    Serial.print(" AY=");
    Serial.print(ay, 2);
    Serial.print(" AZ=");
    Serial.println(az, 2);

    // === CẢNH BÁO ===
    bool alert      = false;
    bool alertTemp  = false;
    bool alertHRLow = false;

    if (!isnan(temp) && temp > 31.0f) {
      alert = true;
      alertTemp = true;
    }
    if (haveMAX && validHR && heartRate < 50.0f) {
      alert = true;
      alertHRLow = true;
    }

    u8g2.clearBuffer();

    if (alert) {
      static bool blinkOn = false;
      blinkOn = !blinkOn;

      if (blinkOn) {
        u8g2.setFont(u8g2_font_ncenB14_tr);
        u8g2.drawStr(10, 24, "CANH BAO!");

        u8g2.setFont(u8g2_font_6x10_tf);
        int y2 = 40;
        if (alertTemp) {
          u8g2.drawStr(0, y2, "Nhiet do > 31C");
          y2 += 12;
        }
        if (alertHRLow) {
          u8g2.drawStr(0, y2, "Nhip tim < 50 bpm");
        }
      }

      u8g2.sendBuffer();
      return;
    }

    // Hiển thị bình thường
    u8g2.setFont(u8g2_font_6x10_tf);
    u8g2.drawStr(0, 10, "ESP32 HEALTH MON");

    char line[32];

    if (haveMAX && validHR) {
      snprintf(line, sizeof(line), "HR  : %3.0f bpm", heartRate);
    } else if (haveMAX) {
      snprintf(line, sizeof(line), "HR  : --- (wait)");
    } else {
      snprintf(line, sizeof(line), "HR  : N/A");
    }
    u8g2.drawStr(0, 22, line);

    if (haveMAX && validSpO2) {
      snprintf(line, sizeof(line), "SpO2: %3.0f %%", spo2);
    } else if (haveMAX) {
      snprintf(line, sizeof(line), "SpO2: --- (wait)");
    } else {
      snprintf(line, sizeof(line), "SpO2: N/A");
    }
    u8g2.drawStr(0, 32, line);

    if (!isnan(temp)) {
      snprintf(line, sizeof(line), "T   : %4.1f C", temp);
    } else {
      snprintf(line, sizeof(line), "T   : Err");
    }
    u8g2.drawStr(0, 42, line);

    u8g2.setFont(u8g2_font_5x8_tf);
    snprintf(line, sizeof(line), "AX:%4.1f AY:%4.1f AZ:%4.1f", ax, ay, az);
    u8g2.drawStr(0, 52, line);

    u8g2.setFont(u8g2_font_6x10_tf);
    u8g2.drawStr(0, 62, "Mode: SENSOR");

    u8g2.sendBuffer();
  }
}

/*
  كاشف هوية الدرون عبر Remote ID — ESP32
  ------------------------------------------------------------
  يستقبل بثّ Remote ID (معيار ASTM F3411 / OpenDroneID) بشكل سلبي
  عبر Bluetooth LE (Legacy Advertising) و WiFi Beacon على القناة 6،
  ويعرض هوية الطائرة على شاشة LCD 16x2 ويطبع التفاصيل كاملة على Serial.

  اللوحة: ESP32 DevKit (أو ESP32-S3)
  المكتبات (من Library Manager):
    - NimBLE-Arduino  (الإصدار 1.4.x)
    - LiquidCrystal I2C (Frank de Brabander)

  التوصيل:
    LCD I2C : SDA -> GPIO21 , SCL -> GPIO22 , VCC -> 5V , GND -> GND
    Buzzer  : GPIO25
    LED أحمر: GPIO26 (يضيء عند وجود درون)
    LED أخضر: GPIO27 (الأجواء خالية)

  ملاحظة: الجهاز يستقبل فقط، لا يرسل ولا يتدخل في الطائرة.
  الطائرات التي لا تبث Remote ID لن تظهر هنا.
*/

#include <Arduino.h>
#include <Wire.h>
#include <LiquidCrystal_I2C.h>
#include <NimBLEDevice.h>
#include <WiFi.h>
#include "esp_wifi.h"

// ---------------- الإعدادات ----------------
const int BUZZER_PIN    = 25;
const int LED_RED_PIN   = 26;
const int LED_GREEN_PIN = 27;
const int WIFI_CHANNEL  = 6;          // القناة المعتمدة لبث Remote ID على WiFi
const unsigned long LOST_MS = 10000;  // إذا انقطع البث 10 ثوانٍ نعتبر الطائرة غادرت
const unsigned long PAGE_MS = 2500;   // مدة عرض كل صفحة على الشاشة

LiquidCrystal_I2C lcd(0x27, 16, 2);   // جرّب 0x3F إذا لم يظهر شيء

// ---------------- بيانات الطائرة ----------------
struct DroneInfo {
  char  uasId[21]   = "";   // الرقم التسلسلي للطائرة
  char  opId[21]    = "";   // رقم تسجيل المشغّل
  char  selfId[24]  = "";   // وصف الرحلة الذي يكتبه المشغّل
  uint8_t uaType    = 0;
  double lat = 0, lon = 0;  // موقع الطائرة
  double opLat = 0, opLon = 0; // موقع الطيار
  float  alt = 0;           // الارتفاع (متر)
  float  speed = 0;         // السرعة الأفقية (م/ث)
  int    rssi = 0;
  char   source[5] = "";    // BLE أو WiFi
  unsigned long lastSeen = 0;
};

DroneInfo drone;
portMUX_TYPE droneMux = portMUX_INITIALIZER_UNLOCKED;
bool wasPresent = false;

// ---------------- أدوات مساعدة ----------------
static int32_t readI32(const uint8_t *p) {
  return (int32_t)((uint32_t)p[0] | ((uint32_t)p[1] << 8) |
                   ((uint32_t)p[2] << 16) | ((uint32_t)p[3] << 24));
}
static uint16_t readU16(const uint8_t *p) { return p[0] | (p[1] << 8); }

static void copyText(char *dst, const uint8_t *src, int len) {
  int j = 0;
  for (int i = 0; i < len; i++) {
    char c = (char)src[i];
    if (c == 0) break;
    if (c >= 32 && c < 127) dst[j++] = c;
  }
  dst[j] = 0;
  while (j > 0 && dst[j - 1] == ' ') dst[--j] = 0;  // حذف المسافات الزائدة
}

const char *uaTypeName(uint8_t t) {
  static const char *names[] = {
    "Unknown", "Aeroplane", "Multirotor", "Gyroplane", "VTOL", "Ornithopter",
    "Glider", "Kite", "Balloon", "Capt.Balloon", "Airship", "Parachute",
    "Rocket", "Tethered", "Obstacle", "Other"};
  return t < 16 ? names[t] : "Unknown";
}

// تصنيف الغرض (شحن / تصوير) من الوصف الذي يبثه المشغّل
const char *purposeOf(const char *desc) {
  String d = String(desc); d.toLowerCase();
  if (d.indexOf("deliver") >= 0 || d.indexOf("cargo") >= 0 ||
      d.indexOf("parcel") >= 0  || d.indexOf("package") >= 0) return "Cargo";
  if (d.indexOf("photo") >= 0 || d.indexOf("video") >= 0 ||
      d.indexOf("film") >= 0  || d.indexOf("survey") >= 0 ||
      d.indexOf("inspect") >= 0 || d.indexOf("mapping") >= 0) return "Photo";
  return "Unknown";
}

// ---------------- فك رسالة Remote ID واحدة (25 بايت) ----------------
void decodeMessage(const uint8_t *m, int rssi, const char *src) {
  uint8_t type = m[0] >> 4;

  portENTER_CRITICAL(&droneMux);
  switch (type) {
    case 0x0:  // Basic ID: الرقم التسلسلي ونوع الطائرة
      drone.uaType = m[1] & 0x0F;
      copyText(drone.uasId, &m[2], 20);
      break;

    case 0x1: {  // Location: الموقع والارتفاع والسرعة
      bool mult = m[1] & 0x01;
      drone.speed = mult ? (m[3] * 0.75f + 63.75f) : (m[3] * 0.25f);
      drone.lat = readI32(&m[5]) / 1e7;
      drone.lon = readI32(&m[9]) / 1e7;
      drone.alt = readU16(&m[15]) * 0.5f - 1000.0f;
      break;
    }

    case 0x3:  // Self ID: وصف الرحلة
      copyText(drone.selfId, &m[2], 23);
      break;

    case 0x4:  // System: موقع الطيار
      drone.opLat = readI32(&m[2]) / 1e7;
      drone.opLon = readI32(&m[6]) / 1e7;
      break;

    case 0x5:  // Operator ID: رقم تسجيل المشغّل
      copyText(drone.opId, &m[2], 20);
      break;
  }
  drone.rssi = rssi;
  strncpy(drone.source, src, sizeof(drone.source) - 1);
  drone.lastSeen = millis();
  portEXIT_CRITICAL(&droneMux);
}

// رسالة مفردة أو حزمة رسائل (Message Pack)
void decodeAny(const uint8_t *d, int len, int rssi, const char *src) {
  if (len < 25) return;
  if ((d[0] >> 4) == 0xF) {               // Message Pack
    uint8_t size = d[1], count = d[2];
    if (size != 25) return;
    for (int i = 0; i < count && 3 + (i + 1) * 25 <= len; i++)
      decodeMessage(&d[3 + i * 25], rssi, src);
  } else {
    decodeMessage(d, rssi, src);
  }
}

// ---------------- الاستقبال عبر Bluetooth ----------------
class RidBleCallbacks : public NimBLEAdvertisedDeviceCallbacks {
  void onResult(NimBLEAdvertisedDevice *dev) override {
    if (!dev->haveServiceData()) return;
    std::string sd = dev->getServiceData(NimBLEUUID((uint16_t)0xFFFA));
    // البنية: [0x0D app code][counter][message...]
    if (sd.size() < 27 || (uint8_t)sd[0] != 0x0D) return;
    decodeAny((const uint8_t *)sd.data() + 2, sd.size() - 2, dev->getRSSI(), "BLE");
  }
};

// ---------------- الاستقبال عبر WiFi Beacon ----------------
void IRAM_ATTR wifiSniffer(void *buf, wifi_promiscuous_pkt_type_t type) {
  if (type != WIFI_PKT_MGMT) return;
  const wifi_promiscuous_pkt_t *pkt = (wifi_promiscuous_pkt_t *)buf;
  const uint8_t *p = pkt->payload;
  int len = pkt->rx_ctrl.sig_len;
  if (len < 40 || p[0] != 0x80) return;   // Beacon فقط

  int i = 36;                              // بداية العناصر بعد الترويسة
  while (i + 2 <= len) {
    uint8_t id = p[i], elen = p[i + 1];
    if (i + 2 + elen > len) break;
    const uint8_t *e = &p[i + 2];
    // Vendor IE بمعرّف ASTM: FA:0B:BC ونوع 0x0D
    if (id == 221 && elen > 5 && e[0] == 0xFA && e[1] == 0x0B &&
        e[2] == 0xBC && e[3] == 0x0D) {
      decodeAny(&e[5], elen - 5, pkt->rx_ctrl.rssi, "WiFi");
    }
    i += 2 + elen;
  }
}

// ---------------- العرض ----------------
void showPage(int page, const DroneInfo &d) {
  char l1[17], l2[17];
  switch (page) {
    case 0:
      snprintf(l1, 17, "DRONE DETECTED!");
      snprintf(l2, 17, "%s %ddBm", d.source, d.rssi);
      break;
    case 1:
      snprintf(l1, 17, "Serial No:");
      snprintf(l2, 17, "%.16s", d.uasId[0] ? d.uasId : "--");
      break;
    case 2:
      snprintf(l1, 17, "Operator ID:");
      snprintf(l2, 17, "%.16s", d.opId[0] ? d.opId : "--");
      break;
    case 3:
      snprintf(l1, 17, "Use:%s", purposeOf(d.selfId));
      snprintf(l2, 17, "%.16s", uaTypeName(d.uaType));
      break;
    case 4:
      snprintf(l1, 17, "Lat:%.5f", d.lat);
      snprintf(l2, 17, "Lon:%.5f", d.lon);
      break;
    default:
      snprintf(l1, 17, "Alt:%.0fm", d.alt);
      snprintf(l2, 17, "Spd:%.1fm/s", d.speed);
      break;
  }
  lcd.clear();
  lcd.setCursor(0, 0); lcd.print(l1);
  lcd.setCursor(0, 1); lcd.print(l2);
}

void printSerial(const DroneInfo &d) {
  Serial.println(F("========== Remote ID =========="));
  Serial.printf("Source      : %s  RSSI %d dBm\n", d.source, d.rssi);
  Serial.printf("Serial No   : %s\n", d.uasId);
  Serial.printf("Operator ID : %s\n", d.opId);
  Serial.printf("UA Type     : %s\n", uaTypeName(d.uaType));
  Serial.printf("Self ID     : %s  -> %s\n", d.selfId, purposeOf(d.selfId));
  Serial.printf("Drone pos   : %.6f, %.6f  Alt %.0f m  Spd %.1f m/s\n",
                d.lat, d.lon, d.alt, d.speed);
  Serial.printf("Pilot pos   : %.6f, %.6f\n", d.opLat, d.opLon);
}

void alarmBeep() {
  for (int i = 0; i < 3; i++) {
    digitalWrite(BUZZER_PIN, HIGH); delay(120);
    digitalWrite(BUZZER_PIN, LOW);  delay(80);
  }
}

// ---------------- البداية ----------------
void setup() {
  Serial.begin(115200);
  pinMode(BUZZER_PIN, OUTPUT);
  pinMode(LED_RED_PIN, OUTPUT);
  pinMode(LED_GREEN_PIN, OUTPUT);

  lcd.init(); lcd.backlight();
  lcd.print("Drone ID Scanner");
  lcd.setCursor(0, 1); lcd.print("Initializing...");

  // WiFi في وضع الالتقاط السلبي على القناة 6
  WiFi.mode(WIFI_STA);
  WiFi.disconnect();
  esp_wifi_set_promiscuous(true);
  esp_wifi_set_promiscuous_rx_cb(&wifiSniffer);
  esp_wifi_set_channel(WIFI_CHANNEL, WIFI_SECOND_CHAN_NONE);

  // Bluetooth في وضع المسح السلبي المستمر
  NimBLEDevice::init("");
  NimBLEScan *scan = NimBLEDevice::getScan();
  scan->setAdvertisedDeviceCallbacks(new RidBleCallbacks(), true);
  scan->setActiveScan(false);
  scan->setInterval(100);
  scan->setWindow(99);
  scan->start(0, nullptr, false);

  delay(1500);
  lcd.clear();
}

// ---------------- الحلقة الرئيسية ----------------
void loop() {
  static unsigned long lastPage = 0, lastPrint = 0;
  static int page = 0;

  DroneInfo d;
  portENTER_CRITICAL(&droneMux);
  d = drone;
  portEXIT_CRITICAL(&droneMux);

  bool present = d.lastSeen && (millis() - d.lastSeen < LOST_MS);

  if (present) {
    digitalWrite(LED_RED_PIN, HIGH);
    digitalWrite(LED_GREEN_PIN, LOW);
    if (!wasPresent) { alarmBeep(); page = 0; lastPage = 0; }

    if (millis() - lastPage > PAGE_MS) {
      showPage(page, d);
      page = (page + 1) % 6;
      lastPage = millis();
    }
    if (millis() - lastPrint > 2000) { printSerial(d); lastPrint = millis(); }
  } else {
    digitalWrite(LED_RED_PIN, LOW);
    digitalWrite(LED_GREEN_PIN, HIGH);
    if (wasPresent || millis() - lastPage > PAGE_MS) {
      lcd.clear();
      lcd.print("Scanning sky...");
      lcd.setCursor(0, 1); lcd.print("Status: Clear");
      lastPage = millis();
    }
  }
  wasPresent = present;
  delay(50);
}

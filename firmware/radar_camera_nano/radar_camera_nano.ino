/*
  الرادار + الكاميرا المتحركة — Arduino Nano
  ------------------------------------------------------------
  هذا الملف يحل محل radar_sensor.ino. لوحة Nano واحدة تشغّل:
    1) الرادار: حساس HC-SR04 على سيرفو يمسح من 15° إلى 165°.
    2) قاعدة الكاميرا (Pan-Tilt): سيرفو للدوران يميناً ويساراً، وسيرفو للرفع والخفض.

  طريقة العمل:
    - وضع البحث (SCAN): الكاميرا تتحرك ببطء يميناً ويساراً.
    - عند رصد الرادار لهدف: تلتفت الكاميرا فوراً نحو زاوية الرادار (رد فعل محلي سريع)،
      ويُرسل الإنذار للـ Raspberry Pi.
    - وضع التتبع (TRACK): الـ Pi يرسل أوامر تصحيح من صورة الكاميرا ليبقى الهدف في المنتصف.
    - إذا لم يصل أمر خلال TRACK_TIMEOUT_MS تعود الكاميرا للبحث تلقائياً.

  الأوامر المستقبلة عبر USB (سطر واحد لكل أمر، سرعة 115200):
    CAM <pan> <tilt>   مثال: CAM 74 60   (زوايا 0–180)
    SCAN               العودة لوضع البحث

  المخرجات عبر USB (سطر JSON لكل رسالة):
    {"sensor":"radar","type":"sweep","angle":74,"distance_cm":120}
    {"sensor":"radar","type":"detection","angle":74,"distance_cm":39,"confidence":1.0,"drone_id":null,"label":"object"}
    {"sensor":"radar","type":"status","cam_mode":"scan","pan":90,"tilt":60}

  التوصيل:
    HC-SR04     : Trig -> D9 , Echo -> D10 , VCC -> 5V , GND -> GND
    سيرفو الرادار (MG90S)       : الإشارة -> D3
    سيرفو دوران الكاميرا (SG90) : الإشارة -> D5
    سيرفو رفع الكاميرا (SG90)   : الإشارة -> D6
    تغذية السيرفوهات الثلاثة من مصدر 5V خارجي (2A على الأقل)، مع GND مشترك مع الـ Nano.
    لا تغذِّ السيرفوهات من منفذ 5V في الـ Nano، فقد تعيد اللوحة التشغيل.

  المكتبات: Servo (مدمجة)
  الجهاز يرصد ويوجّه الكاميرا فقط، بدون أي قاذف أو إطلاق.
*/

#include <Servo.h>

// ---------- التوصيلات ----------
const int TRIG_PIN       = 9;
const int ECHO_PIN       = 10;
const int RADAR_PIN      = 3;
const int PAN_PIN        = 5;
const int TILT_PIN       = 6;

// ---------- إعدادات الرادار ----------
const int RADAR_MIN      = 15;
const int RADAR_MAX      = 165;
const int RADAR_STEP     = 2;
const int DETECT_CM      = 60;
const unsigned long RADAR_STEP_MS   = 40;
const unsigned long RADAR_HOLD_MS   = 1500;   // توقف الرادار عند الهدف
const unsigned long REPEAT_MS       = 4000;   // لا يكرر نفس الهدف قبل هذه المدة

// ---------- إعدادات الكاميرا ----------
const int PAN_MIN        = 20;
const int PAN_MAX        = 160;
const int TILT_MIN       = 30;
const int TILT_MAX       = 120;
const int TILT_SCAN      = 60;     // زاوية الرفع أثناء البحث
const int TILT_TARGET    = 70;     // زاوية الرفع الأولى عند رصد هدف
const int PAN_OFFSET     = 0;      // فرق التركيب بين الرادار وقاعدة الكاميرا (اضبطه بالتجربة)
const int SERVO_MAX_STEP = 3;      // أقصى حركة بالدرجات كل خطوة، لحركة ناعمة
const unsigned long SERVO_STEP_MS    = 20;
const unsigned long SCAN_PERIOD_MS   = 12000;  // زمن دورة البحث كاملة
const unsigned long TRACK_TIMEOUT_MS = 7000;   // العودة للبحث إذا لم تصل أوامر

const unsigned long STATUS_MS = 10000;

Servo radarServo, panServo, tiltServo;

int radarAngle = RADAR_MIN;
int radarDir   = RADAR_STEP;
unsigned long lastRadarStep = 0;
unsigned long radarHoldUntil = 0;
unsigned long lastDetection = 0;
int lastDetAngle = -100;

enum CamMode { MODE_SCAN, MODE_TRACK };
CamMode camMode = MODE_SCAN;
int panNow = 90, tiltNow = TILT_SCAN;
int panTarget = 90, tiltTarget = TILT_SCAN;
unsigned long lastServoStep = 0;
unsigned long trackUntil = 0;
unsigned long scanStart = 0;

unsigned long lastStatus = 0;
char lineBuf[32];
byte lineLen = 0;

// ---------- أدوات ----------
long readDistanceCm() {
  digitalWrite(TRIG_PIN, LOW);  delayMicroseconds(2);
  digitalWrite(TRIG_PIN, HIGH); delayMicroseconds(10);
  digitalWrite(TRIG_PIN, LOW);
  long duration = pulseIn(ECHO_PIN, HIGH, 25000);  // مهلة ~4 أمتار
  if (duration == 0) return 999;
  return duration * 0.0343 / 2;
}

bool confirmed(long &d) {
  long d1 = readDistanceCm(); delay(20);
  long d2 = readDistanceCm();
  d = (d1 + d2) / 2;
  return d1 < DETECT_CM && d2 < DETECT_CM && abs(d1 - d2) < 8;
}

int stepToward(int now, int target) {
  int diff = target - now;
  if (diff >  SERVO_MAX_STEP) diff =  SERVO_MAX_STEP;
  if (diff < -SERVO_MAX_STEP) diff = -SERVO_MAX_STEP;
  return now + diff;
}

const char *modeName() { return camMode == MODE_SCAN ? "scan" : "track"; }

void sendStatus() {
  Serial.print(F("{\"sensor\":\"radar\",\"type\":\"status\",\"cam_mode\":\""));
  Serial.print(modeName());
  Serial.print(F("\",\"pan\":"));  Serial.print(panNow);
  Serial.print(F(",\"tilt\":"));  Serial.print(tiltNow);
  Serial.println(F("}"));
}

void startTrack(int pan, int tilt) {
  camMode = MODE_TRACK;
  panTarget  = constrain(pan,  PAN_MIN,  PAN_MAX);
  tiltTarget = constrain(tilt, TILT_MIN, TILT_MAX);
  trackUntil = millis() + TRACK_TIMEOUT_MS;
}

void startScan() {
  if (camMode != MODE_SCAN) {
    camMode = MODE_SCAN;
    scanStart = millis();
    sendStatus();
  }
}

// ---------- قراءة الأوامر من الـ Pi ----------
void handleLine(char *s) {
  int pan, tilt;
  if (strcmp(s, "SCAN") == 0) {
    startScan();
  } else if (sscanf(s, "CAM %d %d", &pan, &tilt) == 2) {
    if (pan >= 0 && pan <= 180 && tilt >= 0 && tilt <= 180) {
      startTrack(pan, tilt);
    }
  }
  // أي سطر آخر يُتجاهل، فلا ينفّذ الجهاز إلا الأمرين المعرّفين
}

void readSerial() {
  while (Serial.available()) {
    char c = Serial.read();
    if (c == '\n' || c == '\r') {
      if (lineLen > 0) {
        lineBuf[lineLen] = 0;
        handleLine(lineBuf);
        lineLen = 0;
      }
    } else if (lineLen < sizeof(lineBuf) - 1) {
      lineBuf[lineLen++] = c;
    } else {
      lineLen = 0;  // سطر أطول من المسموح: يُرمى
    }
  }
}

// ---------- الرادار ----------
void radarStep() {
  unsigned long now = millis();
  if (now < radarHoldUntil || now - lastRadarStep < RADAR_STEP_MS) return;
  lastRadarStep = now;

  radarServo.write(radarAngle);
  long d;
  bool hit = confirmed(d);

  Serial.print(F("{\"sensor\":\"radar\",\"type\":\"sweep\",\"angle\":"));
  Serial.print(radarAngle);
  Serial.print(F(",\"distance_cm\":"));
  Serial.print(hit ? d : readDistanceCm());
  Serial.println(F("}"));

  bool newTarget = hit &&
    (now - lastDetection > REPEAT_MS || abs(radarAngle - lastDetAngle) > 20);

  if (newTarget) {
    Serial.print(F("{\"sensor\":\"radar\",\"type\":\"detection\",\"angle\":"));
    Serial.print(radarAngle);
    Serial.print(F(",\"distance_cm\":"));
    Serial.print(d);
    Serial.println(F(",\"confidence\":1.0,\"drone_id\":null,\"label\":\"object\"}"));
    lastDetection = now;
    lastDetAngle = radarAngle;
    radarHoldUntil = now + RADAR_HOLD_MS;
    // رد فعل محلي: الكاميرا تلتفت نحو الهدف دون انتظار الـ Pi
    startTrack(radarAngle + PAN_OFFSET, TILT_TARGET);
    return;
  }

  radarAngle += radarDir;
  if (radarAngle >= RADAR_MAX || radarAngle <= RADAR_MIN) radarDir = -radarDir;
}

// ---------- الكاميرا ----------
void cameraStep() {
  unsigned long now = millis();
  if (now - lastServoStep < SERVO_STEP_MS) return;
  lastServoStep = now;

  if (camMode == MODE_TRACK && now > trackUntil) startScan();

  if (camMode == MODE_SCAN) {
    // حركة بحث ناعمة ذهاباً وإياباً بين PAN_MIN و PAN_MAX
    float phase = (float)((now - scanStart) % SCAN_PERIOD_MS) / SCAN_PERIOD_MS;
    float tri = phase < 0.5 ? phase * 2 : (1 - phase) * 2;
    panTarget  = PAN_MIN + (int)(tri * (PAN_MAX - PAN_MIN));
    tiltTarget = TILT_SCAN;
  }

  int p = stepToward(panNow, panTarget);
  int t = stepToward(tiltNow, tiltTarget);
  if (p != panNow)  { panNow = p;  panServo.write(panNow); }
  if (t != tiltNow) { tiltNow = t; tiltServo.write(tiltNow); }
}

void setup() {
  Serial.begin(115200);
  pinMode(TRIG_PIN, OUTPUT);
  pinMode(ECHO_PIN, INPUT);
  radarServo.attach(RADAR_PIN);
  panServo.attach(PAN_PIN);
  tiltServo.attach(TILT_PIN);
  radarServo.write(radarAngle);
  panServo.write(panNow);
  tiltServo.write(tiltNow);
  scanStart = millis();
  delay(500);
  sendStatus();
  lastStatus = millis();
}

void loop() {
  readSerial();
  radarStep();
  cameraStep();
  if (millis() - lastStatus > STATUS_MS) {
    sendStatus();
    lastStatus = millis();
  }
}

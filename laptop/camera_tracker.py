#!/usr/bin/env python3
"""
متتبّع الكاميرا — يعمل على Raspberry Pi داخل الصندوق (الفريقان 3 و5)

يقرر إلى أين تنظر الكاميرا:
  1) عند رسالة رصد من الرادار: يوجّه الكاميرا نحو زاوية الرادار.
  2) عند رصد الكاميرا للطائرة في الصورة: يصحّح الدوران والرفع حتى يبقى الهدف
     في منتصف الصورة، فتتبعه الكاميرا وهو يتحرك.
  3) إذا اختفى الهدف من الصورة مدة LOST_SECONDS: يعيد الكاميرا لوضع البحث.

يرسل الأوامر إلى الموضوع control/camera، والجسر (usb_bridge.py) ينقلها
للوحة Arduino Nano التي تحرّك قاعدة الكاميرا.

مهم لفريق الكاميرا (3): يجب أن تحتوي رسالة رصد الكاميرا على مركز الطائرة
في الصورة كنسبة من 0 إلى 1:
  {"sensor":"camera","type":"detection","confidence":0.82,"label":"photo","cx":0.63,"cy":0.41, ...}
  cx = 0 يسار الصورة، 1 يمينها.  cy = 0 أعلى الصورة، 1 أسفلها.

التشغيل:
  python3 camera_tracker.py
"""

import json
import time
import threading

import paho.mqtt.client as mqtt

MQTT_HOST = "localhost"
MQTT_PORT = 1883
MQTT_USER = None
MQTT_PASS = None

# زاوية رؤية Camera Module 3 العادية تقريباً (بالدرجات)
FOV_H = 66.0
FOV_V = 41.0

GAIN = 0.5             # نسبة التصحيح في كل خطوة (أقل = أنعم، أكثر = أسرع)
DEADZONE = 0.06        # لا يصحّح إذا كان الهدف قريباً من المنتصف بهذه النسبة
MIN_CONFIDENCE = 0.6
TILT_ON_RADAR = 70     # زاوية الرفع عند الالتفات لهدف الرادار
LOST_SECONDS = 4.0     # بعدها تعود الكاميرا للبحث
SEND_INTERVAL = 0.2    # أقصى معدل للأوامر: 5 أوامر في الثانية

# إذا تحركت الكاميرا عكس الاتجاه الصحيح أثناء التجربة، غيّر الإشارة هنا
PAN_SIGN = -1          # الهدف يمين الصورة => تقليل زاوية الدوران
TILT_SIGN = 1          # الهدف أسفل الصورة => زيادة زاوية الرفع

PAN_MIN, PAN_MAX = 20, 160
TILT_MIN, TILT_MAX = 30, 120


class Tracker:
    def __init__(self, client):
        self.client = client
        self.pan = 90.0
        self.tilt = 60.0
        self.tracking = False
        self.last_seen = 0.0
        self.last_sent = 0.0
        self.lock = threading.Lock()

    def send(self, force=False):
        now = time.time()
        if not force and now - self.last_sent < SEND_INTERVAL:
            return
        self.last_sent = now
        self.client.publish("control/camera", f"CAM {int(round(self.pan))} {int(round(self.tilt))}")

    def clamp(self):
        self.pan = max(PAN_MIN, min(PAN_MAX, self.pan))
        self.tilt = max(TILT_MIN, min(TILT_MAX, self.tilt))

    def on_radar(self, msg):
        if msg.get("type") == "status" and not self.tracking:
            self.pan = float(msg.get("pan", self.pan))
            self.tilt = float(msg.get("tilt", self.tilt))
            return
        if msg.get("type") != "detection":
            return
        angle = msg.get("angle")
        if not isinstance(angle, (int, float)):
            return
        with self.lock:
            self.pan = float(angle)
            self.tilt = float(TILT_ON_RADAR)
            self.clamp()
            self.tracking = True
            self.last_seen = time.time()
            self.send(force=True)
        print(f"الرادار رصد هدفاً عند {angle}°، الكاميرا تلتفت إليه")

    def on_camera(self, msg):
        if msg.get("type") != "detection":
            return
        if float(msg.get("confidence", 0)) < MIN_CONFIDENCE:
            return
        cx, cy = msg.get("cx"), msg.get("cy")
        if not isinstance(cx, (int, float)) or not isinstance(cy, (int, float)):
            return
        ex, ey = cx - 0.5, cy - 0.5          # بُعد الهدف عن منتصف الصورة
        with self.lock:
            self.tracking = True
            self.last_seen = time.time()
            if abs(ex) > DEADZONE:
                self.pan += PAN_SIGN * ex * FOV_H * GAIN
            if abs(ey) > DEADZONE:
                self.tilt += TILT_SIGN * ey * FOV_V * GAIN
            self.clamp()
            self.send()

    def watchdog(self):
        while True:
            time.sleep(0.5)
            with self.lock:
                if self.tracking and time.time() - self.last_seen > LOST_SECONDS:
                    self.tracking = False
                    self.client.publish("control/camera", "SCAN")
                    print("فُقد الهدف، الكاميرا تعود للبحث")


def main():
    try:
        client = mqtt.Client(mqtt.CallbackAPIVersion.VERSION2, client_id="camera-tracker")
    except AttributeError:
        client = mqtt.Client(client_id="camera-tracker")
    if MQTT_USER:
        client.username_pw_set(MQTT_USER, MQTT_PASS)
    tracker = Tracker(client)

    def on_connect(c, userdata, flags, rc, properties=None):
        c.subscribe([("drones/radar", 0), ("drones/camera", 0)])
        print("المتتبّع يعمل")

    def on_message(c, userdata, msg):
        try:
            data = json.loads(msg.payload.decode("utf-8"))
        except (UnicodeDecodeError, json.JSONDecodeError):
            return
        if msg.topic == "drones/radar":
            tracker.on_radar(data)
        elif msg.topic == "drones/camera":
            tracker.on_camera(data)

    client.on_connect = on_connect
    client.on_message = on_message
    while True:
        try:
            client.connect(MQTT_HOST, MQTT_PORT, keepalive=30)
            break
        except OSError:
            print("خادم MQTT غير جاهز، إعادة المحاولة بعد 3 ثوانٍ")
            time.sleep(3)
    threading.Thread(target=tracker.watchdog, daemon=True).start()
    client.loop_forever()


if __name__ == "__main__":
    main()

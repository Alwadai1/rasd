#!/usr/bin/env python3
"""
جسر USB إلى MQTT — يعمل على لابتوب العرض (ويندوز أو لينكس)

ما يفعله:
  1) يفتح كل لوحات USB الموصولة (ESP32 وArduino Nano)، ويتعرف على كل لوحة
     من حقل "sensor" في أول رسالة JSON تصله منها، فلا داعي لتحديد المنافذ يدوياً.
  2) يضيف الوقت لكل رسالة، وينشرها على خادم MQTT المحلي في الموضوع drones/<sensor>.
  3) يستقبل أوامر الكاميرا من الموضوع control/camera، ويتأكد أنها أمر مسموح،
     ثم يرسلها للوحة الرادار (Nano) التي تحرّك قاعدة الكاميرا.

التثبيت (مرة واحدة، قبل فصل الإنترنت):
  ويندوز: ثبّت Mosquitto من mosquitto.org ثم:  pip install pyserial paho-mqtt
  لينكس:  sudo apt install mosquitto python3-serial python3-paho-mqtt

التشغيل:
  python usb_bridge.py
"""

import glob
import os
from serial.tools import list_ports
import json
import re
import threading
import time
from datetime import datetime, timezone, timedelta

import serial
import paho.mqtt.client as mqtt

MQTT_HOST = "localhost"
MQTT_PORT = 1883
MQTT_USER = None          # ضع اسم المستخدم وكلمة المرور إذا فعّلتها في Mosquitto
MQTT_PASS = None
BAUD = 115200
PORT_PATTERNS = ["/dev/ttyUSB*", "/dev/ttyACM*"]
RESCAN_SECONDS = 5
RIYADH = timezone(timedelta(hours=3))

# الأوامر الوحيدة المسموح بإرسالها للوحة الرادار
ALLOWED_CMD = re.compile(r"^(CAM (\d{1,3}) (\d{1,3})|SCAN)$")

ports = {}            # path -> {"serial": Serial, "sensor": str or None}
ports_lock = threading.Lock()


def now_iso():
    return datetime.now(RIYADH).isoformat(timespec="seconds")


def make_client():
    try:
        client = mqtt.Client(mqtt.CallbackAPIVersion.VERSION2, client_id="usb-bridge")
    except AttributeError:  # إصدار paho-mqtt أقدم من 2.0
        client = mqtt.Client(client_id="usb-bridge")
    if MQTT_USER:
        client.username_pw_set(MQTT_USER, MQTT_PASS)
    return client


def on_connect(client, userdata, flags, rc, properties=None):
    client.subscribe("control/camera")
    print("متصل بخادم MQTT، بانتظار الرسائل")


def on_message(client, userdata, msg):
    cmd = msg.payload.decode("utf-8", errors="ignore").strip()
    m = ALLOWED_CMD.match(cmd)
    if not m:
        print("أمر مرفوض:", cmd[:40])
        return
    if m.group(2) is not None:
        pan, tilt = int(m.group(2)), int(m.group(3))
        if not (0 <= pan <= 180 and 0 <= tilt <= 180):
            print("زاوية خارج النطاق:", cmd)
            return
    with ports_lock:
        targets = [p for p in ports.values() if p["sensor"] == "radar"]
    if not targets:
        print("لوحة الرادار غير موصولة، لم يُرسل الأمر")
        return
    for p in targets:
        try:
            p["serial"].write((cmd + "\n").encode("ascii"))
        except serial.SerialException as e:
            print("تعذّر الإرسال للوحة الرادار:", e)


def read_port(path, client):
    entry = ports[path]
    ser = entry["serial"]
    while True:
        try:
            raw = ser.readline()
        except (serial.SerialException, OSError):
            print("فُصلت اللوحة:", path)
            break
        if not raw:
            continue
        line = raw.decode("utf-8", errors="ignore").strip()
        if not line.startswith("{"):
            continue
        try:
            msg = json.loads(line)
        except json.JSONDecodeError:
            continue
        sensor = msg.get("sensor")
        if not isinstance(sensor, str) or not re.match(r"^[a-z_]{1,20}$", sensor):
            continue
        if entry["sensor"] != sensor:
            entry["sensor"] = sensor
            print(f"{path} = حساس {sensor}")
        msg["time"] = now_iso()
        client.publish(f"drones/{sensor}", json.dumps(msg, ensure_ascii=False))
    with ports_lock:
        ports.pop(path, None)
    try:
        ser.close()
    except Exception:
        pass


def scan_ports(client):
    while True:
        found = []
        if os.name == "nt":  # ويندوز: المنافذ بأسماء COM3 وCOM4...
            found = [p.device for p in list_ports.comports()]
        else:
            for pattern in PORT_PATTERNS:
                found.extend(glob.glob(pattern))
        for path in found:
            with ports_lock:
                if path in ports:
                    continue
            try:
                ser = serial.Serial(path, BAUD, timeout=1)
            except serial.SerialException:
                continue
            with ports_lock:
                ports[path] = {"serial": ser, "sensor": None}
            print("لوحة جديدة:", path)
            threading.Thread(target=read_port, args=(path, client), daemon=True).start()
        time.sleep(RESCAN_SECONDS)


def main():
    client = make_client()
    client.on_connect = on_connect
    client.on_message = on_message
    while True:
        try:
            client.connect(MQTT_HOST, MQTT_PORT, keepalive=30)
            break
        except OSError:
            print("خادم MQTT غير جاهز، إعادة المحاولة بعد 3 ثوانٍ")
            time.sleep(3)
    client.loop_start()
    scan_ports(client)


if __name__ == "__main__":
    main()

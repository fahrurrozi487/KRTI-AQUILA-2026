#!/usr/bin/env python3
"""
Dashboard web (Flask + MJPEG) -- ganti VNC buat monitoring drone dari laptop
tanpa perlu remote-desktop kamera penuh (VNC berat/lag di link radio terbatas;
MJPEG cuma stream 1 arah, lebih ringan).

Nyubain 2 kamera (down + front) + telemetry ringkas: status misi (WP sekarang,
dari mission_d.py ~status), rangefinder (ToF/TF-Luna), + data tambahan yang
direkomendasikan (baterai, GPS fix, mode/armed, posisi lokal) -- semua ini
indikator kesehatan flight yang biasa dicek manual di GCS, disatuin di 1
halaman ringan.

Param:
  ~port          (int)  default 5000
  ~down_topic    (str)  default /camera_down/image_raw
  ~front_topic   (str)  default /camera_front/image_raw
  ~status_topic  (str)  default /mission_d/status
  ~jpeg_quality  (int)  default 70 (0-100, makin rendah makin hemat bandwidth)

Pakai:
  rosrun mission_control web_dashboard.py
  buka http://<ip-jetson>:5000/ dari browser laptop (1 jaringan/hotspot)

⚠️ Topic rangefinder = "/mavros/distance_sensor/rangefinder_pub" (BUKAN
"_sub" -- itu arah mavros->FC). Override pakai ~rangefinder_topic kalau beda.

Optical flow SENGAJA gak ditampilkan (2026-09-03): FC kirim MAVLink
OPTICAL_FLOW (id 100), tapi mavros cuma support OPTICAL_FLOW_RAD (id 106) --
gak ada plugin/topic yang bisa nerima format itu tanpa bridge custom. Kalau
nanti mau ditambah lagi, butuh node terpisah baca via pymavlink langsung,
bukan lewat mavros.
"""
import os
import sys
import threading
import time

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))

import cv2
import rospy
from flask import Flask, Response, render_template_string, jsonify
from sensor_msgs.msg import Image, BatteryState, Range
from std_msgs.msg import String, Float64
from mavros_msgs.msg import State, GPSRAW
from geometry_msgs.msg import PoseStamped

from image_convert import imgmsg_to_np

app = Flask(__name__)

# ---------- state bersama (ditulis callback ROS, dibaca route Flask) ----------
_lock = threading.Lock()
_frames = {"down": None, "front": None}
_telemetry = {
    "status": "-", "armed": False, "mode": "-", "connected": False,
    "battery_v": None, "battery_pct": None,
    "gps_fix": None, "gps_sats": None, "gps_hdop": None,
    "range_m": None,
    "rel_alt": None,
    "pos_x": None, "pos_y": None, "pos_z": None,
}
JPEG_QUALITY = 70


def _mkgen(key):
    """Generator MJPEG multipart buat 1 kamera (down/front)."""
    def gen():
        while True:
            with _lock:
                frame = _frames[key]
            if frame is None:
                time.sleep(0.1)
                continue
            ok, buf = cv2.imencode(".jpg", frame,
                                    [cv2.IMWRITE_JPEG_QUALITY, JPEG_QUALITY])
            if ok:
                yield (b"--frame\r\nContent-Type: image/jpeg\r\n\r\n" +
                       buf.tobytes() + b"\r\n")
            time.sleep(0.05)  # ~20fps cap, jangan lebih dari framerate kamera asli
    return gen


@app.route("/video/down")
def video_down():
    return Response(_mkgen("down")(),
                     mimetype="multipart/x-mixed-replace; boundary=frame")


@app.route("/video/front")
def video_front():
    return Response(_mkgen("front")(),
                     mimetype="multipart/x-mixed-replace; boundary=frame")


@app.route("/telemetry.json")
def telemetry_json():
    with _lock:
        return jsonify(dict(_telemetry))


_PAGE = """<!doctype html><html><head><title>Drone Dashboard</title>
<style>
body{font-family:sans-serif;background:#111;color:#eee;margin:0;padding:10px}
.row{display:flex;gap:10px;flex-wrap:wrap}
img{width:480px;max-width:100%;border:2px solid #444;background:#000}
table{border-collapse:collapse;margin-top:10px}
td{padding:4px 12px;border-bottom:1px solid #333}
td.k{color:#8ab4f8}
.big{font-size:1.4em;font-weight:bold;color:#7fff7f}
</style></head><body>
<div class="big" id="status">memuat...</div>
<div class="row">
  <div><p>Kamera Bawah</p><img src="/video/down"></div>
  <div><p>Kamera Depan</p><img src="/video/front"></div>
</div>
<table id="tel"></table>
<script>
async function tick(){
  const r = await fetch('/telemetry.json'); const d = await r.json();
  document.getElementById('status').innerText =
    "WP: " + d.status + "  |  mode=" + d.mode + "  armed=" + d.armed;
  const rows = [
    ["Battery", d.battery_v!=null ? d.battery_v.toFixed(2)+"V ("+(d.battery_pct*100).toFixed(0)+"%)" : "-"],
    ["GPS", d.gps_fix!=null ? "fix="+d.gps_fix+" sats="+d.gps_sats+" hdop="+(d.gps_hdop!=null?d.gps_hdop.toFixed(2):"-") : "-"],
    ["Rangefinder (ToF)", d.range_m!=null ? d.range_m.toFixed(2)+" m" : "-"],
    ["Alt Rel (FC raw)", d.rel_alt!=null ? d.rel_alt.toFixed(2)+" m" : "-"],
    ["Posisi lokal", d.pos_x!=null ? "("+d.pos_x.toFixed(1)+", "+d.pos_y.toFixed(1)+", "+d.pos_z.toFixed(1)+")" : "-"],
    ["Connected", d.connected],
  ];
  document.getElementById('tel').innerHTML = rows.map(
    r=>`<tr><td class="k">${r[0]}</td><td>${r[1]}</td></tr>`).join('');
}
setInterval(tick, 1000); tick();
</script>
</body></html>"""


@app.route("/")
def index():
    return render_template_string(_PAGE)


# ---------- callback ROS: cuma nulis state, gak ngapa2in berat ----------
def _cb_img(key):
    def cb(msg):
        try:
            img = imgmsg_to_np(msg)
        except ValueError:
            return
        with _lock:
            _frames[key] = img
    return cb


def _cb_status(msg):
    with _lock:
        _telemetry["status"] = msg.data


def _cb_state(msg):
    with _lock:
        _telemetry["armed"] = bool(msg.armed)
        _telemetry["mode"] = msg.mode
        _telemetry["connected"] = bool(msg.connected)


def _cb_battery(msg):
    with _lock:
        _telemetry["battery_v"] = float(msg.voltage)
        _telemetry["battery_pct"] = float(msg.percentage) if msg.percentage == msg.percentage else None


def _cb_gps(msg):
    with _lock:
        _telemetry["gps_fix"] = int(msg.fix_type)
        _telemetry["gps_sats"] = int(msg.satellites_visible)
        _telemetry["gps_hdop"] = float(msg.eph) / 100.0 if msg.eph < 65535 else None


def _cb_range(msg):
    with _lock:
        _telemetry["range_m"] = float(msg.range)


def _cb_rel_alt(msg):
    with _lock:
        _telemetry["rel_alt"] = float(msg.data)


def _cb_pose(msg):
    with _lock:
        p = msg.pose.position
        _telemetry["pos_x"] = float(p.x)
        _telemetry["pos_y"] = float(p.y)
        _telemetry["pos_z"] = float(p.z)


def main():
    rospy.init_node("web_dashboard")
    port = int(rospy.get_param("~port", 5000))
    down_topic = rospy.get_param("~down_topic", "/camera_down/image_raw")
    front_topic = rospy.get_param("~front_topic", "/camera_front/image_raw")
    status_topic = rospy.get_param("~status_topic", "/mission_d/status")
    # 2026-09-03: dikonfirmasi live di FC beneran -- "rangefinder_sub" itu arah
    # mavros->FC (buat KIRIM data palsu), BUKAN buat nerima. Topic yg BENERAN
    # publish data asli dari FC = "rangefinder_pub" (frame_id="lidar", terbukti
    # ngalir stabil pas dites). Default lama SALAH, sekarang diperbaiki.
    range_topic = rospy.get_param("~rangefinder_topic",
                                   "/mavros/distance_sensor/rangefinder_pub")
    rel_alt_topic = rospy.get_param("~rel_alt_topic",
                                    "/mavros/global_position/rel_alt")
    global JPEG_QUALITY
    JPEG_QUALITY = int(rospy.get_param("~jpeg_quality", 70))

    rospy.Subscriber(down_topic, Image, _cb_img("down"), queue_size=1, buff_size=2**24)
    rospy.Subscriber(front_topic, Image, _cb_img("front"), queue_size=1, buff_size=2**24)
    rospy.Subscriber(status_topic, String, _cb_status, queue_size=1)
    rospy.Subscriber("/mavros/state", State, _cb_state, queue_size=1)
    rospy.Subscriber("/mavros/battery", BatteryState, _cb_battery, queue_size=1)
    rospy.Subscriber("/mavros/gpsstatus/gps1/raw", GPSRAW, _cb_gps, queue_size=1)
    rospy.Subscriber(range_topic, Range, _cb_range, queue_size=1)
    rospy.Subscriber(rel_alt_topic, Float64, _cb_rel_alt, queue_size=1)
    rospy.Subscriber("/mavros/local_position/pose", PoseStamped, _cb_pose, queue_size=1)

    rospy.loginfo("[dashboard] siap di http://0.0.0.0:%d/", port)
    flask_thread = threading.Thread(
        target=lambda: app.run(host="0.0.0.0", port=port, threaded=True,
                                use_reloader=False),
        daemon=True)
    flask_thread.start()
    rospy.spin()


if __name__ == "__main__":
    main()

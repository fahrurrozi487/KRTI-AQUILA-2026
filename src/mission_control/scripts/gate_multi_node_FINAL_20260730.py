#!/usr/bin/env python3
"""
Node ROS detektor Double/Triple Gate (bentuk-U, kamera depan) — Task 2.

Wrapper TIPIS di atas gate_multi_detector.detect_gates() — TIDAK duplikasi
logika deteksi. TERPISAH PENUH dari gate_node.py (Single Gate, tidak
disentuh sama sekali di task ini).

Publish format mission_control/Gate SAMA persis field-nya dengan gate_node.py
lama, supaya mission_d.py (yang sudah ada) bisa pakai node ini tanpa ubah
kode sama sekali — cukup ganti topic yang di-subscribe lewat rosparam
~gate_topic (lihat gate_multi_detect.launch, topic penuh node ini:
/gate_multi_node/gate, SENGAJA beda dari /gate_node/gate lama supaya kedua
node bisa jalan bersamaan saat testing tanpa bentrok).

Param:
  ~image_topic   (str)   default /camera_front/image_raw
  ~publish_debug (bool)  default True

Publish:
  ~gate          mission_control/Gate
  ~debug_image   sensor_msgs/Image (bgr8), bila publish_debug

Catatan pemetaan field Gate.msg utk node ini (detail & rasional lengkap di
docs/GATE_DETECTOR_DOUBLE_TRIPLE.md §8):
  area_frac    = area kontur-luar kandidat terpilih / (W*H) frame -- BEDA
                 definisi dari gate_node.py lama (yg pakai total-piksel-
                 oranye di SELURUH frame). mission_d.py hanya memakai POLA
                 RELATIF (naik-lalu-turun thd peak) utk deteksi transisi
                 antar-lapis, bukan nilai absolut -- tetap kompatibel.
  num_openings = 1 kalau detected, else 0. detect_gates() tidak membedakan
                 sub-bukaan ganda dalam satu struktur gate (selalu ambil 1
                 komponen background TERBESAR per kontur luar, per desain
                 Task 1). Dicek: field ini di mission_d.py HANYA dipakai
                 utk 1 baris log, TIDAK ada percabangan logika berdasar
                 nilai ini -- aman disederhanakan.
  opening_w_frac = 0.0 selalu. detect_gates() tidak mengekspos lebar-bukaan
                 spesifik di return value-nya. Dicek: field ini TIDAK
                 direferensikan sama sekali di mission_d.py.
"""
import os
import sys
sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))

import rospy
from sensor_msgs.msg import Image
from mission_control.msg import Gate

import gate_multi_detector as gm
from image_convert import imgmsg_to_np, np_to_imgmsg


class GateMultiNode:
    def __init__(self):
        topic = rospy.get_param("~image_topic", "/camera_front/image_raw")
        self.publish_debug = bool(rospy.get_param("~publish_debug", True))

        self.pub_gate = rospy.Publisher("~gate", Gate, queue_size=5)
        if self.publish_debug:
            self.pub_debug = rospy.Publisher("~debug_image", Image, queue_size=2)
        rospy.Subscriber(topic, Image, self._cb, queue_size=1, buff_size=2 ** 24)
        rospy.loginfo("[gate_multi] siap. image_topic=%s", topic)
        self._warned = 0

    def _cb(self, msg):
        try:
            img = imgmsg_to_np(msg)
        except ValueError as e:
            if self._warned < 3:
                rospy.logwarn("[gate_multi] %s", e); self._warned += 1
            return
        h, w = img.shape[:2]
        gates = gm.detect_gates(img)

        out = Gate()
        out.header = msg.header
        if gates:
            best = gates[0]  # area kontur-luar terbesar = gate terdekat = prioritas target
            out.detected = True
            out.off_x, out.off_y = best["offset_x"], best["offset_y"]
            out.area_frac = best["area"] / float(w * h)
            out.num_openings = 1
            out.opening_w_frac = 0.0
            out.bbox = list(best["bbox_outer"])
        else:
            out.detected = False
            out.off_x = out.off_y = 0.0
            out.area_frac = 0.0
            out.num_openings = 0
            out.opening_w_frac = 0.0
            out.bbox = [0, 0, 0, 0]
        self.pub_gate.publish(out)

        if self.publish_debug and self.pub_debug.get_num_connections() > 0:
            vis = gm._draw_overlay(img, gates)
            self.pub_debug.publish(
                np_to_imgmsg(vis, msg.header.stamp, msg.header.frame_id))


def main():
    rospy.init_node("gate_multi_node")
    GateMultiNode()
    rospy.spin()


if __name__ == "__main__":
    main()

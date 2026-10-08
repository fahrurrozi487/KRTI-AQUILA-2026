#!/usr/bin/env python3
"""
Node ROS detektor gate oranye (kamera depan) — Fase C.

Subscribe gambar, deteksi gate + bukaan, publish hasil + gambar debug.
Tanpa cv_bridge (lihat image_convert.py).

Param:
  ~image_topic   (str)   default /camera_front/image_raw
  ~hsv_lo        (str)   "H,S,V" batas bawah oranye   default "8,80,80"
  ~hsv_hi        (str)   "H,S,V" batas atas oranye    default "30,255,255"
  ~min_area_frac (float) gate dianggap ada bila >= ini default 0.02
  ~publish_debug (bool)  default True

Publish:
  ~gate          mission_control/Gate
  ~debug_image   sensor_msgs/Image (bgr8), bila publish_debug
"""
import os
import sys
sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))

import rospy
from sensor_msgs.msg import Image
from mission_control.msg import Gate

from gate_detector import GateDetector
from image_convert import imgmsg_to_np, np_to_imgmsg


def _triple(s, default):
    try:
        parts = [int(x) for x in str(s).split(",")]
        return tuple(parts) if len(parts) == 3 else default
    except ValueError:
        return default


class GateNode:
    def __init__(self):
        topic = rospy.get_param("~image_topic", "/camera_front/image_raw")
        lo = _triple(rospy.get_param("~hsv_lo", "8,80,80"), (8, 80, 80))
        hi = _triple(rospy.get_param("~hsv_hi", "30,255,255"), (30, 255, 255))
        min_area = float(rospy.get_param("~min_area_frac", 0.02))
        self.publish_debug = bool(rospy.get_param("~publish_debug", True))

        self.detector = GateDetector(hsv_lo=lo, hsv_hi=hi, min_area_frac=min_area)

        self.pub_gate = rospy.Publisher("~gate", Gate, queue_size=5)
        if self.publish_debug:
            self.pub_debug = rospy.Publisher("~debug_image", Image, queue_size=2)
        rospy.Subscriber(topic, Image, self._cb, queue_size=1, buff_size=2 ** 24)
        rospy.loginfo("[gate] siap. hsv=%s..%s, image_topic=%s", lo, hi, topic)
        self._warned = 0

    def _cb(self, msg):
        try:
            img = imgmsg_to_np(msg)
        except ValueError as e:
            if self._warned < 3:
                rospy.logwarn("[gate] %s", e); self._warned += 1
            return
        r = self.detector.detect(img)

        out = Gate()
        out.header = msg.header
        out.detected = r.detected
        out.off_x, out.off_y = r.off_x, r.off_y
        out.area_frac = r.area_frac
        out.num_openings = r.num_openings
        out.opening_w_frac = r.opening_w_frac
        out.bbox = list(r.bbox)
        self.pub_gate.publish(out)

        if self.publish_debug and self.pub_debug.get_num_connections() > 0:
            vis = self.detector.draw(img, r)
            self.pub_debug.publish(
                np_to_imgmsg(vis, msg.header.stamp, msg.header.frame_id))


def main():
    rospy.init_node("gate_node")
    GateNode()
    rospy.spin()


if __name__ == "__main__":
    main()

#!/usr/bin/env python3
"""
Node ROS detektor garis putus-putus (kamera bawah) -- WP4->WP5.

Wrapper TIPIS di atas line_detector.LineDetector -- TIDAK duplikasi logika
deteksi. Pola SAMA persis gate_node.py (gate_detector.GateDetector).

Param (HSV_lo/hi, BUKAN dark_max lagi -- lih. line_detector.py utk riwayat
kenapa: material tarp asli glossy, V doang gak bisa misahin dari rumput):
  ~image_topic   (str)   default /camera_down/image_raw
  ~hsv_lo        (str)   "H,S,V" batas bawah tarp (venue utama) default "87,35,0"
  ~hsv_hi        (str)   "H,S,V" batas atas tarp (venue utama)  default "107,255,255"
  ~hsv_lo2       (str)   opsional, fallback venue ke-2 (mis. IKN). Kosong = nonaktif.
  ~hsv_hi2       (str)   opsional, fallback venue ke-2. Kosong = nonaktif.
  ~min_area_frac (float) segmen dianggap ada bila >= ini, default 0.01
  ~publish_debug (bool)  default True

Publish:
  ~line          mission_control/Line
  ~debug_image   sensor_msgs/Image (bgr8), bila publish_debug
"""
import os
import sys
sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))

import rospy
from sensor_msgs.msg import Image
from mission_control.msg import Line

from line_detector import LineDetector, HSV_LO, HSV_HI
from image_convert import imgmsg_to_np, np_to_imgmsg


def _triple(s, default):
    try:
        parts = [int(x) for x in str(s).split(",")]
        return tuple(parts) if len(parts) == 3 else default
    except ValueError:
        return default


class LineNode:
    def __init__(self):
        topic = rospy.get_param("~image_topic", "/camera_down/image_raw")
        lo = _triple(rospy.get_param("~hsv_lo", "87,35,0"), HSV_LO)
        hi = _triple(rospy.get_param("~hsv_hi", "107,255,255"), HSV_HI)
        lo2_s = rospy.get_param("~hsv_lo2", "")
        hi2_s = rospy.get_param("~hsv_hi2", "")
        lo2 = _triple(lo2_s, None) if lo2_s else None
        hi2 = _triple(hi2_s, None) if hi2_s else None
        min_area = float(rospy.get_param("~min_area_frac", 0.01))
        self.publish_debug = bool(rospy.get_param("~publish_debug", True))

        self.detector = LineDetector(hsv_lo=lo, hsv_hi=hi, min_area_frac=min_area,
                                      hsv_lo2=lo2, hsv_hi2=hi2)

        self.pub_line = rospy.Publisher("~line", Line, queue_size=5)
        if self.publish_debug:
            self.pub_debug = rospy.Publisher("~debug_image", Image, queue_size=2)
        rospy.Subscriber(topic, Image, self._cb, queue_size=1, buff_size=2 ** 24)
        rospy.loginfo("[line] siap. hsv=%s..%s (fallback2=%s), image_topic=%s",
                      lo, hi, "aktif" if lo2 else "nonaktif", topic)
        self._warned = 0

    def _cb(self, msg):
        try:
            img = imgmsg_to_np(msg)
        except ValueError as e:
            if self._warned < 3:
                rospy.logwarn("[line] %s", e); self._warned += 1
            return
        r = self.detector.detect(img)

        out = Line()
        out.header = msg.header
        out.detected = r.detected
        out.off_x, out.off_y = r.off_x, r.off_y
        out.area_frac = r.area_frac
        out.angle_deg = r.angle_deg
        out.bbox = list(r.bbox)
        self.pub_line.publish(out)

        if self.publish_debug and self.pub_debug.get_num_connections() > 0:
            vis = self.detector.draw(img, r)
            self.pub_debug.publish(
                np_to_imgmsg(vis, msg.header.stamp, msg.header.frame_id))


def main():
    rospy.init_node("line_node")
    LineNode()
    rospy.spin()


if __name__ == "__main__":
    main()

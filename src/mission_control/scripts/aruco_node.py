#!/usr/bin/env python3
"""
Node ROS detektor ArUco (kamera bawah) — Fase C.

Subscribe gambar kamera, deteksi marker 7x7, publish hasil + gambar debug.
TIDAK pakai cv_bridge (rusak di Jetson ini krn OpenCV 4.8 vs cv_bridge 4.2) —
konversi sensor_msgs/Image <-> numpy dilakukan manual.

Param:
  ~image_topic   (str)  topik gambar masuk            default /camera_down/image_raw
  ~dictionary    (str)  dictionary aruco              default DICT_7X7_250
  ~marker_length (float) sisi marker meter (utk pose) default 0.0 (pose mati)
  ~camera_info_topic (str) topik CameraInfo utk pose  default "" (mati)
  ~publish_debug (bool) publish gambar anotasi         default True

Publish:
  ~markers       mission_control/ArucoMarkers
  ~debug_image   sensor_msgs/Image (bgr8), bila publish_debug
"""
import os
import sys
sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))

import numpy as np
import rospy
from sensor_msgs.msg import Image, CameraInfo
from geometry_msgs.msg import Point
from mission_control.msg import ArucoMarker, ArucoMarkers

from aruco_detector import ArucoMarkerDetector
from image_convert import imgmsg_to_np, np_to_imgmsg


class ArucoNode:
    def __init__(self):
        topic = rospy.get_param("~image_topic", "/camera_down/image_raw")
        dictionary = rospy.get_param("~dictionary", "DICT_7X7_50")
        marker_len = float(rospy.get_param("~marker_length", 0.0))
        cam_info_topic = rospy.get_param("~camera_info_topic", "")
        self.publish_debug = bool(rospy.get_param("~publish_debug", True))
        # allowlist ID; "" / "all" = terima semua. Lomba: "1,2,3,4"
        ids_param = str(rospy.get_param("~valid_ids", "1,2,3,4")).strip()
        valid_ids = None if ids_param in ("", "all") else \
            [int(x) for x in ids_param.split(",") if x.strip() != ""]
        # True bila marker lapangan putih-di-hitam (terbalik dari standar)
        invert = bool(rospy.get_param("~invert", True))

        self.detector = ArucoMarkerDetector(dictionary=dictionary,
                                            marker_length=marker_len,
                                            valid_ids=valid_ids,
                                            invert=invert)
        self._cam_info_done = False

        self.pub_markers = rospy.Publisher("~markers", ArucoMarkers, queue_size=5)
        if self.publish_debug:
            self.pub_debug = rospy.Publisher("~debug_image", Image, queue_size=2)

        if cam_info_topic:
            rospy.Subscriber(cam_info_topic, CameraInfo, self._cam_info_cb)
            if marker_len <= 0.0:
                rospy.logwarn("[aruco] camera_info diberi tapi marker_length=0 -> pose tetap mati")

        rospy.Subscriber(topic, Image, self._image_cb, queue_size=1, buff_size=2 ** 24)
        rospy.loginfo("[aruco] siap. dict=%s, valid_ids=%s, invert=%s, image_topic=%s, pose=%s",
                      dictionary, valid_ids or "ALL", invert, topic,
                      self.detector.has_calibration())
        self._warned_count = 0

    def _cam_info_cb(self, msg):
        if self._cam_info_done:
            return
        self.detector.camera_matrix = np.array(msg.K, dtype=np.float64).reshape(3, 3)
        self.detector.dist_coeffs = np.array(msg.D, dtype=np.float64)
        self._cam_info_done = True
        rospy.loginfo("[aruco] kalibrasi kamera diterima -> pose aktif=%s",
                      self.detector.has_calibration())

    def _image_cb(self, msg):
        try:
            img = imgmsg_to_np(msg)
        except ValueError as e:
            if self._warned_count < 3:
                rospy.logwarn("[aruco] %s", e)
                self._warned_count += 1
            return
        markers = self.detector.detect(img)

        out = ArucoMarkers()
        out.header = msg.header
        out.image_height, out.image_width = img.shape[:2]
        for m in markers:
            am = ArucoMarker()
            am.id = m.id
            am.cx, am.cy = m.cx, m.cy
            am.off_x, am.off_y = m.off_x, m.off_y
            am.side_px = m.side_px
            am.has_pose = m.tvec is not None
            if m.tvec is not None:
                am.position = Point(*[float(v) for v in m.tvec])
            out.markers.append(am)
        self.pub_markers.publish(out)

        if self.publish_debug and self.pub_debug.get_num_connections() > 0:
            vis = self.detector.draw(img, markers)
            self.pub_debug.publish(
                np_to_imgmsg(vis, msg.header.stamp, msg.header.frame_id))


def main():
    rospy.init_node("aruco_node")
    ArucoNode()
    rospy.spin()


if __name__ == "__main__":
    main()

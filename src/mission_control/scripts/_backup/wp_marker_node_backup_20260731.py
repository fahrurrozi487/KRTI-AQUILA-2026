#!/usr/bin/env python3
"""wp_marker_node.py — ROS node untuk deteksi marker WP1/WP2/WP3/WP4.

Thin wrapper di atas wp_marker_detector.py. TIDAK menggunakan cv_bridge
(rusak di Jetson ini); konversi via image_convert.py.

Subscribe:
  ~image_topic   (default /camera_down/image_raw)

Publish:
  ~result        mission_control/WpMarkerResult   (setiap frame)
  ~wp2_present   std_msgs/Bool                    (setiap frame)
  ~markers       mission_control/ArucoMarkers     (kompatibel mission_d; hanya saat reason='ok')
  ~debug_image   sensor_msgs/Image                (hanya bila ada subscriber)

~markers mengikuti format persis aruco_node sehingga mission_d bisa dipakai
dengan param markers_topic:=/wp_marker_node/markers tanpa ubah kode mission.

Params:
  ~image_topic        str    /camera_down/image_raw
  ~json_path          str    $(find mission_control)/config/wp_marker_reference.json
  ~hamming_threshold  int    12
  ~wp2_min_area_px    int    3000
  ~throttle_hz        float  5.0
  ~publish_debug      bool   true
"""
import os
import sys
sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))

import numpy as np
import cv2
import rospy
from sensor_msgs.msg import Image
from std_msgs.msg import Bool, Header

from mission_control.msg import WpMarkerResult, ArucoMarker, ArucoMarkers

from wp_marker_detector import decode_frame, detect_wp2_presence, load_refs
from image_convert import imgmsg_to_np, np_to_imgmsg


class WpMarkerNode:
    def __init__(self):
        image_topic   = rospy.get_param('~image_topic',       '/camera_down/image_raw')
        json_path     = rospy.get_param('~json_path',         '')
        self.hamming  = int(rospy.get_param('~hamming_threshold', 12))
        self.min_area = int(rospy.get_param('~wp2_min_area_px',   3000))
        throttle_hz   = float(rospy.get_param('~throttle_hz',  5.0))
        self.pub_dbg  = bool(rospy.get_param('~publish_debug', True))

        if not json_path:
            pkg = os.path.join(os.path.dirname(os.path.abspath(__file__)), '..', 'config')
            json_path = os.path.normpath(os.path.join(pkg, 'wp_marker_reference.json'))

        self.refs = load_refs(json_path, wp_ids=(1, 3, 4))
        rospy.loginfo('[wp_marker] refs loaded from %s  hamming_thr=%d  wp2_area=%d',
                      json_path, self.hamming, self.min_area)

        self.pub_result  = rospy.Publisher('~result',      WpMarkerResult, queue_size=5)
        self.pub_wp2     = rospy.Publisher('~wp2_present', Bool,           queue_size=5)
        self.pub_markers = rospy.Publisher('~markers',     ArucoMarkers,   queue_size=5)
        if self.pub_dbg:
            self.pub_debug = rospy.Publisher('~debug_image', Image, queue_size=2)

        self._throttle_period = 1.0 / max(throttle_hz, 0.1)
        self._last_proc = rospy.Time(0)
        self._warn_count = 0

        rospy.Subscriber(image_topic, Image, self._image_cb,
                         queue_size=1, buff_size=2**24)
        rospy.loginfo('[wp_marker] siap. topic=%s  throttle=%.1f Hz', image_topic, throttle_hz)

    def _image_cb(self, msg):
        now = rospy.Time.now()
        if (now - self._last_proc).to_sec() < self._throttle_period:
            return
        self._last_proc = now

        try:
            bgr = imgmsg_to_np(msg)
        except ValueError as e:
            if self._warn_count < 3:
                rospy.logwarn('[wp_marker] imgmsg_to_np: %s', e)
                self._warn_count += 1
            return

        h_img, w_img = bgr.shape[:2]

        # --- WP1/3/4 ID decode ---
        dec = decode_frame(bgr, self.refs, self.hamming)

        res = WpMarkerResult()
        res.header        = msg.header
        res.found_marker  = bool(dec['found_marker'])
        res.wp_id         = int(dec['wp_id'])
        res.hamming       = int(dec['hamming'])
        res.reliable      = bool(dec['reliable'])
        res.low_conf_tail = bool(dec['low_conf_tail'])
        res.reason        = str(dec['reason'])
        res.aspect        = float(dec['aspect'])
        res.off_x         = float(dec['off_x'])
        res.off_y         = float(dec['off_y'])
        self.pub_result.publish(res)

        # --- WP2 presence ---
        wp2 = detect_wp2_presence(bgr, self.min_area)
        self.pub_wp2.publish(Bool(data=wp2['present']))

        # --- ArucoMarkers (kompatibel mission_d) ---
        # Publish saat ID berhasil decode (reason='ok') ATAU WP2 terdeteksi hadir.
        # WP2 id=2 dipakai saat present=True; off_x/off_y dari blob centroid.
        aruco_msg = ArucoMarkers()
        aruco_msg.header       = msg.header
        aruco_msg.image_width  = w_img
        aruco_msg.image_height = h_img

        if dec['reason'] == 'ok' and dec['wp_id'] != -1:
            am = ArucoMarker()
            am.id    = dec['wp_id']
            am.off_x = dec['off_x']
            am.off_y = dec['off_y']
            am.cx    = (dec['off_x'] + 1.0) * w_img / 2.0
            am.cy    = (dec['off_y'] + 1.0) * h_img / 2.0
            aruco_msg.markers.append(am)

        if wp2['present']:
            am2 = ArucoMarker()
            am2.id    = 2
            am2.off_x = wp2['off_x']
            am2.off_y = wp2['off_y']
            am2.cx    = (wp2['off_x'] + 1.0) * w_img / 2.0
            am2.cy    = (wp2['off_y'] + 1.0) * h_img / 2.0
            aruco_msg.markers.append(am2)

        self.pub_markers.publish(aruco_msg)

        # --- debug image (only when subscriber connected) ---
        if self.pub_dbg and self.pub_debug.get_num_connections() > 0:
            vis = _draw_debug(bgr, dec, wp2)
            self.pub_debug.publish(
                np_to_imgmsg(vis, msg.header.stamp, msg.header.frame_id))


def _draw_debug(bgr, dec, wp2):
    """Minimal annotation: WP id + reason + WP2 area + offsets."""
    vis = bgr.copy()
    h, w = vis.shape[:2]
    color = (0, 255, 0) if dec['wp_id'] != -1 else (0, 0, 255)
    label = (f"WP{dec['wp_id']} H={dec['hamming']} {dec['reason']}"
             f" off=({dec['off_x']:.2f},{dec['off_y']:.2f})"
             if dec['found_marker'] else dec['reason'])
    cv2.putText(vis, label, (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 0.7, color, 2)
    wp2_color = (0, 255, 0) if wp2['present'] else (128, 128, 128)
    cv2.putText(vis, f"WP2 present={wp2['present']} area={wp2['area_px']}",
                (10, 58), cv2.FONT_HERSHEY_SIMPLEX, 0.65, wp2_color, 2)
    # crosshair bila marker terdeteksi
    if dec['found_marker'] and dec['wp_id'] != -1:
        cx = int((dec['off_x'] + 1.0) * w / 2)
        cy = int((dec['off_y'] + 1.0) * h / 2)
        cv2.drawMarker(vis, (cx, cy), (0, 255, 0), cv2.MARKER_CROSS, 20, 2)
    return vis


def main():
    rospy.init_node('wp_marker_node')
    WpMarkerNode()
    rospy.spin()


if __name__ == '__main__':
    main()

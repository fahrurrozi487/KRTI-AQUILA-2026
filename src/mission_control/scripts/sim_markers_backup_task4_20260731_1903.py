#!/usr/bin/env python3
"""
Mock 'kamera bawah virtual' untuk uji Fase D di SITL TANPA kamera nyata.

Menaruh marker ArUco di koordinat dunia (dari waypoints.yaml + leg ber-expected_id),
lalu menghitung offset marker terhadap posisi drone live (/mavros/local_position/pose)
dan mem-publish /aruco_node/markers. Karena offset bergantung posisi drone, loop
align di mission_d.py jadi TERTUTUP (drone bergerak -> offset mengecil -> drop).

Marker sengaja digeser kecil (marker_dx,marker_dy) dari koordinat WP supaya align
benar-benar bekerja (ember tak persis di titik survei).

Param:
  ~waypoints_file, ~mission_file
  ~view_radius (m)  jarak horizontal marker masuk 'pandangan'   default 3.0
  ~marker_dx, ~marker_dy (m)  geser marker dari WP               default 0.5, 0.4
  ~pub_topic        default /aruco_node/markers
"""
import os
import sys
sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))

import yaml
import rospy
from geometry_msgs.msg import PoseStamped
from mission_control.msg import ArucoMarker, ArucoMarkers

W, H = 640, 480


class SimMarkers:
    def __init__(self):
        wp_file = rospy.get_param("~waypoints_file")
        mission_file = rospy.get_param("~mission_file")
        self.view_radius = float(rospy.get_param("~view_radius", 3.0))
        self.mdx = float(rospy.get_param("~marker_dx", 0.5))
        self.mdy = float(rospy.get_param("~marker_dy", 0.4))
        topic = rospy.get_param("~pub_topic", "/aruco_node/markers")

        with open(os.path.expanduser(wp_file)) as f:
            wps = yaml.safe_load(f) or {}
        with open(os.path.expanduser(mission_file)) as f:
            legs = (yaml.safe_load(f) or {}).get("legs", [])

        # {id: (x,y)} dari leg yg punya expected_id
        self.markers = {}
        for leg in legs:
            if "expected_id" in leg and leg["wp"] in wps:
                w = wps[leg["wp"]]
                self.markers[int(leg["expected_id"])] = (
                    float(w["x"]) + self.mdx, float(w["y"]) + self.mdy)
        rospy.loginfo("[sim] %d marker virtual: %s", len(self.markers),
                      {k: tuple(round(c, 1) for c in v) for k, v in self.markers.items()})

        self.pose = None
        rospy.Subscriber("/mavros/local_position/pose", PoseStamped, self._cb)
        self.pub = rospy.Publisher(topic, ArucoMarkers, queue_size=5)

    def _cb(self, msg):
        self.pose = msg.pose.position

    @staticmethod
    def _clamp(v):
        return max(-1.0, min(1.0, v))

    def spin(self):
        rate = rospy.Rate(15)
        while not rospy.is_shutdown():
            if self.pose is not None:
                out = ArucoMarkers()
                out.header.stamp = rospy.Time.now()
                out.header.frame_id = "camera_down"
                out.image_width, out.image_height = W, H
                dx0, dy0 = self.pose.x, self.pose.y
                for mid, (mx, my) in self.markers.items():
                    ex, ey = mx - dx0, my - dy0
                    if abs(ex) > self.view_radius or abs(ey) > self.view_radius:
                        continue                       # di luar pandangan
                    off_x = self._clamp(ex / self.view_radius)
                    off_y = self._clamp(ey / self.view_radius)
                    am = ArucoMarker()
                    am.id = mid
                    am.off_x, am.off_y = off_x, off_y
                    am.cx = W / 2.0 + off_x * W / 2.0
                    am.cy = H / 2.0 + off_y * H / 2.0
                    am.side_px = 120.0
                    am.has_pose = False
                    out.markers.append(am)
                self.pub.publish(out)
            rate.sleep()


def main():
    rospy.init_node("sim_markers")
    SimMarkers().spin()


if __name__ == "__main__":
    main()

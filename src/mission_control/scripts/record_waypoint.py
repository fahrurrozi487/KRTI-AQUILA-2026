#!/usr/bin/env python3
"""
Waypoint recorder: catat posisi T265 (local ENU) sekarang sebagai sebuah waypoint.
Pakai saat survei lapangan: bawa drone ke titik (WP1/WP2/WP3/gate), jalankan ini,
posisi tersimpan ke file YAML.

Pakai:
  rosrun mission_control record_waypoint.py _name:=wp1
  rosrun mission_control record_waypoint.py _name:=gate_double _file:=/home/jetson/catkin_ws/src/mission_control/config/waypoints.yaml

Membaca x,y dari /mavros/local_position/pose (ENU-local, butuh mavros + T265
pipeline aktif & EKF origin di-set). z dari /mavros/global_position/rel_alt
(AGL asli, home-relative) -- 14 Sep: BUKAN lagi local_position/pose.z, biar
konsisten sama datum yang dipakai mavros_helper.py/mission_d.py sekarang.
Setiap pemanggilan MENAMBAH/menimpa entri bernama <name> di file YAML (tidak menghapus yang lain).
"""
import os
import yaml
import rospy
from geometry_msgs.msg import PoseStamped
from std_msgs.msg import Float64

DEFAULT_FILE = os.path.expanduser(
    "~/catkin_ws/src/mission_control/config/waypoints.yaml")


def main():
    rospy.init_node("record_waypoint")
    name = rospy.get_param("~name", None)
    path = rospy.get_param("~file", DEFAULT_FILE)
    if not name:
        rospy.logerr("Wajib beri nama: _name:=wp1")
        return

    rospy.loginfo("[recorder] menunggu /mavros/local_position/pose...")
    try:
        msg = rospy.wait_for_message(
            "/mavros/local_position/pose", PoseStamped, timeout=15.0)
    except rospy.ROSException:
        rospy.logerr("[recorder] timeout: tidak ada pose. T265+mavros aktif & EKF origin di-set?")
        return

    p = msg.pose.position
    o = msg.pose.orientation

    rospy.loginfo("[recorder] menunggu /mavros/global_position/rel_alt...")
    try:
        alt_msg = rospy.wait_for_message(
            "/mavros/global_position/rel_alt", Float64, timeout=15.0)
    except rospy.ROSException:
        rospy.logerr("[recorder] timeout: tidak ada rel_alt. mavros aktif & home di-set?")
        return
    z = alt_msg.data

    data = {}
    if os.path.exists(path):
        with open(path) as f:
            data = yaml.safe_load(f) or {}

    data[name] = {
        "x": round(float(p.x), 3),
        "y": round(float(p.y), 3),
        "z": round(float(z), 3),
        "qz": round(float(o.z), 4),
        "qw": round(float(o.w), 4),
    }

    os.makedirs(os.path.dirname(path), exist_ok=True)
    with open(path, "w") as f:
        yaml.safe_dump(data, f, default_flow_style=False, sort_keys=True)

    rospy.loginfo("[recorder] '%s' = (%.2f, %.2f, %.2f AGL) tersimpan ke %s",
                  name, p.x, p.y, z, path)


if __name__ == "__main__":
    try:
        main()
    except rospy.ROSInterruptException:
        pass

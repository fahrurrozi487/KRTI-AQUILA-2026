#!/usr/bin/env python3
"""
Waypoint recorder versi GPS: catat lat/lon ASLI (bukan ENU lokal) + ketinggian
relatif (AGL) sekarang, sebagai satu entri di waypoints_latlon.yaml.

Companion dari record_waypoint.py (ENU lokal -> waypoints.yaml). Beda: file ini
simpen KOORDINAT DUNIA NYATA (lat/lon), yang PERMANEN lintas sesi -- gak kayak
ENU lokal yang berubah kalau EKF origin beda (T265 manual vs GPS otomatis, lih.
docstring latlon_to_waypoints.py). Cocok dipakai SAAT nav pakai GPS (origin GPS
sudah established, `/mavros/global_position/global` = fix ter-filter EKF).

Alur lengkap:
  1. rosrun mission_control record_waypoint_latlon.py _name:=wp1   <- file ini
     (drone di titik fisik WP1, GUIDED, GPS fix bagus)
  2. ulangi tiap titik (wp1, gate_double, wp2, gate_triple, wp3, ...)
  3. SETIAP SESI TERBANG sesudahnya: rosrun mission_control latlon_to_waypoints.py
     -> convert ke ENU lokal (waypoints.yaml) relatif origin EKF yang aktif SAAT ITU

Pakai:
  rosrun mission_control record_waypoint_latlon.py _name:=wp1
  rosrun mission_control record_waypoint_latlon.py _name:=gate_double _file:=...

Membaca:
  /mavros/global_position/global (sensor_msgs/NavSatFix) -> lat/lon (fix ter-filter)
  /mavros/local_position/pose    (geometry_msgs/PoseStamped) -> z (AGL, sama
    konvensi dgn record_waypoint.py/mission_d.py -- BUKAN altitude AMSL dari GPS)

Setiap pemanggilan MENAMBAH/menimpa entri bernama <name> (tidak menghapus yang lain),
sama seperti record_waypoint.py.
"""
import os
import yaml
import rospy
from sensor_msgs.msg import NavSatFix, NavSatStatus
from geometry_msgs.msg import PoseStamped

DEFAULT_FILE = os.path.expanduser(
    "~/catkin_ws/src/mission_control/config/waypoints_latlon.yaml")


def main():
    rospy.init_node("record_waypoint_latlon")
    name = rospy.get_param("~name", None)
    path = rospy.get_param("~file", DEFAULT_FILE)
    if not name:
        rospy.logerr("Wajib beri nama: _name:=wp1")
        return

    rospy.loginfo("[recorder_latlon] menunggu /mavros/global_position/global (GPS fix)...")
    try:
        fix = rospy.wait_for_message(
            "/mavros/global_position/global", NavSatFix, timeout=15.0)
    except rospy.ROSException:
        rospy.logerr("[recorder_latlon] timeout: belum ada GPS fix. mavros aktif & GPS lock?")
        return

    if fix.status.status < NavSatStatus.STATUS_FIX:
        rospy.logerr("[recorder_latlon] GPS BELUM FIX (status=%d) -- BATAL rekam, "
                    "cek lokasi terbuka/tunggu lock dulu.", fix.status.status)
        return

    rospy.loginfo("[recorder_latlon] menunggu /mavros/local_position/pose (utk z/AGL)...")
    try:
        pose = rospy.wait_for_message(
            "/mavros/local_position/pose", PoseStamped, timeout=15.0)
    except rospy.ROSException:
        rospy.logerr("[recorder_latlon] timeout: local_position/pose tidak ada.")
        return

    data = {}
    if os.path.exists(path):
        with open(path) as f:
            data = yaml.safe_load(f) or {}

    data[name] = {
        "lat": round(float(fix.latitude), 7),
        "lon": round(float(fix.longitude), 7),
        "alt": round(float(pose.pose.position.z), 3),
    }

    os.makedirs(os.path.dirname(path), exist_ok=True)
    with open(path, "w") as f:
        yaml.safe_dump(data, f, default_flow_style=False, sort_keys=True)

    rospy.loginfo("[recorder_latlon] '%s' = (lat=%.7f, lon=%.7f, alt=%.2fm AGL) "
                  "tersimpan ke %s", name, fix.latitude, fix.longitude,
                  pose.pose.position.z, path)
    rospy.loginfo("[recorder_latlon] ingat: jalankan latlon_to_waypoints.py "
                  "buat convert ke waypoints.yaml sebelum misi.")


if __name__ == "__main__":
    try:
        main()
    except rospy.ROSInterruptException:
        pass

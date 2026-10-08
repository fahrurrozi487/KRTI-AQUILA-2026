#!/usr/bin/env python3
# FASE B: bukti Guided autonomous di SITL/Gazebo (ArduCopter).
# Urutan: tunggu FCU -> GUIDED -> arm -> takeoff -> goto 1 koordinat -> hover.
#
# PENTING (ArduCopter GUIDED, BUKAN PX4 OFFBOARD):
#   - JANGAN stream setpoint_position selama takeoff. GUIDED punya sub-mode
#     (TakeOff / Position). NAV_TAKEOFF masuk sub-mode TakeOff; bila langsung
#     diikuti SET_POSITION_TARGET (dari setpoint_position), GUIDED pindah ke
#     sub-mode Position saat masih di tanah -> tidak naik -> auto-disarm.
#     (Terbukti di SITL: dgn stream z tetap 0 lalu disarm; tanpa stream naik OK.)
#   - ArduCopter GUIDED TIDAK butuh stream setpoint kontinu utk tetap di mode
#     (itu syarat PX4 OFFBOARD). Stream setpoint hanya dipakai utk fase goto.
# Param: ~target_x/y/z (default 5,0,2), ~takeoff_alt (=target_z), ~reach_tol (0.3 m)
import os
import sys
# catkin merelay script ini dari devel/lib (shim). Tambah dir SUMBER ke path
# agar 'import mavros_helper' dapat modul asli (berisi class), bukan shim relay.
sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
import rospy
from mavros_helper import MavrosHelper


def main():
    rospy.init_node("mission_node")
    tx = rospy.get_param("~target_x", 5.0)
    ty = rospy.get_param("~target_y", 0.0)
    tz = rospy.get_param("~target_z", 2.0)
    takeoff_alt = rospy.get_param("~takeoff_alt", tz)
    tol = rospy.get_param("~reach_tol", 0.3)

    h = MavrosHelper()
    if not h.wait_for_connection():
        return
    h.wait_for_pose()
    rate = rospy.Rate(20)

    # --- GUIDED + arm (tanpa stream setpoint; ArduCopter tidak butuh) ---
    rospy.loginfo("[mission] set mode GUIDED...")
    while not rospy.is_shutdown() and h.state.mode != "GUIDED":
        h.set_mode("GUIDED")
        rate.sleep()

    rospy.loginfo("[mission] arming...")
    while not rospy.is_shutdown() and not h.state.armed:
        h.arm(True)
        rospy.sleep(0.5)

    # --- takeoff: JANGAN stream setpoint di sini (lihat catatan di header) ---
    rospy.loginfo("[mission] takeoff ke %.1f m...", takeoff_alt)
    h.takeoff(takeoff_alt)
    t0 = rospy.Time.now()
    last_cmd = t0
    while not rospy.is_shutdown():
        z = h.get_xyz()[2]
        if z > takeoff_alt - 0.3:
            break
        if not h.state.armed:
            rospy.logerr("[mission] disarm saat takeoff! batal.")
            return
        # bila belum mulai naik dalam 4 s, kirim ulang perintah takeoff sekali-sekali
        if z < 0.2 and (rospy.Time.now() - last_cmd).to_sec() > 4.0:
            rospy.logwarn("[mission] belum naik, kirim ulang takeoff...")
            h.takeoff(takeoff_alt)
            last_cmd = rospy.Time.now()
        if (rospy.Time.now() - t0).to_sec() > 30:
            rospy.logwarn("[mission] takeoff timeout, lanjut")
            break
        rate.sleep()
    rospy.loginfo("[mission] takeoff selesai (z=%.2f)", h.get_xyz()[2])

    rospy.loginfo("[mission] goto (%.1f, %.1f, %.1f)...", tx, ty, tz)
    while not rospy.is_shutdown():
        h.send_setpoint(tx, ty, tz)
        if h.reached(tx, ty, tz, tol):
            rospy.loginfo("[mission] SAMPAI. Hover.")
            break
        if h.state.mode != "GUIDED":
            rospy.logwarn("[mission] keluar GUIDED (pilot ambil alih) -> STOP")
            return
        rate.sleep()

    while not rospy.is_shutdown() and h.state.mode == "GUIDED":
        h.send_setpoint(tx, ty, tz)
        rate.sleep()


if __name__ == "__main__":
    try:
        main()
    except rospy.ROSInterruptException:
        pass

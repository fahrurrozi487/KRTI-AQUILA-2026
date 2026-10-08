#!/usr/bin/env python3
"""
Kalibrasi align_sign_x/y (ArUco/WP, kamera bawah) atau gate_sign_x/y (gate,
kamera depan) DI DARAT -- tanpa terbang, tanpa kirim setpoint sama sekali.

Kenapa perlu: align_sign_x/y & gate_sign_x/y BELUM PERNAH dikalibrasi fisik
(masih default +1.0 di semua launch file, dicek 2026-08-08). Kalau salah,
loop P-control di mission_d.py jadi POSITIVE FEEDBACK -- drone dikoreksi
MENJAUH dari target, bukan mendekat, sampai mentok clamp align_max_drift/
gate_max_drift. Ini alat buat nemuin sign yang benar SEBELUM resiko itu
kejadian di udara, dengan cara gerakin marker/gerbang manual di depan
kamera & baca prediksi arah koreksi.

Logika HARUS identik dengan mission_d.py (_align_and_drop/_traverse) --
kalau mission_d.py berubah, tool ini perlu disinkronkan lagi.

2026-08-10: mission_d.py sekarang ROTASI koreksi BODY-frame (sign_x/sign_y*
off_x/off_y -- hasil dari tool ini) ke ENU pakai yaw SEKARANG (_body_to_enu(),
bug lama: TIDAK dirotasi sama sekali, cuma valid kalau yaw pas terbang PERSIS
sama kayak yaw pas kalibrasi -- misi punya leg yaw 90 derajat antara WP2 &
gate_triple, jadi salah satu gerbang PASTI salah tanpa fix ini). Konsekuensi
buat tool ini: sign_x/sign_y/swap_xy yang kamu temukan di SINI itu murni
properti MOUNTING kamera (independen dari arah hadap drone) -- CUKUP
kalibrasi SEKALI di sembarang posisi/heading yang nyaman, TIDAK perlu ulang
di tiap heading yang bakal dipakai pas misi (mission_d.py otomatis rotasi
sisanya). de/dn di bawah ini masih dalam BODY frame (SEBELUM rotasi yaw) --
buat ngecek sign_x/sign_y aja, bukan preview ENU final yg beneran dikirim.

Pakai (marker bawah, ArUco/WP -- expected_id sesuai target align):
  rosrun mission_control calibrate_align_sign.py _mode:=align _expected_id:=1

Pakai (gate depan):
  rosrun mission_control calibrate_align_sign.py _mode:=gate

Cara baca hasil per frame:
  off_x/off_y   = offset mentah dari kamera (-1..1), sama definisi kayak
                  dokumentasi ArucoMarker.msg/Gate.msg
  bx/by (dz)    = arah koreksi BODY-frame (mounting kamera, BELUM dirotasi ke
                  ENU -- itu baru terjadi live di mission_d.py pakai yaw
                  sekarang, lih. catatan 2026-08-10 di atas), pakai
                  align_sign_x/y (atau gate_sign_x/y/z) SEKARANG (param,
                  default +1.0, sama seperti mission_d.py)

Cek manual: geser marker/gerbang ke KANAN gambar (off_x makin +) -- drone
SEHARUSNYA gerak ke arah yang bikin dia balik ke tengah kamera (biasanya ke
arah marker itu sendiri, tergantung mounting). Kalau tanda bx/by yang keluar
kebalik dari akal sehat mounting kameramu, set param sign itu ke -1.0. TIDAK
perlu ulang tes di heading lain -- sign_x/sign_y itu properti mounting fisik,
independen dari arah hadap drone (rotasi ke ENU sudah otomatis di mission_d.py).
"""
import sys
import os
sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))

import rospy
from mission_control.msg import ArucoMarkers, Gate


def main():
    rospy.init_node("calibrate_align_sign", anonymous=True)
    mode = rospy.get_param("~mode", "align")  # align | gate
    expected_id = int(rospy.get_param("~expected_id", 1))

    if mode == "align":
        sign_x = float(rospy.get_param("~align_sign_x", 1.0))
        sign_y = float(rospy.get_param("~align_sign_y", 1.0))
        swap_xy = bool(rospy.get_param("~align_swap_xy", False))
        topic = rospy.get_param("~markers_topic", "/aruco_node/markers")
        rospy.loginfo("[calib] mode=align  sign_x=%.1f sign_y=%.1f swap_xy=%s  "
                      "topic=%s  expected_id=%d", sign_x, sign_y, swap_xy,
                      topic, expected_id)

        def cb(msg):
            for mk in msg.markers:
                if mk.id != expected_id:
                    continue
                ox, oy = mk.off_x, mk.off_y
                bx = sign_x * (oy if swap_xy else ox)
                by = sign_y * (ox if swap_xy else oy)
                rospy.loginfo("id=%d  off_x=%+.2f off_y=%+.2f  ->  "
                              "koreksi BODY-frame: bx=%+.3f by=%+.3f "
                              "(diputar ke ENU pakai yaw sekarang saat misi jalan)",
                              mk.id, ox, oy, bx, by)
                return
            rospy.loginfo_throttle(1.0, "id=%d belum kelihatan...", expected_id)

        rospy.Subscriber(topic, ArucoMarkers, cb, queue_size=1)

    elif mode == "gate":
        sign_x = float(rospy.get_param("~gate_sign_x", 1.0))
        sign_y = float(rospy.get_param("~gate_sign_y", 1.0))
        sign_z = float(rospy.get_param("~gate_sign_z", -1.0))
        swap_xy = bool(rospy.get_param("~gate_swap_xy", False))
        topic = rospy.get_param("~gate_topic", "/gate_node/gate")
        rospy.loginfo("[calib] mode=gate  sign_x=%.1f sign_y=%.1f sign_z=%.1f "
                      "swap_xy=%s  topic=%s", sign_x, sign_y, sign_z, swap_xy, topic)

        def cb(msg):
            if not msg.detected:
                rospy.loginfo_throttle(1.0, "gate belum terdeteksi...")
                return
            ox, oy = msg.off_x, msg.off_y
            bx = sign_x * (oy if swap_xy else ox)
            by = sign_y * (ox if swap_xy else oy)
            dz = sign_z * oy
            rospy.loginfo("off_x=%+.2f off_y=%+.2f  ->  koreksi BODY-frame: "
                          "bx=%+.3f by=%+.3f dz(atas-)=%+.3f "
                          "(bx/by diputar ke ENU pakai yaw sekarang saat misi jalan)",
                          ox, oy, bx, by, dz)

        rospy.Subscriber(topic, Gate, cb, queue_size=1)

    else:
        rospy.logerr("~mode harus 'align' atau 'gate', dapat: %s", mode)
        return

    rospy.loginfo("[calib] siap. Gerakin marker/gerbang di depan kamera, "
                  "amati arah 'koreksi' vs akal sehat mounting. "
                  "TIDAK ada setpoint dikirim ke drone -- aman dijalankan kapan saja.")
    rospy.spin()


if __name__ == "__main__":
    try:
        main()
    except rospy.ROSInterruptException:
        pass

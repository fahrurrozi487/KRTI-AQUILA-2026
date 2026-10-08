#!/usr/bin/env python3
"""
Set EKF origin + home position untuk terbang non-GPS (T265).

Non-GPS: EKF tak punya lat/lon absolut, jadi origin & home HARUS di-set tiap boot FC.
Node ini nunggu T265 (local_position) siap, lalu:
  1. publish /mavros/global_position/set_gp_origin  -> acuan global EKF
  2. panggil /mavros/cmd/set_home                    -> titik RTL/land
lalu exit (one-shot). Jalankan di akhir bring-up, sesudah mavros konek & T265 jalan.

Koordinat = param ~lat / ~lon / ~alt. Untuk non-GPS angka absolut TIDAK kritis
(navigasi relatif ke titik takeoff); ganti dgn koord lapangan hanya biar peta MP pas.
"""
import rospy
from geographic_msgs.msg import GeoPointStamped
from geometry_msgs.msg import PoseStamped
from mavros_msgs.srv import CommandHome


def wait_local_pose(timeout=60.0):
    """Tunggu T265 kasih local_position (bukti pipa vision hidup) sebelum set origin."""
    box = {"ok": False}
    sub = rospy.Subscriber("/mavros/local_position/pose", PoseStamped,
                           lambda _m: box.__setitem__("ok", True))
    t0 = rospy.Time.now()
    r = rospy.Rate(5)
    while not rospy.is_shutdown() and not box["ok"]:
        if (rospy.Time.now() - t0).to_sec() > timeout:
            rospy.logwarn("[set_origin_home] local_position belum muncul %.0fs; "
                          "lanjut set origin toh (cek T265 kalau home meleset).", timeout)
            break
        r.sleep()
    sub.unregister()
    return box["ok"]


def main():
    rospy.init_node("set_origin_home")
    lat = rospy.get_param("~lat", -7.0501108)
    lon = rospy.get_param("~lon", 110.3913388)
    alt = rospy.get_param("~alt", 700.0)

    wait_local_pose()

    # 1) EKF global origin (latched — sekali cukup, kirim beberapa kali biar pasti diterima)
    pub = rospy.Publisher("/mavros/global_position/set_gp_origin",
                          GeoPointStamped, queue_size=1, latch=True)
    msg = GeoPointStamped()
    msg.position.latitude = lat
    msg.position.longitude = lon
    msg.position.altitude = alt
    rospy.sleep(1.0)  # beri waktu koneksi publisher ke mavros
    for _ in range(3):
        msg.header.stamp = rospy.Time.now()
        pub.publish(msg)
        rospy.sleep(0.5)
    rospy.loginfo("[set_origin_home] origin dikirim: %.6f, %.6f, %.1f", lat, lon, alt)

    # 2) Home position (service — retry sampai ACCEPTED).
    # current_gps=true: home = posisi/altitude estimasi FC saat ini, biar datum
    # altitude home NYATU dgn datum vertikal EKF (baro, di-nol saat init). Kalau
    # altitude dikirim manual, rel_alt jadi offset (mis. -21m walau di tanah).
    rospy.wait_for_service("/mavros/cmd/set_home")
    set_home = rospy.ServiceProxy("/mavros/cmd/set_home", CommandHome)
    for i in range(5):
        try:
            resp = set_home(current_gps=True, yaw=0.0,
                            latitude=0.0, longitude=0.0, altitude=0.0)
            if resp.success:
                rospy.loginfo("[set_origin_home] home OK (result=%d).", resp.result)
                return
            rospy.logwarn("[set_origin_home] set_home ditolak (coba %d/5)...", i + 1)
        except rospy.ServiceException as e:
            rospy.logwarn("[set_origin_home] service error: %s", e)
        rospy.sleep(1.0)
    rospy.logerr("[set_origin_home] GAGAL set home setelah 5x. Cek link FC.")


if __name__ == "__main__":
    try:
        main()
    except rospy.ROSInterruptException:
        pass

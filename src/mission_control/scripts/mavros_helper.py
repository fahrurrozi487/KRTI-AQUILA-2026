#!/usr/bin/env python3
"""
Helper koneksi mavros: state, posisi lokal, set mode, arm, takeoff, kirim setpoint.
Dipakai bersama oleh mission_node.py dan record_waypoint.py.

Frame setpoint: /mavros/setpoint_position/local = ENU lokal (X=Timur, Y=Utara, Z=Atas),
relatif terhadap EKF origin (= titik takeoff). Sama dengan frame yang dipakai T265.

14 Sep 2026: Z BUKAN lagi ENU-local. x,y tetap dari /mavros/local_position/pose
(ENU-local), tapi z SEKARANG AGL asli dari /mavros/global_position/rel_alt
(home-relative) -- root cause insiden origin basi (home_position.z=0.91m offset
bikin takeoff-check/goto ketipu, lih. mission_d.py). get_xyz()[2] dan argumen z
ke make_setpoint()/send_setpoint() SEMUA AGL. Konversi balik ke ENU-local
(WAJIB krn /mavros/setpoint_position/local cuma ngerti ENU-local, MAV_FRAME_
LOCAL_NED) terjadi PERSIS di make_setpoint(), pakai offset origin<->AGL yang
dihitung ULANG dari nilai live tiap panggilan -- gak nyimpen angka basi.
"""
import math
import rospy
from geometry_msgs.msg import PoseStamped
from std_msgs.msg import Float64
from mavros_msgs.msg import State
from mavros_msgs.srv import CommandBool, SetMode, CommandTOL, CommandLong

MAV_CMD_DO_SET_SERVO = 183


class MavrosHelper:
    def __init__(self):
        self.state = State()
        self.pose = PoseStamped()
        self._have_pose = False
        self._local_z = 0.0        # ENU-local z mentah, dipakai konversi make_setpoint()
        self.rel_alt = 0.0         # AGL asli (home-relative), lih. get_xyz()
        self._have_rel_alt = False
        self._rel_alt_stamp = rospy.Time(0)  # Float64 gak punya header -> catat sendiri

        rospy.Subscriber("/mavros/state", State, self._state_cb)
        rospy.Subscriber("/mavros/local_position/pose", PoseStamped, self._pose_cb)
        rospy.Subscriber("/mavros/global_position/rel_alt", Float64, self._rel_alt_cb)
        self.sp_pub = rospy.Publisher(
            "/mavros/setpoint_position/local", PoseStamped, queue_size=10)

        rospy.loginfo("[mavros_helper] menunggu service mavros...")
        rospy.wait_for_service("/mavros/cmd/arming")
        rospy.wait_for_service("/mavros/set_mode")
        rospy.wait_for_service("/mavros/cmd/takeoff")
        rospy.wait_for_service("/mavros/cmd/command")
        self._arm = rospy.ServiceProxy("/mavros/cmd/arming", CommandBool)
        self._set_mode = rospy.ServiceProxy("/mavros/set_mode", SetMode)
        self._takeoff = rospy.ServiceProxy("/mavros/cmd/takeoff", CommandTOL)
        self._cmd = rospy.ServiceProxy("/mavros/cmd/command", CommandLong)
        rospy.loginfo("[mavros_helper] service siap.")

    def _state_cb(self, msg):
        self.state = msg

    def _pose_cb(self, msg):
        self.pose = msg
        self._local_z = msg.pose.position.z
        self._have_pose = True

    def _rel_alt_cb(self, msg):
        self.rel_alt = msg.data
        self._have_rel_alt = True
        self._rel_alt_stamp = rospy.Time.now()

    def pose_age(self):
        """Umur /mavros/local_position/pose terbaru (detik, x/y). Stale = EKF/mavros
        berhenti update. Dulu bernama _pose_age(), sekarang di sini krn mission_d.py
        butuh gabung dgn agl_age() -- lih. docstring _healthy() di mission_d.py."""
        try:
            stamp = self.pose.header.stamp
            if stamp.is_zero():
                return float("inf")
            age = (rospy.Time.now() - stamp).to_sec()
            return age if age >= 0 else 0.0   # stamp masa depan (clock drift) = segar
        except Exception:
            return float("inf")

    def agl_age(self):
        """Umur /mavros/global_position/rel_alt terbaru (detik, z/AGL). Float64 gak
        punya header -- freshness dicatat sendiri di _rel_alt_cb() saat pesan dateng."""
        try:
            if self._rel_alt_stamp.is_zero():
                return float("inf")
            age = (rospy.Time.now() - self._rel_alt_stamp).to_sec()
            return age if age >= 0 else 0.0
        except Exception:
            return float("inf")

    def wait_for_connection(self, timeout=60.0):
        rospy.loginfo("[mavros_helper] menunggu koneksi FCU...")
        rate = rospy.Rate(5)
        t0 = rospy.Time.now()
        while not rospy.is_shutdown() and not self.state.connected:
            if (rospy.Time.now() - t0).to_sec() > timeout:
                rospy.logerr("[mavros_helper] timeout: FCU tidak terhubung")
                return False
            rate.sleep()
        rospy.loginfo("[mavros_helper] FCU terhubung.")
        return True

    def wait_for_pose(self, timeout=30.0):
        rate = rospy.Rate(5)
        t0 = rospy.Time.now()
        while not rospy.is_shutdown() and not (self._have_pose and self._have_rel_alt):
            if (rospy.Time.now() - t0).to_sec() > timeout:
                rospy.logerr("[mavros_helper] timeout: belum ada local_position/pose "
                             "dan/atau global_position/rel_alt")
                return False
            rate.sleep()
        return True

    def get_xyz(self):
        """x,y dari /mavros/local_position/pose (ENU-local). z dari /mavros/
        global_position/rel_alt (AGL asli, home-relative) -- BUKAN ENU-local
        lagi, lih. docstring modul ini soal insiden origin basi."""
        p = self.pose.pose.position
        return (p.x, p.y, self.rel_alt)

    def get_yaw(self):
        """Yaw sekarang (rad) dari orientation quaternion /mavros/local_position/pose,
        konvensi SAMA kayak make_setpoint() (rotasi Z murni, 0=hadap +X/Timur,
        naik berlawanan jarum jam ke +Y/Utara -- REP-103/ENU standar). Dipakai
        buat rotasi koreksi align/gate dari BODY frame (mounting kamera) ke
        ENU frame SEBELUM ditambahkan ke setpoint -- lih. mission_d.py."""
        q = self.pose.pose.orientation
        return math.atan2(2.0 * (q.w * q.z + q.x * q.y),
                          1.0 - 2.0 * (q.y * q.y + q.z * q.z))

    def set_mode(self, mode):
        try:
            return self._set_mode(base_mode=0, custom_mode=mode).mode_sent
        except rospy.ServiceException as e:
            rospy.logerr("[mavros_helper] set_mode gagal: %s", e)
            return False

    def arm(self, value=True):
        try:
            return self._arm(value).success
        except rospy.ServiceException as e:
            rospy.logerr("[mavros_helper] arm gagal: %s", e)
            return False

    def takeoff(self, altitude):
        try:
            return self._takeoff(altitude=altitude, latitude=0, longitude=0,
                                 min_pitch=0, yaw=0).success
        except rospy.ServiceException as e:
            rospy.logerr("[mavros_helper] takeoff gagal: %s", e)
            return False

    def set_servo(self, channel, pwm):
        """Gerakkan servo output (MAV_CMD_DO_SET_SERVO). channel=output (AUX1=9),
        pwm dalam mikrodetik. Dipakai utk drop payload."""
        try:
            return self._cmd(command=MAV_CMD_DO_SET_SERVO,
                             param1=float(channel), param2=float(pwm)).success
        except rospy.ServiceException as e:
            rospy.logerr("[mavros_helper] set_servo gagal: %s", e)
            return False

    def make_setpoint(self, x, y, z, yaw=None):
        """x,y ENU-local seperti biasa. z MASUK sebagai AGL (samakan sama
        get_xyz()[2]) -- dikonversi ke ENU-local di sini SEBELUM dipublish,
        krn /mavros/setpoint_position/local cuma ngerti ENU-local (MAV_FRAME_
        LOCAL_NED). Offset origin<->AGL dihitung ulang dari nilai live tiap
        panggilan (self._local_z - self.rel_alt), jadi otomatis ngikutin
        kalau origin geser -- gak nyimpen angka basi."""
        z_enu = z + (self._local_z - self.rel_alt)
        sp = PoseStamped()
        sp.header.stamp = rospy.Time.now()
        sp.header.frame_id = "map"
        sp.pose.position.x = x
        sp.pose.position.y = y
        sp.pose.position.z = z_enu
        if yaw is None:
            yaw = self.get_yaw()   # tahan heading SEKARANG -- bukan identitas (0 ENU/90 NED)
        sp.pose.orientation.z = math.sin(yaw / 2.0)
        sp.pose.orientation.w = math.cos(yaw / 2.0)
        return sp

    def send_setpoint(self, x, y, z, yaw=None):
        self.sp_pub.publish(self.make_setpoint(x, y, z, yaw))

    def reached(self, x, y, z, tol=0.3):
        cx, cy, cz = self.get_xyz()
        d = math.sqrt((cx - x) ** 2 + (cy - y) ** 2 + (cz - z) ** 2)
        return d < tol


def demo():
    # self-check matematika tanpa ROS (yaw->quaternion & reached)
    import sys
    import types
    sys.modules[__name__].rospy = types.SimpleNamespace(
        Time=types.SimpleNamespace(now=lambda: types.SimpleNamespace(to_sec=lambda: 0.0)))
    h = MavrosHelper.__new__(MavrosHelper)
    h._local_z = 0.0
    h.rel_alt = 0.0
    sp0 = h.make_setpoint(1, 2, 3, yaw=0.0)
    assert abs(sp0.pose.orientation.w - 1.0) < 1e-9 and abs(sp0.pose.orientation.z) < 1e-9
    sp1 = h.make_setpoint(0, 0, 0, yaw=math.pi)
    assert abs(sp1.pose.orientation.z - 1.0) < 1e-6 and abs(sp1.pose.orientation.w) < 1e-6
    h.pose = PoseStamped()
    # 14 Sep: konversi AGL<->ENU-local. Fabrikasi offset origin 0.91m (persis
    # kasus insiden home_position.z nyata) -- local_z=1.91, rel_alt=1.0 (AGL).
    h._local_z = 1.91
    h.rel_alt = 1.0
    h._have_rel_alt = True
    sp = h.make_setpoint(1.2, 0.0, 1.0, yaw=0.0)  # minta AGL 1.0 (= AGL sekarang)
    assert abs(sp.pose.position.z - 1.91) < 1e-9, "make_setpoint gagal balik ke ENU-local"
    h.pose.pose.position.z = 1.91
    assert abs(h.get_xyz()[2] - 1.0) < 1e-9, "get_xyz()[2] harus ngikut rel_alt, bukan local_z"
    assert h.reached(0.1, 0.1, 1.0, tol=0.3) is True   # z target AGL, cocok sama rel_alt
    assert h.reached(1.0, 0.0, 1.0, tol=0.3) is False  # xy jauh
    # get_yaw(): round-trip lewat quaternion yg sama dgn make_setpoint()
    for test_yaw in (0.0, math.pi / 2, math.pi, -math.pi / 2, 2.5):
        h.pose.pose.orientation = h.make_setpoint(0, 0, 0, yaw=test_yaw).pose.orientation
        got = h.get_yaw()
        # normalisasi beda sudut ke [-pi, pi] biar wrap-around (mis. pi vs -pi) gak keanggep gagal
        diff = math.atan2(math.sin(got - test_yaw), math.cos(got - test_yaw))
        assert abs(diff) < 1e-6, f"get_yaw round-trip gagal: yaw={test_yaw} got={got}"
    print("mavros_helper demo OK")


if __name__ == "__main__":
    demo()

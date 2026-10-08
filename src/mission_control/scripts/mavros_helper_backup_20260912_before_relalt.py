#!/usr/bin/env python3
"""
Helper koneksi mavros: state, posisi lokal, set mode, arm, takeoff, kirim setpoint.
Dipakai bersama oleh mission_node.py dan record_waypoint.py.

Frame setpoint: /mavros/setpoint_position/local = ENU lokal (X=Timur, Y=Utara, Z=Atas),
relatif terhadap EKF origin (= titik takeoff). Sama dengan frame yang dipakai T265.
"""
import math
import rospy
from geometry_msgs.msg import PoseStamped
from mavros_msgs.msg import State
from mavros_msgs.srv import CommandBool, SetMode, CommandTOL, CommandLong

MAV_CMD_DO_SET_SERVO = 183


class MavrosHelper:
    def __init__(self):
        self.state = State()
        self.pose = PoseStamped()
        self._have_pose = False

        rospy.Subscriber("/mavros/state", State, self._state_cb)
        rospy.Subscriber("/mavros/local_position/pose", PoseStamped, self._pose_cb)
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
        self._have_pose = True

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
        while not rospy.is_shutdown() and not self._have_pose:
            if (rospy.Time.now() - t0).to_sec() > timeout:
                rospy.logerr("[mavros_helper] timeout: belum ada local_position/pose")
                return False
            rate.sleep()
        return True

    def get_xyz(self):
        p = self.pose.pose.position
        return (p.x, p.y, p.z)

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
        sp = PoseStamped()
        sp.header.stamp = rospy.Time.now()
        sp.header.frame_id = "map"
        sp.pose.position.x = x
        sp.pose.position.y = y
        sp.pose.position.z = z
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
    sp0 = h.make_setpoint(1, 2, 3, yaw=0.0)
    assert abs(sp0.pose.orientation.w - 1.0) < 1e-9 and abs(sp0.pose.orientation.z) < 1e-9
    sp1 = h.make_setpoint(0, 0, 0, yaw=math.pi)
    assert abs(sp1.pose.orientation.z - 1.0) < 1e-6 and abs(sp1.pose.orientation.w) < 1e-6
    h.pose = PoseStamped()
    assert h.reached(0.1, 0.1, 0.1, tol=0.3) is True
    assert h.reached(1.0, 0.0, 0.0, tol=0.3) is False
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

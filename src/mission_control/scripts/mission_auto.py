#!/usr/bin/env python3
"""
Generator + uploader misi AUTO. Navigasi-agnostic (GPS+OptFlow sekarang, dulu
T265 non-GPS) -- script ini cuma peduli waypoints.yaml, gak peduli sumber EKF.

Baca waypoints.yaml (ENU lokal hasil survei record_waypoint.py) + route AUTO
(mission_auto.yaml), konversi ENU->lat/lon relatif EKF origin, lalu PUSH ke FC
sebagai daftar misi ArduCopter. Setelah ini kamu tinggal:
  takeoff (Stabilize/AltHold) -> naik aman -> switch ke AUTO -> FC terbang sendiri.

PENTING (beda dgn mission_d.py yg GUIDED+ROS):
- AUTO dieksekusi FC SENDIRI. FC TIDAK tahu ArUco. Jadi hanya aksi geometris:
  waypoint (lewati titik), drop BUTA (DO_SET_SERVO di koordinat survei), land.
  Untuk scan/align berbasis kamera -> pakai mission_d.py (GUIDED), bukan ini.
- Origin (~lat/~lon) HARUS sama dgn origin EKF SESI INI. 14 Sep 2026: kalau
  navigasi GPS, origin di-set OTOMATIS tiap boot begitu GPS fix (BUKAN lagi
  angka fix dari set_origin_home, itu T265-only). Kalau beda dari origin sesi
  ini, seluruh peta bergeser.
- 14 Sep 2026 (v2): mode=push SEKARANG ambil origin OTOMATIS dari live
  /mavros/global_position/gp_origin (~use_live_origin, default True) --
  gak perlu lagi rostopic echo manual + ketik ulang ~lat/~lon. Timeout/gagal
  fetch -> fallback ke ~lat/~lon (dipertahankan sebagai jalan keluar manual,
  mis. utk T265/testing/FC belum kasih origin).

Pakai:
  # bring-up (mavros + tunggu GPS fix, lih. docs/PANDUAN_TERBANG_AUTO.md), lalu:
  rosrun mission_control mission_auto.py            # pakai default config
  roslaunch mission_control phaseD_auto.launch      # via launch (disarankan)

Konvensi waypoints.yaml: x=Timur(E), y=Utara(N) relatif origin EKF;
z=AGL asli (/mavros/global_position/rel_alt, 14 Sep 2026 -- bukan lagi
relatif origin/takeoff, lih. record_waypoint.py). Upload pakai frame=
GLOBAL_REL_ALT (home-relative) jadi konsisten sama datum z ini.
Konversi: dLat = dN/111320 ; dLon = dE/(111320*cos(lat)).
"""
import os
import sys
import math
import yaml
import rospy
from geographic_msgs.msg import GeoPointStamped
from mavros_msgs.msg import Waypoint, WaypointList
from mavros_msgs.srv import WaypointPush, WaypointClear, WaypointPull

# MAV_CMD
NAV_WAYPOINT = 16
NAV_TAKEOFF = 22
NAV_LAND = 21
DO_SET_SERVO = 183

# meter per derajat lintang (aproksimasi bola, cukup utk skala lapangan <ratusan m)
M_PER_DEG_LAT = 111320.0

DEFAULT_WP = os.path.expanduser(
    "~/catkin_ws/src/mission_control/config/waypoints.yaml")
DEFAULT_ROUTE = os.path.expanduser(
    "~/catkin_ws/src/mission_control/config/mission_auto.yaml")

GP_ORIGIN_TOPIC = "/mavros/global_position/gp_origin"


def _resolve_origin(use_live, fallback_lat, fallback_lon, fetch_fn, timeout=5.0):
    """Origin buat ENU->lat/lon: coba live gp_origin dulu (SESI INI, gak bisa
    basi), fallback ke ~lat/~lon kalau gak bisa/mati (mis. T265/testing/FC
    belum kasih origin). fetch_fn() -> (lat, lon) atau raise -- dipisah dari
    rospy.wait_for_message langsung biar bisa dites offline (lih. demo())."""
    if use_live:
        try:
            lat, lon = fetch_fn()
            return lat, lon, "live gp_origin (%s)" % GP_ORIGIN_TOPIC
        except Exception as e:
            rospy.logwarn("[AUTO] gagal ambil live origin (%s) -- fallback ke "
                          "~lat/~lon param.", e)
    return fallback_lat, fallback_lon, "param ~lat/~lon (fallback)"


class MissionAuto:
    def __init__(self):
        self.wp_file = rospy.get_param("~waypoints_file", DEFAULT_WP)
        self.route_file = rospy.get_param("~route_file", DEFAULT_ROUTE)
        # ~lat/~lon = fallback manual kalau live origin gak kepakai/gak ada.
        fallback_lat = float(rospy.get_param("~lat", -7.0501108))
        fallback_lon = float(rospy.get_param("~lon", 110.3913388))
        self.use_live_origin = bool(rospy.get_param("~use_live_origin", True))
        self.takeoff_alt = float(rospy.get_param("~takeoff_alt", 2.0))
        self.servo_channel = int(rospy.get_param("~servo_channel", 8))
        self.servo_open = int(rospy.get_param("~servo_open", 1013))
        self.clear_first = bool(rospy.get_param("~clear_first", True))
        # mode: preview = offline (cek YAML+konversi, TANPA FC/mavros);
        #       push    = upload sungguhan ke FC + readback konfirmasi (disarmed, aman).
        self.mode = str(rospy.get_param("~mode", "push")).lower()

        self.waypoints = self._load(self.wp_file)
        self.route = self._load(self.route_file).get("legs", [])

        if self.mode == "preview":
            # preview TETAP offline murni -- live origin butuh mavros, jadi
            # pakai fallback param aja, TIDAK coba fetch (jangan nyentuh FC).
            self.lat0, self.lon0 = fallback_lat, fallback_lon
            rospy.loginfo("[AUTO] mode=preview (OFFLINE) — tak menyentuh FC/mavros. "
                          "Origin pakai ~lat/~lon param (%.7f, %.7f).",
                          self.lat0, self.lon0)
            return
        rospy.wait_for_service("/mavros/mission/push")
        self._push = rospy.ServiceProxy("/mavros/mission/push", WaypointPush)
        if self.clear_first:
            rospy.wait_for_service("/mavros/mission/clear")
            self._clear = rospy.ServiceProxy("/mavros/mission/clear", WaypointClear)

        def _fetch_gp_origin():
            msg = rospy.wait_for_message(GP_ORIGIN_TOPIC, GeoPointStamped, timeout=5.0)
            return msg.position.latitude, msg.position.longitude

        self.lat0, self.lon0, origin_src = _resolve_origin(
            self.use_live_origin, fallback_lat, fallback_lon, _fetch_gp_origin)
        rospy.loginfo("[AUTO] origin dari %s: lat=%.7f lon=%.7f",
                      origin_src, self.lat0, self.lon0)

    def _load(self, path):
        with open(os.path.expanduser(path)) as f:
            return yaml.safe_load(f) or {}

    def _enu_to_latlon(self, x_e, y_n):
        """ENU offset (m) dari origin -> (lat, lon) absolut."""
        dlat = y_n / M_PER_DEG_LAT
        dlon = x_e / (M_PER_DEG_LAT * math.cos(math.radians(self.lat0)))
        return (self.lat0 + dlat, self.lon0 + dlon)

    def _wp(self, cmd, lat=0.0, lon=0.0, alt=0.0,
            p1=0.0, p2=0.0, p3=0.0, p4=0.0, current=False, frame=3):
        """frame=3 = GLOBAL_REL_ALT (altitude relatif home/takeoff)."""
        w = Waypoint()
        w.frame = frame
        w.command = cmd
        w.is_current = current
        w.autocontinue = True
        w.param1, w.param2, w.param3, w.param4 = p1, p2, p3, p4
        w.x_lat, w.y_long, w.z_alt = lat, lon, alt
        return w

    def build(self):
        """Rakit daftar Waypoint dari route. Item 0 = home (konvensi ArduPilot)."""
        wps = []
        # seq 0: home (ArduPilot pakai ini sbg home; isi origin, is_current)
        wps.append(self._wp(NAV_WAYPOINT, self.lat0, self.lon0, 0.0, current=True))
        # seq 1: takeoff
        wps.append(self._wp(NAV_TAKEOFF, self.lat0, self.lon0, self.takeoff_alt))

        for leg in self.route:
            name = leg["wp"]
            act = leg.get("action", "waypoint")
            w = self.waypoints[name]
            lat, lon = self._enu_to_latlon(float(w["x"]), float(w["y"]))
            alt = float(w["z"])
            if act == "land":
                wps.append(self._wp(NAV_LAND, lat, lon, 0.0))
            else:
                hold = float(leg.get("hold", 0.0))
                wps.append(self._wp(NAV_WAYPOINT, lat, lon, alt, p1=hold))
                if act == "drop":
                    # drop BUTA di koordinat survei (bukan align kamera!)
                    wps.append(self._wp(DO_SET_SERVO, 0, 0, 0,
                                        p1=self.servo_channel, p2=self.servo_open,
                                        frame=2))  # FRAME_MISSION utk DO_*
            rospy.loginfo("[AUTO] %s (%s) -> lat=%.7f lon=%.7f alt=%.1f",
                          name, act, lat, lon, alt)
        return wps

    def _readback(self):
        """Pull misi dari FC & tampilkan — konfirmasi apa yg BENAR2 tersimpan."""
        try:
            rospy.wait_for_service("/mavros/mission/pull", timeout=5.0)
            pull = rospy.ServiceProxy("/mavros/mission/pull", WaypointPull)
            n = pull().wp_received
            rospy.loginfo("[AUTO] readback: FC lapor %d item.", n)
            wl = rospy.wait_for_message("/mavros/mission/waypoints",
                                        WaypointList, timeout=5.0)
            rospy.loginfo("[AUTO] --- isi misi di FC (%d item) ---", len(wl.waypoints))
            for i, w in enumerate(wl.waypoints):
                cur = " <-current" if w.is_current else ""
                rospy.loginfo("[AUTO]  [%d] cmd=%d frame=%d lat=%.7f lon=%.7f alt=%.1f%s",
                              i, w.command, w.frame, w.x_lat, w.y_long, w.z_alt, cur)
        except rospy.ROSException as e:
            rospy.logwarn("[AUTO] readback gagal (%s) — push mungkin tetap sukses.", e)

    def run(self):
        wps = self.build()

        if self.mode == "preview":
            rospy.loginfo("[AUTO] === PREVIEW: %d item (TIDAK dikirim ke FC) ===",
                          len(wps))
            rospy.loginfo("[AUTO] Jalankan dgn mode:=push (mavros hidup) utk upload.")
            return True

        if self.clear_first:
            self._clear()
            rospy.loginfo("[AUTO] misi lama di-clear")
        res = self._push(start_index=0, waypoints=wps)
        if res.success:
            rospy.loginfo("[AUTO] PUSH sukses: %d item terkirim ke FC.",
                          res.wp_transfered)
            self._readback()
            rospy.loginfo("[AUTO] DRY-RUN aman (disarmed): verifikasi item di atas "
                          "cocok. Utk terbang: takeoff Stabilize -> hover cek -> "
                          "switch AUTO. Jari di Stabilize.")
        else:
            rospy.logerr("[AUTO] PUSH GAGAL. Cek koneksi/route.")
        return res.success


def _selftest_resolve_origin():
    """Offline, no ROS master: cek _resolve_origin() -- live origin dipakai
    kalau fetch_fn sukses, fallback ke param kalau fetch_fn raise/gak dipakai."""
    ok_fetch = lambda: (1.111, 2.222)
    bad_fetch = lambda: (_ for _ in ()).throw(RuntimeError("timeout simulasi"))

    lat, lon, src = _resolve_origin(True, 9.0, 9.0, ok_fetch)
    assert (lat, lon) == (1.111, 2.222) and "live" in src, "harus pakai live origin"

    lat, lon, src = _resolve_origin(True, 9.0, 9.0, bad_fetch)
    assert (lat, lon) == (9.0, 9.0) and "fallback" in src, "harus fallback pas fetch gagal"

    lat, lon, src = _resolve_origin(False, 9.0, 9.0, ok_fetch)
    assert (lat, lon) == (9.0, 9.0), "use_live=False harus tetap pakai fallback"

    print("mission_auto _resolve_origin selftest: PASS")


def main():
    if "--selftest" in sys.argv:
        _selftest_resolve_origin()
        return
    rospy.init_node("mission_auto")
    ok = MissionAuto().run()
    sys.exit(0 if ok else 1)


if __name__ == "__main__":
    try:
        main()
    except rospy.ROSInterruptException:
        pass

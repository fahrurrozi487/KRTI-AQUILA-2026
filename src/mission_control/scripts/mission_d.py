#!/usr/bin/env python3
"""
FASE D: state machine misi. Menyatukan Fase B (takeoff/goto) +
detektor ArUco (scan / align-drop) + detektor gate (traverse).

⚠️ BELUM DIUJI SITL — JANGAN LANGSUNG TERBANG. File ini mengontrol
setpoint/motor drone sungguhan. Jalur abort/LAND dan traverse multi-lapis
(gate_layers>1) belum diverifikasi di SITL. Uji penuh di SITL dulu; saat uji
lapangan, pilot WAJIB siaga ambil alih (Stabilize).

Alur: WAIT -> TAKEOFF(opsional) -> [tiap leg: GOTO -> aksi] -> DONE
  aksi: scan | drop | traverse | yaw | land | line_follow | goto
    goto (2026-09-16): no-op eksplisit -- WP ini cuma dilewati (goto pre-leg
    di atas sudah cukup), tanpa scan/traverse/dsb. Dipakai mis. utk
    gate_single_final saat mau skip deteksi gate (goto ke koordinatnya saja).

Konsumsi:
  /aruco_node/markers (mission_control/ArucoMarkers)
  /gate_node/gate    (mission_control/Gate)  — utk traverse
  /line_node/line    (mission_control/Line)  — utk line_follow
Kontrol : mavros setpoint_position/local + servo (DO_SET_SERVO).

Param utama (lihat phaseD_mission.launch):
  ~waypoints_file, ~mission_file
  ~do_takeoff (bool), ~takeoff_alt
  ~reach_tol, ~scan_timeout, ~scan_frames, ~scan_leak
  ~align_tol, ~align_gain, ~align_timeout, ~align_frames, ~align_sign_x/y, ~align_swap_xy
  ~gate_topic (/gate_node/gate, Single Gate), ~gate_topic2 (/gate_multi_node/gate,
    Double/Triple Gate bentuk-U) -- DUA detektor gate BERBEDA topologi fisik,
    lih. leg field gate_source di bawah. ~gate_tol, ~gate_gain, ~gate_timeout, ~gate_frames
  ~gate_sign_x/y/z, ~gate_swap_xy, ~traverse_through_m
  ~gate_layer_timeout, ~gate_area_drop_ratio  (utk gate multi-lapis)
  ~line_topic, ~line_gain, ~line_sign_x/y, ~line_swap_xy, ~line_max_drift
    (utk line_follow -- lih. docstring _line_follow())
  ~yaw_tol, ~yaw_timeout, ~yaw_frames
  ~yaw_direction_override, ~yaw_deg_override — 2026-09-22: override SEMUA leg
    action:yaw di mission yaml (arah/besar), lih. Field leg opsional di bawah.
  ~servo_channel (8), ~servo_open (1013), ~servo_close (2015)
  ~markers_topic (/aruco_node/markers)
  ~wp_marker_result_topic (/wp_marker_node/result) — Task 4: sumber tail_side
  ~enable_heading_align (bool, default False) — Task 4: aktifkan yaw ke arah ekor
    setelah tiap scan/drop. ~tail_dir_rotate_steps, ~tail_mirror = kalibrasi
    mounting kamera (lih. _tail_to_yaw_cmd()). ~tail_result_stale_sec (default
    2.0) = umur maks WpMarkerResult dipakai (guard basi/wp_id salah).

Field leg opsional:
  hold_heading (bool, default False) — 2026-09-15: leg ini terbang MUNDUR (tail
    duluan) ke WP-nya, tahan yaw SEKARANG sepanjang transit alih-alih auto-hadap
    arah tujuan (default _goto()). Dipakai pada leg traverse gate_double/gate_triple
    biar drone gak muter hadap gate duluan sebelum sampai -- centering/fly-through
    di _traverse() yang urus hadap gate (gate_yaw dihitung dari vektor approach WP
    sebelumnya->gate, TIDAK dipengaruhi hold_heading).
  hold_sec (float, default 0) — 2026-09-22: jeda diam (hover) di WP ini SEBELUM
    aksi (SESUDAH _line_follow() utk leg line_follow, krn goto-nya baru terjadi
    di situ). scan/drop sudah dapat dwell alami dari scan_timeout/align_timeout
    -- field ini utk leg yang TIDAK (goto/land/traverse), spy tetap ada jeda yg
    sebanding meski gak pakai vision (mis. gate_single_final/wp5 saat gate/scan
    tak diandalkan).
  yaw_deg, direction (leg action:yaw) — BISA di-override SEMUA sekaligus lewat
    rosparam ~yaw_direction_override/~yaw_deg_override (lih. Param utama di atas)
    tanpa edit yaml, dipakai kalau arena fisik punya >1 layout (arah putar beda tiap
    sesi). Default rosparam kosong/-1 = TIDAK override, nilai leg yaml dipakai.
  gate_layers (int, default 1) — jumlah bidang gate berlapis pd satu leg traverse.
  gate_source ("1"|"2", default "1") — 2026-09-04: pilih detektor traverse per-leg.
    "1" = gate_topic (Single Gate, gate_node.py — tunnel oranye solid, bukaan
    dicari via profil kolom). "2" = gate_topic2 (Double/Triple Gate bentuk-U,
    gate_multi_node.py — TIDAK PERNAH divalidasi utk struktur Single Gate,
    ditulis dari nol khusus topologi-U/terbuka-bawah, lih. gate_multi_detector.py).
    Ditambahkan setelah ketahuan mission_final.yaml butuh KEDUANYA dalam satu
    run (gate_double/gate_triple = Double/Triple fisik -> "2"; gate_single_final
    = Single Gate fisik -> default "1") -- sebelum ini gate_topic cuma satu,
    di-override penuh via arg use_gate_multi (WAJIB pilih salah satu utk
    seluruh misi), gate_single_final ikut kebaca detektor Double/Triple yg
    salah topologi. Kedua node BISA jalan bersamaan (topic beda, sengaja).

Sinyal transisi antar-lapis (gate_layers>1): SINYAL UTAMA = area_frac naik-lalu-
turun (drone mendekat -> oranye membesar -> tembus -> mengecil). Berpindah lapis
saat gate.detected jadi False ATAU area_frac turun < peak*(1-gate_area_drop_ratio).
Jarak/waktu (gate_layer_timeout) HANYA fallback pengaman, BUKAN sinyal utama.

Konvensi align ArUco (kamera bawah): off_x,off_y = offset marker di gambar (-1..1).
Pemetaan ke ENU: dE = sign_x*off(x|y), dN = sign_y*off(y|x) (swap & sign utk
sesuai mounting -> DIKALIBRASI di lapangan, mirip walk test).

Konvensi traverse gate (kamera DEPAN): off_x = kiri/kanan bukaan, off_y = atas/bawah.
Center dulu, lalu fly-through sepanjang vektor approach (WP sebelumnya -> WP gate).

Konvensi line_follow (kamera bawah, garis putus-putus): SAMA pola align_sign_x/y
ArUco (off_x,off_y dari mission_control/Line). Beda dari scan/drop/traverse:
line_follow TIDAK berhenti/nunggu "sudah center" -- ini goto() ke WP target
(mis. WP5) yang dinudge lateral kalau dash kedeteksi (mission_control/Line
detected=True). GAP (Line.detected=False, di antara dash) BUKAN kegagalan --
goto tetap lanjut ke WP target pakai posisi EKF apa adanya (dead-reckon),
persis pola fallback "align timeout -> drop apa adanya"/"gate TAK TERLIHAT ->
through geometris WP" yang sudah ada. WAJIB WP target (mis. WP5) sudah
disurvei/valid -- line_follow bukan pengganti survei, cuma koreksi visual.
"""
import os
import sys
import math
sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))

import yaml
import rospy
from std_msgs.msg import Int32, String
from mission_control.msg import ArucoMarkers, Gate, WpMarkerResult, Line
from mavros_helper import MavrosHelper


class MissionD:
    def __init__(self):
        self.wp_file = rospy.get_param("~waypoints_file")
        self.mission_file = rospy.get_param("~mission_file")
        self.do_takeoff = bool(rospy.get_param("~do_takeoff", True))
        self.takeoff_alt = float(rospy.get_param("~takeoff_alt", 1.2))
        self.reach_tol = float(rospy.get_param("~reach_tol", 0.3))
        self.reach_tol_xy = float(rospy.get_param("~reach_tol_xy", self.reach_tol))
        self.reach_tol_z = float(rospy.get_param("~reach_tol_z", self.reach_tol))
        self.goto_timeout = float(rospy.get_param("~goto_timeout", 60.0))
        self.pose_stale_sec = float(rospy.get_param("~pose_stale_sec", 2.0))
        self.max_recede = float(rospy.get_param("~max_recede", 2.0))
        self.align_max_drift = float(rospy.get_param("~align_max_drift", 1.5))
        self.scan_timeout = float(rospy.get_param("~scan_timeout", 8.0))
        self.scan_frames = int(rospy.get_param("~scan_frames", 5))
        self.scan_leak = int(rospy.get_param("~scan_leak", 1))  # lih. _scan()
        self.align_tol = float(rospy.get_param("~align_tol", 0.08))
        self.align_gain = float(rospy.get_param("~align_gain", 1.0))
        self.align_timeout = float(rospy.get_param("~align_timeout", 2.0))
        self.align_frames = int(rospy.get_param("~align_frames", 8))
        self.align_sign_x = float(rospy.get_param("~align_sign_x", 1.0))
        self.align_sign_y = float(rospy.get_param("~align_sign_y", 1.0))
        self.align_swap_xy = bool(rospy.get_param("~align_swap_xy", False))
        # marker kecil (pelengkap): toleransi align lebih ketat dari marker besar.
        # dipakai HANYA sbg tahap-2 opsional; marker besar tetap penentu utama.
        # ponytail: DEPRECATED (Task 4) — expected_id_small/align lateral tahap-2 di
        # _scan()/_align_and_drop() salah sasaran (marker "kecil" itu sebenarnya ekor,
        # butuh HEADING bukan offset lateral). Kode dibiarkan ada utk referensi historis
        # tapi TIDAK dipanggil aktif oleh mission yaml manapun. Ganti: _align_heading_to_tail().
        self.small_marker_align_tol = float(rospy.get_param("~small_marker_align_tol", 0.04))
        # Task 4: heading align ke arah ekor marker WP (ganti mekanisme lateral di atas).
        # Default OFF -> harus di-set True eksplisit di mission yaml/launch yang mau pakai.
        self.enable_heading_align = bool(rospy.get_param("~enable_heading_align", False))
        # kalibrasi lapangan (mirip align_sign_x/y): mounting kamera bawah menentukan
        # bagaimana label N/E/S/W (frame GAMBAR) berpadanan dgn kiri/kanan drone.
        # BELUM diverifikasi fisik -> sesuaikan saat uji lapangan (mirip walk test).
        self.tail_dir_rotate_steps = int(rospy.get_param("~tail_dir_rotate_steps", 0)) % 4
        self.tail_mirror = bool(rospy.get_param("~tail_mirror", False))
        # guard: WpMarkerResult dipakai HANYA kalau segar & dari wp_id yang sedang di-scan/drop
        self.tail_result_stale_sec = float(rospy.get_param("~tail_result_stale_sec", 2.0))
        self.servo_channel = int(rospy.get_param("~servo_channel", 8))
        self.servo_open = int(rospy.get_param("~servo_open", 1013))
        self.servo_close = int(rospy.get_param("~servo_close", 2015))
        self.yaw_tol = float(rospy.get_param("~yaw_tol", 0.12))          # rad ~7°
        self.yaw_timeout = float(rospy.get_param("~yaw_timeout", 15.0))
        self.yaw_frames = int(rospy.get_param("~yaw_frames", 8))
        # 2026-09-22: override arah/besar SEMUA leg action:yaw di mission yaml,
        # diisi dari launch arg -- dipakai kalau arena fisik punya >1 layout
        # (mis. yaw WP2->WP3 kadang perlu kanan, kadang kiri) tanpa edit yaml
        # tiap sesi. Default kosong/-1 = TIDAK override (pakai nilai leg yaml).
        self.yaw_direction_override = str(rospy.get_param("~yaw_direction_override", "")).strip()
        self.yaw_deg_override = float(rospy.get_param("~yaw_deg_override", -1.0))
        markers_topic = rospy.get_param("~markers_topic", "/aruco_node/markers")
        # Task 4: tail_side (ekor) utk heading align — datang dari wp_marker_node,
        # topic terpisah dari markers_topic (yang dipakai utk off_x/off_y align lateral).
        wp_marker_result_topic = rospy.get_param("~wp_marker_result_topic",
                                                  "/wp_marker_node/result")
        # 2026-08-08: kasih tau wp_marker_node kapan lagi aktif nyari WP tertentu
        # (_scan()) -- gate buat CLAHE fallback yg mahal CPU, lih. wp_marker_node.py.
        expected_id_topic = rospy.get_param("~wp_marker_expected_id_topic",
                                             "/wp_marker_node/expected_id")
        # gate traverse (kamera depan)
        gate_topic = rospy.get_param("~gate_topic", "/gate_node/gate")
        # 2026-09-04: sumber KEDUA, topologi fisik beda total dari gate_topic
        # (lih. docstring field leg gate_source) -- dipilih per-leg, BUKAN
        # exclusive-pilih-satu-buat-seluruh-misi kayak sebelumnya.
        gate_topic2 = rospy.get_param("~gate_topic2", "/gate_multi_node/gate")
        self.gate_tol = float(rospy.get_param("~gate_tol", 0.12))
        self.gate_gain = float(rospy.get_param("~gate_gain", 1.0))
        self.gate_timeout = float(rospy.get_param("~gate_timeout", 20.0))
        self.gate_frames = int(rospy.get_param("~gate_frames", 5))
        self.gate_sign_x = float(rospy.get_param("~gate_sign_x", 1.0))
        self.gate_sign_y = float(rospy.get_param("~gate_sign_y", 1.0))
        self.gate_sign_z = float(rospy.get_param("~gate_sign_z", -1.0))  # off_y+ = bawah gambar -> turun
        self.gate_swap_xy = bool(rospy.get_param("~gate_swap_xy", False))
        self.traverse_through_m = float(rospy.get_param("~traverse_through_m", 1.5))
        self.gate_max_drift = float(rospy.get_param("~gate_max_drift", 1.5))
        # gate multi-lapis: transisi antar-lapis via area_frac naik-lalu-turun
        self.gate_layer_timeout = float(rospy.get_param("~gate_layer_timeout", 10.0))
        self.gate_area_drop_ratio = float(rospy.get_param("~gate_area_drop_ratio", 0.4))
        # line_follow (kamera bawah, garis putus-putus WP3->WP4, dikonfirmasi
        # panitia 2026-09-03): pola SAMA align_sign_x/y/swap_xy/gain/max_drift
        # ArUco -- reuse, bukan reinvent. Dipakai di mission_final.yaml.
        line_topic = rospy.get_param("~line_topic", "/line_node/line")
        self.line_gain = float(rospy.get_param("~line_gain", 1.0))
        self.line_sign_x = float(rospy.get_param("~line_sign_x", 1.0))
        self.line_sign_y = float(rospy.get_param("~line_sign_y", 1.0))
        self.line_swap_xy = bool(rospy.get_param("~line_swap_xy", False))
        self.line_max_drift = float(rospy.get_param("~line_max_drift", 1.5))
        # retry dari WP terdekat (2026-09-03): kalau drone crash/keluar kendali,
        # gak perlu ulang dari awal -- ~start_wp="wp3" motong self.legs ke leg
        # PERTAMA yang wp-nya cocok, sisanya (leg sebelum itu) di-skip total.
        # Default kosong = perilaku lama (mulai dari leg 0), gak ada regresi.
        self.start_wp = str(rospy.get_param("~start_wp", "")).strip()

        self.waypoints = self._load_waypoints(self.wp_file)
        self.legs = self._load_mission(self.mission_file)
        if self.start_wp:
            idx = self._find_leg_index(self.legs, self.start_wp)
            if idx is None:
                rospy.logerr("[D] start_wp=%s TIDAK KETEMU di %d leg mission_file -- "
                             "cek ejaan/nama WP. Lanjut dari leg 0 (fallback aman).",
                             self.start_wp, len(self.legs))
            else:
                skipped = idx
                self.legs = self.legs[idx:]
                rospy.logwarn("[D] RESTART dari WP '%s' (leg %d) -- %d leg sebelumnya "
                              "DI-SKIP (retry/recovery, bukan misi penuh)",
                              self.start_wp, idx, skipped)

        self._markers = None          # ArucoMarkers terbaru
        self._gate = None             # Gate terbaru (sumber "1", Single Gate)
        self._gate2 = None            # Gate terbaru (sumber "2", Double/Triple Gate)
        self._wp_marker_result = None  # WpMarkerResult terbaru (Task 4: tail_side)
        self._line = None             # Line terbaru (line_follow)
        self._pilot_override = False  # True kalau mode != GUIDED terdeteksi, lih. _healthy()
        rospy.Subscriber(markers_topic, ArucoMarkers, self._markers_cb)
        rospy.Subscriber(gate_topic, Gate, self._gate_cb)
        rospy.Subscriber(gate_topic2, Gate, self._gate2_cb)
        rospy.Subscriber(wp_marker_result_topic, WpMarkerResult, self._wp_marker_result_cb)
        rospy.Subscriber(line_topic, Line, self._line_cb)
        self.pub_expected_id = rospy.Publisher(expected_id_topic, Int32, queue_size=1, latch=True)
        self.pub_expected_id.publish(Int32(data=-1))  # idle di awal
        self.h = MavrosHelper()
        self.rate = rospy.Rate(20)
        # status leg sekarang, buat konsumen luar (mis. web_dashboard.py) --
        # latched biar subscriber yg connect belakangan tetap dapet nilai terakhir.
        self.pub_status = rospy.Publisher("~status", String, queue_size=1, latch=True)
        self.pub_status.publish(String(data="WAIT"))

    # ---------- config ----------
    @staticmethod
    def _find_leg_index(legs, wp_name):
        """Index leg PERTAMA yang wp-nya == wp_name, atau None. Static (tanpa
        self) biar bisa dites offline dari _selftest_start_wp()."""
        return next((i for i, leg in enumerate(legs) if leg.get("wp") == wp_name), None)

    def _load_waypoints(self, path):
        with open(os.path.expanduser(path)) as f:
            d = yaml.safe_load(f) or {}
        rospy.loginfo("[D] %d waypoint dimuat dari %s", len(d), path)
        return d

    def _load_mission(self, path):
        with open(os.path.expanduser(path)) as f:
            d = yaml.safe_load(f) or {}
        legs = d.get("legs", [])
        rospy.loginfo("[D] %d leg dimuat dari %s", len(legs), path)
        return legs

    def _markers_cb(self, msg):
        self._markers = msg

    def _gate_cb(self, msg):
        self._gate = msg

    def _gate2_cb(self, msg):
        self._gate2 = msg

    def _gate_live(self, gate_source):
        """Pilih sumber Gate live sesuai leg field gate_source ("1"/"2")."""
        return self._gate2 if str(gate_source) == "2" else self._gate

    def _line_cb(self, msg):
        self._line = msg

    def _wp_marker_result_cb(self, msg):
        self._wp_marker_result = msg

    def _find_marker(self, marker_id):
        """Return (off_x, off_y) marker dgn id ini dari frame terbaru, atau None."""
        m = self._markers
        if m is None:
            return None
        for mk in m.markers:
            if mk.id == marker_id:
                return (mk.off_x, mk.off_y)
        return None

    def _wp_xyz(self, name):
        w = self.waypoints[name]
        return (float(w["x"]), float(w["y"]), float(w["z"]))

    def _get_yaw(self):
        """Yaw ENU (rad) dari quaternion local_position. +CCW dari atas (Z up)."""
        q = self.h.pose.pose.orientation
        # yaw = atan2(2(wz+xy), 1-2(y^2+z^2))
        siny = 2.0 * (q.w * q.z + q.x * q.y)
        cosy = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
        return math.atan2(siny, cosy)

    @staticmethod
    def _angle_diff(a, b):
        """a-b dinormalisasi ke [-pi, pi]."""
        d = a - b
        while d > math.pi:
            d -= 2.0 * math.pi
        while d < -math.pi:
            d += 2.0 * math.pi
        return d

    # ---------- health ----------
    def _pose_age(self):
        """Umur feedback posisi (pose lokal x/y) DAN AGL (rel_alt, z) terbaru --
        ambil yg PALING basi (max). Stale = EKF/mavros/GPS berhenti update salah
        satu. 14 Sep: dulu cuma cek pose lokal; z sekarang dari topic terpisah
        (rel_alt, lih. mavros_helper.py) jadi harus ikut dicek di sini juga --
        1 tempat, tetap dipanggil dari _healthy() yg sudah berlaku ke semuanya."""
        return max(self.h.pose_age(), self.h.agl_age())

    def _healthy(self):
        """Cek flight-critical: mode GUIDED + armed + pose segar. Return False + log
        kalau bermasalah. Dipanggil di SETIAP loop tick di semua primitive (goto/
        scan/align/traverse/line_follow/yaw) -- 1 tempat, berlaku ke semuanya.

        2026-09-04: cek mode ditambahkan PALING AWAL (sebelumnya gak ada sama
        sekali -- kalau pilot switch RC keluar GUIDED buat ambil alih manual,
        mission_d dulu BARU berhenti lewat timeout per-leg, bisa sampai puluhan
        detik). self._pilot_override dipakai _abort() supaya kasus ini TIDAK
        maksa set_mode("LAND") -- pilot udah pegang kendali, jangan dilawan."""
        self._pilot_override = self.h.state.mode != "GUIDED"
        if self._pilot_override:
            rospy.logerr("[D] STOP misi: mode berubah jadi %s (pilot ambil alih?) -- "
                         "mission_d BERHENTI, TIDAK maksa mode", self.h.state.mode)
            return False
        if not self.h.state.armed:
            rospy.logerr("[D] ABORT: drone disarm di tengah misi")
            return False
        age = self._pose_age()
        if age > self.pose_stale_sec:
            rospy.logerr("[D] ABORT: pose stale %.1fs (>EKF/vision drop?)", age)
            return False
        return True

    def _reached(self, x, y, z):
        """Toleransi XY dan Z terpisah (vision outdoor Z lebih noisy)."""
        cx, cy, cz = self.h.get_xyz()
        return (math.hypot(cx - x, cy - y) < self.reach_tol_xy
                and abs(cz - z) < self.reach_tol_z)

    def _abort(self, reason):
        """Abort: log + coba LAND (turun aman). Pilot wajib ambil alih Stabilize.

        2026-09-04: kalau penyebabnya pilot UDAH ambil alih (mode != GUIDED,
        lih. _healthy()), JANGAN set_mode("LAND") -- itu bakal melawan pilot
        yang sengaja pindah mode buat kendali manual. Cukup berhenti diam-diam."""
        # TODO: (1) uji SITL penuh utk verifikasi abort jalan; (2) LAND butuh EKF,
        # saat EKF sangat bad LAND bisa gak stabil -> pertimbang RTL atau pilot-only.
        if getattr(self, "_pilot_override", False):
            rospy.logerr("[D] ABORT misi: %s -- TIDAK maksa mode (pilot sudah pegang kendali)", reason)
            return
        rospy.logerr("[D] ABORT misi: %s -> LAND", reason)
        self.h.set_mode("LAND")

    # ---------- primitives ----------
    def _takeoff_seq(self):
        rospy.loginfo("[D] GUIDED + arm + takeoff %.1fm", self.takeoff_alt)
        while not rospy.is_shutdown() and self.h.state.mode != "GUIDED":
            self.h.set_mode("GUIDED"); self.rate.sleep()
        while not rospy.is_shutdown() and not self.h.state.armed:
            self.h.arm(True); rospy.sleep(0.5)
        # JANGAN stream setpoint selama takeoff (ArduCopter GUIDED, lihat Fase B)
        self.h.takeoff(self.takeoff_alt)
        t0 = rospy.Time.now()
        while not rospy.is_shutdown():
            if not self.h.state.armed:
                rospy.logerr("[D] disarm saat takeoff!"); return False
            if self.h.get_xyz()[2] > self.takeoff_alt - 0.3:
                break
            if (rospy.Time.now() - t0).to_sec() > 30:
                rospy.logwarn("[D] takeoff timeout"); break
            self.rate.sleep()
        rospy.loginfo("[D] takeoff selesai (z=%.2f)", self.h.get_xyz()[2])
        return True

    def _body_to_enu(self, bx, by):
        """Rotasi koreksi align/gate dari BODY frame (hasil sign_x/sign_y*off_x/off_y --
        pemetaan tetap thd mounting kamera, TIDAK berubah krn arah hadap drone) ke
        ENU frame (dE, dN) pakai yaw SEKARANG.

        KENAPA (bug ditemukan 2026-08-10): sebelum ini, (bx,by) LANGSUNG dipakai
        sbg (dE,dN) tanpa rotasi -- cuma benar kalau yaw drone PERSIS sama kayak
        pas kalibrasi_align_sign.py dites di darat. Misi PUNYA leg yaw 90 derajat
        (antara WP2 & gate_triple) -- gate_double (sebelum belok) & gate_triple
        (sesudah belok) beda heading, jadi tanpa rotasi ini salah satunya PASTI
        salah arah (positive-feedback runaway) walau sign sudah dikalibrasi benar
        di 1 heading. Konvensi yaw SAMA kayak MavrosHelper.make_setpoint()
        (rotasi Z murni, 0=+X/Timur, naik CCW ke +Y/Utara -- REP-103/ENU)."""
        yaw = self.h.get_yaw()
        c, s = math.cos(yaw), math.sin(yaw)
        de = bx * c - by * s
        dn = bx * s + by * c
        return de, dn

    def _face_yaw(self, x, y, z, target, label=""):
        """Putar di tempat (tahan x,y,z) sampai yaw ~= target (absolute, ENU rad).
        Dipanggil _goto() SEBELUM translasi kalau yaw dihitung otomatis (bearing
        ke target) -- drone selesai muter dulu, baru maju (bukan bareng).
        ponytail: duplikasi kecil dari _yaw_turn() (target relatif vs absolut) --
        sengaja dipisah biar _yaw_turn() yg sudah tervalidasi terbang gak kesentuh."""
        t0 = rospy.Time.now()
        good = 0
        while not rospy.is_shutdown():
            if not self._healthy():
                self._abort("face yaw %s unhealthy" % label)
                return False
            self.h.send_setpoint(x, y, z, yaw=target)
            err = abs(self._angle_diff(self._get_yaw(), target))
            if err < self.yaw_tol:
                good += 1
                if good >= self.yaw_frames:
                    return True
            else:
                good = 0
            if (rospy.Time.now() - t0).to_sec() > self.yaw_timeout:
                rospy.logwarn("[D]  face yaw %s timeout (err=%.1f deg) -> lanjut",
                              label, math.degrees(err))
                return True
            self.rate.sleep()
        return False

    def _goto(self, x, y, z, label="", yaw=None):
        rospy.loginfo("[D] GOTO %s (%.1f,%.1f,%.1f)", label, x, y, z)
        t0 = rospy.Time.now()
        min_dist = None
        if yaw is None:
            cx0, cy0, cz0 = self.h.get_xyz()
            if math.hypot(x - cx0, y - cy0) > 0.1:
                yaw = math.atan2(y - cy0, x - cx0)   # bearing ENU ke target, dihitung sekali
                if not self._face_yaw(cx0, cy0, cz0, yaw, label):
                    return False
        while not rospy.is_shutdown():
            self.h.send_setpoint(x, y, z, yaw=yaw)
            if not self._healthy():
                self._abort("goto %s unhealthy" % label)
                return False
            cx, cy, cz = self.h.get_xyz()
            d = math.hypot(cx - x, cy - y)
            if self._reached(x, y, z):
                rospy.loginfo("[D]  sampai %s", label); return True
            min_dist = d if min_dist is None else min(min_dist, d)
            if min_dist is not None and d > min_dist + self.max_recede:
                rospy.logerr("[D] ABORT: %s menjauh %.1f>min+%.1f (drift vision?)",
                             label, d, self.max_recede)
                self._abort("goto %s drift" % label)
                return False
            if (rospy.Time.now() - t0).to_sec() > self.goto_timeout:
                rospy.logerr("[D] ABORT: goto %s timeout %.0fs", label, self.goto_timeout)
                self._abort("goto %s timeout" % label)
                return False
            self.rate.sleep()
        return False

    def _scan(self, expected_id, x, y, z, expected_id_small=None):
        """Hover di WP, konfirmasi marker expected_id terlihat beberapa frame.

        expected_id_small (opsional): marker kecil PELENGKAP. Bila diberikan &
        terlihat, cuma di-LOG sbg konfirmasi ganda — TIDAK mengubah sukses/gagal
        scan (marker besar tetap satu-satunya penentu).

        GANTI (2026-08-07): counter hits HYSTERESIS (leaky-bucket) -- naik +1
        per hit (dipatok scan_frames), turun -scan_leak per miss (BUKAN reset
        ke 0 kayak sebelumnya). Kenapa: audit dataset WP nunjukin akurasi
        per-frame (end-to-end incl. foto gagal decode) WP1~37%, WP2~18%,
        WP3~65%, WP4~44% (lih. wp_decode_audit.py --base dataset_arucode).
        Dgn hard-reset, waktu tunggu RATA-RATA sampai 5-hit-beruntun pd
        p=0.37 (WP1) ~225 frame (~45s @5Hz) -- jauh lampaui scan_timeout=8s
        (rumus waktu-tunggu-N-sukses-beruntun klasik) -- scan bakal TIMEOUT
        hampir selalu walau marker beneran ada & kelihatan, krn 1 frame
        meleset buang semua progress. Leaky-bucket = teknik hysteresis/
        debounce standar (turunan sederhana dari Wald Sequential Probability
        Ratio Test, 1945 -- akumulasi bukti dari waktu ke waktu, bukan
        all-or-nothing per sampel): toleran ke noise terselang-seling, tp
        tetap butuh scan_frames NET evidence -- rentetan miss murni tetap
        gagal confirm (safety tak berubah, threshold & timeout tak berubah).
        scan_leak=1 (default): simetris naik/turun, paling konservatif.

        Publish `~expected_id`=expected_id selama scan (gate CLAHE fallback
        mahal-CPU di wp_marker_node.py, lih. docstring modul situ) -- dikembalikan
        ke -1 (idle) di `finally` APAPUN cara keluarnya (sukses/timeout/abort),
        biar CLAHE gak nyala terus-menerus di luar jendela scan.
        """
        rospy.loginfo("[D] SCAN id=%d (hover)", expected_id)
        self.pub_expected_id.publish(Int32(data=int(expected_id)))
        try:
            t0 = rospy.Time.now()
            hits = 0
            while not rospy.is_shutdown():
                self.h.send_setpoint(x, y, z)              # tahan posisi
                if not self._healthy():
                    self._abort("scan id=%d unhealthy" % expected_id)
                    return False
                if self._find_marker(expected_id) is not None:
                    hits = min(self.scan_frames, hits + 1)
                    if hits >= self.scan_frames:
                        # marker besar terkonfirmasi = kondisi sukses (TIDAK berubah).
                        # marker kecil hanya info tambahan bila kebetulan terlihat.
                        if (expected_id_small is not None
                                and self._find_marker(int(expected_id_small)) is not None):
                            rospy.loginfo("[D]  konfirmasi ganda: marker kecil id=%d juga terlihat",
                                          int(expected_id_small))
                        rospy.loginfo("[D]  id=%d TERKONFIRMASI", expected_id)
                        return True
                else:
                    hits = max(0, hits - self.scan_leak)
                if (rospy.Time.now() - t0).to_sec() > self.scan_timeout:
                    rospy.logwarn("[D]  id=%d TIDAK terlihat (timeout) -> lanjut",
                                  expected_id)
                    return False
                self.rate.sleep()
            return False
        finally:
            self.pub_expected_id.publish(Int32(data=-1))

    def _align_and_drop(self, expected_id, x, y, z, expected_id_small=None):
        """Align di atas marker (P-control offset->0) lalu drop servo.

        expected_id_small (opsional): marker kecil PELENGKAP utk presisi ekstra.
        Dijalankan sbg tahap-2 SETELAH align marker besar (logika marker besar
        TIDAK berubah). Kalau marker kecil tak terlihat dlm align_timeout, JANGAN
        blok -> langsung DROP pakai hasil align marker besar (graceful fallback).
        """
        rospy.loginfo("[D] ALIGN id=%d lalu DROP", expected_id)
        tx, ty = x, y
        t0 = rospy.Time.now()
        good = 0
        while not rospy.is_shutdown():
            if not self._healthy():
                self._abort("align id=%d unhealthy" % expected_id)
                return False
            off = self._find_marker(expected_id)
            if off is not None:
                ox, oy = off
                # offset gambar -> koreksi BODY frame (swap/sign utk mounting kamera)
                # -> ROTASI ke ENU pakai yaw sekarang (lih. _body_to_enu())
                bx = self.align_sign_x * (oy if self.align_swap_xy else ox)
                by = self.align_sign_y * (ox if self.align_swap_xy else oy)
                de, dn = self._body_to_enu(bx, by)
                tx += self.align_gain * de * 0.1       # langkah kecil (gain*offset)
                ty += self.align_gain * dn * 0.1
                # clamp: jangan drift jauh dari WP asli (anti runaway vision drift)
                tx = max(x - self.align_max_drift, min(x + self.align_max_drift, tx))
                ty = max(y - self.align_max_drift, min(y + self.align_max_drift, ty))
                if abs(ox) < self.align_tol and abs(oy) < self.align_tol:
                    good += 1
                else:
                    good = 0
                if good >= self.align_frames:
                    rospy.loginfo("[D]  terpusat (off<%.2f) -> DROP", self.align_tol)
                    break
            self.h.send_setpoint(tx, ty, z)
            if (rospy.Time.now() - t0).to_sec() > self.align_timeout:
                rospy.logwarn("[D]  align timeout -> drop apa adanya")
                break
            self.rate.sleep()
        # ---- tahap-2 (opsional): align presisi ke marker kecil ----
        # ponytail: sengaja duplikasi pola P-control tahap-1 (bukan helper) supaya
        # jalur align marker BESAR di atas tetap byte-for-byte tak tersentuh.
        # Fallback: marker kecil tak terlihat -> DROP pakai (tx,ty) marker besar.
        if expected_id_small is not None:
            sid = int(expected_id_small)
            rospy.loginfo("[D] ALIGN tahap-2 marker kecil id=%d (tol=%.3f)",
                          sid, self.small_marker_align_tol)
            t0 = rospy.Time.now()
            good = 0
            saw_small = False
            while not rospy.is_shutdown():
                if not self._healthy():
                    self._abort("align kecil id=%d unhealthy" % sid)
                    return False
                off = self._find_marker(sid)
                if off is not None:
                    saw_small = True
                    ox, oy = off
                    bx = self.align_sign_x * (oy if self.align_swap_xy else ox)
                    by = self.align_sign_y * (ox if self.align_swap_xy else oy)
                    de, dn = self._body_to_enu(bx, by)
                    tx += self.align_gain * de * 0.1
                    ty += self.align_gain * dn * 0.1
                    tx = max(x - self.align_max_drift, min(x + self.align_max_drift, tx))
                    ty = max(y - self.align_max_drift, min(y + self.align_max_drift, ty))
                    if (abs(ox) < self.small_marker_align_tol
                            and abs(oy) < self.small_marker_align_tol):
                        good += 1
                    else:
                        good = 0
                    if good >= self.align_frames:
                        rospy.loginfo("[D]  marker kecil terpusat (off<%.3f) -> DROP",
                                      self.small_marker_align_tol)
                        break
                self.h.send_setpoint(tx, ty, z)
                if (rospy.Time.now() - t0).to_sec() > self.align_timeout:
                    if saw_small:
                        rospy.logwarn("[D]  align kecil timeout -> DROP pakai hasil marker besar")
                    else:
                        rospy.logwarn("[D]  marker kecil id=%d TAK TERLIHAT -> DROP "
                                      "pakai hasil marker besar (fallback)", sid)
                    break
                self.rate.sleep()
        # DROP: buka servo
        ok = self.h.set_servo(self.servo_channel, self.servo_open)
        rospy.loginfo("[D]  servo ch%d -> %d (buka/drop) success=%s",
                      self.servo_channel, self.servo_open, ok)
        # tahan posisi sebentar biar payload jatuh
        t1 = rospy.Time.now()
        while not rospy.is_shutdown() and (rospy.Time.now() - t1).to_sec() < 2.0:
            if not self._healthy():
                self._abort("drop hold unhealthy")
                return False
            self.h.send_setpoint(tx, ty, z); self.rate.sleep()
        return True

    def _tail_to_yaw_cmd(self, tail_side):
        """Konversi sisi ekor (N/E/S/W, label FRAME GAMBAR kamera bawah, lih.
        detect_tail_side() di wp_decode_audit.py) jadi (yaw_deg, direction)
        RELATIF thd yaw drone SEKARANG, siap pakai ke _yaw_turn() yang sudah ada.

        ASUMSI MOUNTING (BELUM diverifikasi fisik -> kalibrasi lapangan mirip
        walk test utk align_sign_x/y): atas-gambar('N') = depan drone saat ini
        (tak perlu putar), kanan-gambar('E') = kanan drone -> putar KANAN
        (CW) 90 deg, kiri-gambar('W') -> putar KIRI (CCW) 90 deg, 'S' -> 180 deg.
        ~tail_dir_rotate_steps / ~tail_mirror: knob kalibrasi bila mounting
        fisik berbeda dari asumsi ini (rotasi 90/180/270 atau gambar mirror).

        Return (yaw_deg, direction) atau None kalau tail_side tak dikenal.
        """
        step = {'N': 0, 'E': 1, 'S': 2, 'W': 3}.get(tail_side)
        if step is None:
            return None
        if self.tail_mirror:
            step = {0: 0, 1: 3, 2: 2, 3: 1}[step]      # tukar E<->W
        step = (step + self.tail_dir_rotate_steps) % 4
        return {0: (0.0, "left"), 1: (90.0, "right"),
                2: (180.0, "right"), 3: (90.0, "left")}[step]

    def _wp_marker_result_age(self, res):
        """Umur WpMarkerResult terbaru (detik). Sama pola dgn _pose_age()."""
        try:
            stamp = res.header.stamp
            if stamp.is_zero():
                return float("inf")
            age = (rospy.Time.now() - stamp).to_sec()
            return age if age >= 0 else 0.0
        except Exception:
            return float("inf")

    def _align_heading_to_tail(self, x, y, z, expected_id):
        """Task 4, tahap OPSIONAL setelah scan/drop: putar (yaw) menghadap
        arah ekor marker WP. Non-blocking by design: tail_side kosong/low-conf
        -> SKIP (log warning, lanjut), JANGAN macet nunggu -- marker besar
        tetap satu-satunya penentu sukses/gagal scan/drop (logika itu TIDAK
        disentuh). Return False HANYA kalau _yaw_turn() abort (unhealthy).

        Guard (fix): WpMarkerResult TERAKHIR bisa saja basi atau dari WP lain
        (mis. deteksi transien salah sesaat sebelum fungsi ini dipanggil) --
        cek umur pesan (~tail_result_stale_sec) DAN wp_id cocok expected_id
        leg yang baru selesai, sebelum dipakai buat putar drone."""
        if not self.enable_heading_align:
            return True
        res = self._wp_marker_result
        if res is None:
            rospy.logwarn("[D]  heading align: belum ada WpMarkerResult -> skip")
            return True
        age = self._wp_marker_result_age(res)
        if age > self.tail_result_stale_sec:
            rospy.logwarn("[D]  heading align: WpMarkerResult basi (%.1fs > %.1fs) -> skip",
                          age, self.tail_result_stale_sec)
            return True
        if res.wp_id != expected_id:
            rospy.logwarn("[D]  heading align: WpMarkerResult wp_id=%d != expected=%d -> skip",
                          res.wp_id, expected_id)
            return True
        if res.low_conf_tail or not res.tail_side:
            rospy.logwarn("[D]  heading align: tail_side tak terdeteksi (low_conf=%s) -> skip",
                          res.low_conf_tail)
            return True
        cmd = self._tail_to_yaw_cmd(res.tail_side)
        if cmd is None:
            rospy.logwarn("[D]  heading align: tail_side='%s' tak dikenal -> skip", res.tail_side)
            return True
        yaw_deg, direction = cmd
        if yaw_deg == 0.0:
            rospy.loginfo("[D]  heading align: ekor='N' (sudah menghadap) -> tak perlu putar")
            return True
        rospy.loginfo("[D]  heading align ke ekor='%s' -> yaw %s %.0f deg",
                      res.tail_side, direction, yaw_deg)
        return self._yaw_turn(x, y, z, yaw_deg=yaw_deg, direction=direction)

    def _land(self, x, y):
        rospy.loginfo("[D] LAND")
        while not rospy.is_shutdown() and self.h.state.mode != "LAND":
            self.h.set_mode("LAND"); rospy.sleep(0.3)
        t0 = rospy.Time.now()
        while not rospy.is_shutdown() and self.h.state.armed:
            if (rospy.Time.now() - t0).to_sec() > 30:
                rospy.logwarn("[D]  land timeout"); break
            rospy.sleep(0.5)
        rospy.loginfo("[D]  mendarat (armed=%s)", self.h.state.armed)
        return True

    def _traverse(self, x, y, z, prev_xyz=None, gate_layers=1, gate_source="1"):
        """Center di bukaan gate (kamera depan) lalu fly-through sepanjang approach.

        gate_layers>1: ulangi (center -> maju pelan lewati bidang) utk tiap lapis.
        Sinyal transisi antar-lapis = area_frac naik-lalu-turun (BUKAN jarak/waktu);
        gate_layer_timeout hanya fallback pengaman. Lapis TERAKHIR = fly-through
        penuh (traverse_through_m), identik versi single-gate.
        """
        rospy.loginfo("[D] TRAVERSE gate (%d lapis, through %.1fm)",
                      gate_layers, self.traverse_through_m)
        # vektor approach (prev->gate) atau X+ default; sama utk semua lapis
        if prev_xyz is not None:
            px, py, pz = prev_xyz
            vx, vy = x - px, y - py
            n = math.hypot(vx, vy)
            if n < 0.1:
                vx, vy, n = 1.0, 0.0, 1.0
            ux, uy = vx / n, vy / n
        else:
            ux, uy = 1.0, 0.0
        gate_yaw = math.atan2(uy, ux)   # arah lintasan gate, konstan sepanjang traverse

        tx, ty, tz = x, y, z
        for layer_num in range(1, gate_layers + 1):
            # ---------- CENTER (logika align PERSIS versi single-gate) ----------
            rospy.loginfo("[D]  lapis %d/%d: center bukaan", layer_num, gate_layers)
            t0 = rospy.Time.now()
            good = 0
            saw_gate = False
            while not rospy.is_shutdown():
                if not self._healthy():
                    self._abort("traverse unhealthy")
                    return False
                g = self._gate_live(gate_source)
                if g is not None and g.detected:
                    saw_gate = True
                    ox, oy = g.off_x, g.off_y
                    # off gambar -> koreksi BODY frame (kamera DEPAN: off_x=kiri/kanan,
                    # off_y=atas/bawah) -> ROTASI ke ENU pakai yaw sekarang (dz/Up
                    # gak perlu dirotasi, yaw gak mempengaruhi sumbu vertikal)
                    bx = self.gate_sign_x * (oy if self.gate_swap_xy else ox)
                    by = self.gate_sign_y * (ox if self.gate_swap_xy else oy)
                    de, dn = self._body_to_enu(bx, by)
                    dz = self.gate_sign_z * oy
                    tx += self.gate_gain * de * 0.1
                    ty += self.gate_gain * dn * 0.1
                    tz += self.gate_gain * dz * 0.1
                    # clamp dari WP gate
                    tx = max(x - self.gate_max_drift, min(x + self.gate_max_drift, tx))
                    ty = max(y - self.gate_max_drift, min(y + self.gate_max_drift, ty))
                    tz = max(z - self.gate_max_drift, min(z + self.gate_max_drift, tz))
                    if abs(ox) < self.gate_tol and abs(oy) < self.gate_tol:
                        good += 1
                    else:
                        good = 0
                    if good >= self.gate_frames:
                        rospy.loginfo("[D]  gate terpusat (off<%.2f, n=%d) -> through",
                                      self.gate_tol, g.num_openings)
                        break
                self.h.send_setpoint(tx, ty, tz, yaw=gate_yaw)
                if (rospy.Time.now() - t0).to_sec() > self.gate_timeout:
                    if saw_gate:
                        rospy.logwarn("[D]  gate center timeout -> through apa adanya")
                    else:
                        rospy.logwarn("[D]  gate TAK TERLIHAT -> through geometris WP")
                    break
                self.rate.sleep()

            if layer_num < gate_layers:
                # -------- BUKAN lapis terakhir: maju pelan sampai lewat bidang ini --------
                # ponytail: lead 0.25m hardcoded (target selalu ~0.25m di depan drone,
                # jadi target tak balapan setpoint -> creep aman). Jadikan rosparam kalau
                # butuh tuning lapangan.
                lead = 0.25
                rospy.loginfo("[D]  lapis %d: maju pelan lewati bidang (lead %.2fm)",
                              layer_num, lead)
                ta = rospy.Time.now()
                area_peak = 0.0
                while not rospy.is_shutdown():
                    if not self._healthy():
                        self._abort("traverse advance lapis %d unhealthy" % layer_num)
                        return False
                    g = self._gate_live(gate_source)
                    area = g.area_frac if g is not None else 0.0
                    if area > area_peak:               # peak terus di-update selama maju
                        area_peak = area
                    detected = bool(g is not None and g.detected)
                    # (a) bidang lapis ini sudah lewat -> tak terlihat lagi
                    if not detected:
                        rospy.loginfo("[D]  lapis %d lewat (gate hilang dr frame)", layer_num)
                        break
                    # (b) area_frac turun dr peak -> sudah melewati bidang terdekat
                    if area_peak > 0.0 and area < area_peak * (1.0 - self.gate_area_drop_ratio):
                        rospy.loginfo("[D]  lapis %d lewat (area %.3f < peak %.3f * %.2f)",
                                      layer_num, area, area_peak, 1.0 - self.gate_area_drop_ratio)
                        break
                    # (c) fallback pengaman: timeout tanpa sinyal visual
                    if (rospy.Time.now() - ta).to_sec() > self.gate_layer_timeout:
                        rospy.logwarn("[D]  lapis %d: gate_layer_timeout tercapai, lanjut "
                                      "paksa ke lapis berikutnya (TIDAK ADA konfirmasi visual)",
                                      layer_num)
                        break
                    # creep maju: target dipatok ~lead meter di depan posisi drone kini
                    cx, cy, _ = self.h.get_xyz()
                    tx = cx + ux * lead
                    ty = cy + uy * lead
                    self.h.send_setpoint(tx, ty, tz, yaw=gate_yaw)
                    self.rate.sleep()
            else:
                # -------- lapis TERAKHIR: fly-through penuh (PERSIS kode asli) --------
                fx = tx + ux * self.traverse_through_m
                fy = ty + uy * self.traverse_through_m
                rospy.loginfo("[D]  through -> (%.1f,%.1f,%.1f)", fx, fy, tz)
                return self._goto(fx, fy, tz, "gate_through", yaw=gate_yaw)

        return True

    def _line_follow(self, x, y, z, label=""):
        """Terbang ke (x,y,z) [WP target, mis. WP5] sambil koreksi lateral pakai
        garis putus-putus (kamera bawah). Beda dari _align_and_drop()/_traverse():
        TIDAK ada kondisi "sudah center -> berhenti" -- dash cuma nudge tx,ty
        sekilas kalau kelihatan (pola align_gain/align_max_drift ArUco, reuse),
        sukses/gagal-nya SAMA kayak _goto() biasa (reach_tol_xy/z, goto_timeout,
        max_recede). GAP (Line.detected=False, di antara dash) BUKAN kegagalan --
        loop lanjut kirim setpoint ke (x,y,z) apa adanya (dead-reckon EKF),
        persis fallback "align timeout -> drop apa adanya" yang sudah ada.
        """
        rospy.loginfo("[D] LINE_FOLLOW -> %s (%.1f,%.1f,%.1f)", label, x, y, z)
        tx, ty = x, y
        t0 = rospy.Time.now()
        min_dist = None
        while not rospy.is_shutdown():
            if not self._healthy():
                self._abort("line_follow %s unhealthy" % label)
                return False
            ln = self._line
            if ln is not None and ln.detected:
                ox, oy = ln.off_x, ln.off_y
                bx = self.line_sign_x * (oy if self.line_swap_xy else ox)
                by = self.line_sign_y * (ox if self.line_swap_xy else oy)
                de, dn = self._body_to_enu(bx, by)
                tx += self.line_gain * de * 0.1
                ty += self.line_gain * dn * 0.1
                # clamp: jangan drift jauh dari WP target (anti runaway vision drift)
                tx = max(x - self.line_max_drift, min(x + self.line_max_drift, tx))
                ty = max(y - self.line_max_drift, min(y + self.line_max_drift, ty))
            self.h.send_setpoint(tx, ty, z)
            # cek "sampai" ke target TERKOREKSI (tx,ty), BUKAN koordinat asli (x,y).
            # BUG ditemukan SITL 2026-09-03: koreksi visual persisten (bukan cuma
            # sesekali -- garis beneran melengkung/gak lurus ke WP survei) bikin
            # tx,ty settle di ujung line_max_drift dan MENETAP di situ (gak pernah
            # balik ke 0 sendiri) -- kalau _reached() ngecek ke (x,y) asli, itu
            # gak akan pernah kesampaian selama koreksi masih aktif -> timeout->
            # abort->LAND padahal drone udah PERSIS di titik yang dikejar kamera.
            # (x,y) asli tetap jadi acuan SAFETY (max_recede di bawah, TIDAK
            # diubah) -- cuma kriteria SUKSES yang pindah ke target dinamis.
            if self._reached(tx, ty, z):
                rospy.loginfo("[D]  line_follow sampai %s", label)
                return True
            cx, cy, _ = self.h.get_xyz()
            d = math.hypot(cx - x, cy - y)
            min_dist = d if min_dist is None else min(min_dist, d)
            if min_dist is not None and d > min_dist + self.max_recede:
                rospy.logerr("[D] ABORT: line_follow %s menjauh %.1f>min+%.1f (drift vision?)",
                             label, d, self.max_recede)
                self._abort("line_follow %s drift" % label)
                return False
            if (rospy.Time.now() - t0).to_sec() > self.goto_timeout:
                rospy.logerr("[D] ABORT: line_follow %s timeout %.0fs", label, self.goto_timeout)
                self._abort("line_follow %s timeout" % label)
                return False
            self.rate.sleep()
        return False

    def _yaw_turn(self, x, y, z, yaw_deg=90.0, direction="left"):
        """Putar di tempat (tahan x,y,z). left = +yaw (CCW / kiri drone), right = -yaw.
        yaw_deg: besar sudut relatif (default 90)."""
        yaw_deg = float(yaw_deg)
        direction = str(direction).lower()
        sign = 1.0 if direction in ("left", "kiri", "ccw", "+") else -1.0
        yaw0 = self._get_yaw()
        target = yaw0 + sign * math.radians(yaw_deg)
        rospy.loginfo("[D] YAW %s %.0f deg (from %.1f -> %.1f deg)",
                      direction, yaw_deg, math.degrees(yaw0), math.degrees(target))
        t0 = rospy.Time.now()
        good = 0
        while not rospy.is_shutdown():
            if not self._healthy():
                self._abort("yaw unhealthy")
                return False
            self.h.send_setpoint(x, y, z, yaw=target)
            err = abs(self._angle_diff(self._get_yaw(), target))
            if err < self.yaw_tol:
                good += 1
                if good >= self.yaw_frames:
                    rospy.loginfo("[D]  yaw OK (err=%.1f deg)", math.degrees(err))
                    return True
            else:
                good = 0
            if (rospy.Time.now() - t0).to_sec() > self.yaw_timeout:
                rospy.logwarn("[D]  yaw timeout (err=%.1f deg) -> lanjut",
                              math.degrees(err))
                return True
            self.rate.sleep()
        return False

    def _hold(self, x, y, z, seconds):
        """Hover diam di (x,y,z) selama `seconds` detik -- delay generik lewat
        leg field hold_sec (2026-09-22). Dipakai leg tanpa aksi vision (goto/
        land) yang butuh jeda seperti scan/drop punya secara alami (scan_timeout/
        align_timeout). Pola sama seperti hold 2s sesudah drop di
        _align_and_drop() -- cuma dijadikan reusable & dikontrol dari mission
        yaml, bukan hardcoded 2.0."""
        if seconds <= 0:
            return True
        rospy.loginfo("[D]  hold %.1fs di tempat", seconds)
        t0 = rospy.Time.now()
        while not rospy.is_shutdown() and (rospy.Time.now() - t0).to_sec() < seconds:
            if not self._healthy():
                self._abort("hold unhealthy")
                return False
            self.h.send_setpoint(x, y, z)
            self.rate.sleep()
        return True

    # ---------- main ----------
    def run(self):
        if not self.h.wait_for_connection():
            return
        self.h.wait_for_pose()
        if self.do_takeoff and not self._takeoff_seq():
            return

        prev_xyz = None
        for i, leg in enumerate(self.legs):
            if rospy.is_shutdown():
                return
            name = leg["wp"]
            x, y, z = self._wp_xyz(name)
            act = leg.get("action", "scan")
            rospy.loginfo("[D] === leg %d/%d: %s (%s) ===",
                          i + 1, len(self.legs), name, act)
            self.pub_status.publish(String(data="%s (%s) [%d/%d]" %
                                            (name, act, i + 1, len(self.legs))))
            # yaw di tempat: tetap goto dulu (biasanya sudah di WP setelah drop)
            # line_follow DIKECUALIKAN: _line_follow() SENDIRI sudah menempuh
            # seluruh jarak (goto-equivalent + koreksi visual) -- goto polos di
            # sini bakal nerbangin BUTA (tanpa kamera) duluan sampai TARGET
            # PERSIS, bikin _line_follow() gak kebagian jarak sama sekali buat
            # dikoreksi (ketemu 2026-08-29 pas SITL: line_follow selesai instan,
            # <0.02s buat jarak 7m -- bug, bukan behavior benar).
            # hold_heading (2026-09-15): leg minta terbang MUNDUR ke WP ini --
            # tahan yaw SEKARANG (bukan auto-hadap arah tujuan, default _goto()
            # kalau yaw=None) sepanjang transit. Dipakai WP1->gate_double &
            # WP2->gate_triple (setelah leg yaw kiri) biar kamera depan/badan
            # drone TETAP menghadap arah datang, bukan muter duluan ke gate --
            # traverse sesudahnya yang urus hadap ke gate (gate_yaw dari vektor
            # approach, lih. _traverse()).
            goto_yaw = self._get_yaw() if leg.get("hold_heading") else None
            if goto_yaw is not None:
                rospy.loginfo("[D]  hold_heading: tahan yaw=%.1f deg (mundur) -> %s",
                              math.degrees(goto_yaw), name)
            if act != "line_follow" and not self._goto(x, y, z, name, yaw=goto_yaw):
                return
            # hold_sec (2026-09-22): jeda eksplisit di WP ini SEBELUM aksi --
            # scan/drop otomatis dapat delay dari scan_timeout/align_timeout,
            # tapi goto/land/traverse gak punya dwell alami; leg yang butuh
            # jeda (mis. gate_single_final/wp5 saat gate/scan gak dipakai)
            # set hold_sec di mission yaml. line_follow DIKECUALIKAN di sini
            # (dia belum "sampai" -- goto-nya sendiri baru terjadi di bawah),
            # hold-nya dipasang SESUDAH _line_follow() selesai.
            if act != "line_follow" and not self._hold(x, y, z, float(leg.get("hold_sec", 0))):
                return
            # marker kecil pelengkap (opsional): None kalau leg tak punya field ini
            expected_id_small = leg.get("expected_id_small")
            if act == "scan":
                self._scan(int(leg["expected_id"]), x, y, z, expected_id_small)
                if not self._align_heading_to_tail(x, y, z, int(leg["expected_id"])):
                    return
            elif act == "drop":
                self._align_and_drop(int(leg["expected_id"]), x, y, z, expected_id_small)
                if not self._align_heading_to_tail(x, y, z, int(leg["expected_id"])):
                    return
            elif act == "traverse":
                layers = int(leg.get("gate_layers", 1))
                gsrc = leg.get("gate_source", "1")
                if not self._traverse(x, y, z, prev_xyz, gate_layers=layers, gate_source=gsrc):
                    return
            elif act == "yaw":
                yaw_deg = float(leg.get("yaw_deg", 90.0))
                direction = leg.get("direction", "left")
                # override rosparam (2026-09-22): ~yaw_direction_override/
                # ~yaw_deg_override, diisi lewat arg launch (mis. arena punya
                # 2 bentuk fisik, arah putar WP2->WP3 perlu diganti per sesi
                # tanpa edit mission yaml). Default kosong/-1 = TIDAK override,
                # pakai nilai leg yaml apa adanya -- semua mission lain (mis.
                # yaw kiri 90 sebelum gate_triple di mission_seleksi/final)
                # tidak berubah kecuali argnya eksplisit di-set.
                if self.yaw_direction_override:
                    direction = self.yaw_direction_override
                if self.yaw_deg_override >= 0:
                    yaw_deg = self.yaw_deg_override
                if not self._yaw_turn(x, y, z, yaw_deg=yaw_deg, direction=direction):
                    return
            elif act == "line_follow":
                if not self._line_follow(x, y, z, name):
                    return
                if not self._hold(x, y, z, float(leg.get("hold_sec", 0))):
                    return
            elif act == "goto":
                pass  # sudah sampai lewat goto pre-leg di atas, tidak ada aksi tambahan
            elif act == "land":
                self._land(x, y)
            else:
                rospy.logwarn("[D]  aksi tak dikenal: %s", act)
            prev_xyz = (x, y, z)

        rospy.loginfo("[D] === MISI SELESAI ===")
        self.pub_status.publish(String(data="DONE"))


def _selftest_body_to_enu():
    """Cek matematika _body_to_enu() tanpa ROS -- rotasi body->ENU harus:
    (a) identitas persis di yaw=0 (de=bx, dn=by, no-op -- kompatibel behavior
    lama sebelum fix), (b) muter 90 derajat CCW yg benar (REP-103/ENU) di
    yaw=pi/2, (c) magnitude vektor gak berubah di rotasi manapun (rotasi murni,
    bukan skala)."""
    import types
    md = MissionD.__new__(MissionD)
    for test_yaw in (0.0, math.pi / 2, math.pi, -math.pi / 2, 1.234):
        md.h = types.SimpleNamespace(get_yaw=lambda yw=test_yaw: yw)
        for bx, by in [(1.0, 0.0), (0.0, 1.0), (0.7, -0.3), (0.0, 0.0)]:
            de, dn = md._body_to_enu(bx, by)
            mag_in = math.hypot(bx, by)
            mag_out = math.hypot(de, dn)
            assert abs(mag_in - mag_out) < 1e-9, \
                f"rotasi ubah magnitude: yaw={test_yaw} in=({bx},{by}) out=({de},{dn})"
    # (a) yaw=0 harus identitas
    md.h = types.SimpleNamespace(get_yaw=lambda: 0.0)
    de, dn = md._body_to_enu(0.7, -0.3)
    assert abs(de - 0.7) < 1e-9 and abs(dn - (-0.3)) < 1e-9, "yaw=0 harusnya no-op"
    # (b) yaw=pi/2: body-X (1,0) harus muter jadi (0,1) -- CCW 90 derajat
    md.h = types.SimpleNamespace(get_yaw=lambda: math.pi / 2)
    de, dn = md._body_to_enu(1.0, 0.0)
    assert abs(de - 0.0) < 1e-6 and abs(dn - 1.0) < 1e-6, \
        f"rotasi 90deg CCW salah: dapat ({de},{dn}), harusnya (0,1)"
    print("mission_d _body_to_enu selftest: PASS")


def _selftest_start_wp():
    """Cek _find_leg_index() -- retry-dari-WP (2026-09-03), tanpa ROS."""
    legs = [
        {"wp": "wp1", "action": "scan"},
        {"wp": "gate_double", "action": "traverse"},
        {"wp": "wp2", "action": "drop"},
        {"wp": "wp2", "action": "yaw"},   # wp2 muncul 2x -- harus ambil yg PERTAMA
        {"wp": "wp3", "action": "scan"},
    ]
    assert MissionD._find_leg_index(legs, "wp1") == 0
    assert MissionD._find_leg_index(legs, "wp2") == 2, "harus leg PERTAMA yg cocok, bukan yg ke-2 (yaw)"
    assert MissionD._find_leg_index(legs, "wp3") == 4
    assert MissionD._find_leg_index(legs, "wp99_gak_ada") is None
    print("mission_d _find_leg_index selftest: PASS")


def main():
    import sys as _sys
    if "--selftest" in _sys.argv:
        _selftest_body_to_enu()
        _selftest_start_wp()
        return
    rospy.init_node("mission_d")
    MissionD().run()


if __name__ == "__main__":
    main()

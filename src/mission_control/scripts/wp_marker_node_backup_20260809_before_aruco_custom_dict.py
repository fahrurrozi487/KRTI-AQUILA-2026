#!/usr/bin/env python3
"""wp_marker_node.py — ROS node untuk deteksi marker WP1/WP2/WP3/WP4.

v2 (2026-08-07): rebuild total pakai dataset lapangan baru (dataset_arucode/
Aruco_yang_baru/, ~4000 foto). Ganti dari wp_marker_detector.decode_frame() (dataset
lama) ke wp_decode_v2.decode_frame_v2() -- lih. docstring wp_decode_v2.py utk alasan
teknis (mask dibatasi tarp, marker-kecil dipisah via profil proyeksi bukan morfologi,
ID-decode pakai rotation-search thd referensi -- gak gantung ke deteksi-arah).

v2.1 (2026-08-08): AGREGASI OR/LATCH. Diagnosis dataset penuh nunjukin 58% kegagalan
WP1 itu `no_match` di foto yg KONDISINYA UDAH BAGUS (terang, tarp asli) -- noise acak
per-frame (corner/warp meleset dikit beda arah tiap frame), bukan soal cahaya. Drone
hover beberapa detik pas scan (`scan_frames`/`scan_timeout` mission_d.py) -> banyak
frame yg SEBELUMNYA dinilai sendiri-sendiri, dibuang percuma.

Percobaan pertama (majority-vote per-sel, TemporalAggregator) DIBATALKAN sesudah
divalidasi empiris pakai burst foto realistis (dikelompokkan per sesi hover via
timestamp filename): rata-rata bit dari window JUSTRU lebih jarang berhasil drpd
single-frame (sinyal bersih WP1 sering cuma 2-3 dari 100+ frame/burst -- gak pernah
jadi mayoritas, malah teredam noise), dan SEKALI pernah salah tebak padahal
single-frame di burst yg sama tidak pernah salah. Diganti (2026-08-08) ke
LatchAggregator (wp_decode_v2.py): OR sederhana, bukan average -- begitu 1 frame
individually reason='ok' (match_bits_to_refs() YANG SAMA dgn single-frame, bukan
threshold baru), ID itu dipakai & ditahan (`~latch_sec`) biar gak kedip ke -1 di
antara 2 bacaan ok yg berdekatan. TIDAK PERNAH lebih buruk dari single-frame
(union), precision = persis precision single-frame 'ok' (95.7-100%, sudah
tervalidasi). TIDAK ada ML/model baru.

~result & ~markers SEKARANG mencerminkan verdict LATCH (yg dipakai mission_d.py
buat scan/align). ~result_small DIUBAH PERANNYA (topic lama dipertahankan, isinya
beda): sekarang cerminan pembacaan SINGLE-FRAME mentah (utk debug/bandingkan vs
verdict latch), BUKAN lagi cerminan ~result spt v2 kemarin.

TIDAK menggunakan cv_bridge (rusak di Jetson ini); konversi via image_convert.py.

Subscribe:
  ~image_topic   (default /camera_down/image_raw)

Publish:
  ~result        mission_control/WpMarkerResult   (verdict LATCH -- ID dari frame
                   'ok' terakhir, ditahan ~latch_sec; off_x/off_y/aspect dari frame
                   SEKARANG biar align tetap responsif)
  ~result_small  mission_control/WpMarkerResult   (verdict SINGLE-FRAME mentah, debug)
  ~wp2_present   std_msgs/Bool                    (setiap frame)
  ~markers       mission_control/ArucoMarkers     (dari verdict LATCH; hanya saat reason='ok')
  ~debug_image   sensor_msgs/Image                (hanya bila ada subscriber)

~markers mengikuti format persis aruco_node sehingga mission_d bisa dipakai
dengan param markers_topic:=/wp_marker_node/markers tanpa ubah kode mission.

⚠️ Catatan interaksi dgn `enable_heading_align` (Task 4, default OFF): guard
`_align_heading_to_tail()` di mission_d.py cek `res.wp_id == expected_id` dari
`~result` -- kalau belum pernah ok SAMA SEKALI atau latch sudah kadaluarsa
(`reason='no_signal'`), wp_id sementara -1, heading-align skip transient pas itu.
Non-blocking by design (fitur ini opt-in, bukan penentu sukses/gagal scan).

Params:
  ~image_topic          str    /camera_down/image_raw
  ~json_path            str    $(find mission_control)/config/wp_marker_reference_v2.json
  ~hamming_threshold    int    7
  ~ambiguous_margin      int    3
  ~wp2_min_area_px      int    3000
  ~throttle_hz          float  5.0
  ~latch_sec            float  2.0  (tahan bacaan 'ok' terakhir sekian detik sebelum expired)
  ~publish_debug        bool   true
  ~clahe_fallback_wp_ids str  "1"  (wp_id yg BOLEH pakai CLAHE fallback -- WP3 & WP4
                          DIKECUALIKAN default (dua-duanya pernah nyumbang salah
                          tebak pas divalidasi skala PENUH di resolusi kamera live
                          640x480). Lih. komentar __init__.)

Subscribe tambahan:
  ~expected_id   std_msgs/Int32   (dari mission_d.py._scan(), -1 = idle. CLAHE
                   fallback (2026-08-08) HANYA aktif selama wp_id di
                   ~clahe_fallback_wp_ids -- ~91% lebih lambat/frame, cuma boleh
                   dibayar saat mission BENERAN lagi nyari WP yg tervalidasi
                   aman. Lih. enhance_clahe() docstring wp_decode_v2.py.)
"""
import os
import sys
sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))

import cv2
import rospy
from sensor_msgs.msg import Image
from std_msgs.msg import Bool, Int32

from mission_control.msg import WpMarkerResult, ArucoMarker, ArucoMarkers

from wp_marker_detector import detect_wp2_presence
from wp_decode_v2 import decode_frame_v2_robust, load_refs_v2, LatchAggregator
from image_convert import imgmsg_to_np, np_to_imgmsg


class WpMarkerNode:
    def __init__(self):
        image_topic   = rospy.get_param('~image_topic',       '/camera_down/image_raw')
        json_path     = rospy.get_param('~json_path',         '')
        self.hamming  = int(rospy.get_param('~hamming_threshold', 7))
        self.ambig_margin = int(rospy.get_param('~ambiguous_margin', 3))
        self.min_area = int(rospy.get_param('~wp2_min_area_px',   3000))
        throttle_hz   = float(rospy.get_param('~throttle_hz',  5.0))
        self.pub_dbg  = bool(rospy.get_param('~publish_debug', True))

        latch_sec = float(rospy.get_param('~latch_sec', 2.0))
        self.latch = LatchAggregator(latch_sec=latch_sec)

        # CLAHE fallback: dipicu HANYA selama mission_d.py aktif nyari WP tertentu
        # (~expected_id dari _scan(), lih. mission_d.py) -- mahal (~91% lbh lambat/
        # frame, lih. enhance_clahe() docstring), TIDAK boleh nyala terus-menerus.
        self.expected_id = -1   # -1 = gak lagi scan spesifik, CLAHE fallback OFF
        rospy.Subscriber('~expected_id', Int32, self._expected_id_cb, queue_size=1)

        # 2026-08-08/09: divalidasi ULANG 2x di resolusi KAMERA LIVE asli (640x480).
        # (1) 2026-08-08, tes CLAHE-tunggal skala kecil: WP3 presisi turun 100%->
        #     99.4% (1 salah), WP1/WP4 keliatan aman -- WP3 DIKELUARKAN.
        # (2) 2026-08-09, decode_frame_v2_robust (5 varian HSV + CLAHE combo) DITES
        #     ULANG skala PENUH (837 foto): WP4 TERNYATA JUGA gak aman -- 2 salah
        #     tebak, KEDUANYA dari stage CLAHE-kombinasi (`clahe+hsv_variant_3` &
        #     `clahe+hsv_variant_4`). 5 varian HSV MURNI (tanpa CLAHE) tetap bersih
        #     (0 salah) di WP4. Kesimpulan: CLAHE (sendirian ATAU dikombinasi varian
        #     HSV) TIDAK bisa dipercaya aman kecuali sudah divalidasi skala PENUH
        #     per-WP -- WP4 DIKELUARKAN juga. Default sekarang cuma {1} (WP1 doang).
        # User eksplisit gak mau salah prediksi. Ganti via param kalau ada data
        # baru (validasi skala PENUH) yg membenarkan WP3/WP4 aman lagi.
        clahe_wp_ids = rospy.get_param('~clahe_fallback_wp_ids', '1')
        self.clahe_wp_ids = {int(x) for x in str(clahe_wp_ids).split(',') if x.strip()}

        if not json_path:
            pkg = os.path.join(os.path.dirname(os.path.abspath(__file__)), '..', 'config')
            json_path = os.path.normpath(os.path.join(pkg, 'wp_marker_reference_v2.json'))

        self.refs = load_refs_v2(json_path, wp_ids=(1, 3, 4))
        rospy.loginfo('[wp_marker] v2.1 (latch). refs=%s  hamming_thr=%d  ambig_margin=%d  '
                      'wp2_area=%d  latch_sec=%.1fs',
                      json_path, self.hamming, self.ambig_margin, self.min_area, latch_sec)

        self.pub_result  = rospy.Publisher('~result',      WpMarkerResult, queue_size=5)
        self.pub_result_small = rospy.Publisher('~result_small', WpMarkerResult, queue_size=5)
        self.pub_wp2     = rospy.Publisher('~wp2_present', Bool,           queue_size=5)
        self.pub_markers = rospy.Publisher('~markers',     ArucoMarkers,   queue_size=5)
        if self.pub_dbg:
            self.pub_debug = rospy.Publisher('~debug_image', Image, queue_size=2)

        self._throttle_period = 1.0 / max(throttle_hz, 0.1)
        self._last_proc = rospy.Time(0)
        self._warn_count = 0

        rospy.Subscriber(image_topic, Image, self._image_cb,
                         queue_size=1, buff_size=2**24)
        rospy.loginfo('[wp_marker] siap. topic=%s  throttle=%.1f Hz', image_topic, throttle_hz)

    def _expected_id_cb(self, msg):
        self.expected_id = int(msg.data)

    def _image_cb(self, msg):
        now = rospy.Time.now()
        if (now - self._last_proc).to_sec() < self._throttle_period:
            return
        self._last_proc = now

        try:
            bgr = imgmsg_to_np(msg)
        except ValueError as e:
            if self._warn_count < 3:
                rospy.logwarn('[wp_marker] imgmsg_to_np: %s', e)
                self._warn_count += 1
            return

        h_img, w_img = bgr.shape[:2]

        # --- WP1/3/4 ID decode SINGLE-FRAME: baseline -> 5 varian HSV (selalu
        # aktif, murah ~7Hz) -> CLAHE+varian (HANYA selama mission_d.py aktif
        # scan WP yg divalidasi aman, ~clahe_fallback_wp_ids default {1} --
        # WP3 & WP4 DIKECUALIKAN dari CLAHE, lih. komentar __init__ & docstring
        # decode_frame_v2_robust() wp_decode_v2.py) ---
        allow_clahe = self.expected_id in self.clahe_wp_ids
        dec = decode_frame_v2_robust(bgr, self.refs, self.hamming, self.ambig_margin,
                                      allow_clahe=allow_clahe)

        self.pub_result_small.publish(_dec_to_msg(dec, msg.header))  # debug: baca mentah

        # --- OR/latch: pakai bacaan 'ok' terakhir, tahan ~latch_sec ---
        v = self.latch.update(dec['wp_id'], dec['hamming'], dec['reliable'],
                              dec['reason'], dec['tail_side'], now.to_sec())
        # posisi (off_x/off_y/aspect) tetap dari frame SEKARANG biar align responsif;
        # ID/hamming/reliable/reason/tail_side dari verdict latch.
        dec_t = dict(dec, wp_id=v['wp_id'], hamming=v['hamming'], reliable=v['reliable'],
                    reason=v['reason'], tail_side=v['tail_side'])
        self.pub_result.publish(_dec_to_msg(dec_t, msg.header))

        # --- WP2 presence (red box drop trigger, tidak berubah dari v1) ---
        wp2 = detect_wp2_presence(bgr, self.min_area)
        self.pub_wp2.publish(Bool(data=wp2['present']))

        # --- ArucoMarkers (kompatibel mission_d) -- dari verdict TEMPORAL ---
        aruco_msg = ArucoMarkers()
        aruco_msg.header       = msg.header
        aruco_msg.image_width  = w_img
        aruco_msg.image_height = h_img

        if dec_t['reason'] == 'ok' and dec_t['wp_id'] != -1:
            am = ArucoMarker()
            am.id    = dec_t['wp_id']
            am.off_x = dec_t['off_x']
            am.off_y = dec_t['off_y']
            am.cx    = (dec_t['off_x'] + 1.0) * w_img / 2.0
            am.cy    = (dec_t['off_y'] + 1.0) * h_img / 2.0
            aruco_msg.markers.append(am)

        if wp2['present']:
            am2 = ArucoMarker()
            am2.id    = 2
            am2.off_x = wp2['off_x']
            am2.off_y = wp2['off_y']
            am2.cx    = (wp2['off_x'] + 1.0) * w_img / 2.0
            am2.cy    = (wp2['off_y'] + 1.0) * h_img / 2.0
            aruco_msg.markers.append(am2)

        self.pub_markers.publish(aruco_msg)

        # --- debug image (only when subscriber connected) ---
        if self.pub_dbg and self.pub_debug.get_num_connections() > 0:
            vis = _draw_debug(bgr, dec, dec_t, wp2)
            self.pub_debug.publish(
                np_to_imgmsg(vis, msg.header.stamp, msg.header.frame_id))


def _dec_to_msg(dec, header):
    """dict (dari decode_frame_v2() atau verdict latch) -> WpMarkerResult."""
    res = WpMarkerResult()
    res.header        = header
    res.found_marker  = bool(dec['found_marker'])
    res.wp_id         = int(dec['wp_id'])
    res.hamming       = int(dec['hamming'])
    res.reliable      = bool(dec['reliable'])
    res.low_conf_tail = (dec['tail_side'] == '')
    res.tail_side     = str(dec['tail_side'])
    res.reason        = str(dec['reason'])
    res.aspect        = float(dec['aspect'])
    res.off_x         = float(dec['off_x'])
    res.off_y         = float(dec['off_y'])
    return res


def _draw_debug(bgr, dec, dec_t, wp2):
    """Overlay: verdict LATCH (baris 1, utama) + baca single-frame (baris 2,
    pembanding) + WP2."""
    vis = bgr.copy()
    h, w = vis.shape[:2]
    color_t = (0, 255, 0) if dec_t['wp_id'] != -1 else (0, 165, 255)
    label_t = (f"[LATCH] WP{dec_t['wp_id']} "
               f"H={dec_t['hamming']} {dec_t['reason']} arah={dec_t['tail_side'] or '?'}")
    cv2.putText(vis, label_t, (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 0.7, color_t, 2)

    color_s = (0, 255, 0) if dec['wp_id'] != -1 else (0, 0, 255)
    label_s = (f"[frame ini] WP{dec['wp_id']} H={dec['hamming']} {dec['reason']} "
               f"off=({dec['off_x']:.2f},{dec['off_y']:.2f})" if dec['found_marker'] else
               f"[frame ini] {dec['reason']}")
    cv2.putText(vis, label_s, (10, 58), cv2.FONT_HERSHEY_SIMPLEX, 0.6, color_s, 2)

    wp2_color = (0, 255, 0) if wp2['present'] else (128, 128, 128)
    cv2.putText(vis, f"WP2 present={wp2['present']} area={wp2['area_px']}",
                (10, 84), cv2.FONT_HERSHEY_SIMPLEX, 0.6, wp2_color, 2)
    if dec_t['found_marker'] and dec_t['wp_id'] != -1:
        cx = int((dec_t['off_x'] + 1.0) * w / 2)
        cy = int((dec_t['off_y'] + 1.0) * h / 2)
        cv2.drawMarker(vis, (cx, cy), (0, 255, 0), cv2.MARKER_CROSS, 20, 2)
    return vis


def main():
    rospy.init_node('wp_marker_node')
    WpMarkerNode()
    rospy.spin()


if __name__ == '__main__':
    main()

#!/usr/bin/env python3
"""exposure_bracket_node.py (2026-08-09) — multi-exposure bracketing v4l2, OPT-IN.

KENAPA: `low_quality` (marker area gelap, brightness core < MIN_CORE_BRIGHTNESS)
itu bottleneck TERBESAR sisa WP1/WP3 (65%/44% dari kegagalan) -- SEMUA teknik
software post-hoc (gamma correction 2 varian, CLAHE) TERBUKTI GAGAL total
(dites empiris, 0 manfaat di versi paling hati-hati). Analisis brightness
core vs seluruh-gambar (2026-08-09) nunjukin brightness core SANGAT
BERVARIASI antar-frame (contoh WP1: rata2 133, tapi minimum 47) -- ini bukan
"kamera selalu kurang terang", tapi momen tertentu (sudut/framing) bikin area
marker spesifik jadi gelap sesaat.

TEKNIK: exposure bracketing (HDR klasik, non-ML) -- BEDA FUNDAMENTAL dari
gamma/CLAHE krn nangkep frame BARU di exposure sensor yg BENERAN beda,
bukan olah-ulang 1 frame yg udah ditangkep. Selama scan aktif
(~expected_id != -1, gate SAMA PERSIS kayak CLAHE fallback di wp_marker_node.py),
node ini siklus `exposure_absolute` v4l2 kamera bawah di antara beberapa
preset -- biar ada peluang nangkep momen "beruntung" (marker kebetulan lebih
terang di exposure tertentu) tanpa terus-menerus overexpose kondisi yg udah
bagus.

⚠️ BATASAN JUJUR: preset exposure_absolute DEFAULT di bawah cuma tebakan awal
(dites indoor: exposure=2000 -> brightness 2x, TAPI itu bukan kondisi
lapangan). WAJIB dikalibrasi ulang di lokasi lomba asli sebelum dipercaya --
lih. `~exposure_presets` param. Analisis brightness core menunjukkan ini
CUMA nolongin kasus borderline (core brightness ~70-90), BUKAN kasus yg
beneran gelap parah (core<50) -- realistiskan ekspektasi.

Params:
  ~video_device       str    /dev/video1
  ~expected_id_topic  str    /wp_marker_node/expected_id
  ~exposure_presets    str   "auto,1200,2000" (comma-separated; "auto" = balik
                        exposure_auto=3/otomatis, angka = exposure_absolute manual)
  ~cycle_sec          float  1.0  (ganti preset tiap sekian detik selama scan aktif)

TIDAK auto-dijalankan di launch manapun (opt-in, sama pola kayak
calibrate_align_sign.py / set_camera_exposure.sh) -- jalankan manual/tambahkan
ke launch cuma kalau sudah dikalibrasi & mau dipakai.
"""
import subprocess
import sys

import rospy
from std_msgs.msg import Int32


class ExposureBracketNode:
    def __init__(self):
        self.device = rospy.get_param('~video_device', '/dev/video1')
        presets_str = rospy.get_param('~exposure_presets', 'auto,1200,2000')
        self.presets = [p.strip() for p in presets_str.split(',') if p.strip()]
        self.cycle_sec = float(rospy.get_param('~cycle_sec', 1.0))

        self.expected_id = -1
        self._idx = 0
        self._current_mode = None   # cache biar gak spam v4l2-ctl kalau gak ganti

        rospy.Subscriber('~expected_id', Int32, self._expected_id_cb, queue_size=1)
        self._timer = rospy.Timer(rospy.Duration(self.cycle_sec), self._tick)
        rospy.loginfo('[exposure_bracket] siap. device=%s presets=%s cycle=%.1fs '
                      '-- OPT-IN, WAJIB dikalibrasi lapangan sebelum dipercaya',
                      self.device, self.presets, self.cycle_sec)

    def _expected_id_cb(self, msg):
        was_idle = (self.expected_id == -1)
        self.expected_id = int(msg.data)
        if self.expected_id == -1 and not was_idle:
            self._set_mode('auto')   # balik auto begitu scan selesai

    def _set_mode(self, mode):
        if mode == self._current_mode:
            return
        try:
            if mode == 'auto':
                subprocess.run(['v4l2-ctl', '-d', self.device,
                                '--set-ctrl=exposure_auto=3'],
                               check=True, capture_output=True, timeout=2)
            else:
                subprocess.run(['v4l2-ctl', '-d', self.device,
                                '--set-ctrl=exposure_auto=1',
                                f'--set-ctrl=exposure_absolute={mode}'],
                               check=True, capture_output=True, timeout=2)
            self._current_mode = mode
        except (subprocess.CalledProcessError, subprocess.TimeoutExpired) as e:
            rospy.logwarn_throttle(5.0, '[exposure_bracket] gagal set %s: %s', mode, e)

    def _tick(self, event):
        if self.expected_id == -1:
            return   # idle -- biarkan auto, jangan bracket terus-menerus
        preset = self.presets[self._idx % len(self.presets)]
        self._idx += 1
        self._set_mode(preset)

    def shutdown(self):
        self._set_mode('auto')   # WAJIB balikin auto pas node mati, jangan nyangkut manual


def main():
    rospy.init_node('exposure_bracket_node')
    node = ExposureBracketNode()
    rospy.on_shutdown(node.shutdown)
    rospy.spin()


if __name__ == '__main__':
    main()

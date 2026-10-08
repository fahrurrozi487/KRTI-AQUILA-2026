#!/usr/bin/env python3
"""
Detektor ArUco (OpenCV murni, TANPA ROS) — inti Fase C untuk kamera bawah.

Dipakai untuk: scan marker di WP1, dan align drone di atas ember (WP2) sebelum
drop payload. Detektor hanya melaporkan posisi marker di GAMBAR; pemetaan ke
gerak drone dilakukan di Fase D sesuai konvensi mounting kamera.

API: OpenCV >= 4.7 (cv2.aruco.ArucoDetector). Default dictionary 7x7 (DICT_7X7_250).

Konvensi offset (untuk align):
  Gambar: origin kiri-atas, x ke kanan, y ke bawah.
  off_x = (cx - W/2) / (W/2)  -> [-1..1], + = marker di KANAN gambar
  off_y = (cy - H/2) / (H/2)  -> [-1..1], + = marker di BAWAH gambar
  Drone dianggap "di atas marker" saat |off_x|,|off_y| <= center_tol.

Jalankan mandiri:
  python3 aruco_detector.py --selftest          # uji pakai marker sintetis (tanpa kamera)
  python3 aruco_detector.py --image foto.png     # deteksi di 1 gambar
  python3 aruco_detector.py --camera 0           # live dari /dev/video0 (saat kamera ada)
"""
import math

import cv2
import numpy as np
import cv2.aruco as aruco


# Peta nama dictionary 7x7 -> konstanta cv2.aruco
_DICTS = {
    "DICT_7X7_50": aruco.DICT_7X7_50,
    "DICT_7X7_100": aruco.DICT_7X7_100,
    "DICT_7X7_250": aruco.DICT_7X7_250,
    "DICT_7X7_1000": aruco.DICT_7X7_1000,
}


class Marker:
    """Satu marker terdeteksi."""
    def __init__(self, marker_id, corners, img_w, img_h):
        self.id = int(marker_id)
        self.corners = corners                      # (4,2) float piksel
        c = corners.reshape(4, 2)
        self.cx = float(c[:, 0].mean())             # pusat x piksel
        self.cy = float(c[:, 1].mean())             # pusat y piksel
        self.off_x = (self.cx - img_w / 2.0) / (img_w / 2.0)
        self.off_y = (self.cy - img_h / 2.0) / (img_h / 2.0)
        # ukuran marker di gambar (rata-rata panjang sisi, piksel) — proxy jarak
        self.side_px = float(np.mean([
            np.linalg.norm(c[i] - c[(i + 1) % 4]) for i in range(4)
        ]))
        # pose metrik (diisi bila kalibrasi tersedia): x,y,z meter di frame kamera
        self.tvec = None
        self.rvec = None

    def centered(self, tol=0.08):
        return abs(self.off_x) <= tol and abs(self.off_y) <= tol

    def __repr__(self):
        s = "Marker(id=%d, center=(%.0f,%.0f), off=(%.2f,%.2f), side=%.0fpx" % (
            self.id, self.cx, self.cy, self.off_x, self.off_y, self.side_px)
        if self.tvec is not None:
            s += ", xyz=(%.2f,%.2f,%.2f)m" % tuple(self.tvec)
        return s + ")"


class ArucoMarkerDetector:
    def __init__(self, dictionary="DICT_7X7_50", marker_length=0.0,
                 camera_matrix=None, dist_coeffs=None, valid_ids=None,
                 invert=False, poly_accuracy_rate=None,
                 adaptive_thresh_win_max=None, adaptive_thresh_win_step=None,
                 adaptive_thresh_constant=None):
        if dictionary not in _DICTS:
            raise ValueError("dictionary tak dikenal: %s (pilihan: %s)"
                             % (dictionary, list(_DICTS)))
        self.dictionary = aruco.getPredefinedDictionary(_DICTS[dictionary])
        self.params = aruco.DetectorParameters()
        # sub-pixel refinement: align lebih presisi
        self.params.cornerRefinementMethod = aruco.CORNER_REFINE_SUBPIX
        # marker bisa mepet tepi frame (kamera bawah dekat terpal)
        self.params.minDistanceToBorder = 1
        # ponytail: diturunkan dari 0.02 -> marker kecil di frame tetap kebaca;
        # naikkan lagi kalau muncul false-positive dari noise
        self.params.minMarkerPerimeterRate = 0.01
        # poly_accuracy_rate: opt-in, default None = perilaku cv2 asli (gak
        # berubah, biar aruco_node.py 19 Juli TETAP APA ADANYA per permintaan
        # user). Kalau di-set, override polygonalApproxAccuracyRate (default
        # cv2 ~0.03) -- dinaikkan (mis. 0.06) bikin approxPolyDP lebih toleran
        # ke bentuk quad yg gak rapi (lipatan tarp dll). Divalidasi 2026-08-10:
        # tuning+holdout dataset WP1/WP3 (Aruco_yang_baru), 0.06 = titik optimal
        # (0.10 kelewat longgar, hasil malah turun) -- lih. project_task3_status.md.
        if poly_accuracy_rate is not None:
            self.params.polygonalApproxAccuracyRate = float(poly_accuracy_rate)
        # adaptive_thresh_win_max/step: opt-in juga (default None = perilaku cv2
        # asli, default win_max=23/step=10). Literatur resmi OpenCV: "if missing
        # markers, increase winSizeMax" -- window adaptive-threshold lebih lebar
        # & langkah lebih halus (step kecil) bikin lebih banyak kombinasi window
        # dicoba, nolongin kontur yg pecah krn lipatan tarp/pantulan cahaya gak
        # rata (sebelum tahap approxPolyDP/poly_accuracy_rate di atas). Divalidasi
        # 2026-08-10 round 2 (win_max=45, win_step=5): holdout WP1 18.2%->26.8%,
        # WP3 77.2%->78.8%, presisi 100% keduanya -- lih. project_task3_status.md.
        if adaptive_thresh_win_max is not None:
            self.params.adaptiveThreshWinSizeMax = int(adaptive_thresh_win_max)
        if adaptive_thresh_win_step is not None:
            self.params.adaptiveThreshWinSizeStep = int(adaptive_thresh_win_step)
        # adaptive_thresh_constant: opt-in (default None = cv2 asli, default=7).
        # Round 3 (2026-08-10): dinaikkan ke 10 -- konstanta lebih besar =
        # ambang biner lebih longgar, nolongin kontur pecah krn kontras lokal
        # rendah (lipatan/pantulan). Divalidasi: holdout WP1 26.8%->30.4%,
        # WP3 78.8%->83.4%, presisi 100% keduanya -- lih. project_task3_status.md.
        if adaptive_thresh_constant is not None:
            self.params.adaptiveThreshConstant = float(adaptive_thresh_constant)
        self.detector = aruco.ArucoDetector(self.dictionary, self.params)
        self.marker_length = float(marker_length)         # meter (untuk pose)
        self.camera_matrix = camera_matrix
        self.dist_coeffs = dist_coeffs
        # allowlist ID: hanya marker ini yang dianggap valid (buang false-positive).
        # None = terima semua. Lomba: {1,2,3,4} (WP1=1, WP2=2, WP3=3, WP4=4).
        self.valid_ids = set(valid_ids) if valid_ids else None
        # invert=True: marker lapangan putih-di-hitam (terbalik dari standar hitam-di-putih)
        self.invert = bool(invert)

    def has_calibration(self):
        return (self.camera_matrix is not None and self.marker_length > 0.0)

    def _detect_raw(self, gray):
        """Return (corners, ids) dari grayscale siap-deteksi."""
        return self.detector.detectMarkers(gray)

    def detect(self, image):
        """image: BGR atau grayscale numpy. Return list[Marker]."""
        if image.ndim == 3:
            gray = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)
        else:
            gray = image
        h, w = gray.shape[:2]
        if self.invert:
            gray = 255 - gray
            # quiet zone: pad putih, lalu geser corner kembali ke frame asli
            pad = 40
            gpad = cv2.copyMakeBorder(gray, pad, pad, pad, pad,
                                      cv2.BORDER_CONSTANT, value=255)
            corners, ids, _ = self._detect_raw(gpad)
            if ids is not None and corners is not None:
                off = np.array([[[pad, pad]]], dtype=np.float32)
                corners = [c.astype(np.float32) - off for c in corners]
        else:
            corners, ids, _ = self._detect_raw(gray)
        markers = []
        if ids is None:
            return markers
        for i, mid in enumerate(ids.flatten()):
            if self.valid_ids is not None and int(mid) not in self.valid_ids:
                continue                                  # buang ID di luar allowlist
            m = Marker(mid, corners[i], w, h)
            if self.has_calibration():
                self._estimate_pose(m)
            markers.append(m)
        # bila banyak (besar+kecil id sama), urut side_px terbesar dulu
        markers.sort(key=lambda m: m.side_px, reverse=True)
        return markers

    def _estimate_pose(self, m):
        """Isi tvec/rvec (meter) lewat solvePnP; perlu marker_length & kalibrasi."""
        half = self.marker_length / 2.0
        # titik objek marker di frame marker (z=0), urutan sama dgn corners aruco
        obj = np.array([[-half, half, 0], [half, half, 0],
                        [half, -half, 0], [-half, -half, 0]], dtype=np.float32)
        img_pts = m.corners.reshape(4, 2).astype(np.float32)
        ok, rvec, tvec = cv2.solvePnP(obj, img_pts, self.camera_matrix,
                                      self.dist_coeffs, flags=cv2.SOLVEPNP_IPPE_SQUARE)
        if ok:
            m.rvec = rvec.flatten()
            m.tvec = tvec.flatten()

    def draw(self, image, markers):
        """Gambar anotasi untuk debug (return salinan BGR)."""
        out = image.copy() if image.ndim == 3 else cv2.cvtColor(image, cv2.COLOR_GRAY2BGR)
        h, w = out.shape[:2]
        cv2.drawMarker(out, (w // 2, h // 2), (255, 0, 0), cv2.MARKER_CROSS, 20, 2)
        for m in markers:
            pts = m.corners.reshape(4, 2).astype(int)
            cv2.polylines(out, [pts], True, (0, 255, 0), 2)
            cv2.circle(out, (int(m.cx), int(m.cy)), 4, (0, 0, 255), -1)
            cv2.putText(out, "id=%d" % m.id, (pts[0][0], pts[0][1] - 8),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 0), 2)
        return out


def _make_synthetic(dictionary_name, marker_id, side=200, pad=80):
    """Buat gambar marker sintetis (marker putih di tengah kanvas) untuk self-test."""
    d = aruco.getPredefinedDictionary(_DICTS[dictionary_name])
    if hasattr(aruco, "generateImageMarker"):
        marker = aruco.generateImageMarker(d, marker_id, side)     # OpenCV >=4.7
    else:
        marker = aruco.drawMarker(d, marker_id, side)              # OpenCV lama
    canvas = np.full((side + 2 * pad, side + 2 * pad), 255, dtype=np.uint8)
    canvas[pad:pad + side, pad:pad + side] = marker
    return canvas


def _selftest():
    print("== ArUco self-test (marker sintetis, tanpa kamera) ==")
    det = ArucoMarkerDetector(dictionary="DICT_7X7_250")
    ok_all = True
    for test_id in (0, 7, 42, 123):
        img = _make_synthetic("DICT_7X7_250", test_id)
        found = det.detect(img)
        ids = [m.id for m in found]
        passed = ids == [test_id]
        # marker di tengah kanvas -> offset harus ~0
        centered = bool(found) and found[0].centered(tol=0.05)
        print("  id=%-3d -> terdeteksi=%s center_ok=%s %s"
              % (test_id, ids, centered, "OK" if (passed and centered) else "GAGAL"))
        ok_all = ok_all and passed and centered
    # uji negatif: kanvas kosong -> tidak ada deteksi
    empty = det.detect(np.full((300, 300), 255, dtype=np.uint8))
    neg_ok = (empty == [])
    print("  kanvas kosong -> %d deteksi %s" % (len(empty), "OK" if neg_ok else "GAGAL"))
    ok_all = ok_all and neg_ok
    print("HASIL:", "SEMUA LULUS ✅" if ok_all else "ADA YANG GAGAL ❌")
    return ok_all


def _main():
    import argparse
    ap = argparse.ArgumentParser()
    ap.add_argument("--selftest", action="store_true")
    ap.add_argument("--image")
    ap.add_argument("--camera", type=int)
    ap.add_argument("--dict", default="DICT_7X7_50")
    ap.add_argument("--invert", action="store_true",
                    help="marker putih-di-hitam (cetakan terbalik di lapangan)")
    ap.add_argument("--save", help="simpan gambar anotasi ke path ini")
    args = ap.parse_args()

    if args.selftest:
        import sys
        sys.exit(0 if _selftest() else 1)

    det = ArucoMarkerDetector(dictionary=args.dict, invert=args.invert)
    if args.image:
        img = cv2.imread(args.image)
        if img is None:
            print("gagal baca:", args.image)
            return
        markers = det.detect(img)
        print("terdeteksi %d marker:" % len(markers))
        for m in markers:
            print("  ", m)
        if args.save:
            cv2.imwrite(args.save, det.draw(img, markers))
            print("anotasi disimpan:", args.save)
    elif args.camera is not None:
        cap = cv2.VideoCapture(args.camera)
        while True:
            ret, frame = cap.read()
            if not ret:
                break
            markers = det.detect(frame)
            vis = det.draw(frame, markers)
            cv2.imshow("aruco", vis)
            if cv2.waitKey(1) & 0xFF == 27:
                break
        cap.release()
        cv2.destroyAllWindows()
    else:
        print("pakai --selftest | --image FILE | --camera N")


if __name__ == "__main__":
    _main()

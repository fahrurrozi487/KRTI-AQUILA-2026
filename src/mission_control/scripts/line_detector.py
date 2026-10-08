#!/usr/bin/env python3
"""
Detektor garis putus-putus (OpenCV murni, TANPA ROS) — kamera bawah,
buat leg line-follower WP4->WP5 (lih. docs/, masih nunggu konfirmasi panitia
soal misi ini beneran ada di final atau enggak -- detector ini disiapin duluan
biar tinggal tuning begitu dataset/kejelasan misi ada).

RIWAYAT PENTING (2026-08-29): versi pertama pakai threshold V doang (dark_max),
asumsi "garis hitam = akromatik, cuma V yang berarti". SALAH buat material
fisik beneran: dataset asli (line_following/, 462 foto, tarp abu-abu GLOSSY
di rumput kering) nunjukin V tarp (median ~128-166) TUMPANG TINDIH sama V
rumput (median ~120-125) -- tarp mantul cahaya matahari (glossy), gak
matte-hitam kayak diasumsikan. Analisis 25 foto: H (Hue) tarp median=103
(rentang p1-p99: 87-107) vs H rumput median=26 (rentang p1-p99: 18-36,
maks 49) -- terpisah bersih, margin 38 poin. Makanya sekarang HSV_lo/hi
6 angka (pola SAMA gate_detector.py), BUKAN dark_max 1 angka lagi.

⚠️ Default HSV_LO/HI di bawah dikalibrasi dari rumput KERING/COKLAT (dataset
lokal, bukan lapangan final IKN). Rumput IKN yang lebih hijau bisa geser H
rumput ke atas (mendekati tarp) -- WAJIB validasi ulang pakai foto/kondisi
IKN sebelum lomba, jangan asumsikan angka ini otomatis benar di lapangan lain.

Output (LineResult):
  detected  : ada segmen garis yg cukup besar?
  off_x, off_y : offset ternormalisasi centroid segmen (-1..1; + kanan/bawah)
  area_frac : fraksi piksel tarp di frame (proxy ukuran/jarak segmen)
  angle_deg : arah dash, (-90,90], 0=horizontal. Ambigu 180 deg
  bbox      : (x,y,w,h) kotak segmen terpilih

Jalankan mandiri:
  python3 line_detector.py --selftest
  python3 line_detector.py --image frame.png --save out.png
  python3 line_detector.py --camera 0
"""
import math
import numpy as np
import cv2

# Hasil tune_line.py (batch, split tuning=370/holdout=92, seed=42, 2026-08-29):
# margin=0 (H 87-107 mentah, dari analisis 25-foto awal) SUDAH plateau recall
# 100% di tuning MAUPUN holdout -- margin lebih lebar gak nambah recall, cuma
# nambah resiko nyenggol rumput.
#
# RIWAYAT S_lo (2026-08-29, lanjutan investigasi "aman gak di rumput hijau
# IKN?"): dicek rumput kering kampus (H 18-49) DAN rumput Zoysia matrella asli
# (foto Wikimedia Commons rumput terawat + foto venue IKN sendiri di buku
# panduan, H 35-78 tergantung sumber) -- SEMUA jauh di bawah H=87, rumput
# BUKAN sumber masalah. Yang KETEMU beneran jadi false-positive (foto real,
# bukan simulasi): permukaan abu-abu desaturasi non-organik BERBAYANG --
# kerikil/aspal, & tiang/papan metal -- signature-nya mirip tarp (H nyasar
# ke 87-107 krn gelap+desaturasi, bukan krn warnanya beneran mirip).
# S_lo dinaikkan 25->35 (S kerikil FP p90=64, S tarp asli p50=75, p1=34):
# HILANGKAN false-positive kerikil TANPA korbanin recall (tetap 100% di
# tuning MAUPUN holdout, diverifikasi ulang). S_lo=65 akan hilangkan JUGA
# FP tiang/papan metal, tapi recall anjlok ke 81.7% -- gak worth trade-off-nya
# (tiang/papan metal kemungkinan lebih jarang persis di jalur terbang
# dibanding kerikil/paving yang emang ada di dekat venue -- Plaza Timur/
# Barat). Residual risk tiang/papan metal BELUM ditangani -- mitigasi
# operasional: hindari terbang deket struktur metal tipis/tiang.
HSV_LO = (87, 35, 0)
HSV_HI = (107, 255, 255)


class LineResult:
    def __init__(self):
        self.detected = False
        self.off_x = 0.0
        self.off_y = 0.0
        self.area_frac = 0.0
        self.bbox = (0, 0, 0, 0)
        self.angle_deg = 0.0     # arah dash, (-90,90], 0=horizontal. Ambigu 180 deg
                                  # (garis gak punya "depan/belakang") -- lih. _selftest.

    def __repr__(self):
        return ("LineResult(detected=%s, off=(%.2f,%.2f), area=%.1f%%, angle=%.1f)" %
                (self.detected, self.off_x, self.off_y, self.area_frac * 100, self.angle_deg))


class LineDetector:
    def __init__(self, hsv_lo=HSV_LO, hsv_hi=HSV_HI, min_area_frac=0.01,
                 hsv_lo2=None, hsv_hi2=None):
        """hsv_lo2/hsv_hi2 (opsional): rentang FALLBACK ke-2, dicoba HANYA kalau
        rentang utama gagal deteksi (pola sama gate_multi_detector.py CLAHE
        fallback -- gak nambah beban tiap frame kalau rentang utama sukses,
        beda dari HSV_FALLBACK_VARIANTS WP decoder yg coba semua config tiap
        frame & DITOLAK krn latensi). BELUM diisi angka default -- nunggu
        data rumput venue ke-2 (mis. IKN, Zoysia matrella) tervalidasi lewat
        tune_line.py; jangan isi tebakan literatur, resiko nabrak H tarp
        (87-107), lih. diskusi 2026-08-29."""
        self.hsv_lo = np.array(hsv_lo, np.uint8)
        self.hsv_hi = np.array(hsv_hi, np.uint8)
        self.hsv_lo2 = np.array(hsv_lo2, np.uint8) if hsv_lo2 is not None else None
        self.hsv_hi2 = np.array(hsv_hi2, np.uint8) if hsv_hi2 is not None else None
        self.min_area_frac = float(min_area_frac)

    def _raw_mask(self, image, lo, hi):
        hsv = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)
        m = cv2.inRange(hsv, lo, hi)
        k = cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (5, 5))
        m = cv2.morphologyEx(m, cv2.MORPH_OPEN, k)
        m = cv2.morphologyEx(m, cv2.MORPH_CLOSE, k)
        return m

    def mask(self, image):
        """Return mask tarp (uint8 0/255) sesudah morphology, rentang utama."""
        return self._raw_mask(image, self.hsv_lo, self.hsv_hi)

    def detect(self, image):
        h, w = image.shape[:2]
        res = LineResult()
        m = self.mask(image)
        cnts, _ = cv2.findContours(m, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        if not cnts and self.hsv_lo2 is not None:
            # fallback: rentang utama nihil -> coba rentang ke-2 (venue lain)
            m = self._raw_mask(image, self.hsv_lo2, self.hsv_hi2)
            cnts, _ = cv2.findContours(m, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        if not cnts:
            return res
        big = max(cnts, key=cv2.contourArea)
        area = cv2.contourArea(big)
        res.area_frac = area / float(w * h)
        if res.area_frac < self.min_area_frac:
            return res
        gx, gy, gw, gh = cv2.boundingRect(big)
        res.bbox = (gx, gy, gw, gh)
        cx, cy = gx + gw / 2.0, gy + gh / 2.0
        res.off_x = (cx - w / 2.0) / (w / 2.0)
        res.off_y = (cy - h / 2.0) / (h / 2.0)
        # arah dash: best-fit line lewat titik-titik kontur (cocok buat dash
        # yg memanjang, dash bundar/kotak -> arahnya kurang berarti tapi gak error)
        vx, vy, _, _ = cv2.fitLine(big, cv2.DIST_L2, 0, 0.01, 0.01).flatten()
        angle = math.degrees(math.atan2(vy, vx)) % 180.0
        res.angle_deg = angle - 180.0 if angle > 90.0 else angle
        res.detected = True
        return res

    def draw(self, image, res):
        out = image.copy()
        h, w = out.shape[:2]
        cv2.drawMarker(out, (w // 2, h // 2), (255, 0, 0), cv2.MARKER_CROSS, 20, 2)
        gx, gy, gw, gh = res.bbox
        if gw > 0:
            cv2.rectangle(out, (gx, gy), (gx + gw, gy + gh), (0, 255, 255), 2)
        if res.detected:
            tx = int(w / 2 + res.off_x * w / 2)
            ty = int(h / 2 + res.off_y * h / 2)
            cv2.circle(out, (tx, ty), 8, (0, 0, 255), -1)
            a = math.radians(res.angle_deg)
            dx, dy = int(40 * math.cos(a)), int(40 * math.sin(a))
            cv2.line(out, (tx - dx, ty - dy), (tx + dx, ty + dy), (0, 255, 0), 2)
        cv2.putText(out, "area=%.1f%% angle=%.0f" % (res.area_frac * 100, res.angle_deg),
                    (10, 25), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 255), 2)
        return out


def _hsv_to_bgr_px(h, s, v):
    """1 piksel HSV (skala OpenCV 0-179/0-255) -> tuple BGR, buat gambar sintetis."""
    px = np.uint8([[[h, s, v]]])
    b, g, r = cv2.cvtColor(px, cv2.COLOR_HSV2BGR)[0, 0]
    return (int(b), int(g), int(r))


# warna sintetis dari median dataset asli: tarp H=103,S=75,V=141 ; rumput H=26,S=110,V=125
_TARP_BGR = _hsv_to_bgr_px(103, 75, 141)
_GRASS_BGR = _hsv_to_bgr_px(26, 110, 125)


def _make_synthetic(w, h, dash_boxes):
    """Bangun garis putus-putus sintetis: rumput di background, dash tarp di
    posisi dash_boxes=[(x0,y0,x1,y1),...]. Return BGR."""
    img = np.full((h, w, 3), _GRASS_BGR, np.uint8)
    for (x0, y0, x1, y1) in dash_boxes:
        cv2.rectangle(img, (x0, y0), (x1, y1), _TARP_BGR, -1)
    return img


def _make_synthetic_rotated(w, h, cx, cy, length, thickness, angle_deg):
    """Dash tarp berbentuk garis tebal miring angle_deg (buat tes deteksi arah)."""
    img = np.full((h, w, 3), _GRASS_BGR, np.uint8)
    a = math.radians(angle_deg)
    dx, dy = math.cos(a) * length / 2, math.sin(a) * length / 2
    p1 = (int(cx - dx), int(cy - dy))
    p2 = (int(cx + dx), int(cy + dy))
    cv2.line(img, p1, p2, _TARP_BGR, thickness)
    return img


def _angle_err(measured, expected):
    """Selisih sudut terpendek, toleran ambiguitas 180 deg (garis gak punya arah)."""
    return abs(((measured - expected + 90) % 180) - 90)


def _selftest():
    print("== Line self-test (garis sintetis warna tarp asli, tanpa kamera) ==")
    det = LineDetector()
    ok = True

    # 1) satu dash di tengah -> terdeteksi, off dekat 0
    W, H = 640, 480
    img1 = _make_synthetic(W, H, [(int(W*0.45), int(H*0.40), int(W*0.55), int(H*0.60))])
    r1 = det.detect(img1)
    c1 = r1.detected and abs(r1.off_x) < 0.1 and abs(r1.off_y) < 0.1
    print("  dash tengah: %s  -> %s" % (r1, "OK" if c1 else "GAGAL"))
    ok = ok and c1

    # 2) dash di kanan-bawah -> off_x>0, off_y>0
    img2 = _make_synthetic(W, H, [(int(W*0.70), int(H*0.65), int(W*0.85), int(H*0.80))])
    r2 = det.detect(img2)
    c2 = r2.detected and r2.off_x > 0.2 and r2.off_y > 0.2
    print("  dash kanan-bawah: %s  -> %s" % (r2, "OK" if c2 else "GAGAL"))
    ok = ok and c2

    # 3) gap (di antara dash, cuma rumput dalam frame) -> tidak terdeteksi
    r3 = det.detect(np.full((H, W, 3), _GRASS_BGR, np.uint8))
    c3 = not r3.detected
    print("  gap (rumput saja): %s  -> %s" % (r3, "OK" if c3 else "GAGAL"))
    ok = ok and c3

    # 4) dua dash sekaligus kelihatan -> ambil yg kontur terbesar (bukan crash)
    img4 = _make_synthetic(W, H, [(int(W*0.10), int(H*0.10), int(W*0.20), int(H*0.20)),
                                   (int(W*0.40), int(H*0.40), int(W*0.60), int(H*0.60))])
    r4 = det.detect(img4)
    c4 = r4.detected
    print("  dua dash: %s  -> %s" % (r4, "OK" if c4 else "GAGAL"))
    ok = ok and c4

    # 5) arah dash: garis miring 30 deg -> angle_deg terdeteksi harus dekat 30
    img5 = _make_synthetic_rotated(W, H, W // 2, H // 2, 220, 18, 30)
    r5 = det.detect(img5)
    c5 = r5.detected and _angle_err(r5.angle_deg, 30) < 5
    print("  arah 30 deg: %s  -> %s" % (r5, "OK" if c5 else "GAGAL"))
    ok = ok and c5

    # 6) arah dash: garis miring -60 deg (curam) -> angle_deg dekat -60
    img6 = _make_synthetic_rotated(W, H, W // 2, H // 2, 220, 18, -60)
    r6 = det.detect(img6)
    c6 = r6.detected and _angle_err(r6.angle_deg, -60) < 5
    print("  arah -60 deg: %s  -> %s" % (r6, "OK" if c6 else "GAGAL"))
    ok = ok and c6

    # 7) arah dash: hampir horizontal (0 deg)
    img7 = _make_synthetic_rotated(W, H, W // 2, H // 2, 220, 18, 0)
    r7 = det.detect(img7)
    c7 = r7.detected and _angle_err(r7.angle_deg, 0) < 5
    print("  arah 0 deg: %s  -> %s" % (r7, "OK" if c7 else "GAGAL"))
    ok = ok and c7

    # 8) fallback venue ke-2: rentang utama nihil (dash warna beda total, di
    # luar H 87-107), rentang ke-2 (dipasang manual buat tes ini, H 40-60,
    # simulasi "warna venue lain") -> tetap kedeteksi lewat fallback
    det_fb = LineDetector(hsv_lo=HSV_LO, hsv_hi=HSV_HI, hsv_lo2=(40, 25, 0), hsv_hi2=(60, 255, 255))
    other_bgr = _hsv_to_bgr_px(50, 110, 130)   # dash "venue lain", H=50 (di luar rentang utama)
    img8 = np.full((H, W, 3), _GRASS_BGR, np.uint8)
    cv2.rectangle(img8, (int(W*0.45), int(H*0.40)),
                  (int(W*0.55), int(H*0.60)), other_bgr, -1)
    r8_no_fb = det.detect(img8)          # detector TANPA fallback -> harus nihil
    r8_fb = det_fb.detect(img8)          # detector DENGAN fallback -> harus ketemu
    c8 = (not r8_no_fb.detected) and r8_fb.detected
    print("  fallback venue-2: tanpa_fb=%s dgn_fb=%s  -> %s" %
          (r8_no_fb.detected, r8_fb, "OK" if c8 else "GAGAL"))
    ok = ok and c8

    print("HASIL:", "SEMUA LULUS ✅" if ok else "ADA YANG GAGAL ❌")
    return ok


def _triple(s, default):
    try:
        parts = [int(x) for x in str(s).split(",")]
        return tuple(parts) if len(parts) == 3 else default
    except ValueError:
        return default


def _main():
    import argparse
    ap = argparse.ArgumentParser()
    ap.add_argument("--selftest", action="store_true")
    ap.add_argument("--image")
    ap.add_argument("--camera", type=int)
    ap.add_argument("--save")
    ap.add_argument("--hsv_lo", default="87,35,0")
    ap.add_argument("--hsv_hi", default="107,255,255")
    args = ap.parse_args()

    if args.selftest:
        import sys
        sys.exit(0 if _selftest() else 1)

    lo = _triple(args.hsv_lo, HSV_LO)
    hi = _triple(args.hsv_hi, HSV_HI)
    det = LineDetector(hsv_lo=lo, hsv_hi=hi)
    if args.image:
        img = cv2.imread(args.image)
        if img is None:
            print("gagal baca:", args.image); return
        r = det.detect(img)
        print(r)
        if args.save:
            cv2.imwrite(args.save, det.draw(img, r))
            print("anotasi disimpan:", args.save)
    elif args.camera is not None:
        cap = cv2.VideoCapture(args.camera)
        while True:
            ret, frame = cap.read()
            if not ret:
                break
            r = det.detect(frame)
            cv2.imshow("line", det.draw(frame, r))
            if cv2.waitKey(1) & 0xFF == 27:
                break
        cap.release(); cv2.destroyAllWindows()
    else:
        print("pakai --selftest | --image FILE | --camera N")


if __name__ == "__main__":
    _main()

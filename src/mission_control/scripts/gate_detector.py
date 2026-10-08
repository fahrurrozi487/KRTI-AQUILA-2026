#!/usr/bin/env python3
"""
Detektor gate oranye (OpenCV murni, TANPA ROS) — inti Fase C untuk kamera depan.

Gate = struktur oranye solid (spec R233 G146 B17 ~ HSV 18,236,233) berbentuk
terowongan yang dilewati drone. Drone harus mengincar BUKAAN (celah), bukan
pusat oranye — pada double gate, pusat massa oranye = panel pembagi (kalau
diincar -> nabrak). Maka bukaan dicari lewat profil kolom: kolom dgn sedikit
oranye = bukaan; jumlah celah membedakan double vs triple.

Output (GateResult):
  detected     : ada gate yg cukup besar?
  off_x, off_y : offset ternormalisasi pusat BUKAAN terpilih (-1..1; + kanan/bawah)
  area_frac    : fraksi piksel oranye di frame (proxy kedekatan/jarak)
  num_openings : jumlah celah terdeteksi (1=single, 2=double, dst)
  opening_w_frac: lebar bukaan terpilih / lebar gambar
  bbox         : (x,y,w,h) kotak oranye keseluruhan

Jalankan mandiri:
  python3 gate_detector.py --selftest
  python3 gate_detector.py --image gate.png --save out.png
  python3 gate_detector.py --camera 0
"""
import numpy as np
import cv2


class GateResult:
    def __init__(self):
        self.detected = False
        self.off_x = 0.0
        self.off_y = 0.0
        self.area_frac = 0.0
        self.num_openings = 0
        self.opening_w_frac = 0.0
        self.bbox = (0, 0, 0, 0)
        self.openings = []          # list (cx,cy,w) piksel, semua celah

    def __repr__(self):
        return ("GateResult(detected=%s, off=(%.2f,%.2f), area=%.1f%%, "
                "openings=%d, op_w=%.2f)" % (self.detected, self.off_x, self.off_y,
                 self.area_frac * 100, self.num_openings, self.opening_w_frac))


class GateDetector:
    def __init__(self, hsv_lo=(8, 80, 80), hsv_hi=(30, 255, 255),
                 min_area_frac=0.02, open_col_thresh=0.20, min_gap_frac=0.08,
                 min_open_img_frac=0.05, keep_frac_of_max=0.35):
        self.hsv_lo = np.array(hsv_lo, np.uint8)
        self.hsv_hi = np.array(hsv_hi, np.uint8)
        self.min_area_frac = float(min_area_frac)
        self.open_col_thresh = float(open_col_thresh)
        self.min_gap_frac = float(min_gap_frac)
        self.min_open_img_frac = float(min_open_img_frac)
        self.keep_frac_of_max = float(keep_frac_of_max)

    def mask(self, image):
        """Return mask oranye (uint8 0/255) sesudah morphology."""
        hsv = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)
        m = cv2.inRange(hsv, self.hsv_lo, self.hsv_hi)
        k = cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (5, 5))
        m = cv2.morphologyEx(m, cv2.MORPH_OPEN, k)
        m = cv2.morphologyEx(m, cv2.MORPH_CLOSE, k)
        return m

    def detect(self, image):
        h, w = image.shape[:2]
        res = GateResult()
        m = self.mask(image)
        res.area_frac = float(m.mean() / 255.0)
        if res.area_frac < self.min_area_frac:
            return res

        cnts, _ = cv2.findContours(m, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        if not cnts:
            return res
        big = max(cnts, key=cv2.contourArea)
        gx, gy, gw, gh = cv2.boundingRect(big)
        res.bbox = (gx, gy, gw, gh)
        if gw < 10 or gh < 10:
            return res

        # profil kolom di bagian BAWAH gate (di bawah balok atas)
        y0 = gy + int(0.35 * gh)
        band = m[y0:gy + gh, gx:gx + gw]
        col_cov = band.mean(axis=0) / 255.0
        open_cols = col_cov < self.open_col_thresh

        min_gap_px = max(8, int(self.min_gap_frac * gw),
                         int(self.min_open_img_frac * w))
        gaps = []
        start = None
        for i, o in enumerate(open_cols):
            if o and start is None:
                start = i
            elif not o and start is not None:
                if i - start >= min_gap_px:
                    gaps.append((start, i))
                start = None
        if start is not None and len(open_cols) - start >= min_gap_px:
            gaps.append((start, len(open_cols)))
        if not gaps:
            return res

        max_w = max(b - a for a, b in gaps)
        gaps = [(a, b) for a, b in gaps
                if (b - a) >= self.keep_frac_of_max * max_w]
        if not gaps:
            return res

        res.num_openings = len(gaps)
        for (a, b) in gaps:
            cx = gx + (a + b) / 2.0
            res.openings.append((cx, y0 + (gy + gh - y0) / 2.0, b - a))

        a, b = max(gaps, key=lambda g: g[1] - g[0])
        cx = gx + (a + b) / 2.0
        cy = y0 + (gy + gh - y0) / 2.0
        res.detected = True
        res.off_x = (cx - w / 2.0) / (w / 2.0)
        res.off_y = (cy - h / 2.0) / (h / 2.0)
        res.opening_w_frac = (b - a) / float(w)
        return res

    def draw(self, image, res):
        out = image.copy()
        h, w = out.shape[:2]
        cv2.drawMarker(out, (w // 2, h // 2), (255, 0, 0), cv2.MARKER_CROSS, 20, 2)
        gx, gy, gw, gh = res.bbox
        if gw > 0:
            cv2.rectangle(out, (gx, gy), (gx + gw, gy + gh), (0, 165, 255), 2)
        for (cx, cy, gwid) in res.openings:
            cv2.line(out, (int(cx), gy), (int(cx), gy + gh), (0, 255, 0), 2)
        if res.detected:
            tx = int(w / 2 + res.off_x * w / 2)
            ty = int(h / 2 + res.off_y * h / 2)
            cv2.circle(out, (tx, ty), 8, (0, 0, 255), -1)
            cv2.putText(out, "TARGET", (tx + 10, ty),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 0, 255), 2)
        cv2.putText(out, "openings=%d area=%.0f%%" % (res.num_openings, res.area_frac * 100),
                    (10, 25), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 255), 2)
        return out


def _make_synthetic(w, h, gaps_x):
    """Bangun gate sintetis: persegi oranye penuh, lalu 'lubangi' celah vertikal
    (terbuka ke bawah) pada posisi gaps_x=[(x0,x1),...]. Return BGR."""
    img = np.full((h, w, 3), 30, np.uint8)            # latar gelap
    orange = (17, 146, 233)                            # BGR dari R233G146B17
    # struktur oranye: kotak besar di tengah
    gx0, gy0, gx1, gy1 = int(w*0.2), int(h*0.15), int(w*0.8), int(h*0.85)
    cv2.rectangle(img, (gx0, gy0), (gx1, gy1), orange, -1)
    # lubangi celah (dari bawah balok atas ke dasar) = bukaan
    top = gy0 + int(0.25 * (gy1 - gy0))
    for (xa, xb) in gaps_x:
        cv2.rectangle(img, (xa, top), (xb, gy1), (30, 30, 30), -1)
    return img


def _selftest():
    print("== Gate self-test (gate sintetis, tanpa kamera) ==")
    det = GateDetector()
    ok = True

    # 1) single gate: 1 celah di tengah
    W, H = 640, 480
    img1 = _make_synthetic(W, H, [(int(W*0.42), int(W*0.58))])
    r1 = det.detect(img1)
    c1 = r1.detected and r1.num_openings == 1 and abs(r1.off_x) < 0.08
    print("  single: %s  -> %s" % (r1, "OK" if c1 else "GAGAL"))
    ok = ok and c1

    # 2) double gate: 2 celah (kiri & kanan pembagi tengah)
    img2 = _make_synthetic(W, H, [(int(W*0.28), int(W*0.42)),
                                   (int(W*0.58), int(W*0.72))])
    r2 = det.detect(img2)
    # target harus salah satu celah (off_x != 0), BUKAN pusat (pembagi)
    c2 = r2.detected and r2.num_openings == 2 and abs(r2.off_x) > 0.1
    print("  double: %s  -> %s" % (r2, "OK" if c2 else "GAGAL"))
    ok = ok and c2

    # 3) tanpa gate: frame gelap -> tidak terdeteksi
    r3 = det.detect(np.full((H, W, 3), 30, np.uint8))
    c3 = not r3.detected
    print("  kosong: %s  -> %s" % (r3, "OK" if c3 else "GAGAL"))
    ok = ok and c3

    print("HASIL:", "SEMUA LULUS ✅" if ok else "ADA YANG GAGAL ❌")
    return ok


def _main():
    import argparse
    ap = argparse.ArgumentParser()
    ap.add_argument("--selftest", action="store_true")
    ap.add_argument("--image")
    ap.add_argument("--camera", type=int)
    ap.add_argument("--save")
    args = ap.parse_args()

    if args.selftest:
        import sys
        sys.exit(0 if _selftest() else 1)

    det = GateDetector()
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
            cv2.imshow("gate", det.draw(frame, r))
            if cv2.waitKey(1) & 0xFF == 27:
                break
        cap.release(); cv2.destroyAllWindows()
    else:
        print("pakai --selftest | --image FILE | --camera N")


if __name__ == "__main__":
    _main()

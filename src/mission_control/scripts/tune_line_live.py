#!/usr/bin/env python3
"""
Tuner interaktif dark_max buat line_detector.py -- BELUM ada dataset, jadi ini
trackbar live (bukan tune_line.py batch/anti-overfit kayak tune_step1.py, yang
nunggu dataset numpuk lebih dari 1 sesi foto sebelum dibikin).

Geser slider dark_max sambil lihat mask + centroid + arah update real-time,
berhenti begitu garis kepisah bersih dari background di semua sample yang ada.

Pakai:
  python3 tune_line_live.py --image frame.png
  python3 tune_line_live.py --image "dataset_line/*.jpg"   # banyak file -> panah/space ganti
  python3 tune_line_live.py --camera 0                     # langsung dari kamera bawah

Kontrol:
  slider dark_max : geser buat ubah ambang
  n / SPACE        : gambar berikutnya (mode --image dgn banyak file)
  p                : gambar sebelumnya
  ESC / q          : keluar, cetak dark_max terakhir
"""
import argparse
import glob
import sys
import cv2

sys.path.insert(0, __file__.rsplit("/", 1)[0])
from line_detector import LineDetector


def _run(images_fn, get_dark_max):
    """images_fn: () -> list gambar BGR (statis, dari file) ATAU None (pakai --camera).
    Kalau images_fn None, jalan mode kamera langsung."""
    cv2.namedWindow("tune_line")
    cv2.createTrackbar("dark_max", "tune_line", 70, 255, lambda v: None)

    if images_fn is not None:
        imgs = images_fn()
        if not imgs:
            print("Tidak ada gambar terbaca."); return None
        idx = 0
        while True:
            dark_max = cv2.getTrackbarPos("dark_max", "tune_line")
            det = LineDetector(dark_max=dark_max)
            r = det.detect(imgs[idx])
            vis = det.draw(imgs[idx], r)
            cv2.putText(vis, "[%d/%d] n/p ganti, q keluar" % (idx + 1, len(imgs)),
                        (10, vis.shape[0] - 12), cv2.FONT_HERSHEY_SIMPLEX, 0.5,
                        (255, 255, 0), 1)
            cv2.imshow("tune_line", vis)
            k = cv2.waitKey(30) & 0xFF
            if k in (ord('q'), 27):
                break
            elif k in (ord('n'), ord(' ')):
                idx = (idx + 1) % len(imgs)
            elif k == ord('p'):
                idx = (idx - 1) % len(imgs)
        cv2.destroyAllWindows()
        return cv2.getTrackbarPos("dark_max", "tune_line")

    # mode kamera langsung
    cap = cv2.VideoCapture(0)
    while True:
        ret, frame = cap.read()
        if not ret:
            break
        dark_max = cv2.getTrackbarPos("dark_max", "tune_line")
        det = LineDetector(dark_max=dark_max)
        r = det.detect(frame)
        cv2.imshow("tune_line", det.draw(frame, r))
        k = cv2.waitKey(1) & 0xFF
        if k in (ord('q'), 27):
            break
    cap.release()
    cv2.destroyAllWindows()
    return cv2.getTrackbarPos("dark_max", "tune_line")


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--image", help="path 1 file ATAU glob pattern (\"dir/*.jpg\")")
    ap.add_argument("--camera", type=int, help="index kamera, mis. 0")
    args = ap.parse_args()

    if args.image:
        paths = sorted(glob.glob(args.image)) if any(c in args.image for c in "*?[") \
            else [args.image]
        imgs = [im for p in paths if (im := cv2.imread(p)) is not None]
        final = _run(lambda: imgs, None)
    elif args.camera is not None:
        final = _run(None, None)
    else:
        print("pakai --image FILE_ATAU_GLOB | --camera N")
        return

    if final is not None:
        print("dark_max terpilih: %d" % final)
        print("Pakai di line_detect.launch: <param name=\"dark_max\" value=\"%d\" />" % final)


if __name__ == "__main__":
    main()

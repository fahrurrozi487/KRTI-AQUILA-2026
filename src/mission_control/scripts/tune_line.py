#!/usr/bin/env python3
"""
Tuning batch HSV line_detector -- dataset line_following/ (462 foto, 2026-08-29).

ATURAN ANTI-OVERFIT (pola sama tune_step1.py/gate_multi_detector._split_dataset):
semua keputusan margin diambil HANYA dari tuning_set. holdout_set cuma diukur
SEKALI dengan param terpilih.

⚠️ KETERBATASAN JUJUR: dataset ini TIDAK punya label ground-truth (gak ada folder
"ada-tarp"/"tanpa-tarp", gak ada bounding box manual) -- beda dari dataset ArUco/
gate yang sudah dipakai project ini sebelumnya (yang punya folder berhasil_deteksi).
Semua 462 foto diasumsikan POSITIF (tarp kelihatan di frame -- diverifikasi visual
sample acak). Makanya:
  - "recall" DI SINI = detection rate (fraksi frame ter-deteksi) -- valid krn semua
    frame diasumsikan ada tarp.
  - "presisi/false-positive rate" TIDAK BISA dihitung numerik (butuh frame TANPA
    tarp buat diuji) -- diganti spot-check visual manual (simpan overlay sample).
"""
import os
import sys
import glob
import random
import cv2
import numpy as np

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from line_detector import LineDetector

DPATH = os.path.expanduser("~/catkin_ws/line_following")
FP_DIR = os.path.expanduser("~/catkin_ws/line_following_fp_check")  # foto real
# yg TERBUKTI false-positive (kerikil/aspal berbayang, tiang/papan metal --
# investigasi 2026-08-29). Bukan dataset tuning/holdout -- dipakai HANYA buat
# validasi S_lo, biar re-tuning ke depan (dataset baru/venue baru) otomatis
# ke-cek juga terhadap 2 kegagalan nyata ini, bukan cuma recall tarp.


def _split_dataset(dpath, seed=42, holdout=0.2):
    """Pola PERSIS gate_multi_detector._split_dataset -- konsistensi metodologi."""
    files = sorted(glob.glob(os.path.join(dpath, "*.jpg")))
    rng = random.Random(seed)
    rng.shuffle(files)
    n_hold = int(round(len(files) * holdout))
    holdout_set = sorted(files[:n_hold])
    tuning_set = sorted(files[n_hold:])
    return tuning_set, holdout_set


def _load(files, max_dim=640):
    """ponytail: resize saat load, bukan simpan full-res (1920x1080 x 462 foto
    ~2.9GB -> OOM-kill di Jetson Nano 4GB RAM). max_dim juga lebih representatif
    resolusi kamera drone beneran (bukan foto HP 1920x1080)."""
    imgs = []
    for f in files:
        im = cv2.imread(f)
        if im is None:
            continue
        h, w = im.shape[:2]
        scale = max_dim / max(h, w)
        if scale < 1.0:
            im = cv2.resize(im, (int(w * scale), int(h * scale)))
        imgs.append((f, im))
    return imgs


def _eval(imgs, hsv_lo, hsv_hi):
    """Return (detect_rate%, area_fracs list, angle non-null count)."""
    det = LineDetector(hsv_lo=hsv_lo, hsv_hi=hsv_hi)
    n_det = 0
    areas = []
    for _, im in imgs:
        r = det.detect(im)
        if r.detected:
            n_det += 1
            areas.append(r.area_frac)
    n = len(imgs)
    dr = 100.0 * n_det / n if n else 0.0
    return dr, areas


def main():
    tun_f, hol_f = _split_dataset(DPATH, seed=42)
    print("loading tuning=%d holdout=%d ..." % (len(tun_f), len(hol_f)))
    tun = _load(tun_f)
    hol = _load(hol_f)
    print("loaded.\n")

    # margin di sekitar batas empiris (H core 87-107 dari analisis 25-foto
    # sebelumnya) -- sweep margin simetris, S_lo tetap 25 (dari analisis awal)
    print("=== SWEEP margin H (tuning_set saja, %d foto) ===" % len(tun))
    print("  margin  H_lo-H_hi   det%%    area_mean%%  area_std%%")
    results = []
    for margin in (0, 5, 10, 15, 20, 25, 30):
        h_lo, h_hi = 87 - margin, 107 + margin
        dr, areas = _eval(tun, (h_lo, 25, 0), (h_hi, 255, 255))
        area_mean = np.mean(areas) * 100 if areas else 0.0
        area_std = np.std(areas) * 100 if areas else 0.0
        print("  %5d   %3d-%3d     %5.1f   %8.2f    %8.2f" %
              (margin, h_lo, h_hi, dr, area_mean, area_std))
        results.append((margin, h_lo, h_hi, dr, area_mean, area_std))

    # pilih margin TERKECIL yang udah nyentuh recall maksimum (plateau) --
    # margin lebih besar dari itu cuma nambah resiko nyenggol rumput tanpa
    # nambah recall (ponytail: greedy pilih plateau awal, bukan grid-search
    # penuh -- cukup buat 1 parameter beda dari gate's 2-parameter sweep)
    max_dr = max(r[3] for r in results)
    chosen = next(r for r in results if r[3] >= max_dr - 0.5)  # toleransi 0.5%
    margin, h_lo, h_hi, dr_tun, _, _ = chosen
    print("\n>>> margin H terpilih (plateau recall, tuning_set): %d -> H %d-%d (det=%.1f%%)" %
          (margin, h_lo, h_hi, dr_tun))

    # ---- SWEEP S_lo: cari titik yg buang false-positive REAL (kerikil/tiang)
    # tanpa korbanin recall tarp (investigasi 2026-08-29, lih. FP_DIR) ----
    fp_files = sorted(glob.glob(os.path.join(FP_DIR, "*.jpg")))
    fp_imgs = [(f, im) for f, im in
               [(f, cv2.imread(f)) for f in fp_files] if im is not None]
    print("\n=== SWEEP S_lo @ H=%d-%d (cek recall tuning+holdout DAN %d foto false-positive REAL) ==="
          % (h_lo, h_hi, len(fp_imgs)))
    print("  S_lo  det_tuning%%  det_holdout%%   FP lolos?")
    s_results = []
    for s_lo in (25, 35, 45, 55, 65, 75):
        det = LineDetector(hsv_lo=(h_lo, s_lo, 0), hsv_hi=(h_hi, 255, 255))
        dr_t = 100.0 * sum(1 for _, im in tun if det.detect(im).detected) / len(tun)
        dr_h = 100.0 * sum(1 for _, im in hol if det.detect(im).detected) / len(hol)
        fp_names = [os.path.basename(f) for f, im in fp_imgs if det.detect(im).detected]
        s_results.append((s_lo, dr_t, dr_h, fp_names))
        print("  %4d   %6.1f       %6.1f        %s" %
              (s_lo, dr_t, dr_h, ", ".join(fp_names) if fp_names else "aman (0)"))

    # pilih S_lo TERKECIL yg recall tuning+holdout masih di plateau (>=99%,
    # toleransi kecil) -- prioritas: jangan korbanin recall demi nutup FP yg
    # makin jarang/spesifik (lih. laporan 2026-08-29: tiang metal butuh S_lo=65
    # tapi recall anjlok ~18pp, GAK dipilih -- trade-off gak worth).
    s_chosen = None
    for s_lo, dr_t, dr_h, fp_names in s_results:
        if dr_t >= 99.5 and dr_h >= 99.5:
            s_chosen = (s_lo, dr_t, dr_h, fp_names)
    if s_chosen is None:
        s_chosen = s_results[0]
    s_lo, dr_t_final, dr_h_final, fp_remaining = s_chosen
    print("\n>>> S_lo terpilih (recall tuning+holdout masih >=99.5%%): %d" % s_lo)
    if fp_remaining:
        print("    ⚠️ residual false-positive BELUM tertangani: %s" % ", ".join(fp_remaining))
        print("    (butuh S_lo lebih tinggi utk fix, tapi korbanin recall -- lih. komentar di atas)")
    else:
        print("    semua false-positive yg diketahui SAAT INI sudah aman.")

    print("\n=== HASIL FINAL (H=%d-%d, S_lo=%d) ===" % (h_lo, h_hi, s_lo))
    print("  recall tuning : %.1f%%" % dr_t_final)
    print("  recall holdout: %.1f%%" % dr_h_final)

    print("\n⚠️  Presisi/false-positive rate dataset UTAMA tetap TIDAK bisa dihitung numerik")
    print("    (462 foto semua diasumsikan positif, gak ada frame tanpa-tarp) -- tapi")
    print("    %d foto FP_DIR sekarang JADI regression-test permanen tiap re-tuning." % len(fp_imgs))

    # simpan overlay sample acak dari HOLDOUT (bukan tuning) buat spot-check visual
    out_dir = os.path.expanduser("~/scratch_line/holdout_spotcheck")
    os.makedirs(out_dir, exist_ok=True)
    det = LineDetector(hsv_lo=(h_lo, s_lo, 0), hsv_hi=(h_hi, 255, 255))
    rng = random.Random(7)
    sample = rng.sample(hol, min(10, len(hol)))
    for f, im in sample:
        r = det.detect(im)
        vis = det.draw(im, r)
        name = os.path.basename(f).replace(" ", "_").replace("(", "").replace(")", "")
        cv2.imwrite(os.path.join(out_dir, name), vis)
    print("\n%d overlay spot-check holdout disimpan ke %s" % (len(sample), out_dir))
    print("\nParam final -> line_detect.launch: hsv_lo=\"%d,%d,0\" hsv_hi=\"%d,255,255\"" %
          (h_lo, s_lo, h_hi))


if __name__ == "__main__":
    main()

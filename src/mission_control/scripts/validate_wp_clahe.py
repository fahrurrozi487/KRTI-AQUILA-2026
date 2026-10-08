#!/usr/bin/env python3
"""validate_wp_clahe.py — validasi akhir pipeline robust (2026-08-08/09) thd
dataset lapangan asli (dataset_arucode/Aruco_yang_baru/). Bandingkan baseline
decode_frame_v2() vs +decode_frame_v2_robust() (reuse fungsi PRODUKSI
wp_decode_v2.py, BUKAN reimplementasi -- biar hasil tes selalu cerminan kode
yg beneran jalan di wp_marker_node.py, TERMASUK kebijakan allow_clahe per-WP
yg sama persis kayak ~clahe_fallback_wp_ids default "1").

Print per WP: reliable/agreement/n dari referensi JSON, presisi & recall
baseline, presisi & recall +robust, delta, breakdown kegagalan yg TERSISA.

`--resize WxH` (PENTING): dataset diambil kamera C920 di resolusi NATIVE
(1920x1080), TAPI live pipeline (`camera_down.launch`) sengaja stream di
640x480 (Jetson gak kuat resolusi lebih tinggi). Tanpa `--resize`, hasil tes
ini cerminan AKURASI ALGORITMA di foto resolusi tinggi -- BUKAN jaminan sama
persis di kamera drone asli. Pakai `--resize 640x480` biar apples-to-apples.

Jalanin: python3 validate_wp_clahe.py [--base DIR] [--resize 640x480]
"""
import argparse
import json
import os
import sys
from collections import Counter

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
import cv2
from wp_decode_v2 import decode_frame_v2, decode_frame_v2_robust, load_refs_v2

WP_IDS = {'wp1': 1, 'wp3': 3, 'wp4': 4}
# kebijakan allow_clahe SAMA PERSIS default ~clahe_fallback_wp_ids di wp_marker_node.py
CLAHE_ALLOWED = {1}


def eval_folder(folder, true_id, refs, resize=None):
    files = sorted(f for f in os.listdir(folder) if f.lower().endswith(('.jpg', '.jpeg', '.png')))
    n_total = 0
    base_ok = base_correct = 0
    robust_ok = robust_correct = 0
    reason_after = Counter()   # breakdown kegagalan SESUDAH robust (apa yg TERSISA)
    allow_clahe = true_id in CLAHE_ALLOWED
    for f in files:
        bgr = cv2.imread(os.path.join(folder, f))
        if bgr is None:
            continue
        if resize:
            bgr = cv2.resize(bgr, resize)
        n_total += 1
        dec0 = decode_frame_v2(bgr, refs, 7, 3)
        if dec0['reason'] == 'ok':
            base_ok += 1
            if dec0['wp_id'] == true_id:
                base_correct += 1
        dec = decode_frame_v2_robust(bgr, refs, 7, 3, allow_clahe=allow_clahe)
        if dec['reason'] == 'ok':
            robust_ok += 1
            if dec['wp_id'] == true_id:
                robust_correct += 1
        else:
            reason_after[dec['reason']] += 1
    return dict(n_total=n_total, base_ok=base_ok, base_correct=base_correct,
                robust_ok=robust_ok, robust_correct=robust_correct, reason_after=reason_after)


def pct(a, b):
    return f"{a/b*100:.1f}%" if b else "n/a"


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--base', default=os.path.expanduser('~/catkin_ws/dataset_arucode/Aruco_yang_baru'))
    ap.add_argument('--refs', default=os.path.join(
        os.path.dirname(os.path.abspath(__file__)), '..', 'config', 'wp_marker_reference_v2.json'))
    ap.add_argument('--resize', default=None,
                     help="mis. 640x480 -- samakan resolusi ke live camera_down.launch")
    a = ap.parse_args()
    resize = None
    if a.resize:
        w, h = a.resize.lower().split('x')
        resize = (int(w), int(h))

    refs = load_refs_v2(a.refs)
    with open(a.refs) as f:
        raw = json.load(f)

    print(f"resize={resize or '(asli, TIDAK direpresentasikan resolusi kamera live)'}")
    print(f"{'WP':<5} {'reliable':<9} {'agreement':<10} {'n_ref':<6} | "
          f"{'baseline P/R':<20} | {'+robust P/R':<20} | delta")
    print("-" * 100)
    for name, wid in WP_IDS.items():
        folder = os.path.join(a.base, name)
        if not os.path.isdir(folder):
            print(f"[SKIP] {name}: folder tak ada ({folder})")
            continue
        r = eval_folder(folder, wid, refs, resize)
        ref = raw[str(wid)]
        base_p = pct(r['base_correct'], r['base_ok'])
        base_r = pct(r['base_ok'], r['n_total'])
        robust_p = pct(r['robust_correct'], r['robust_ok'])
        robust_r = pct(r['robust_ok'], r['n_total'])
        delta = r['robust_ok'] - r['base_ok']
        wrong = r['robust_ok'] - r['robust_correct']
        print(f"WP{wid:<4} {str(ref['reliable']):<9} {ref['agreement']:<10.4f} {ref['n']:<6} | "
              f"P={base_p:<7} R={base_r:<9} | P={robust_p:<7} R={robust_r:<9} | "
              f"+{delta} bacaan baru, wrong={wrong}")
        print(f"      sisa kegagalan sesudah robust ({r['n_total']-r['robust_ok']} foto): "
              f"{dict(r['reason_after'])}")
    print("\nDONE")


if __name__ == '__main__':
    main()

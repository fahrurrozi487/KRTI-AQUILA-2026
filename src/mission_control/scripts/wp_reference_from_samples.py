#!/usr/bin/env python3
"""wp_reference_from_samples.py — bangun referensi dari SAMPEL BERKUALITAS,
bukan 1 foto tetap (WP*_inverted.jpg / aruco_bersih) dan bukan rata-rata SEMUA
foto dataset (termasuk yang buram/miring, yang bikin WP1 agreement cuma 0.671).

Reuse penuh pipeline `wp_decode_audit.py` (READ-ONLY, tidak diedit) — cuma
nambah 2 tahap filter di atasnya:
  1. quality gate per-foto: tail terdeteksi (bukan low-conf) + aspek warp ketat
     (|aspect-1| <= 0.15, lebih ketat dari AR_TOL=0.25 pipeline asli)
  2. trimmed vote: dari yang lolos gate, buang trim_frac terjauh (hamming ke
     modal awal) sebelum vote ulang -> modal akhir dari sampel yg PALING
     konsisten satu sama lain, bukan disetir 1 foto asing.

Tidak menulis wp_marker_reference.json otomatis — cuma cetak laporan supaya
bisa dicek dulu sebelum menimpa referensi produksi (riwayat rebuild v1 gagal
verifikasi visual, jangan diulang tanpa review).
"""
import argparse
import os
import sys

import numpy as np

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from wp_decode_audit import process_image, aggregate, bits_str, GRID_N  # noqa: E402


def quality_pool(folder, aspect_tol=0.15, limit=None):
    files = sorted(f for f in os.listdir(folder)
                    if f.lower().endswith(('.jpg', '.jpeg', '.png')))
    if limit:
        files = files[:limit]
    pool = []   # list of (fname, bits)
    n_ok = 0
    for f in files:
        r = process_image(os.path.join(folder, f))
        if not r['ok']:
            continue
        n_ok += 1
        if r['tail'] is None:                       # orientasi tak diketahui -> buang
            continue
        if abs(r['aspect'] - 1.0) > aspect_tol:       # warp miring -> buang
            continue
        pool.append((f, r['bits']))
    return files, n_ok, pool


def trimmed_vote(pool, trim_frac=0.2):
    """Buang trim_frac sampel terjauh dari modal awal, vote ulang."""
    bits_list = [b for _, b in pool]
    modal0, _ = aggregate(bits_list)
    if modal0 is None:
        return None, 0.0, []
    dists = [int((b != modal0).sum()) for b in bits_list]
    order = np.argsort(dists)
    keep_n = max(1, int(np.ceil(len(pool) * (1 - trim_frac))))
    keep_idx = sorted(order[:keep_n].tolist())
    kept = [pool[i] for i in keep_idx]
    modal1, agree1 = aggregate([b for _, b in kept])
    return modal1, agree1, kept


def run(base, wps, aspect_tol, trim_frac, limit, top_n):
    finals = {}
    for wp in wps:
        folder = os.path.join(base, wp)
        if not os.path.isdir(folder):
            print(f"[SKIP] {wp}: folder tak ada")
            continue
        files, n_ok, pool = quality_pool(folder, aspect_tol, limit)
        print(f"\n===== {wp} =====")
        print(f"  foto total={len(files)}  decode-ok={n_ok}  lolos quality-gate={len(pool)}")
        if not pool:
            print("  (pool kosong, skip)")
            continue
        modal, agree, kept = trimmed_vote(pool, trim_frac)
        print(f"  trimmed vote: pakai {len(kept)}/{len(pool)} sampel "
              f"(buang {trim_frac:.0%} terjauh dari modal awal)")
        print(f"  agreement akhir: {agree:.3f}")
        print("  pola:")
        for line in bits_str(modal).splitlines():
            print("    " + line)
        dists = sorted(((int((b != modal).sum()), f) for f, b in kept))
        print(f"  {top_n} sampel PALING cocok (paling representatif):")
        for d, f in dists[:top_n]:
            print(f"    H={d:2d}  {f}")
        finals[wp] = modal

    names = list(finals.keys())
    if len(names) > 1:
        print("\n===== jarak antar pola akhir (hamming, dari max"
              f"={GRID_N*GRID_N}) =====")
        for i in range(len(names)):
            for j in range(i + 1, len(names)):
                a, b = names[i], names[j]
                d = int((finals[a] != finals[b]).sum())
                print(f"  {a} vs {b}: {d}")
    return finals


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--base', default=os.path.expanduser('~/catkin_ws/dataset_arucode'))
    ap.add_argument('--wps', nargs='+', default=['wp1', 'wp3', 'wp4'])
    ap.add_argument('--aspect-tol', type=float, default=0.15)
    ap.add_argument('--trim-frac', type=float, default=0.2)
    ap.add_argument('--limit', type=int, default=None)
    ap.add_argument('--top-n', type=int, default=5)
    a = ap.parse_args()
    run(a.base, a.wps, a.aspect_tol, a.trim_frac, a.limit, a.top_n)


if __name__ == '__main__':
    main()

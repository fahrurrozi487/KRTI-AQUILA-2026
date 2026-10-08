#!/usr/bin/env python3
"""wp_decode_gridsweep.py — Task 1/2: uji apakah GRID_N=6 (dikunci di
wp_decode_audit.py) benar-benar resolusi cetak marker, atau agreement
rendah disebabkan grid-size mismatch.

READ-ONLY. Hitung corners+tail+warp SEKALI per foto (mahal), lalu re-sample
grid utk N=4..12 dari warp yang sama (murah) -> agreement per N per WP.
Kalau agreement naik tajam di N tertentu dan modal pattern-nya terlihat
lebih "solid" (blok besar, bukan noise checkerboard), itu kandidat GRID_N asli.

ponytail: satu script eksperimen, dibuang setelah kesimpulan diambil.
"""
import os
import sys
import json
import cv2
import numpy as np

sys.path.insert(0, os.path.dirname(__file__))
import wp_decode_audit as W

BASE = os.path.expanduser('~/catkin_ws/dataset_arucode')


def decode_grid_n(warp, N):
    """Sama seperti W.decode_grid tapi grid size N parametrik (bukan GRID_N global)."""
    _hsv = cv2.cvtColor(warp, cv2.COLOR_BGR2HSV)
    _h, _s = _hsv[:, :, 0], _hsv[:, :, 1]
    _red = ((_h < 7) | (_h > 168)) & (_s > 100)
    warp = warp.copy()
    warp[_red] = [255, 255, 255]
    g = cv2.cvtColor(warp, cv2.COLOR_BGR2GRAY)
    thr, bw = cv2.threshold(g, 0, 255, cv2.THRESH_BINARY_INV + cv2.THRESH_OTSU)
    if (bw > 0).sum() < 50:
        return np.zeros((N, N), dtype=int)
    span = W._body_yspan(bw)
    if span is None:
        return np.zeros((N, N), dtype=int)
    y0, y1 = span
    xs_body = np.where(bw[y0:y1].sum(axis=0) > 0)[0]
    x0, x1 = int(xs_body.min()), int(xs_body.max()) + 1
    cw, ch = (x1 - x0) / N, (y1 - y0) / N
    half = max(3, int(min(cw, ch) * 0.16))   # patch proporsional ke ukuran sel (bukan 8px tetap)
    bits = np.zeros((N, N), dtype=int)
    for r in range(N):
        for c in range(N):
            cy, cx = int(y0 + (r + 0.5) * ch), int(x0 + (c + 0.5) * cw)
            patch = g[max(0, cy - half):cy + half, max(0, cx - half):cx + half]
            bits[r, c] = 1 if patch.size and np.median(patch) < thr else 0
    return bits


def collect_warps(folder):
    """Proses semua foto SEKALI: corners+tail+warp (mahal), cache di memori."""
    files = sorted(f for f in os.listdir(folder)
                   if f.lower().endswith(('.jpg', '.png', '.jpeg')))
    warps = []
    for f in files:
        path = os.path.join(folder, f)
        bgr = cv2.imread(path)
        if bgr is None:
            continue
        body = W.largest_component(W.marker_mask(bgr))
        if body is None:
            continue
        _hsv = cv2.cvtColor(bgr, cv2.COLOR_BGR2HSV)
        _h, _s = _hsv[:, :, 0], _hsv[:, :, 1]
        _red = ((_h < 7) | (_h > 168)) & (_s > 100)
        body_tail = body.copy()
        body_tail[_red] = 0
        body_tail = cv2.morphologyEx(body_tail, cv2.MORPH_CLOSE,
                                      cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (15, 15)))
        tail = W.detect_tail_side(body_tail)
        corners = W.corners_of(body)
        if corners is None:
            continue
        warp, aspect = W.warp_marker(bgr, corners)
        if abs(aspect - 1.0) > W.AR_TOL:
            continue
        warp = W.rotate_to_canonical(warp, tail)
        warps.append((f, warp, tail))
    return warps


def agreement_for_n(warps, N):
    stack = []
    for f, warp, tail in warps:
        stack.append(decode_grid_n(warp, N))
    stack = np.array(stack)
    modal = (stack.mean(axis=0) >= 0.5).astype(int)
    agree = float((stack == modal).mean())
    return agree, modal


def bits_str(bits):
    return '\n'.join(''.join('#' if b else '.' for b in row) for row in bits)


def main():
    folders = sys.argv[1:] if len(sys.argv) > 1 else ['wp1_new', 'wp2', 'wp3', 'wp4']
    for name in folders:
        folder = os.path.join(BASE, name)
        print(f"\n===== sweep {name} =====")
        warps = collect_warps(folder)
        print(f"  n_ok(ar-gate)={len(warps)}")
        for N in [4, 5, 6, 7, 8, 9, 10, 12]:
            agree, modal = agreement_for_n(warps, N)
            print(f"  N={N:2d}  agreement={agree:.4f}")
        # cetak pola modal utk N terbaik
        best_n, best_agree = None, -1
        for N in [4, 5, 6, 7, 8, 9, 10, 12]:
            agree, modal = agreement_for_n(warps, N)
            if agree > best_agree:
                best_agree, best_n = agree, N
        agree, modal = agreement_for_n(warps, best_n)
        print(f"  BEST: N={best_n} agreement={agree:.4f}")
        print("  pola modal:")
        for line in bits_str(modal).splitlines():
            print("    " + line)


if __name__ == '__main__':
    main()

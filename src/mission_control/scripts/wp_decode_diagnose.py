#!/usr/bin/env python3
"""wp_decode_diagnose.py — Task 1 (lanjutan Task 3): breakdown akar masalah
agreement < 1.0 per WP.

READ-ONLY terhadap dataset & wp_decode_audit.py (hanya import, tidak ubah).
Untuk tiap WP: hitung Hamming distance tiap foto 'ok' ke pola modal, ranking
foto paling menyimpang, dan simpan overlay diagnostik (corners+tail+warp
+grid decode) ke dataset_arucode/_diag/<wp>/ untuk audit visual.

ponytail: satu script analisis read-only, bukan tool umum. Tidak ada CLI
canggih, cuma argumen yang dipakai sesi ini.
"""
import os
import sys
import json
import cv2
import numpy as np

sys.path.insert(0, os.path.dirname(__file__))
import wp_decode_audit as W

BASE = os.path.expanduser('~/catkin_ws/dataset_arucode')
DIAG = os.path.join(BASE, '_diag')


def process_folder_named(folder, limit=None):
    """Seperti W.process_folder tapi simpan filename tiap hasil (ok & buang)."""
    files = sorted(f for f in os.listdir(folder)
                   if f.lower().endswith(('.jpg', '.png', '.jpeg')))
    if limit:
        files = files[:limit]
    recs = []
    for f in files:
        r = W.process_image(os.path.join(folder, f))
        r['file'] = f
        recs.append(r)
    return recs


def save_diag_overlay(path, out_path, tail, corners, aspect, bits, modal, hamming):
    bgr = cv2.imread(path)
    if bgr is None:
        return
    vis = bgr.copy()
    if corners is not None:
        o = W.order_corners(corners).astype(int)
        cv2.polylines(vis, [o], True, (0, 0, 255), 3)
    cv2.putText(vis, f'tail={tail} ar={aspect:.2f} hamming={hamming}/36',
                (20, 40), cv2.FONT_HERSHEY_SIMPLEX, 0.9, (0, 255, 255), 2)

    # warp kanonik + grid decode, side-by-side kalau ada
    canvas_h = vis.shape[0]
    warp_vis = np.full((canvas_h, W.WARP_PX, 3), 255, np.uint8)
    if corners is not None:
        warp, _ = W.warp_marker(bgr, corners)
        warp = W.rotate_to_canonical(warp, tail)
        wv = warp.copy()
        gh, gw = wv.shape[0] / W.GRID_N, wv.shape[1] / W.GRID_N
        for r in range(W.GRID_N):
            for c in range(W.GRID_N):
                y0, y1 = int(r * gh), int((r + 1) * gh)
                x0, x1 = int(c * gw), int((c + 1) * gw)
                cv2.rectangle(wv, (x0, y0), (x1, y1), (0, 255, 0), 1)
                if bits is not None:
                    mark = '#' if bits[r, c] else '.'
                    bad = modal is not None and bits[r, c] != modal[r, c]
                    color = (0, 0, 255) if bad else (255, 128, 0)
                    cv2.putText(wv, mark, (x0 + 10, y0 + 30),
                                cv2.FONT_HERSHEY_SIMPLEX, 0.9, color, 2)
        warp_vis[:wv.shape[0], :wv.shape[1]] = wv

    combo = np.hstack([vis, warp_vis])
    os.makedirs(os.path.dirname(out_path), exist_ok=True)
    cv2.imwrite(out_path, combo)


def diagnose(wp_name, folder_name, ref_id, ref_json, topn=15):
    folder = os.path.join(BASE, folder_name)
    recs = process_folder_named(folder)
    good = [r for r in recs if r['ok']]
    bad = [r for r in recs if not r['ok']]

    modal = np.array(ref_json[ref_id]['pattern'])

    # per-foto hamming ke modal
    for r in good:
        r['hamming'] = int((r['bits'] != modal).sum())
    good_sorted = sorted(good, key=lambda r: -r['hamming'])

    n_ok = len(good)
    n_bad = len(bad)
    n_total = len(recs)

    # kategori penyebab utk foto ok tapi menyimpang:
    # - low_conf (tail=None, ekor tak terdeteksi -> kanonikalisasi acak)
    # - tail terdeteksi tapi hamming tinggi tetap (>25% sel beda, ambang arbitrer
    #   utk "menyimpang signifikan" dipakai laporan, bukan gating)
    low_conf_n = sum(1 for r in good if r.get('low_conf'))
    high_hamming_n = sum(1 for r in good if r['hamming'] > 9)  # >25% dari 36

    # buang: breakdown alasan (sudah dihitung wp_decode_audit, tapi re-derive dgn nama)
    bad_reason_hist = {}
    for r in bad:
        key = r['reason'].split('(')[0]
        bad_reason_hist[key] = bad_reason_hist.get(key, 0) + 1

    print(f"\n===== {wp_name} ({folder_name}, id={ref_id}) =====")
    print(f"  total={n_total} ok={n_ok} buang={n_bad}  buang_breakdown={bad_reason_hist}")
    print(f"  agreement (recompute)={1 - np.mean([r['hamming'] for r in good])/36:.3f}"
          if good else "  agreement: n/a")
    print(f"  low_conf (tail=None) di antara ok: {low_conf_n}/{n_ok}")
    print(f"  hamming>9/36 (>25% sel beda) di antara ok: {high_hamming_n}/{n_ok}")

    # dump top-N worst utk audit visual
    outdir = os.path.join(DIAG, folder_name)
    top = good_sorted[:topn]
    for r in top:
        out_path = os.path.join(outdir, f"h{r['hamming']:02d}_{r['file']}")
        corners = W.corners_of(W.largest_component(W.marker_mask(cv2.imread(os.path.join(folder, r['file'])))))
        save_diag_overlay(os.path.join(folder, r['file']), out_path,
                           r['tail'], corners, r['aspect'], r['bits'], modal, r['hamming'])
    print(f"  top-{len(top)} worst-hamming disimpan ke {outdir}/")
    for r in top:
        print(f"    {r['file']}: hamming={r['hamming']}/36 tail={r['tail']} ar={r['aspect']:.3f} low_conf={r.get('low_conf')}")

    return dict(wp=wp_name, n_total=n_total, n_ok=n_ok, n_bad=n_bad,
                bad_reason_hist=bad_reason_hist, low_conf_n=low_conf_n,
                high_hamming_n=high_hamming_n, good=good, bad=bad, modal=modal)


def main():
    with open(os.path.expanduser(
            '~/catkin_ws/src/mission_control/config/wp_marker_reference.json')) as f:
        ref = json.load(f)
    wps = [('WP1', 'wp1_new', '1'), ('WP2', 'wp2', '2'),
           ('WP3', 'wp3', '3'), ('WP4', 'wp4', '4')]
    all_results = {}
    for wp_name, folder, rid in wps:
        all_results[wp_name] = diagnose(wp_name, folder, rid, ref)
    return all_results


if __name__ == '__main__':
    main()

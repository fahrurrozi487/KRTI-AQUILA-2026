#!/usr/bin/env python3
"""wp_decode_experiment.py — Task 2: uji fix "tail detection setelah
rektifikasi" (bukan sebelum).

Akar masalah (Task 1, dikonfirmasi audit visual): detect_tail_side()
di wp_decode_audit.py jalan di mask foto MENTAH (masih skew perspektif +
rotasi in-plane kamera). Profil rowsum/colsum dihitung di sumbu PIKSEL
FOTO, bukan sumbu lokal marker -> kalau marker difoto miring (rotasi
in-plane, BUKAN cuma foreshortening), N/S/E/W jadi salah total (dibuktikan:
2 foto WP1 dgn ekor jelas kelihatan mata di sisi berlawanan dari yang
dideteksi algoritma).

Fix yang diuji: warp corners TAPI dengan margin (padding) supaya tab ekor
(yang sengaja dibuang dari `corners_of` via MORPH_OPEN) ikut kebawa masuk
kanvas warp. Deteksi ekor di kanvas warp yang SUDAH axis-aligned (bebas
rotasi in-plane), bukan di foto mentah.

READ-ONLY thd dataset. Tidak mengubah wp_decode_audit.py.
"""
import os
import sys
import cv2
import numpy as np

sys.path.insert(0, os.path.dirname(__file__))
import wp_decode_audit as W

BASE = os.path.expanduser('~/catkin_ws/dataset_arucode')
PAD_FRAC = 0.35   # margin sekitar core square, relatif ke WARP_PX


def warp_padded(bgr, corners):
    o = W.order_corners(corners)
    pad = int(W.WARP_PX * PAD_FRAC)
    full = W.WARP_PX + 2 * pad
    dst = np.array([[pad, pad], [pad + W.WARP_PX, pad],
                     [pad + W.WARP_PX, pad + W.WARP_PX], [pad, pad + W.WARP_PX]],
                    dtype=np.float32)
    M = cv2.getPerspectiveTransform(o, dst)
    return cv2.warpPerspective(bgr, M, (full, full)), pad


def detect_tail_side_warped(warp_pad, pad):
    """Ekor di kanvas warp axis-aligned (bebas rotasi in-plane, beda dgn
    versi asli yang jalan di foto mentah skew). Pakai ULANG W.detect_tail_side
    (profil rowsum/colsum vs core 60%) -- sekarang input-nya sudah
    rektifikasi jadi asumsi 'core besar vs tab sempit' berlaku benar,
    tidak perlu geometri pad manual.
    """
    mask = W.marker_mask(warp_pad)
    body = W.largest_component(mask)
    if body is None:
        return None
    return W.detect_tail_side(body)


def process_image_v2(path):
    bgr = cv2.imread(path)
    if bgr is None:
        return dict(ok=False, reason='baca-gagal')
    mask = W.marker_mask(bgr)
    body = W.largest_component(mask)
    if body is None:
        return dict(ok=False, reason='no-marker')
    corners = W.corners_of(body)
    if corners is None:
        return dict(ok=False, reason='no-corners')
    warp, aspect = W.warp_marker(bgr, corners)
    if abs(aspect - 1.0) > W.AR_TOL:
        return dict(ok=False, reason=f'warp-buruk(ar={aspect:.2f})', aspect=aspect)
    wpad, pad = warp_padded(bgr, corners)
    tail = detect_tail_side_warped(wpad, pad)
    warp = W.rotate_to_canonical(warp, tail)
    bits = W.decode_grid(warp)
    return dict(ok=True, tail=tail, aspect=aspect, bits=bits,
                low_conf=(tail is None), reason='ok')


def process_folder_v2(folder):
    files = sorted(f for f in os.listdir(folder)
                   if f.lower().endswith(('.jpg', '.png', '.jpeg')))
    recs = []
    for f in files:
        r = process_image_v2(os.path.join(folder, f))
        r['file'] = f
        recs.append(r)
    return recs


def aggregate(good_bits):
    if not good_bits:
        return None, 0.0
    stack = np.array(good_bits)
    modal = (stack.mean(axis=0) >= 0.5).astype(int)
    agree = float((stack == modal).mean())
    return modal, agree


def bits_str(bits):
    return '\n'.join(''.join('#' if b else '.' for b in row) for row in bits)


def compare(folder_name):
    folder = os.path.join(BASE, folder_name)
    old_recs = W.process_folder(folder)   # pipeline lama (utk n_ok, tail hist lama)
    new_recs = process_folder_v2(folder)

    old_good = [r for r in old_recs['good']]
    old_modal, old_agree = aggregate(old_good)

    new_good = [r for r in new_recs if r['ok']]
    new_bits = [r['bits'] for r in new_good]
    new_modal, new_agree = aggregate(new_bits)

    old_tails = old_recs['tails']
    new_tails = [r['tail'] for r in new_good]
    from collections import Counter
    old_hist = dict(Counter('None' if t is None else t for t in old_tails))
    new_hist = dict(Counter('None' if t is None else t for t in new_tails))

    print(f"\n===== {folder_name}: lama vs baru (tail post-rektifikasi) =====")
    print(f"  n_ok: lama={len(old_good)} baru={len(new_good)}")
    print(f"  agreement: lama={old_agree:.4f} baru={new_agree:.4f}  delta={new_agree-old_agree:+.4f}")
    print(f"  tail hist lama: {old_hist}")
    print(f"  tail hist baru: {new_hist}")
    if new_modal is not None:
        print("  pola modal BARU:")
        for line in bits_str(new_modal).splitlines():
            print("    " + line)

    return dict(old_agree=old_agree, new_agree=new_agree, new_recs=new_recs, old_recs=old_recs)


def main():
    folders = sys.argv[1:] if len(sys.argv) > 1 else ['wp1_new', 'wp2', 'wp3', 'wp4']
    results = {}
    for name in folders:
        results[name] = compare(name)
    return results


if __name__ == '__main__':
    main()

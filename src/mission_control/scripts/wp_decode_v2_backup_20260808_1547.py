#!/usr/bin/env python3
"""wp_decode_v2.py — Task 3 REBUILD: decode marker WP1/2/3/4 dari dataset lapangan
baru (~/catkin_ws/dataset_arucode/Aruco_yang_baru/), foto diambil BERDIRI dari jarak
jauh (bukan nadir kayak kamera drone) -> beda kondisi dari dataset lama yg jadi dasar
wp_decode_audit.py. Reuse fungsi yg scale/rotation-agnostic dari situ (order_corners,
warp_marker, decode_grid, rotate_to_canonical); tulis ulang yg sensitif skala/geometri
foto (marker_mask, pemisahan badan besar vs marker kecil, pencarian sudut).

Beda kunci dari wp_decode_audit.py (v1, dataset lama):
  1. marker_mask_v2: dibatasi ke DALAM area tarp oranye dulu (foto v2 lebar, ada
     rumput/paving background yg ikut kesegmentasi "bukan-oranye" kalau gak dibatasi).
  2. split_core_protrusion: marker kecil (arah navigasi, Task baru) nempel LANGSUNG
     ke badan besar -- bukan "ekor" tipis yg gampang diputus morfologi kayak desain
     lama. Dipisah via profil proyeksi baris/kolom (generalisasi detect_tail_side),
     bukan MORPH_OPEN. Sisi tempat marker kecil nempel = arah navigasi (posisi,
     BUKAN isi pola marker kecil -- keputusan user 2026-08-07).
  3. corners_small: corners_of() lama pakai downscale 0.25 + kernel morfologi besar,
     di-tune utk marker ~1000px+ (dataset lama, foto dekat). Marker v2 cuma ~250-400px
     dlm frame -> kernel yg sama overerode, motong kuadran. Versi ringan tanpa
     downscale/opening agresif (protrusion udah dibuang di langkah 2).
  4. AR_TOL_V2 dinaikkan jauh dari AR_TOL lama (0.25): foto BERDIRI/miring bikin
     rasio warp asli marker persegi secara SISTEMATIS jauh dari 1.0 (foreshortening),
     divalidasi visual: warp tetap rapi meski aspect~1.7-2.6 (lih. commit message/
     percakapan). Longgarkan berdasar distribusi empiris (lihat main()), bukan tebakan.

ponytail: diagnostik + rebuild reference, gaya sama seperti wp_decode_audit.py asli.
"""
import argparse
import os
import sys
import numpy as np
import cv2

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from wp_decode_audit import order_corners, warp_marker, rotate_to_canonical, decode_grid, bits_str, GRID_N

ORANGE_H_LO, ORANGE_H_HI, ORANGE_S_MIN = 8, 32, 40


def marker_mask_v2(bgr):
    """'Bukan-oranye' TAPI dibatasi ke dalam tarp terbesar (buang background luar tarp)."""
    hsv = cv2.cvtColor(bgr, cv2.COLOR_BGR2HSV)
    h, s = hsv[:, :, 0], hsv[:, :, 1]
    orange = ((h >= ORANGE_H_LO) & (h <= ORANGE_H_HI) & (s > ORANGE_S_MIN)).astype(np.uint8) * 255
    k = cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (15, 15))
    orange = cv2.morphologyEx(orange, cv2.MORPH_CLOSE, k)
    cnts, _ = cv2.findContours(orange, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
    if not cnts:
        return None
    c = max(cnts, key=cv2.contourArea)
    if cv2.contourArea(c) < 5000:
        return None
    tarp_solid = np.zeros_like(orange)
    cv2.drawContours(tarp_solid, [c], -1, 255, -1)          # isi solid (nutup lubang marker)
    not_orange = (~(orange > 0)).astype(np.uint8) * 255
    body_region = cv2.bitwise_and(not_orange, tarp_solid)
    return cv2.morphologyEx(body_region, cv2.MORPH_CLOSE, k)


def largest_component(mask, min_area=1):
    if mask is None:
        return None
    n, lbl, stats, _ = cv2.connectedComponentsWithStats(mask, 8)
    if n <= 1:
        return None
    idx = 1 + int(np.argmax(stats[1:, cv2.CC_STAT_AREA]))
    if stats[idx, cv2.CC_STAT_AREA] < min_area:
        return None
    return (lbl == idx).astype(np.uint8) * 255


def split_core_protrusion(body, thr_frac=0.6):
    """Pisah body jadi (core=badan besar tanpa marker kecil, side='N'/'S'/'E'/'W').

    Generalisasi detect_tail_side() (wp_decode_audit.py) dari 'ekor tipis' ke
    'marker kecil nempel langsung' -- caranya sama (profil proyeksi, threshold 60%
    baris/kolom "inti" vs bagian yg lebih sempit), tapi di sini dipakai buat MOTONG
    (bukan cuma mendeteksi sisi), krn protrusion terlalu tebal utk diputus morfologi.
    Return (None, None) kalau tak ada sisi menonjol jelas (badan sudah nyaris persegi).
    """
    rowsum = (body > 0).sum(axis=1)
    colsum = (body > 0).sum(axis=0)
    if rowsum.max() == 0:
        return None, None
    row_thr = rowsum.max() * thr_frac
    col_thr = colsum.max() * thr_frac
    body_ys = np.where(rowsum > 0)[0]; core_ys = np.where(rowsum >= row_thr)[0]
    body_xs = np.where(colsum > 0)[0]; core_xs = np.where(colsum >= col_thr)[0]
    if not len(core_ys) or not len(core_xs):
        return None, None
    sides = {
        'N': int(core_ys[0]) - int(body_ys[0]),
        'S': int(body_ys[-1]) - int(core_ys[-1]),
        'W': int(core_xs[0]) - int(body_xs[0]),
        'E': int(body_xs[-1]) - int(core_xs[-1]),
    }
    side = max(sides, key=lambda k: sides[k])
    noise_px = max(5, int(min(core_ys[-1] - core_ys[0], core_xs[-1] - core_xs[0]) * 0.03))
    if sides[side] < noise_px:
        return body, None                     # gak ada protrusion jelas
    core = body.copy()
    y0, y1 = int(core_ys[0]), int(core_ys[-1]) + 1
    x0, x1 = int(core_xs[0]), int(core_xs[-1]) + 1
    if side == 'N': core[:y0, :] = 0
    elif side == 'S': core[y1:, :] = 0
    elif side == 'W': core[:, :x0] = 0
    elif side == 'E': core[:, x1:] = 0
    return core, side


def corners_small(core):
    """Sudut badan besar, TANPA downscale/opening agresif (marker v2 kecil di frame,
    ~200-400px -- lih. docstring modul). core diasumsikan sudah bersih dari protrusion.

    Sanity check luas: approxPolyDP kadang nyusutin kuadran < luas core asli kalau
    tepi mask gak mulus (lipatan kain/bayangan) -- ketauan lewat overlay visual
    (kuadran motong isi grid asli). Fallback minAreaRect kalau kuadran hasil
    approxPolyDP < 75% luas core (ditemukan & divalidasi 2026-08-07).
    """
    k = cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (5, 5))
    clean = cv2.morphologyEx(core, cv2.MORPH_CLOSE, k)
    cnts, _ = cv2.findContours(clean, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
    if not cnts:
        return None
    c = max(cnts, key=cv2.contourArea)
    core_area = cv2.contourArea(c)
    peri = cv2.arcLength(c, True)
    for eps_frac in (0.01, 0.02, 0.03, 0.04, 0.05, 0.07, 0.10, 0.13, 0.16):
        approx = cv2.approxPolyDP(c, eps_frac * peri, True)
        if len(approx) == 4 and cv2.isContourConvex(approx):
            if cv2.contourArea(approx) >= 0.75 * core_area:
                return approx.reshape(-1, 2).astype(np.float32)
    return cv2.boxPoints(cv2.minAreaRect(c)).astype(np.float32)


def _centroid_offset(corners, img_shape):
    h, w = img_shape[:2]
    cx, cy = np.asarray(corners).mean(axis=0)
    return float((cx - w / 2) / (w / 2)), float((cy - h / 2) / (h / 2))


MIN_CORE_BRIGHTNESS = 100   # kalibrasi dari distribusi confident-correct wp3 (n=178,
                             # min=109, p10=114) vs confident-wrong (n=20, min=115,
                             # MEDIAN LEBIH TERANG dari correct -- brightness TIDAK
                             # diskriminatif korrect-vs-salah, cuma diskriminatif
                             # marker-vs-bukan-marker (bayangan~47, tanah~93). 120
                             # (percobaan pertama) kepotong 60/178 correct match --
                             # diturunkan ke 100 (di bawah tanah=93, di bawah correct
                             # min=109).
MIN_CORE_AREA_PX = 5000     # confident-correct area min=25638 (n=178); confident-wrong
                             # bisa serendah 497 (silau/pantulan kecil). 5000 aman jauh
                             # di bawah correct min sambil masih buang kasus silau.
                             # ponytail: area TETAP gak nolong kasus "marker asli tapi
                             # misread" (median area confident-wrong=55147, overlap besar
                             # dgn correct) -- itu perlu perbaikan presisi corner/decode,
                             # bukan quality-gate. Lih. WP_MARKER_DECODER_V2.md §recall.


def _quality_gate(bgr, core):
    """True kalau core KEMUNGKINAN BESAR marker asli (bukan bayangan/tanah/silau)."""
    gray = cv2.cvtColor(bgr, cv2.COLOR_BGR2GRAY)
    vals = gray[core > 0]
    if vals.size < MIN_CORE_AREA_PX:
        return False, float(vals.mean()) if vals.size else 0.0
    return float(vals.mean()) >= MIN_CORE_BRIGHTNESS, float(vals.mean())


def decode_frame_v2(bgr, refs, hamming_threshold=7, ambiguous_margin=3):
    """Decode marker WP1/3/4 di 1 frame. TIDAK pakai split_core_protrusion's 'side'
    utk nentuin rotasi kanonik (side kadang salah pilih, lih. docstring modul) --
    sebagai gantinya coba KE-4 rotasi bit thd tiap referensi, ambil (wp_id, rotasi)
    dgn hamming distance minimum. 'side' cuma dipakai laporan arah-navigasi terpisah
    (posisi marker-kecil, sesuai keputusan user), independen dari ID-decode.

    refs: dict wp_id(str) -> {"pattern": [[..]], "reliable": bool}
    Returns dict: found_marker, wp_id, hamming, reliable, tail_side, reason, aspect, off_x, off_y
    """
    mask = marker_mask_v2(bgr)
    body = largest_component(mask, min_area=300)
    if body is None:
        return dict(found_marker=False, wp_id=-1, hamming=-1, reliable=False,
                    tail_side='', reason='no_marker', aspect=0.0, off_x=0.0, off_y=0.0)

    core, side = split_core_protrusion(body)
    tail_side = side or ''
    if core is None:
        ox, oy = _centroid_offset(np.column_stack(np.where(body > 0)[::-1]), bgr.shape)
        return dict(found_marker=True, wp_id=-1, hamming=-1, reliable=False,
                    tail_side=tail_side, reason='no_core', aspect=0.0, off_x=ox, off_y=oy)

    ok_quality, bright = _quality_gate(bgr, core)
    if not ok_quality:
        ox, oy = _centroid_offset(np.column_stack(np.where(core > 0)[::-1]), bgr.shape) \
            if (core > 0).any() else (0.0, 0.0)
        return dict(found_marker=True, wp_id=-1, hamming=-1, reliable=False,
                    tail_side=tail_side, reason='low_quality', aspect=0.0, off_x=ox, off_y=oy)

    corners = corners_small(core)
    if corners is None:
        return dict(found_marker=True, wp_id=-1, hamming=-1, reliable=False,
                    tail_side=tail_side, reason='no_corners', aspect=0.0, off_x=0.0, off_y=0.0)
    ox, oy = _centroid_offset(corners, bgr.shape)

    warp, aspect = warp_marker(bgr, corners)
    bits = decode_grid(warp)
    if bits.sum() == 0:
        return dict(found_marker=True, wp_id=-1, hamming=-1, reliable=False,
                    tail_side=tail_side, reason='decode_failed', aspect=float(aspect),
                    off_x=ox, off_y=oy)

    # rotation-search: cocokkan ke-4 rotasi thd tiap referensi, ambil terbaik
    distances = {}
    for wid, info in refs.items():
        ref = np.array(info['pattern'], dtype=int)
        distances[wid] = min(int((np.rot90(bits, k) != ref).sum()) for k in range(4))

    best_id = min(distances, key=lambda k: (distances[k], not refs[k]['reliable']))
    best_dist = distances[best_id]
    if best_dist > hamming_threshold:
        return dict(found_marker=True, wp_id=-1, hamming=int(best_dist), reliable=False,
                    tail_side=tail_side, reason='no_match', aspect=float(aspect),
                    off_x=ox, off_y=oy)
    second_dist = min((d for wid, d in distances.items() if wid != best_id), default=None)
    if second_dist is not None and (second_dist - best_dist) < ambiguous_margin:
        return dict(found_marker=True, wp_id=-1, hamming=int(best_dist), reliable=False,
                    tail_side=tail_side, reason='ambiguous', aspect=float(aspect),
                    off_x=ox, off_y=oy)
    return dict(found_marker=True, wp_id=int(best_id), hamming=int(best_dist),
                reliable=bool(refs[best_id]['reliable']), tail_side=tail_side,
                reason='ok', aspect=float(aspect), off_x=ox, off_y=oy)


def load_refs_v2(json_path, wp_ids=(1, 3, 4)):
    import json
    with open(json_path) as f:
        data = json.load(f)
    return {str(w): {'pattern': data[str(w)]['pattern'], 'reliable': data[str(w)]['reliable']}
            for w in wp_ids}


def process_image(path, ar_tol=1.3, debug_dir=None):
    """-> dict(ok, side, aspect, bits, reason). side = arah marker kecil (N/E/S/W/None)."""
    bgr = cv2.imread(path)
    if bgr is None:
        return dict(ok=False, reason='baca-gagal')
    mask = marker_mask_v2(bgr)
    body = largest_component(mask, min_area=300)
    if body is None:
        return dict(ok=False, reason='no-marker')
    core, side = split_core_protrusion(body)
    if core is None:
        return dict(ok=False, reason='no-core')
    corners = corners_small(core)
    if corners is None:
        return dict(ok=False, reason='no-corners', side=side)
    warp, aspect = warp_marker(bgr, corners)
    if debug_dir is not None:
        _save_overlay(bgr, corners, side, os.path.basename(path), debug_dir)
    if abs(aspect - 1.0) > ar_tol:
        return dict(ok=False, reason=f'warp-buruk(ar={aspect:.2f})', side=side, aspect=aspect)
    cwarp = rotate_to_canonical(warp, side)
    bits = decode_grid(cwarp)
    return dict(ok=True, side=side, aspect=aspect, bits=bits,
                low_conf=(side is None), reason='ok')


def _save_overlay(bgr, corners, side, name, debug_dir):
    vis = bgr.copy()
    o = order_corners(corners).astype(int)
    cv2.polylines(vis, [o], True, (0, 0, 255), 3)
    cv2.putText(vis, f'side={side}', (20, 40), cv2.FONT_HERSHEY_SIMPLEX, 1.0, (255, 0, 0), 2)
    os.makedirs(debug_dir, exist_ok=True)
    cv2.imwrite(os.path.join(debug_dir, name.rsplit('.', 1)[0] + '_ov.png'), vis)


def process_folder(folder, limit=None, ar_tol=1.3, debug_dir=None):
    files = sorted(f for f in os.listdir(folder) if f.lower().endswith(('.jpg', '.png', '.jpeg')))
    if limit:
        files = files[:limit]
    good, sides, aspects, bad = [], [], [], {}
    for f in files:
        r = process_image(os.path.join(folder, f), ar_tol, debug_dir)
        if r.get('aspect') is not None:
            aspects.append(r['aspect'])
        if r['ok']:
            good.append(r['bits']); sides.append(r.get('side'))
        else:
            key = r['reason'].split('(')[0]
            bad[key] = bad.get(key, 0) + 1
    return dict(n_total=len(files), n_good=len(good), bad=bad, aspects=aspects, sides=sides, good=good)


def aggregate(good):
    if not good:
        return None, 0.0
    stack = np.array(good)
    modal = (stack.mean(axis=0) >= 0.5).astype(int)
    agree = (stack == modal).mean()
    return modal, float(agree)


def _side_hist(sides):
    from collections import Counter
    return dict(Counter('None' if s is None else s for s in sides))


def run(base, folders, limit, ar_tol, debug):
    results = {}
    for name in folders:
        folder = os.path.join(base, name)
        if not os.path.isdir(folder):
            print(f"[SKIP] {name}: folder tak ada")
            continue
        dbg = os.path.join(base, '_debug_v2', name.replace('/', '_')) if debug else None
        R = process_folder(folder, limit, ar_tol, dbg)
        modal, agree = aggregate(R['good'])
        results[name] = (modal, agree, R)
        ar = np.array(R['aspects']) if R['aspects'] else np.array([0])
        print(f"\n===== {name} =====")
        print(f"  foto: {R['n_total']}  ok: {R['n_good']}  buang: {R['bad']}")
        print(f"  aspek-rasio warp: mean={ar.mean():.3f} std={ar.std():.3f} min={ar.min():.3f} max={ar.max():.3f}")
        print(f"  sisi marker-kecil (arah): {_side_hist(R['sides'])}")
        print(f"  agreement sel: {agree:.3f}")
        if modal is not None:
            print("  pola modal (# = hitam):")
            for line in bits_str(modal).splitlines():
                print("    " + line)
    classify_report(results)
    return results


def classify_report(results):
    names = [n for n in results if results[n][0] is not None]
    if len(names) < 2:
        print("\n[classify] perlu >=2 folder dgn modal utk klasifikasi")
        return
    refs = {n: results[n][0] for n in names}
    print(f"\n===== KLASIFIKASI GROUND-TRUTH (nearest-match: {names}) =====")
    for true_name in names:
        R = results[true_name][2]
        confusion = {n: 0 for n in names}
        for bits in R['good']:
            dists = {n: int((bits != refs[n]).sum()) for n in names}
            confusion[min(dists, key=dists.get)] += 1
        n_dec = sum(confusion.values())
        n_ok = confusion[true_name]
        n_tot = R['n_total']
        others = {k: v for k, v in confusion.items() if k != true_name and v > 0}
        acc_dec = n_ok / n_dec if n_dec else 0.0
        acc_e2e = n_ok / n_tot if n_tot else 0.0
        print(f"  {true_name}: {n_ok}/{n_dec} benar dari yg ke-decode ({acc_dec:.1%})"
              f"  |  end-to-end: {n_ok}/{n_tot} ({acc_e2e:.1%})"
              + (f"  salah->{others}" if others else ""))


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--base', default=os.path.expanduser('~/catkin_ws/dataset_arucode/Aruco_yang_baru'))
    ap.add_argument('--folders', nargs='+', default=['wp1', 'wp3', 'wp4', 'wp2/wp2_outside_box/wp2_part1'])
    ap.add_argument('--limit', type=int, default=None)
    ap.add_argument('--ar-tol', type=float, default=1.3)
    ap.add_argument('--debug', action='store_true')
    a = ap.parse_args()
    run(a.base, a.folders, a.limit, a.ar_tol, a.debug)


if __name__ == '__main__':
    main()

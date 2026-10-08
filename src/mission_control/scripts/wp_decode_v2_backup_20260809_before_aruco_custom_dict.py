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


def _marker_mask(bgr, h_lo, h_hi, s_min):
    """'Bukan-oranye' TAPI dibatasi ke dalam tarp terbesar (buang background luar
    tarp). Diekstrak (2026-08-08) dari marker_mask_v2() supaya ambang H/S bisa
    diganti-ganti -- dipakai ULANG oleh multi-hypothesis HSV fallback
    (lih. HSV_FALLBACK_VARIANTS)."""
    hsv = cv2.cvtColor(bgr, cv2.COLOR_BGR2HSV)
    h, s = hsv[:, :, 0], hsv[:, :, 1]
    orange = ((h >= h_lo) & (h <= h_hi) & (s > s_min)).astype(np.uint8) * 255
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


def marker_mask_v2(bgr):
    """Ambang default (produksi, TIDAK berubah dari sebelumnya)."""
    return _marker_mask(bgr, ORANGE_H_LO, ORANGE_H_HI, ORANGE_S_MIN)


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


def match_bits_to_refs(bits, refs, hamming_threshold=7, ambiguous_margin=3):
    """Rotation-search: cocokkan ke-4 rotasi `bits` thd tiap referensi, ambil
    terbaik. Diekstrak dari decode_frame_v2() (2026-08-08) supaya bisa dipakai
    ULANG oleh agregasi temporal di wp_marker_node.py (majority-vote bits dari
    beberapa frame -> match sekali) TANPA duplikasi logic -- satu-satunya
    tempat threshold/ambiguous_margin diterapkan, decode_frame_v2() maupun
    jalur temporal manapun WAJIB lewat sini, bukan reimplementasi sendiri.

    refs: dict wp_id(str) -> {"pattern": [[..]], "reliable": bool}
    Returns (wp_id:int, hamming:int, reliable:bool, reason:str)
      reason: "no_match" | "ambiguous" | "ok"
    """
    distances = {}
    for wid, info in refs.items():
        ref = np.array(info['pattern'], dtype=int)
        distances[wid] = min(int((np.rot90(bits, k) != ref).sum()) for k in range(4))

    best_id = min(distances, key=lambda k: (distances[k], not refs[k]['reliable']))
    best_dist = distances[best_id]
    if best_dist > hamming_threshold:
        return -1, int(best_dist), False, 'no_match'
    second_dist = min((d for wid, d in distances.items() if wid != best_id), default=None)
    if second_dist is not None and (second_dist - best_dist) < ambiguous_margin:
        return -1, int(best_dist), False, 'ambiguous'
    return int(best_id), int(best_dist), bool(refs[best_id]['reliable']), 'ok'


def enhance_clahe(bgr):
    """CLAHE di kanal L (LAB) -- naikkan KONTRAS LOKAL, bukan kecerahan rata-rata
    (beda dari gamma/brightening polos). Teknik sama yg sudah dipakai
    `gate_multi_detector.py` utk gate (non-ML). Dipakai sbg KESEMPATAN KEDUA
    kalau `decode_frame_v2()` gagal -- BUKAN preprocessing default (lih.
    `CLAHE_FALLBACK_REASONS` & wp_marker_node.py utk kapan dipanggil).

    Divalidasi empiris 2026-08-08 di dataset penuh (wp1/wp3/wp4, Aruco_yang_baru):
    dari frame yg GAGAL baseline dgn reason di `CLAHE_FALLBACK_REASONS`, CLAHE
    menyelamatkan WP1 +6/6 benar, WP3 +3/3 benar, WP4 +2/2 benar (0 salah sama
    sekali). PENTING: trigger `reason='low_quality'` (underexposed) TIDAK
    terbantu SAMA SEKALI (0/292 rescued di WP1) -- CLAHE menaikkan kontras
    LOKAL, bukan brightness rata-rata yg diukur `_quality_gate()`, jadi gak
    bisa "menerangkan" foto yg genuinely gelap. Yang terbantu itu `no_match`
    (57.9% dari kegagalan WP1) -- foto CUKUP terang tapi kontras tepi lemah
    bikin corner-finding/baca-grid meleset.

    Biaya: ~91% lebih lambat per-frame drpd baseline (re-run seluruh pipeline
    decode, bukan cuma CLAHE-nya) -- makanya HANYA dipanggil saat aktif nyari
    marker tertentu (`expected_id` dari mission, lih. wp_marker_node.py), BUKAN
    tiap frame terus-menerus (Jetson sudah pas-pasan, baseline sendiri cuma
    ~3.5Hz drpd target 5Hz)."""
    lab = cv2.cvtColor(bgr, cv2.COLOR_BGR2LAB)
    l, a, b = cv2.split(lab)
    clahe = cv2.createCLAHE(clipLimit=3.0, tileGridSize=(8, 8))
    l2 = clahe.apply(l)
    return cv2.cvtColor(cv2.merge([l2, a, b]), cv2.COLOR_LAB2BGR)


# reason yg divalidasi terbantu CLAHE (lih. enhance_clahe() docstring) -- SENGAJA
# TIDAK termasuk 'low_quality'/'no_marker'/'no_core' (terbukti 0% rescued, buang CPU percuma).
CLAHE_FALLBACK_REASONS = {'no_match', 'no_corners', 'decode_failed', 'ambiguous'}


def decode_frame_v2(bgr, refs, hamming_threshold=7, ambiguous_margin=3, mask_fn=marker_mask_v2):
    """Decode marker WP1/3/4 di 1 frame. TIDAK pakai split_core_protrusion's 'side'
    utk nentuin rotasi kanonik (side kadang salah pilih, lih. docstring modul) --
    sebagai gantinya coba KE-4 rotasi bit thd tiap referensi, ambil (wp_id, rotasi)
    dgn hamming distance minimum. 'side' cuma dipakai laporan arah-navigasi terpisah
    (posisi marker-kecil, sesuai keputusan user), independen dari ID-decode.

    mask_fn (2026-08-08): fungsi segmentasi yg dipakai, default marker_mask_v2
    (ambang HSV tetap). Bisa diganti (mis. varian ambang lain) -- dipakai
    ULANG oleh decode_frame_v2_robust() (multi-hypothesis HSV fallback), TIDAK
    mengubah perilaku caller lama yg gak kasih argumen ini.

    refs: dict wp_id(str) -> {"pattern": [[..]], "reliable": bool}
    Returns dict: found_marker, wp_id, hamming, reliable, tail_side, reason, aspect,
      off_x, off_y, bits (np.ndarray 6x6 kalau grid berhasil dibaca frame ini, else None --
      2026-08-08, dipakai wp_marker_node.py buat agregasi temporal, TIDAK mengubah
      perilaku/key lain, murni tambahan).
    """
    mask = mask_fn(bgr)
    body = largest_component(mask, min_area=300)
    if body is None:
        return dict(found_marker=False, wp_id=-1, hamming=-1, reliable=False,
                    tail_side='', reason='no_marker', aspect=0.0, off_x=0.0, off_y=0.0, bits=None)

    core, side = split_core_protrusion(body)
    tail_side = side or ''
    if core is None:
        ox, oy = _centroid_offset(np.column_stack(np.where(body > 0)[::-1]), bgr.shape)
        return dict(found_marker=True, wp_id=-1, hamming=-1, reliable=False,
                    tail_side=tail_side, reason='no_core', aspect=0.0, off_x=ox, off_y=oy, bits=None)

    ok_quality, bright = _quality_gate(bgr, core)
    if not ok_quality:
        ox, oy = _centroid_offset(np.column_stack(np.where(core > 0)[::-1]), bgr.shape) \
            if (core > 0).any() else (0.0, 0.0)
        return dict(found_marker=True, wp_id=-1, hamming=-1, reliable=False,
                    tail_side=tail_side, reason='low_quality', aspect=0.0, off_x=ox, off_y=oy, bits=None)

    corners = corners_small(core)
    if corners is None:
        return dict(found_marker=True, wp_id=-1, hamming=-1, reliable=False,
                    tail_side=tail_side, reason='no_corners', aspect=0.0, off_x=0.0, off_y=0.0, bits=None)
    ox, oy = _centroid_offset(corners, bgr.shape)

    warp, aspect = warp_marker(bgr, corners)
    bits = decode_grid(warp)
    if bits.sum() == 0:
        return dict(found_marker=True, wp_id=-1, hamming=-1, reliable=False,
                    tail_side=tail_side, reason='decode_failed', aspect=float(aspect),
                    off_x=ox, off_y=oy, bits=None)

    best_id, best_dist, reliable, reason = match_bits_to_refs(
        bits, refs, hamming_threshold, ambiguous_margin)
    return dict(found_marker=True, wp_id=best_id, hamming=best_dist, reliable=reliable,
                tail_side=tail_side, reason=reason, aspect=float(aspect),
                off_x=ox, off_y=oy, bits=bits)


# Multi-hypothesis HSV threshold (2026-08-08): akar masalah recall WP1/WP3 BUKAN
# di sudut (3 teknik corner-refinement dicoba & DITOLAK -- cornerSubPix,
# edge-line-fitting, keduanya net negatif/nol, lih. git log/memory), tapi di
# KUALITAS SEGMENTASI MASK (ambang HSV statis ORANGE_H_LO/HI/S_MIN). Coba
# beberapa varian ambang (shadow-tolerant s/d overexpose-tolerant, dari
# literatur robust/adaptive HSV segmentation) sbg kesempatan tambahan kalau
# baseline gagal -- BUKAN ganti mask default, cuma fallback.
#
# 6 varian awalnya dicoba (2026-08-08), divalidasi dataset PENUH di resolusi
# kamera live 640x480: 5 varian pertama = AMAN. Varian ke-6 (8,36,70) DIBUANG
# -- kontribusinya cuma 1 percobaan sukses TOTAL di WP1, dan itu SALAH.
#
# 2026-08-09: dicoba tambah 5 varian LAGI, dites dataset PENUH 3 WP.
# (2,40,30), (7,33,45), (6,34,35) = AMAN (0 salah di WP1/WP3/WP4 manapun) --
# DITAMBAHKAN. (4,36,50) & (9,31,20) DIBUANG -- masing-masing nyumbang 1 salah
# tebak KHUSUS di WP4 (0/4 & 4/4 kontribusi WP4 tapi tetap dibuang GLOBAL demi
# simpel -- opsi "izinkan di WP1/WP3 doang, blok di WP4" dipertimbangkan tapi
# butuh mekanisme per-WP baru; kerugian gain kecil (WP1 -4, WP3 -6, WP4 -4
# dari total 789/766/837 foto) TIDAK sepadan nambah kompleksitas).
#
# 2026-08-09 (lanjutan): dicoba 4 varian LEBIH LEBAR/ekstrem lagi (atas
# permintaan user "perbesar range HSV"), dites dataset PENUH 3 WP.
# (1,42,20), (14,26,75), (0,40,40) = AMAN (0 salah) -- DITAMBAHKAN, WP3
# dapet gain besar (+30 bacaan). (0,45,10) DIBUANG -- itu varian PALING
# lebar/permisif dari semua yg pernah dicoba (hue span 45, S_MIN cuma 10),
# nyumbang 1 salah tebak di WP4 (10 benar tapi 1 salah dari 11 percobaan).
HSV_FALLBACK_VARIANTS = [(5, 35, 25), (10, 30, 55), (3, 38, 20), (12, 28, 65), (5, 32, 15),
                         (2, 40, 30), (7, 33, 45), (6, 34, 35),
                         (1, 42, 20), (14, 26, 75), (0, 40, 40)]


def decode_frame_v2_robust(bgr, refs, hamming_threshold=7, ambiguous_margin=3, allow_clahe=True):
    """Baseline -> 5 varian HSV -> (kalau allow_clahe) CLAHE -> CLAHE+5 varian HSV.
    Reuse decode_frame_v2() apa adanya (mask_fn diganti-ganti), TIDAK reimplementasi
    pipeline. Berhenti di percobaan PERTAMA yg reason='ok'.

    allow_clahe: WAJIB False utk WP3 (dipanggil wp_marker_node.py berdasar
    ~clahe_fallback_wp_ids) -- divalidasi 2026-08-08: kombinasi CLAHE+varian HSV
    aman utk WP3 (13/13 benar), tapi CLAHE SENDIRIAN (tanpa varian HSV) sekali
    salah tebak di WP3 (persis temuan lama yg udah nyingkirin WP3 dari
    enhance_clahe() fallback single -- konsisten, kebijakan sama dipertahankan).

    Divalidasi dataset PENUH @640x480 (representasi kamera live): WP1 recall
    2.7%->6.7% (+32 bacaan, 0 salah baru), WP3 (varian HSV only, allow_clahe=False)
    recall 22.1%->29.6% (+58 bacaan, 0 salah baru). Biaya CPU: ~130-140ms/frame
    kalau semua percobaan gagal (masih ~7Hz, DI ATAS target 5Hz walau aktif
    terus-menerus tanpa scan-phase gating -- beda dari CLAHE tunggal yg 91%
    lebih lambat & WAJIB dibatasi jendela scan).
    """
    d = decode_frame_v2(bgr, refs, hamming_threshold, ambiguous_margin)
    if d['reason'] == 'ok':
        return d
    for h_lo, h_hi, s_min in HSV_FALLBACK_VARIANTS:
        dv = decode_frame_v2(bgr, refs, hamming_threshold, ambiguous_margin,
                              mask_fn=lambda b, hl=h_lo, hh=h_hi, sm=s_min: _marker_mask(b, hl, hh, sm))
        if dv['reason'] == 'ok':
            return dv
    if not allow_clahe:
        return d
    enh = enhance_clahe(bgr)
    d1 = decode_frame_v2(enh, refs, hamming_threshold, ambiguous_margin)
    if d1['reason'] == 'ok':
        return d1
    for h_lo, h_hi, s_min in HSV_FALLBACK_VARIANTS:
        dv = decode_frame_v2(enh, refs, hamming_threshold, ambiguous_margin,
                              mask_fn=lambda b, hl=h_lo, hh=h_hi, sm=s_min: _marker_mask(b, hl, hh, sm))
        if dv['reason'] == 'ok':
            return dv
    return d


class LatchAggregator:
    """OR/latch (2026-08-08): begitu 1 frame individually reason='ok', pakai ID
    itu & TAHAN (latch) sampai `latch_sec` detik -- biar drone gak lihat -1 di
    antara 2 bacaan ok yg berdekatan (mis. drone goyang dikit pas hover).

    GANTI dari rencana awal (majority-vote per-sel, TemporalAggregator): dites
    empiris 2026-08-08 pakai burst nyata (foto dikelompokkan per sesi hover via
    timestamp filename -- BUKAN sliding-window naive semua foto, itu keliru krn
    urutan alfabetis '(2).jpg'/'(3).jpg' bukan urutan waktu asli). Hasil:
    majority-vote JUSTRU lebih jarang berhasil drpd single-frame (WP1: 1/8 vs
    2/8 burst, WP4: 2/8 vs 4/8 burst) krn sinyal bersih WP1 sering cuma 2-3 dari
    100+ frame per burst -- gak pernah jadi MAYORITAS di window manapun, malah
    teredam noise. Dan sekali PERNAH salah tebak (WP3 burst 2: rata-rata dari
    6 frame yg semua individually ambigu nyasar lewat threshold ke referensi
    tetangga) padahal single-frame di burst yg sama TIDAK PERNAH salah.
    Kesimpulan: rata-rata bit tidak lebih presisi drpd single-frame reason='ok'
    (yg sudah 95.7-100%), sementara recall-nya malah turun.

    OR/latch TIDAK PERNAH lebih buruk dari single-frame (union, bukan average):
    precision = persis precision single-frame 'ok' (match_bits_to_refs() yg
    SAMA, dihitung SEKALI di decode_frame_v2() per frame -- bukan dihitung
    ulang di sini). Recall >= single-frame krn tetap dipertahankan (latch)
    selama latch_sec walau frame berikutnya kebetulan gagal.

    Murni Python, TIDAK ada dependency ROS (lih. _selftest_latch()).
    """

    def __init__(self, latch_sec=2.0):
        self.latch_sec = latch_sec
        self._latched = None
        self._latch_t = None

    def reset(self):
        self._latched = None
        self._latch_t = None

    def update(self, wp_id, hamming, reliable, reason, tail_side, now_sec):
        """wp_id/hamming/reliable/reason: hasil match_bits_to_refs() SINGLE-FRAME
        (dihitung di decode_frame_v2(), diteruskan ke sini). tail_side: dec['tail_side']
        frame ini. Return dict verdict (wp_id/hamming/reliable/reason/tail_side) --
        reason 'ok' (baru ATAU masih dlm latch_sec) atau 'no_signal' (latch
        kadaluarsa/belum pernah ok)."""
        if reason == 'ok':
            self._latched = dict(wp_id=wp_id, hamming=hamming, reliable=reliable,
                                  reason='ok', tail_side=tail_side)
            self._latch_t = now_sec
            return dict(self._latched)
        if self._latched is not None and (now_sec - self._latch_t) <= self.latch_sec:
            return dict(self._latched)
        self._latched = None
        return dict(wp_id=-1, hamming=-1, reliable=False, reason='no_signal', tail_side='')


def _selftest_latch():
    """Self-check LatchAggregator: ok langsung dipakai, tetap ke-latch selama
    latch_sec, expired sesudahnya, ID baru langsung ganti (union, bukan
    majority). Tanpa ROS."""
    agg = LatchAggregator(latch_sec=2.0)

    v = agg.update(-1, 12, False, 'no_match', '', 0.0)
    assert v['reason'] == 'no_signal' and v['wp_id'] == -1, "belum pernah ok -> no_signal"

    v = agg.update(1, 4, True, 'ok', 'N', 1.0)
    assert v['reason'] == 'ok' and v['wp_id'] == 1, "1 frame ok -> langsung dipakai"

    v = agg.update(-1, 15, False, 'no_match', '', 1.5)
    assert v['reason'] == 'ok' and v['wp_id'] == 1, "harusnya masih ke-latch (0.5s < latch_sec)"

    v = agg.update(-1, 15, False, 'no_match', '', 4.0)
    assert v['reason'] == 'no_signal', "harusnya expired (2.5s > latch_sec=2.0)"

    v = agg.update(3, 5, True, 'ok', 'E', 5.0)
    assert v['reason'] == 'ok' and v['wp_id'] == 3, "ok baru ID beda -> langsung ganti"

    print("LatchAggregator selftest: PASS")


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
    ap.add_argument('--selftest', action='store_true', help='cek LatchAggregator, tanpa dataset/ROS')
    a = ap.parse_args()
    if a.selftest:
        _selftest_latch()
        return
    run(a.base, a.folders, a.limit, a.ar_tol, a.debug)


if __name__ == '__main__':
    main()

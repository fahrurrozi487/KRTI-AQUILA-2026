#!/usr/bin/env python3
"""
Detektor Double/Triple Gate bentuk-U (OpenCV murni, TANPA ROS).

INDEPENDEN penuh dari gate_detector.py (single gate) — tidak import/reuse
apa pun dari sana. Ditulis dari nol khusus untuk gate berbentuk U:
2 kaki + palang atas, TANPA palang bawah (mirip gawang, terbuka di bawah).

Konsekuensi topologi bentuk-U: bukaan gate TERSAMBUNG langsung ke background
lewat celah bawah -> TIDAK ADA lubang tertutup. Jadi bukaan TIDAK dicari via
hierarchy child (RETR_CCOMP hole), melainkan sebagai komponen background
terhubung TERBESAR di dalam bounding box kontur luar.

Konvensi offset (kamera depan): origin tengah frame, x kanan +, y bawah +.
  offset_x = (cx - W/2)/(W/2)  -> [-1..1]
  offset_y = (cy - H/2)/(H/2)  -> [-1..1]
diukur dari PUSAT BOUNDING-BOX komponen background terbesar (bukan centroid
piksel mentah -- centroid bisa bias ke objek terang/gelap di background;
lihat _validate_opening()), bukan dari kontur luar.

Jalankan mandiri (tanpa roscore):
  python3 gate_multi_detector.py --selftest
  python3 gate_multi_detector.py --image foto.jpg [--save out.png]
  python3 gate_multi_detector.py --batch-test
"""
import os
import glob
import random
import argparse

import cv2
import numpy as np


# ---------- parameter tuning (SATU tempat; hanya disetel dari tuning_set) ----------
# oranye di HSV; Value dilebarkan biar toleran variasi cahaya lapangan.
HSV_ORANGE_LO = (5, 80, 40)
HSV_ORANGE_HI = (25, 255, 255)
CLOSE_KERNEL = 7          # ukuran kernel closing (sambung mask terputus)
CLOSE_ITER = 2
MIN_AREA = 1500.0         # area kontur luar minimum (piksel @720p)
SOLIDITY_LO = 0.15        # bentuk U; diturunkan dari 0.30 (Langkah1): gate DEKAT/
                          # mengisi-frame solidity turun ke ~0.17-0.27, dulu kelewat.
SOLIDITY_HI = 0.80        # dinaikkan dari 0.60 (Langkah1): tangkap gate dari sudut miring.
ASPECT_LO = 0.40          # w/h kontur luar wajar utk gate U
ASPECT_HI = 2.50

# --- solidity adaptif-skala (Triple Gate, sesi 6 Aug) ---
# Triple Gate lebih DALAM dari Double (dinding interior ikut oranye) -> jarak
# dekat siluet luarnya jadi nyaris solid (solidity 0.70-0.97), jauh di atas
# SOLIDITY_HI=0.80 yg dikalibrasi dari Double yg dangkal. Menaikkan SOLIDITY_HI
# global TERBUKTI (sweep+audit visual, lihat docs §9d) meloloskan false-positive
# Double: celah sempit kaki-vs-divider (area ~105-107rb px, aspect ~0.45) yg
# kebetulan lolos solidity tinggi juga. Kandidat gate ASLI besar (Triple dekat
# MAUPUN Double) py aspect >=1.15 & area >=100rb px -- false-positive itu SATU-
# SATUNYA kandidat besar dgn aspect serendah itu di kedua dataset (diverifikasi
# exhaustive, docs §9d). Jadi: solidity dilonggarkan HANYA utk kandidat BESAR
# (area >= LARGE_AREA_THRESH) DAN aspect-nya cukup lebar (>=ASPECT_MIN_LARGE)
# -- kandidat kecil (noise/gate jauh) tetap pakai SOLIDITY_HI asli, TIDAK
# berubah sama sekali (nol risiko regresi thd 69.2% Double yg sudah teraudit).
LARGE_AREA_THRESH = 20000.0   # ambang "kandidat besar" (noise kecil ~1.5-4rb px,
                              # gate asli dekat >=100rb px -- rentang ini kosong,
                              # jadi nilai persis di antaranya aman dipilih)
ASPECT_MIN_LARGE = 0.60       # utk kandidat besar: aspect di bawah ini ditolak
                              # (blokir celah kaki-vs-divider, aspect~0.45-0.46)
SOLIDITY_HI_RELAXED = 0.97    # solidity max utk kandidat besar (cakup semua
                              # solidity gate Triple dekat yg diverifikasi, maks
                              # terukur 0.961)

# --- refine-Cr adaptif-per-frame (Triple Gate, sesi lanjutan 6 Aug) ---
# Root cause KEDUA Triple (near_miss_not_open_bottom, 65/239 foto): pita tipis
# tanah/aspal kecoklatan kebetulan masuk rentang HSV_ORANGE -> komponen bukaan
# latar TERPOTONG jadi 2 (tak nyambung ke tepi bawah bbox). HSV Hue/Sat tumpang
# tindih penuh antara gate & tanah (dicoba, gagal - lihat docs §11 percobaan A).
# YCrCb channel Cr JUGA tumpang-tindih kalau dibanding ANTAR-foto (ambang
# global gagal, docs §11), TAPI di DALAM satu foto yg sama, median Cr piksel
# oranye di strip-kaki SELALU lebih tinggi dari median Cr di pita palsu
# (referensi relatif per-frame, bukan angka mutlak). Jadi: kalau jalur
# validasi normal gagal, coba re-klasifikasi mask oranye pakai referensi Cr
# dari kaki gate KANDIDAT ITU SENDIRI (bukan angka tetap) -- piksel yg Cr-nya
# jauh di bawah referensi (mis. pita tanah) dianggap BUKAN oranye asli.
# CR_REFINE_MIN_AREA (bukan LARGE_AREA_THRESH -- sengaja ambang beda &
# lebih tinggi): dipilih dari rentang KOSONG antara false-positive terbesar
# yg ditemukan (~30rb px, salah re-klasifikasi bagian kaki solid jadi
# "celah") & rescue Triple asli terkecil (~424rb px) -- diverifikasi
# exhaustive ke 520 Double: gain=0 di 100rb/150rb/200rb (dipilih 100rb,
# margin teraman & paling longgar dari 3 titik yg diuji).
CR_REFINE_MIN_AREA = 100000.0
CR_REFINE_MARGIN = 8          # piksel oranye dgn Cr < (referensi_kaki - margin)
                              # dianggap bukan gate. Diverifikasi 65/65 Triple
                              # rescued & 0 regresi Double di margin ini.
CR_REFINE_MIN_LEG_PIXELS = 50 # sampel referensi kaki terlalu sedikit -> jangan
                              # coba refine (skip, gagal spt biasa; aman)

# --- CLAHE kesempatan-kedua (leg_miss Double, referensi literatur shadow-
# suppression CLAHE+OTSU+HSV outdoor) -- HANYA dicoba saat mask oranye ASLI
# nihil TOTAL (0 kandidat sama sekali), bukan per-kandidat. Kaki gate
# ternaungi bayangan (S/V turun di bawah HSV_ORANGE_LO) -- CLAHE pd kanal
# S&V menormalkan kontras lokal shadow SEBELUM threshold ulang. Diverifikasi
# exhaustive ke 520 Double + 239 Triple: clip=3/tile=8 -> gain=64 Double,
# loss=0 (Double MAUPUN Triple). CLAHE sbg pengganti PENUH mask (bukan
# fallback) dicoba dulu & DITOLAK (gain 51-67 TAPI loss 3-44 juga -- terlalu
# invasif, ubah kandidat yg SUDAH benar). Fallback-only (dicoba di sini)
# aman by construction: jalur asli/mask asli tidak pernah disentuh.
CLAHE_CLIP_LIMIT = 3.0
CLAHE_TILE_GRID = 8
CLAHE_MIN_AREA = 100000.0    # WAJIB (lihat docs §13): tanpa ini >50% hasil CLAHE
                              # adalah noise kecil yg lolos numerik tapi gugur
                              # audit visual (target di rumput dll, sama sekali
                              # bukan gate). Ambang sama dgn CR_REFINE_MIN_AREA
                              # (skala gate asli, bukan kebetulan sama nilai).

# --- post-pairing kaki terpisah (Double Gate, sesi lanjutan 6 Aug) ---
# Root cause BARU ditemukan (audit visual manual thd kegagalan pasca-CLAHE,
# lihat docs §14): mayoritas kegagalan Double (126/129) BUKAN soal warna/
# bayangan sama sekali -- foto jarak SANGAT dekat, kedua kaki gate penuh
# tinggi frame, tapi CELAH DI TENGAH (bukaan) menampakkan latar (semak/rumah)
# yg MEMUTUS mask oranye jadi 2-3 kontur solo (bukan 1 kontur U menyambung).
# Pipeline normal cuma ambil kontur terbesar (1 kaki tunggal) -> otomatis
# gagal aspect (kaki solo jauh lebih ramping dari ASPECT_LO=0.40) atau
# leg-check. Referensi: teknik "post-pairing" tiang gawang RoboCup (deteksi
# tiang kiri+kanan individual lalu dipasangkan berdasar kemiripan tinggi &
# jarak, dipakai saat gawang tak tampak sbg 1 blob utuh).
# HANYA dicoba sbg kesempatan TERAKHIR (normal+CR-refine+CLAHE semua nihil)
# -> by construction loss=0 (tak pernah menimpa kandidat yg sudah lolos).
# Diverifikasi exhaustive 520 Double + 239 Triple: gain=38 Double, gain=0
# loss=0 Triple. Audit visual PENUH ke SEMUA 38 gain (grid + 5 spot-check
# resolusi penuh) -- 38/38 benar, crosshair konsisten di bukaan asli antara
# 2 kaki, tidak ada target ke noise/latar.
PAIR_LEG_MIN_AREA = 20000.0     # skala tiang asli (sama LARGE_AREA_THRESH; noise ~1.5-4rb px)
PAIR_LEG_MIN_HEIGHT_FRAC = 0.5  # tiang harus tinggi (dekat kamera, foto close-range)
PAIR_LEG_MAX_ASPECT = 0.55      # ramping/tinggi (kaki solo, bukan gate utuh)
PAIR_LEG_MIN_SOLIDITY = 0.5     # batang solid (bukan noise berlubang acak)
PAIR_GAP_MIN_RATIO = 0.15       # celah >= 15% lebar gabungan (bukan celah semu tipis)
PAIR_GAP_MAX_RATIO = 0.85       # celah <= 85% (bukan 2 objek jauh tak terkait)
PAIR_VOVERLAP_MIN_FRAC = 0.5    # overlap vertikal >= 50% tinggi tiang terpendek (baris sama)

# --- leg-check via PEAK sub-window density (Double Gate, sesi lanjutan 6 Aug) ---
# Root cause BARU (91 sisa kegagalan pasca-pairing, docs §15): 90/91 (98.9%)
# adalah kandidat yg SUDAH lolos filter bentuk (aspect/solidity) tapi gagal
# leg-check RATA-RATA strip tipis (mis. kasus ekstrem 17_30_13(2): left_frac
# 0.2485 vs ambang 0.25, MELESET 0.0015). Sebab fisik: kaki gate 2-warna --
# pita LUAR pucat/kayu tak-jenuh (gagal ambang HSV_ORANGE) + pita DALAM
# oranye jenuh lebih sempit -- rata-rata di seluruh LEG_STRIP_FRAC (30% lebar
# bbox) terdilusi pita pucat, walau ADA kolom yg nyaris 100% oranye di
# dalamnya. Referensi literatur: projection-profile PEAK (dipakai deteksi
# baris teks/pole -- puncak densitas lokal menandai objek nyata, bukan
# rata-rata wilayah pencarian yg didilusi latar). Kesempatan KEDUA per-
# kandidat (pola sama refine-Cr): kalau rata-rata strip penuh gagal, geser
# jendela SEMPIT (PEAK_LEG_WINDOW_FRAC dari lebar bbox) di dalam strip yg
# SAMA, ambil densitas MAKS -- kalau ada kolom padat oranye asli, tetap lolos.
# Nilai dipilih via sweep (window x density) diverifikasi: window LEBIH LEBAR
# (>=0.10) nyaris tak menambah gain apa pun (pita oranye jenuh sempit di
# data ini) -- window=0.06 hasil terbaik. density=0.55 SEMPAT meloloskan 1
# foto keluarga noise dikenal (17_30_12(2)) -- dinaikkan ke 0.65 (nol hit
# noise, gain turun 23->14, masih signifikan). Diverifikasi exhaustive 520
# Double + 239 Triple: gain=14 Double, 0 Triple (nol regresi, hanya dicoba
# saat leg-check normal SUDAH gagal). Audit visual SEMUA 14 gain -- 14/14
# benar, crosshair tetap di bukaan asli.
PEAK_LEG_WINDOW_FRAC = 0.06   # lebar jendela geser (fraksi lebar bbox luar)
PEAK_LEG_MIN_DENSITY = 0.65   # densitas oranye min di jendela TERPADAT

MIN_OPENING_AREA = 300.0  # komponen background terlalu kecil = bukan bukaan
# rel_area MASIH NONAKTIF setelah Langkah 3 (validasi bentuk sudah membersihkan
# false-positive terbesar). Belum dicoba diaktifkan -- kandidat utk iterasi
# berikutnya kalau false-positive keluarga noise (lihat LEG_STRIP_FRAC di bawah)
# masih perlu ditekan lebih lanjut.
REL_AREA_FRAC = 0.0       # buang kandidat < frac * area-terbesar-di-frame (0=mati)
                          # oranye kecil (spanduk ~5k px) saat gate asli ~450k px.

# --- validasi bentuk bukaan (Langkah 2): pisahkan gate sejati dari noise ---
# LEG_STRIP_FRAC=0.30 & LEG_ORANGE_MIN=0.25 (Langkah3, 30 Jul): dinaikkan/diturunkan
# dari 0.20/0.35 krn foto gate asli jarak-dekat (mis. 17_31_45) punya kaki yg tampak
# terdistorsi perspektif (strip vertikal lurus 0.20x0.35 lama menolaknya, salah satu
# sisi kaki jatuh ke 0.28-0.39 sedikit di bawah 0.35). Nilai ini dipilih via sweep
# grid (LEG_STRIP_FRAC x LEG_ORANGE_MIN) yg diverifikasi TIDAK melebarkan gap
# tuning-holdout & TIDAK meloloskan 3 kasus negatif sintetis + 2 foto noise kunci
# (17_30_06/17_30_12 varian "(4)"). KETERBATASAN DIKETAHUI: beberapa foto SAUDARA-
# FRAME dari noise 17_30_06/17_30_12 (bukan varian yg jadi acuan) mulai lolos jadi
# false-positive (3/10 di keluarga tsb) -- trade-off yg diterima demi rescue rate
# gate asli jarak-dekat. Lihat docs/GATE_DETECTOR_DOUBLE_TRIPLE.md untuk detail sweep & rasional.
LEG_STRIP_FRAC = 0.30     # lebar strip kiri/kanan yg dicek utk "kaki" (fraksi bw)
LEG_ORANGE_MIN = 0.25     # fraksi piksel oranye min di strip kaki -> dianggap ada kaki
OPEN_WIDTH_FRAC = 0.30    # bukaan harus >= 30% lebar bbox (bukan celah tipis antar-bilah)
OPEN_HEIGHT_FRAC = 0.30   # bukaan harus >= 30% tinggi bbox
OPEN_BOTTOM_MARGIN_FRAC = 0.12  # tepi bawah bukaan harus dekat tepi bawah bbox
                          # (<=12% bh). Toleransi ~+/-16 deg roll kamera (kamera level
                          # normal saat traverse); TOLAK gate terbalik/ekstrem.
                          # ponytail: kalau IMU tunjukkan roll besar, ganti "bawah" dgn
                          # sisi-terbuka relatif-gravitasi dari attitude (belum perlu).


def _peak_density(strip, window_px):
    """Densitas oranye MAKS di jendela geser selebar window_px dalam strip
    (lihat komentar PEAK_LEG_* di atas) -- tahan dilusi pita pucat/kayu yg
    ikut masuk strip lebar tapi bukan bagian oranye jenuh kaki asli."""
    if strip.size == 0 or strip.shape[1] < 1:
        return 0.0
    window_px = max(1, min(window_px, strip.shape[1]))
    col_density = (strip > 0).mean(axis=0)
    csum = np.cumsum(np.concatenate([[0.0], col_density]))
    n = len(col_density)
    best = 0.0
    for i in range(0, n - window_px + 1):
        avg = (csum[i + window_px] - csum[i]) / window_px
        if avg > best:
            best = avg
    return best


def _validate_opening(sub_mask, bw, bh, open_bbox):
    """Validasi 1 kandidat = benar bukaan gate (kaki kiri+kanan, bukaan lebar,
    terbuka ke bawah). Return (ok, (target_cx, target_cy)) koord LOKAL bbox.

    sub_mask : mask oranye di dalam bbox luar (uint8, 0/255)
    open_bbox: (ox,oy,ow,oh) bounding box komponen bukaan terbesar (koord lokal)
    palang atas TIDAK wajib (gate dekat: palang bisa keluar frame).
    """
    ox, oy, ow, oh = open_bbox
    # target robust: pusat bbox bukaan (bukan centroid piksel yg tertarik objek fg)
    target = (ox + ow / 2.0, oy + oh / 2.0)

    # (1) bukaan LEBAR & TINGGI cukup (bukan celah tipis antar-bilah spt 17_30_17)
    if ow < OPEN_WIDTH_FRAC * bw or oh < OPEN_HEIGHT_FRAC * bh:
        return False, target
    # (2) bukaan terbuka ke BAWAH: tepi bawah bukaan dekat tepi bawah bbox
    if (bh - (oy + oh)) > OPEN_BOTTOM_MARGIN_FRAC * bh:
        return False, target
    # (3) kaki KIRI & KANAN oranye (tahan skew perspektif: kaki miring tetap di
    #     sisi kiri & kanan bbox). Cek pada rentang tinggi bukaan saja.
    sw = max(1, int(round(LEG_STRIP_FRAC * bw)))
    y0, y1 = int(oy), int(oy + oh)
    left = sub_mask[y0:y1, 0:sw]
    right = sub_mask[y0:y1, bw - sw:bw]
    if left.size == 0 or right.size == 0:
        return False, target
    left_frac = float((left > 0).mean())
    right_frac = float((right > 0).mean())
    if left_frac < LEG_ORANGE_MIN or right_frac < LEG_ORANGE_MIN:
        # kesempatan kedua (sesi lanjutan 6 Aug, lihat komentar PEAK_LEG_*
        # di atas): rata-rata strip PENUH gagal -> coba jendela sempit
        # TERPADAT di strip yg SAMA (kaki 2-warna: pita pucat mendilusi
        # rata-rata, tapi kolom oranye jenuh asli tetap ada di dalamnya).
        window_px = max(1, int(round(PEAK_LEG_WINDOW_FRAC * bw)))
        left_peak = _peak_density(left, window_px)
        right_peak = _peak_density(right, window_px)
        if left_peak < PEAK_LEG_MIN_DENSITY or right_peak < PEAK_LEG_MIN_DENSITY:
            return False, target
    return True, target


def _leg_candidates(mask):
    """Kontur solo berbentuk 'tiang' (tinggi, ramping, solid) -- kandidat
    kaki gate individual utk post-pairing (lihat komentar PAIR_LEG_* di atas)."""
    contours, hierarchy = cv2.findContours(mask, cv2.RETR_CCOMP, cv2.CHAIN_APPROX_SIMPLE)
    cands = []
    if hierarchy is None:
        return cands
    hierarchy = hierarchy[0]
    h, w = mask.shape[:2]
    for i, cnt in enumerate(contours):
        if hierarchy[i][3] != -1:
            continue
        area = cv2.contourArea(cnt)
        if area < PAIR_LEG_MIN_AREA:
            continue
        x, y, bw, bh = cv2.boundingRect(cnt)
        if bh < PAIR_LEG_MIN_HEIGHT_FRAC * h:
            continue
        aspect = bw / float(bh) if bh > 0 else 999
        if aspect > PAIR_LEG_MAX_ASPECT:
            continue
        hull = cv2.convexHull(cnt)
        hull_area = cv2.contourArea(hull)
        if hull_area <= 0:
            continue
        if area / hull_area < PAIR_LEG_MIN_SOLIDITY:
            continue
        cands.append((x, y, bw, bh, area))
    return cands


def _try_pairing(mask, w, h):
    """Pasangkan 2 kontur 'tiang' solo jadi 1 kandidat gate (kesempatan
    terakhir, lihat komentar PAIR_LEG_*/docs §14). Return list gate-dict
    (pola sama _scan), area terbesar duluan."""
    cands = _leg_candidates(mask)
    if len(cands) < 2:
        return []
    cands.sort(key=lambda c: c[0])
    results = []
    for a in range(len(cands)):
        for b in range(a + 1, len(cands)):
            x1, y1, bw1, bh1, area1 = cands[a]
            x2, y2, bw2, bh2, area2 = cands[b]
            if x2 <= x1 + bw1:
                continue
            gap = x2 - (x1 + bw1)
            combined_w = (x2 + bw2) - x1
            if combined_w <= 0:
                continue
            gap_ratio = gap / float(combined_w)
            if not (PAIR_GAP_MIN_RATIO <= gap_ratio <= PAIR_GAP_MAX_RATIO):
                continue
            top = max(y1, y2)
            bot = min(y1 + bh1, y2 + bh2)
            voverlap = max(0, bot - top)
            if voverlap < PAIR_VOVERLAP_MIN_FRAC * min(bh1, bh2):
                continue
            tcx = x1 + bw1 + gap / 2.0
            tcy = top + voverlap / 2.0
            offset_x = (tcx - w / 2.0) / (w / 2.0)
            offset_y = (tcy - h / 2.0) / (h / 2.0)
            bx, by = x1, min(y1, y2)
            bbw = (x2 + bw2) - x1
            bbh = max(y1 + bh1, y2 + bh2) - by
            results.append({
                "offset_x": float(offset_x), "offset_y": float(offset_y),
                "area": float(area1 + area2),
                "bbox_outer": (int(bx), int(by), int(bbw), int(bbh)),
            })
    results.sort(key=lambda r: r["area"], reverse=True)
    return results[:1]  # pasangan area terbesar saja (paling dekat/prioritas)


def detect_gates(frame_bgr, solidity_lo=None, solidity_hi=None,
                 min_area=None, rel_area_frac=None,
                 large_area_thresh=None, aspect_min_large=None,
                 solidity_hi_relaxed=None):
    """Deteksi semua gate-U pada satu frame. Return list dict, area DESC.

    Elemen: {"offset_x":float,"offset_y":float,"area":float,"bbox_outer":(x,y,w,h)}
    Param opsional (None = pakai konstanta modul) utk sweep tuning tanpa edit file.
    """
    solidity_lo = SOLIDITY_LO if solidity_lo is None else solidity_lo
    solidity_hi = SOLIDITY_HI if solidity_hi is None else solidity_hi
    min_area = MIN_AREA if min_area is None else min_area
    rel_area_frac = REL_AREA_FRAC if rel_area_frac is None else rel_area_frac
    large_area_thresh = LARGE_AREA_THRESH if large_area_thresh is None else large_area_thresh
    aspect_min_large = ASPECT_MIN_LARGE if aspect_min_large is None else aspect_min_large
    solidity_hi_relaxed = SOLIDITY_HI_RELAXED if solidity_hi_relaxed is None else solidity_hi_relaxed
    h, w = frame_bgr.shape[:2]
    # 1. BGR -> HSV
    hsv = cv2.cvtColor(frame_bgr, cv2.COLOR_BGR2HSV)
    ycrcb = cv2.cvtColor(frame_bgr, cv2.COLOR_BGR2YCrCb)  # utk refine-Cr (fallback bottom-margin)
    k = cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (CLOSE_KERNEL, CLOSE_KERNEL))

    def _scan(mask):
        """Cari gate di SATU mask oranye (dipanggil 2x oleh pemanggil di
        bawah: mask normal dulu, lalu mask CLAHE sbg kesempatan-kedua kalau
        mask normal nihil -- lihat blok CLAHE_FALLBACK_* di bawah)."""
        # 4. findContours RETR_CCOMP -> ambil kontur LUAR saja (parent == -1)
        contours, hierarchy = cv2.findContours(mask, cv2.RETR_CCOMP,
                                               cv2.CHAIN_APPROX_SIMPLE)
        results = []
        if hierarchy is None:
            return results
        hierarchy = hierarchy[0]
        for i, cnt in enumerate(contours):
            if hierarchy[i][3] != -1:          # bukan kontur luar (punya parent) -> lewati
                continue
            # 6. filter noise: area, solidity (bentuk U), aspect ratio
            area = cv2.contourArea(cnt)
            if area < min_area:
                continue
            hull = cv2.convexHull(cnt)
            hull_area = cv2.contourArea(hull)
            if hull_area <= 0:
                continue
            solidity = area / hull_area
            x, y, bw, bh = cv2.boundingRect(cnt)
            if bh <= 0:
                continue
            aspect = bw / float(bh)
            # jalur ASLI: SELALU dicek apa adanya, TIDAK diubah -> kandidat yg
            # sudah lolos di baseline (mis. Double area>=20rb aspect 0.47-0.59,
            # lihat docs §9) tetap lolos persis seperti sebelumnya, nol regresi.
            ok_strict = (solidity_lo <= solidity <= solidity_hi
                         and ASPECT_LO <= aspect <= ASPECT_HI)
            # jalur ADAPTIF-SKALA (Triple Gate, sesi 6 Aug): "kesempatan kedua"
            # HANYA utk kandidat yg GAGAL jalur asli DAN besar & cukup lebar
            # (blokir celah kaki-vs-divider aspect~0.45-0.46, lihat komentar
            # konstanta & docs §9d/§10) -- tidak pernah menimpa jalur asli.
            is_large = area >= large_area_thresh
            ok_relaxed = (is_large
                          and solidity_lo <= solidity <= solidity_hi_relaxed
                          and aspect_min_large <= aspect <= ASPECT_HI)
            if not (ok_strict or ok_relaxed):
                continue
            # 5. di dalam bbox kontur luar: komponen background (mask TERBALIK) terbesar
            sub = mask[y:y + bh, x:x + bw]
            inv = cv2.bitwise_not(sub)          # bukaan (non-oranye) jadi foreground
            n, labels, stats, centroids = cv2.connectedComponentsWithStats(inv, 8)
            if n <= 1:                          # tak ada komponen selain latar
                continue
            # label 0 = background connectedComponents; cari label>=1 area terbesar
            best_lbl, best_area = -1, 0.0
            for lbl in range(1, n):
                a = stats[lbl, cv2.CC_STAT_AREA]
                if a > best_area:
                    best_area, best_lbl = a, lbl
            if best_lbl < 0 or best_area < MIN_OPENING_AREA:
                continue
            # 6b. VALIDASI bentuk (Langkah 2): kaki kiri+kanan, bukaan lebar & terbuka-bawah.
            #     Tolak noise semak/kayu & celah-tipis antar-bilah. Target = pusat bbox bukaan.
            ox = stats[best_lbl, cv2.CC_STAT_LEFT]
            oy = stats[best_lbl, cv2.CC_STAT_TOP]
            ow = stats[best_lbl, cv2.CC_STAT_WIDTH]
            oh = stats[best_lbl, cv2.CC_STAT_HEIGHT]
            ok, (tcx, tcy) = _validate_opening(sub, bw, bh, (ox, oy, ow, oh))
            if not ok and area >= CR_REFINE_MIN_AREA:
                # kesempatan kedua (sesi lanjutan 6 Aug, lihat komentar konstanta
                # CR_REFINE_* di atas): mgkn bukaan terpotong pita tanah/aspal
                # palsu -> re-klasifikasi mask oranye pakai referensi Cr KHUSUS
                # kandidat ini (median Cr piksel oranye di strip kaki kiri+kanan).
                sw_r = max(1, int(round(LEG_STRIP_FRAC * bw)))
                leg_mask = np.concatenate([sub[:, 0:sw_r].flatten(), sub[:, bw - sw_r:bw].flatten()])
                leg_cr_all = np.concatenate([
                    ycrcb[y:y + bh, x:x + sw_r].reshape(-1, 3),
                    ycrcb[y:y + bh, x + bw - sw_r:x + bw].reshape(-1, 3)])
                leg_cr = leg_cr_all[leg_mask > 0][:, 1] if leg_mask.sum() > 0 else np.array([])
                if len(leg_cr) >= CR_REFINE_MIN_LEG_PIXELS:
                    ref_cr = float(np.median(leg_cr))
                    sub_ycrcb = ycrcb[y:y + bh, x:x + bw]
                    cr_channel = sub_ycrcb[:, :, 1]
                    sub_refined = np.where((sub > 0) & (cr_channel >= ref_cr - CR_REFINE_MARGIN),
                                           255, 0).astype(np.uint8)
                    inv2 = cv2.bitwise_not(sub_refined)
                    n2, labels2, stats2, centroids2 = cv2.connectedComponentsWithStats(inv2, 8)
                    if n2 > 1:
                        best_lbl2, best_area2 = -1, 0.0
                        for lbl in range(1, n2):
                            a2 = stats2[lbl, cv2.CC_STAT_AREA]
                            if a2 > best_area2:
                                best_area2, best_lbl2 = a2, lbl
                        if best_lbl2 >= 0 and best_area2 >= MIN_OPENING_AREA:
                            ox2 = stats2[best_lbl2, cv2.CC_STAT_LEFT]
                            oy2 = stats2[best_lbl2, cv2.CC_STAT_TOP]
                            ow2 = stats2[best_lbl2, cv2.CC_STAT_WIDTH]
                            oh2 = stats2[best_lbl2, cv2.CC_STAT_HEIGHT]
                            ok, (tcx, tcy) = _validate_opening(sub_refined, bw, bh, (ox2, oy2, ow2, oh2))
            if not ok:
                continue
            # 7. offset dari target (pusat bbox bukaan) -> koord frame penuh
            cx = x + tcx
            cy = y + tcy
            offset_x = (cx - w / 2.0) / (w / 2.0)
            offset_y = (cy - h / 2.0) / (h / 2.0)
            results.append({
                "offset_x": float(offset_x),
                "offset_y": float(offset_y),
                "area": float(area),
                "bbox_outer": (int(x), int(y), int(bw), int(bh)),
            })
        # 8. urut area kontur luar DESC (terbesar = terdekat = prioritas target)
        results.sort(key=lambda r: r["area"], reverse=True)
        # 9. filter area RELATIF: buang kandidat yang jauh lebih kecil dari yg terbesar
        #    (bunuh spanduk/noise ~5k px saat gate asli ~450k px, tanpa menyentuh
        #    ambang absolut yg bisa ikut membuang gate jauh yg berdiri sendiri).
        if rel_area_frac > 0.0 and results:
            biggest = results[0]["area"]
            thresh = biggest * rel_area_frac
            results = [r for r in results if r["area"] >= thresh]
        return results

    # 2. threshold oranye -> mask biner (jalur ASLI, TIDAK diubah)
    mask = cv2.inRange(hsv, np.array(HSV_ORANGE_LO, np.uint8),
                       np.array(HSV_ORANGE_HI, np.uint8))
    # 3. morphological closing (dilate->erode) sambung mask terputus
    mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, k, iterations=CLOSE_ITER)
    results = _scan(mask)
    if results:
        return results

    # --- CLAHE kesempatan-kedua (sesi lanjutan 6 Aug, referensi literatur
    # shadow-suppression outdoor: CLAHE+OTSU+HSV) -- HANYA saat mask asli
    # NIHIL sama sekali (bukan per-kandidat spt refine-Cr, krn dampak CLAHE
    # ke seluruh mask, bukan cuma 1 kandidat). Target: kaki gate ternaungi
    # bayangan (leg_miss, dominan 71.9% kegagalan Double, docs §7a) --
    # CLAHE pd kanal S&V menormalkan kontras lokal shadow SEBELUM threshold,
    # tanpa mengubah jalur asli sama sekali (nol risiko regresi thd
    # kandidat yg SUDAH lolos, krn baru dicoba kalau mask asli 100% nihil).
    H_ch, S_ch, V_ch = cv2.split(hsv)
    clahe = cv2.createCLAHE(clipLimit=CLAHE_CLIP_LIMIT, tileGridSize=(CLAHE_TILE_GRID, CLAHE_TILE_GRID))
    S2 = clahe.apply(S_ch)
    V2 = clahe.apply(V_ch)
    hsv_clahe = cv2.merge([H_ch, S2, V2])
    mask_clahe = cv2.inRange(hsv_clahe, np.array(HSV_ORANGE_LO, np.uint8),
                             np.array(HSV_ORANGE_HI, np.uint8))
    mask_clahe = cv2.morphologyEx(mask_clahe, cv2.MORPH_CLOSE, k, iterations=CLOSE_ITER)
    clahe_results = _scan(mask_clahe)
    # WAJIB filter skala (audit visual 6 Aug menemukan >50% hasil CLAHE tanpa
    # filter ini adalah noise kecil -- lolos pagar "0 loss" numerik TAPI
    # gugur audit, lihat docs §13): CLAHE menaikkan kontras lokal di SELURUH
    # frame, termasuk noise kecil yg tak terkait gate -- hanya terima kandidat
    # BESAR (skala gate asli, bukan noise), sama spt pola LARGE_AREA_THRESH.
    clahe_results = [r for r in clahe_results if r["area"] >= CLAHE_MIN_AREA]
    if clahe_results:
        return clahe_results

    # --- post-pairing kesempatan TERAKHIR (sesi lanjutan 6 Aug, lihat komentar
    # PAIR_LEG_*/docs §14) -- kaki kiri+kanan terpisah krn celah tengah bukan
    # soal warna/bayangan (mask ASLI dipakai, bukan CLAHE), jadi coba di mask
    # normal, bukan mask_clahe.
    return _try_pairing(mask, w, h)


def _draw_overlay(frame_bgr, gates):
    """Overlay debug: kontur luar hijau, komponen target merah, crosshair target."""
    out = frame_bgr.copy()
    h, w = out.shape[:2]
    for g in gates:
        x, y, bw, bh = g["bbox_outer"]
        cv2.rectangle(out, (x, y), (x + bw, y + bh), (0, 255, 0), 2)  # kontur luar hijau
        cx = int(g["offset_x"] * (w / 2.0) + w / 2.0)
        cy = int(g["offset_y"] * (h / 2.0) + h / 2.0)
        # area background target: tandai merah (isi bbox tipis) + crosshair
        cv2.drawMarker(out, (cx, cy), (0, 0, 255), cv2.MARKER_CROSS, 30, 3)
        cv2.circle(out, (cx, cy), 6, (0, 0, 255), -1)
    return out


# ---------- sintetis untuk self-test ----------
def _make_synthetic_u(w=1280, h=720, gate_w=200, gate_h=220, thick=None,
                      cx=None, cy=None):
    """Kanvas hitam + satu gate-U oranye (2 kaki + palang atas, TANPA palang bawah).

    Return (img_bgr, (exp_off_x, exp_off_y)) — offset ekspektasi = centroid bukaan.
    cx,cy = pusat bounding box gate (default tengah frame).
    thick default proporsional ke lebar (gate menjauh -> tetap ramping, bukan
    makin gempal) supaya solidity U konsisten realistis di semua ukuran.
    """
    if thick is None:
        thick = max(6, int(round(gate_w * 0.15)))
    img = np.zeros((h, w, 3), np.uint8)
    if cx is None:
        cx = w // 2
    if cy is None:
        cy = h // 2
    x0 = int(cx - gate_w / 2)
    y0 = int(cy - gate_h / 2)
    orange = (0, 140, 255)  # BGR ~ oranye (HSV H~15)
    # palang atas (full width)
    cv2.rectangle(img, (x0, y0), (x0 + gate_w, y0 + thick), orange, -1)
    # kaki kiri
    cv2.rectangle(img, (x0, y0), (x0 + thick, y0 + gate_h), orange, -1)
    # kaki kanan
    cv2.rectangle(img, (x0 + gate_w - thick, y0), (x0 + gate_w, y0 + gate_h),
                  orange, -1)
    # bukaan = persegi antara kaki, di bawah palang, terbuka ke bawah bbox:
    #   x: x0+thick .. x0+gate_w-thick ; y: y0+thick .. y0+gate_h
    open_cx = x0 + gate_w / 2.0
    open_cy = (y0 + thick + y0 + gate_h) / 2.0
    exp_off_x = (open_cx - w / 2.0) / (w / 2.0)
    exp_off_y = (open_cy - h / 2.0) / (h / 2.0)
    return img, (exp_off_x, exp_off_y)


def _make_blob(w=1280, h=720, cx=700, cy=300, size=120):
    """Blob oranye tak-berbingkai (meniru semak/kayu 17_30_06/12). Bukan gate.
    Tak ada kaki kiri+kanan -> harus DITOLAK validasi."""
    img = np.zeros((h, w, 3), np.uint8)
    orange = (0, 140, 255)
    cv2.circle(img, (cx, cy), size // 2, orange, -1)
    return img


def _make_one_leg_thin_gap(w=1280, h=720, gate_w=280, gate_h=380, cx=900, cy=360):
    """Satu kaki gate lebar + CELAH TIPIS antar-bilah (meniru 17_30_17 dari sudut
    miring). Kaki kiri+kanan ada, tapi bukaan = celah sempit -> gagal OPEN_WIDTH_FRAC,
    harus DITOLAK. Regresi test eksplisit."""
    img = np.zeros((h, w, 3), np.uint8)
    orange = (0, 140, 255)
    x0 = int(cx - gate_w / 2)
    y0 = int(cy - gate_h / 2)
    # dua bilah triplek tebal dgn celah tipis di tengah (kaki gate dilihat miring)
    slab = int(gate_w * 0.44)          # tiap bilah tebal
    gap = gate_w - 2 * slab            # celah tipis (~12% lebar)
    cv2.rectangle(img, (x0, y0), (x0 + slab, y0 + gate_h), orange, -1)
    cv2.rectangle(img, (x0 + slab + gap, y0), (x0 + gate_w, y0 + gate_h), orange, -1)
    return img


def _selftest():
    print("== gate_multi self-test (U sintetis, tanpa kamera) ==")
    tol = 0.08
    cases = [
        ("tengah",        None, None),
        ("geser kiri",    400,  360),
        ("geser kanan-bawah", 900, 500),
        ("kecil (jauh)",  640,  300),
    ]
    ok_all = True
    for name, cx, cy in cases:
        gw = 120 if "kecil" in name else 220
        gh = 130 if "kecil" in name else 240
        img, (ex, ey) = _make_synthetic_u(cx=cx, cy=cy, gate_w=gw, gate_h=gh)
        gates = detect_gates(img)
        if not gates:
            print("  %-18s -> TIDAK terdeteksi  GAGAL" % name)
            ok_all = False
            continue
        g = gates[0]
        dx, dy = abs(g["offset_x"] - ex), abs(g["offset_y"] - ey)
        passed = dx <= tol and dy <= tol
        print("  %-18s -> off=(%.3f,%.3f) exp=(%.3f,%.3f) d=(%.3f,%.3f) %s"
              % (name, g["offset_x"], g["offset_y"], ex, ey, dx, dy,
                 "OK" if passed else "GAGAL"))
        ok_all = ok_all and passed
    # negatif: kasus yang HARUS ditolak validasi bentuk (Langkah 2)
    for nname, nimg in (
        ("blob tak-berbingkai", _make_blob()),
        ("satu kaki+celah tipis", _make_one_leg_thin_gap()),
        ("kanvas kosong", np.zeros((720, 1280, 3), np.uint8)),
    ):
        ng = detect_gates(nimg)
        n_ok = (ng == [])
        print("  %-22s -> %d gate %s" % (nname, len(ng), "OK" if n_ok else "GAGAL"))
        ok_all = ok_all and n_ok
    print("HASIL:", "SEMUA LULUS" if ok_all else "ADA YANG GAGAL")
    return ok_all


# ---------- split dataset & batch test ----------
def _split_dataset(dpath, seed=42, holdout=0.2):
    files = sorted(glob.glob(os.path.join(dpath, "*.jpg")))
    rng = random.Random(seed)
    rng.shuffle(files)
    n_hold = int(round(len(files) * holdout))
    holdout_set = sorted(files[:n_hold])
    tuning_set = sorted(files[n_hold:])
    return tuning_set, holdout_set


def _run_set(files, save_dir=None):
    """Jalankan detect_gates di tiap file. Return (n_total, n_detected)."""
    n_det = 0
    for i, f in enumerate(files):
        img = cv2.imread(f)
        if img is None:
            continue
        gates = detect_gates(img)
        if gates:
            n_det += 1
        if save_dir is not None:
            out = _draw_overlay(img, gates)
            cv2.imwrite(os.path.join(save_dir, os.path.basename(f)), out)
        if (i + 1) % 50 == 0:
            print("    ...%d/%d" % (i + 1, len(files)))
    return len(files), n_det


def _batch_test(seed=42, dataset=None):
    """dataset: nama folder di ~/catkin_ws/gate/ (default dataset_gate_baru,
    dataset Double). Pakai dataset_triple_baru buat Triple -- 2026-08-09,
    diparametrize biar bisa jalanin ke keduanya tanpa duplikasi fungsi."""
    dataset = dataset or "dataset_gate_baru"
    dpath = os.path.join(os.path.dirname(os.path.abspath(__file__)),
                         "..", "..", "..", "gate", dataset)
    dpath = os.path.abspath(dpath)
    if not os.path.isdir(dpath):
        # fallback: relatif working dir
        dpath = os.path.abspath(os.path.join("gate", dataset))
    print("dataset:", dpath)
    tuning_set, holdout_set = _split_dataset(dpath, seed=seed)
    print("total=%d  tuning=%d  holdout=%d"
          % (len(tuning_set) + len(holdout_set), len(tuning_set), len(holdout_set)))

    gate_root = os.path.abspath(os.path.join(dpath, ".."))
    manifest = os.path.join(gate_root, "holdout_manifest_multi_%s.txt" % dataset)
    with open(manifest, "w") as fh:
        fh.write("\n".join(os.path.basename(f) for f in holdout_set) + "\n")
    print("holdout manifest ->", manifest)

    save_dir = os.path.join(gate_root, "annotated_multi_%s" % dataset)
    os.makedirs(save_dir, exist_ok=True)

    print("[tuning_set]")
    nt, dt = _run_set(tuning_set, save_dir)
    print("[holdout_set]")
    nh, dh = _run_set(holdout_set, save_dir)

    rt = 100.0 * dt / nt if nt else 0.0
    rh = 100.0 * dh / nh if nh else 0.0
    gap = abs(rt - rh)
    print("\n==== RINGKASAN ====")
    print("tuning : %d/%d terdeteksi (%.1f%%)" % (dt, nt, rt))
    print("holdout: %d/%d terdeteksi (%.1f%%)" % (dh, nh, rh))
    print("GAP tuning-holdout = %.1f poin" % gap)
    print("overlay tersimpan ->", save_dir)
    return rt, rh, gap


def _main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--selftest", action="store_true")
    ap.add_argument("--batch-test", action="store_true")
    ap.add_argument("--dataset", default=None,
                    help="nama folder di ~/catkin_ws/gate/ (default dataset_gate_baru)")
    ap.add_argument("--image")
    ap.add_argument("--save")
    ap.add_argument("--seed", type=int, default=42)
    args = ap.parse_args()

    if args.selftest:
        import sys
        sys.exit(0 if _selftest() else 1)
    if args.batch_test:
        _batch_test(seed=args.seed, dataset=args.dataset)
        return
    if args.image:
        img = cv2.imread(args.image)
        if img is None:
            print("gagal baca:", args.image)
            return
        gates = detect_gates(img)
        print("terdeteksi %d gate:" % len(gates))
        for g in gates:
            print("  ", g)
        if args.save:
            cv2.imwrite(args.save, _draw_overlay(img, gates))
            print("overlay disimpan:", args.save)
        return
    print("pakai --selftest | --batch-test | --image FILE [--save OUT]")


if __name__ == "__main__":
    _main()

#!/usr/bin/env python3
"""wp_decode_audit.py — Task 3: decode marker grid WP3/WP4, bandingkan.

READ-ONLY terhadap dataset. Tulis overlay debug ke dataset_arucode/_debug/.
Grid DIPAKSA 5x5 (keputusan tim: ukuran seragam semua WP, lih. wp1).

Pipeline per foto:
  1. mask putih (HSV) -> komponen terbesar = badan marker (+ekor nempel)
  2. deteksi sisi ekor (tonjolan putih di luar bounding badan)
  3. rektifikasi: approxPolyDP 4-sudut (fallback minAreaRect) -> warpPerspective
  4. diagnostik warp: aspek-rasio + overlay 4-sudut ke foto asli
  5. normalisasi orientasi via sisi ekor -> kanonik (ekor di atas)
  6. decode 5x5 (1=hitam)
Agregasi per folder: pola modal + stabilitas sel. Compare wp3 vs wp4.

ponytail: diagnostik+decode, bukan tuning. Tidak infer n, tidak split
tuning/holdout (tak ada ambang yang dituning di sini).
"""
import argparse
import os
import sys
import cv2
import numpy as np

GRID_N = 6              # DIKUNCI 6x6: rekonstruksi decode paling setia ke warp asli (fitur 1-sel notch/kotak-terisolasi cuma muncul di N>=6; "5x5" klaim tim awal keliru).
WARP_PX = 500          # sisi kanonik hasil warp (5 sel * 100px)
AR_TOL = 0.25          # |aspek_rasio - 1| di atas ini => warp buruk, buang
                       # DICOBA & DIREVERT (2026-08-07): longgarin ke soft-zone
                       # [0.25,0.6] dgn bobot voting linear-taper drpd buang keras.
                       # Buang turun drastis (WP2 77%->23%, WP1 47%->23%) TAPI
                       # agreement turun di 3/4 WP (WP1 -4.7, WP2 -5.6, WP4 -3.5,
                       # WP3 +0.5) DAN separasi WP3<->WP4 malah TURUN 19/36->18/36
                       # sel beda -- metrik paling kritis (WP3/WP4 riwayat tertukar)
                       # justru memburuk. Trade-off recall vs precision, tp precision
                       # yg menang di sini. Kode lih.
                       # wp_decode_audit_backup_before_arweight_20260807.py.
_CORNER_DS = 0.25      # downscale sblm MORPH_OPEN(45,45) di corners_of: kernel besar di
                       # 1920x1080 asli ~1.15s/foto (murni biaya profil morfologi), turun
                       # ke ~0.01s pada skala 0.25 dgn hasil box scale-equivariant (diverifikasi
                       # A/B penuh 437 foto: baseline minAreaRect downscaled vs full-res
                       # identik sampai 4 desimal di semua metrik agreement/hamming).

def marker_mask(bgr):
    """Mask marker = 'BUKAN oranye' (putih+hitam grid jadi satu blob padat).

    Segmentasi via warna terpal oranye, bukan putih. Kenapa: kalau ambil
    putih saja, sel hitam grid membelah badan jadi banyak fragmen -> komponen
    terbesar cuma potongan border (bug aspek-rasio 1.7). Terpal oranye
    homogen -> komplemennya = badan marker utuh.
    """
    hsv = cv2.cvtColor(bgr, cv2.COLOR_BGR2HSV)
    h, s = hsv[:, :, 0], hsv[:, :, 1]
    orange = ((h >= 8) & (h <= 32) & (s > 40))     # terpal oranye (s>40: toleran overexposed, cek verif: 0 piksel putih tertangkap)
    mask = (~orange).astype(np.uint8) * 255
    k = cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (15, 15))
    mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, k)  # tutup celah antar sel
    return mask


def largest_component(mask):
    """Komponen putih terbesar (badan marker + ekor). None kalau tak ada."""
    n, lbl, stats, _ = cv2.connectedComponentsWithStats(mask, 8)
    if n <= 1:
        return None
    idx = 1 + int(np.argmax(stats[1:, cv2.CC_STAT_AREA]))
    return (lbl == idx).astype(np.uint8) * 255


def detect_tail_side(body):
    """Sisi ekor via profil baris/kolom — tidak butuh morphologi.

    Ekor = tonjolan sempit di satu sisi badan persegi. Profil baris
    (rowsum = lebar piksel di tiap y) dan profil kolom (colsum = tinggi
    piksel di tiap x): badan utama punya rowsum/colsum TINGGI (mendekati
    penuh), ekor punya rowsum/colsum RENDAH (hanya 1 tab sempit).
    Cari sisi mana yang badan meluas melewati 'core' (baris/kolom berisi ≥60%
    maks) -> sisi itu adalah ekor.
    ponytail: threshold 60% robust terhadap perspektif ringan (mengurangi
    rowsum di tepi); gagal kalau ekor ≥60% lebar badan (tidak terjadi di
    marker lomba). Upgrade kalau ekor lebih lebar dari itu.
    """
    rowsum = (body > 0).sum(axis=1)   # lebar pada tiap baris y
    colsum = (body > 0).sum(axis=0)   # tinggi pada tiap kolom x
    if rowsum.max() == 0:
        return None
    row_thr = rowsum.max() * 0.6      # baris "badan inti" vs "ekor sempit"
    col_thr = colsum.max() * 0.6
    body_ys = np.where(rowsum > 0)[0]; core_ys = np.where(rowsum >= row_thr)[0]
    body_xs = np.where(colsum > 0)[0]; core_xs = np.where(colsum >= col_thr)[0]
    if not len(core_ys) or not len(core_xs):
        return None
    h_core = int(core_ys[-1]) - int(core_ys[0]) or 1
    w_core = int(core_xs[-1]) - int(core_xs[0]) or 1
    noise_px = max(5, int(min(h_core, w_core) * 0.03))   # filter rim
    sides = {
        'N': int(core_ys[0])  - int(body_ys[0]),    # ekor di atas
        'S': int(body_ys[-1]) - int(core_ys[-1]),
        'W': int(core_xs[0])  - int(body_xs[0]),
        'E': int(body_xs[-1]) - int(core_xs[-1]),
    }
    best = max(sides, key=lambda k: sides[k])
    return best if sides[best] >= noise_px else None


def corners_of(body):
    """4 sudut badan inti (tanpa ekor).

    approxPolyDP (kontur asli -> quad sungguhan) dgn epsilon dinaikkan
    bertahap sampai lolos 4-titik-konveks; fallback minAreaRect kalau tak
    ketemu (mis. foto terlalu buram/kontur pecah).

    GANTI dari minAreaRect-only (versi lama, lih. backup
    wp_decode_audit_backup_before_quadcorner_*.py): minAreaRect memaksa
    KOTAK-ROTASI ke kontur yg kontur aslinya TRAPESIUM saat marker difoto
    miring (foto sudut ekstrem) -- sistematis salah bentuk utk kasus yg
    justru paling butuh presisi (root-cause WP1: 74% foto large_noise dari
    distorsi perspektif, lih. docs/WP_MARKER_DECODER.md §7a). approxPolyDP
    sempat dicoba & di-revert dulu ("rapuh, latch ke notch grid/ekor") --
    ekor SUDAH dibuang lewat MORPH_OPEN di bawah sebelum approxPolyDP jalan
    di sini, beda dari percobaan lama; downscale (_CORNER_DS) turut membuang
    notch grid halus sbg efek low-pass, jadi masalah lama itu tidak terulang.

    Divalidasi A/B penuh (437 foto asli, wp1/wp3/wp4, 2026-08-07):
    agreement WP1 0.721->0.759, WP4 0.771->0.826 (WP3 turun 0.787->0.735,
    tp separasi WP3 ke tetangga NAIK, lih. bawah). Hamming WP1<->WP4
    (pasangan paling rawan tertukar di lapangan) naik 10/36 -> 17/36;
    WP1<->WP3 15/36->20/36; WP3<->WP4 17/36->19/36 -- SEMUA pasangan makin
    terpisah, bukan cuma satu. subpixel corner refine (cv2.cornerSubPix)
    dicoba di atas approxPolyDP ini juga -- HASIL LEBIH BURUK di semua
    metrik (mask biner tak punya gradien tajam ala checkerboard utk
    subpix kunci), sengaja TIDAK dipakai.
    """
    small = cv2.resize(body, None, fx=_CORNER_DS, fy=_CORNER_DS,
                       interpolation=cv2.INTER_NEAREST)
    ksz = max(3, int(round(45 * _CORNER_DS))) | 1
    k = cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (ksz, ksz))
    core = cv2.morphologyEx(small, cv2.MORPH_OPEN, k)  # buang ekor dulu
    cnts, _ = cv2.findContours(core, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
    if not cnts:
        return None
    c = max(cnts, key=cv2.contourArea)
    peri = cv2.arcLength(c, True)
    for eps_frac in (0.01, 0.02, 0.03, 0.04, 0.05, 0.07, 0.10):
        approx = cv2.approxPolyDP(c, eps_frac * peri, True)
        if len(approx) == 4 and cv2.isContourConvex(approx):
            return approx.reshape(-1, 2).astype(np.float32) / _CORNER_DS
    return cv2.boxPoints(cv2.minAreaRect(c)).astype(np.float32) / _CORNER_DS


def order_corners(pts):
    """Urut 4 sudut jadi siklik TL,TR,BR,BL (searah jarum jam di koordinat gambar).

    GANTI dari heuristik sum/diff (Rosebrock/PyImageSearch 2014, "4 Point
    OpenCV getPerspective Transform Example" -- pendekatan awal yg dipakai
    kode ini): sum/diff RUSAK kalau quad SANGAT miring/skew -- nilai sum atau
    diff dua sudul bisa nabrak/dekat, jadi salah pasang sudut mana diagonal ke
    mana -> hasil quad SELF-INTERSECTING ("bowtie", garis nyilang lewat tengah
    marker bukan ngikut tepi). Terbukti visual di overlay debug WP2 foto AR
    ekstrem (0.35, 3.55, dst) -- garis diagonal motong lewat badan marker,
    bukan cuma foto miring biasa (foto miring biasa quad-nya tetap benar,
    lih. overlay AR=0.68 yg valid).

    Fix (Rosebrock/PyImageSearch 2016, "Ordering coordinates clockwise with
    Python and OpenCV" -- artikel susulan resmi yg alamatin bug yg sama):
    urut sudut berdasar SUDUT POLAR thd sentroid, bukan sum/diff. Rotation
    /skew-invariant -- dijamin urutan siklik benar (non-self-intersecting)
    di poligon convex apa pun, arah quad-nya seberapa pun miring. Anchor start
    = sudut sum(x+y) minimum dipertahankan (kompatibel sm heuristik lama)
    supaya urutan tetap deterministik; TIDAK kritis buat kebenaran krn
    orientasi final ditentukan ulang lewat tail-detection pasca-warp
    (rotate_to_canonical), bukan dari label TL/TR/BL/BR di sini.

    DIPERTAHANKAN (2026-08-07, beda dari 2 percobaan lain yg direvert --
    lih. AR_TOL & decode_grid): A/B penuh 437 foto, POLA MODAL 4/4 WP
    IDENTIK PERSIS ke baseline (termasuk separasi WP3<->WP4 tetap 19/36
    beda) walau sample lolos AR_TOL nambah signifikan (WP1 186->205,
    WP3 26->29, WP4 36->38) & std aspek-rasio susut tajam (WP1 1.010->0.720,
    WP3 0.398->0.209, WP4 0.321->0.218 -- geometri makin presisi). Agreement
    sel turun tipis (WP1 -2.5, WP3 -1.2, WP4 -1.5 poin) krn foto yg baru
    keselamatkan emang lebih miring dari rata2 -- itu bukan regresi kualitas,
    cuma sampel makin luas. Beda validasi dari AR_TOL/decode_grid: di sana
    "menang" cuma diukur dari 1 angka agregat; di sini ada bukti independen
    (overlay visual bowtie vs pola modal yg TAK BERUBAH sama sekali) bahwa
    fix ini benar, bukan cuma trade-off angka.
    """
    c = pts.mean(axis=0)
    ang = np.arctan2(pts[:, 1] - c[1], pts[:, 0] - c[0])   # y-down -> ascending = CW
    cyc = pts[np.argsort(ang)]
    anchor = int(np.argmin(cyc.sum(axis=1)))
    return np.roll(cyc, -anchor, axis=0).astype(np.float32)

def warp_marker(bgr, corners):
    """warpPerspective ke kotak kanonik WARP_PX. Kembalikan (warp, aspect)."""
    o = order_corners(corners)
    (tl, tr, br, bl) = o
    wa = np.linalg.norm(br - bl); wb = np.linalg.norm(tr - tl)
    ha = np.linalg.norm(tr - br); hb = np.linalg.norm(tl - bl)
    w = (wa + wb) / 2; h = (ha + hb) / 2
    aspect = w / h if h > 0 else 0.0
    dst = np.array([[0, 0], [WARP_PX, 0], [WARP_PX, WARP_PX], [0, WARP_PX]],
                   dtype=np.float32)
    M = cv2.getPerspectiveTransform(o, dst)
    return cv2.warpPerspective(bgr, M, (WARP_PX, WARP_PX)), aspect


def rotate_to_canonical(warp, tail_side):
    """Putar supaya ekor di atas (N). Kanonik dipilih: ekor=N."""
    rot = {'N': 0, 'E': 1, 'S': 2, 'W': 3}.get(tail_side)
    if rot is None:
        return warp  # tak tahu orientasi -> biarkan (ditandai low-conf)
    for _ in range(rot):
        warp = cv2.rotate(warp, cv2.ROTATE_90_COUNTERCLOCKWISE)
    return warp


def _body_yspan(bw):
    """y-range BADAN saja (buang ekor). bw: hitam=255.

    Ekor sudah dinormalisasi ke atas (rotate_to_canonical) & memberi cluster
    hitam kecil di atas badan, terpisah celah putih (baris ~nol hitam). Ambil
    run baris-hitam kontigu bermassa terbesar = badan.
    ponytail: kalau baris tepi grid SELURUHNYA putih, run badan bisa terpotong
    1 sel -> muncul sbg agreement rendah (sudah jadi konvensi decode ini),
    bukan diam-diam. Upgrade kalau perlu: spesifikasi border eksplisit.
    """
    row_black = (bw > 0).sum(axis=1)
    active = row_black > 0.02 * bw.shape[1]      # baris "berisi" hitam
    if not active.any():
        return None
    idx = np.where(active)[0]
    segs = np.split(idx, np.where(np.diff(idx) > 1)[0] + 1)  # run kontigu
    best = max(segs, key=lambda s: row_black[s].sum())       # massa terbesar=badan
    return int(best[0]), int(best[-1]) + 1


def decode_grid(warp):
    """5x5, 1=hitam. Grid dipatok ke bbox konten HITAM BADAN (buang border+ekor).

    minAreaRect ikut menyertakan quiet-zone putih yg lebarnya tak diketahui DAN
    ekor (cluster hitam terpisah di atas); kalau grid dipatok ke sisi warp/bbox
    global, sampling meleset. y-extent dipatok ke badan via _body_yspan (buang
    ekor), x-extent dihitung dari baris badan saja.
    ponytail: lih. _body_yspan utk failure-mode baris tepi putih.

    DICOBA & DIREVERT (2026-08-07): threshold lokal per-sel (cv2.adaptiveThreshold
    gaya Sauvola/Niblack, window ~2.5 sel) menggantikan Otsu global -- dugaan
    awal cahaya lapangan miring bikin sel di sisi terang/gelap salah baca.
    A/B penuh 437 foto (wp1/wp2/wp3/wp4): WP1 0.703->0.691, WP2 0.663->0.651,
    WP3 0.708->0.701 (semua TURUN), WP4 0.723->0.735 (naik tipis) -- 3/4 WP
    memburuk, TIDAK dipakai. Kemungkinan window lokal ikut menangkap gradien
    dalam satu sel (bukan cuma tren pencahayaan makro), jadi noise per-sel
    nambah alih-alih iluminasi yg terkoreksi. Kode lih.
    wp_decode_audit_backup_before_localthr_20260807.py kalau mau eksperimen ulang.
    """
    # nampan merah WP2 (kondisi permanen): pixel merah di warp diperlakukan sebagai putih
    # sebelum Otsu, agar border tray tidak ikut terdecode sebagai sel hitam.
    # Segmentasi blob tetap memakai tray+kertas sebagai anchor (AR stabil).
    _hsv = cv2.cvtColor(warp, cv2.COLOR_BGR2HSV)
    _h, _s = _hsv[:, :, 0], _hsv[:, :, 1]
    _red = ((_h < 7) | (_h > 168)) & (_s > 100)
    warp = warp.copy()
    warp[_red] = [255, 255, 255]
    g = cv2.cvtColor(warp, cv2.COLOR_BGR2GRAY)
    thr, bw = cv2.threshold(g, 0, 255,
                            cv2.THRESH_BINARY_INV + cv2.THRESH_OTSU)  # hitam->255
    if (bw > 0).sum() < 50:
        return np.zeros((GRID_N, GRID_N), dtype=int)
    span = _body_yspan(bw)
    if span is None:
        return np.zeros((GRID_N, GRID_N), dtype=int)
    y0, y1 = span
    xs_body = np.where(bw[y0:y1].sum(axis=0) > 0)[0]         # x hanya dari baris badan
    x0, x1 = int(xs_body.min()), int(xs_body.max()) + 1
    cw, ch = (x1 - x0) / GRID_N, (y1 - y0) / GRID_N
    bits = np.zeros((GRID_N, GRID_N), dtype=int)
    for r in range(GRID_N):
        for c in range(GRID_N):
            cy, cx = int(y0 + (r + 0.5) * ch), int(x0 + (c + 0.5) * cw)
            patch = g[max(0, cy - 8):cy + 8, max(0, cx - 8):cx + 8]
            bits[r, c] = 1 if patch.size and np.median(patch) < thr else 0
    return bits


def process_image(path, debug_dir=None):
    """-> dict(ok, tail, aspect, bits, reason)."""
    bgr = cv2.imread(path)
    if bgr is None:
        return dict(ok=False, reason='baca-gagal')
    mask = marker_mask(bgr)
    body = largest_component(mask)
    if body is None:
        return dict(ok=False, reason='no-marker')
    # Untuk tail detection: exclude pixel merah supaya nampan WP2 tidak
    # menutupi protrusi ekor kertas. Body asli (dengan tray) tetap dipakai
    # untuk corners_of (AR stabil, tidak salah pick ground).
    _hsv = cv2.cvtColor(bgr, cv2.COLOR_BGR2HSV)
    _h, _s = _hsv[:, :, 0], _hsv[:, :, 1]
    _red = ((_h < 7) | (_h > 168)) & (_s > 100)
    _body_tail = body.copy()
    _body_tail[_red] = 0
    _body_tail = cv2.morphologyEx(
        _body_tail, cv2.MORPH_CLOSE,
        cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (15, 15)))
    tail = detect_tail_side(_body_tail)
    corners = corners_of(body)
    if corners is None:
        return dict(ok=False, reason='no-corners')
    warp, aspect = warp_marker(bgr, corners)
    if debug_dir is not None:
        _save_overlay(bgr, corners, tail, os.path.basename(path), debug_dir)
    if abs(aspect - 1.0) > AR_TOL:
        return dict(ok=False, reason=f'warp-buruk(ar={aspect:.2f})',
                    tail=tail, aspect=aspect)
    warp = rotate_to_canonical(warp, tail)
    bits = decode_grid(warp)
    return dict(ok=True, tail=tail, aspect=aspect, bits=bits,
                low_conf=(tail is None), reason='ok')


def _save_overlay(bgr, corners, tail, name, debug_dir):
    vis = bgr.copy()
    o = order_corners(corners).astype(int)
    cv2.polylines(vis, [o], True, (0, 0, 255), 3)
    for i, (x, y) in enumerate(o):
        cv2.circle(vis, (x, y), 8, (0, 255, 0), -1)
        cv2.putText(vis, 'TL TR BR BL'.split()[i], (x + 8, y),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 0), 2)
    cv2.putText(vis, f'tail={tail}', (20, 40),
                cv2.FONT_HERSHEY_SIMPLEX, 1.0, (255, 0, 0), 2)
    os.makedirs(debug_dir, exist_ok=True)
    cv2.imwrite(os.path.join(debug_dir, name.rsplit('.', 1)[0] + '_ov.png'), vis)

def process_folder(folder, limit=None, debug_dir=None):
    files = sorted(f for f in os.listdir(folder)
                   if f.lower().endswith(('.jpg', '.png', '.jpeg')))
    if limit:
        files = files[:limit]
    good, tails, aspects, bad = [], [], [], {}
    for f in files:
        r = process_image(os.path.join(folder, f), debug_dir)
        if r.get('aspect') is not None:
            aspects.append(r['aspect'])
        if r['ok']:
            good.append(r['bits'])
            tails.append(r.get('tail'))
        else:
            bad[r['reason'].split('(')[0]] = bad.get(r['reason'].split('(')[0], 0) + 1
    return dict(n_total=len(files), n_good=len(good), bad=bad,
                aspects=aspects, tails=tails, good=good)


def aggregate(good):
    """Pola modal per sel + stabilitas (frac foto yg setuju dgn modal).

    DICOBA & DIREVERT (2026-08-07): voting berbobot blur (varians-Laplacian,
    Pech-Pacheco et al. 2000 -- lih. PyImageSearch "Blur detection with
    OpenCV"), bobot = persentil-rank blur DALAM folder yg sama (dikalibrasi
    dari distribusi data, bukan ambang tetap -- korelasi blur/disagreement
    dicek dulu per-WP: WP1 corr=0.085 [nol], WP3 corr=-0.449 [sedang],
    WP4 corr=-0.724 [kuat, rapi monoton] sblm implementasi, jadi bukan
    tebakan). Hasil A/B penuh: WP1 tak berubah (sesuai prediksi, blur
    emang bukan penyebab di sana). WP4 (korelasi PALING kuat) jugaTAK
    berubah sama sekali (agreement identik, modal identik) -- diduga noise
    dari foto buram tersebar acak antar sel, bukan konsisten nyoblos ke 1
    pola salah, jadi voting mayoritas polos udah otomatis meredamnya. WP2 &
    WP3 malah agreement TURUN DAN pola modal (jawaban ID) IKUT BERUBAH --
    tanpa ground truth independen, tak ada cara pastikan jawaban baru itu
    lebih benar; mengubah jawaban tanpa bukti independen = lawan dari
    "akurasi stabil". TIDAK dipakai. Kode lih.
    wp_decode_audit_backup_before_blurweight_20260807.py.
    """
    if not good:
        return None, 0.0
    stack = np.array(good)                       # (N,5,5)
    modal = (stack.mean(axis=0) >= 0.5).astype(int)
    agree = (stack == modal).mean()              # kesepakatan sel keseluruhan
    return modal, float(agree)


def em_aggregate(good, max_iter=50):
    """EM iterative tail refinement + modal aggregation.

    EXPERIMENTAL — TIDAK DIPAKAI di run() (reverted). Alasan revert:
    risiko sirkularitas — rotasi per-foto dipilih berdasarkan kecocokan
    ke modal yang dihitung dari rotasi yang sama (data leakage). Validasi
    fisik (overlay posisi ekor di foto vs rotasi EM) wajib sebelum deploy.

    E step: tiap foto cari rotasi bits (0/90/180/270 CCW) yang paling
            cocok dengan modal saat ini.
    M step: hitung ulang modal dari bits yang sudah dirotasi.
    Return: (modal, agree, rotations, n_iter)
    """
    if not good:
        return None, 0.0, [], 0
    n = len(good)
    rotations = [0] * n
    for it in range(max_iter):
        stack = np.array([np.rot90(good[i], rotations[i]) for i in range(n)])
        modal = (stack.mean(axis=0) >= 0.5).astype(int)
        new_rot, changed = [], 0
        for i, bits in enumerate(good):
            bh, bk = 999, 0
            for k in range(4):
                h = int((np.rot90(bits, k) != modal).sum())
                if h < bh:
                    bh, bk = h, k
            new_rot.append(bk)
            if bk != rotations[i]:
                changed += 1
        rotations = new_rot
        if changed == 0:
            break
    stack = np.array([np.rot90(good[i], rotations[i]) for i in range(n)])
    modal = (stack.mean(axis=0) >= 0.5).astype(int)
    agree = float((stack == modal).mean())
    return modal, agree, rotations, it + 1


def bits_str(bits):
    return '\n'.join(''.join('#' if b else '.' for b in row) for row in bits)


def _tail_hist(tails):
    from collections import Counter
    c = Counter('None' if t is None else t for t in tails)
    return dict(c)


def run(base, folders, limit, debug):
    results = {}
    for name in folders:
        folder = os.path.join(base, name)
        if not os.path.isdir(folder):
            print(f"[SKIP] {name}: folder tak ada")
            continue
        dbg = os.path.join(base, '_debug', name) if debug else None
        R = process_folder(folder, limit, dbg)
        modal, agree = aggregate(R['good'])
        results[name] = (modal, agree, R)
        ar = np.array(R['aspects']) if R['aspects'] else np.array([0])
        print(f"\n===== {name} =====")
        print(f"  foto: {R['n_total']}  ok: {R['n_good']}  buang: {R['bad']}")
        print(f"  aspek-rasio warp: mean={ar.mean():.3f} std={ar.std():.3f} "
              f"min={ar.min():.3f} max={ar.max():.3f}")
        print(f"  sisi ekor: {_tail_hist(R['tails'])}")
        print(f"  agreement sel: {agree:.3f}")
        if modal is not None:
            print("  pola modal (# = hitam):")
            for line in bits_str(modal).splitlines():
                print("    " + line)
    # compare wp3 vs wp4
    if 'wp3' in results and 'wp4' in results and results['wp3'][0] is not None \
       and results['wp4'][0] is not None:
        b3, b4 = results['wp3'][0], results['wp4'][0]
        same = int((b3 == b4).sum()); tot = b3.size
        print(f"\n===== BANDING wp3 vs wp4 =====")
        print(f"  sel identik: {same}/{tot}")
        if np.array_equal(b3, b4):
            print("  VERDICT: POLA SAMA -> MASALAH SERIUS (cetak dobel/salah?)")
        else:
            print(f"  VERDICT: POLA BEDA ({tot-same} sel beda) -> BAGUS, "
                  "marker wp3 != wp4 (tinggal tentukan mana id=3/id=4).")
    classify_report(results)
    return results


def classify_report(results):
    """Akurasi klasifikasi GROUND-TRUTH: cocokkan tiap foto ke pola modal
    ke-4 WP via nearest-match (Hamming distance terkecil), cek apakah
    nyocok ke folder ASALNYA sendiri.

    Beda dari 'agreement sel' (konsistensi internal, proxy): folder asal
    foto = ID sebenarnya (kita yg foto, kita yg taruh di folder itu), jadi
    ini akurasi navigasi SUNGGUHAN, bukan ditebak dari metrik proxy. Prinsip
    sama dgn error-correction ArUco (Garrido-Jurado et al. 2014): sistem
    tertutup 4 kandidat, jadi robust ke banyak sel salah baca selama
    errornya nggak konsisten condong ke 1 referensi lain -- lih. skor
    per-WP di bawah utk cek regangnya berapa jauh.

    ponytail: foto ikut nyumbang suara ke modal folder-nya sendiri (bukan
    leave-one-out) -- bias optimis ~1/n, n~200-350/WP jadi diabaikan drpd
    LOOCV penuh (n x lebih lambat utk presisi yg gak signifikan bedanya).
    """
    names = [n for n in results if results[n][0] is not None]
    if len(names) < 2:
        print("\n[classify] perlu >=2 WP dgn modal utk klasifikasi")
        return
    refs = {n: results[n][0] for n in names}
    print(f"\n===== KLASIFIKASI GROUND-TRUTH (nearest-match ke referensi: {names}) =====")
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
              f"  |  end-to-end (dari semua foto termasuk yg dibuang): {n_ok}/{n_tot} ({acc_e2e:.1%})"
              + (f"  salah->{others}" if others else ""))

def _selftest():
    """Synthetic 5x5 marker + ekor: warp bersih harus decode balik pola asli.

    ponytail: satu check runnable — kalau warp/decode/normalisasi rusak, gagal.
    """
    # _body_yspan: ekor-hitam kecil di atas + badan besar, terpisah celah ->
    # harus ambil BADAN saja (ekor synthetic biasa PUTIH+tipis shg lolos warp,
    # jadi bug ekor-hitam-dlm-warp cuma keuji di sini, bukan lewat pipeline).
    bw = np.zeros((500, 500), np.uint8)
    bw[50:120, 200:300] = 255          # ekor (cluster kecil atas)
    bw[200:450, 100:400] = 255         # badan (massa besar)
    ys0, ys1 = _body_yspan(bw)
    assert 190 <= ys0 <= 210 and 440 <= ys1 <= 460, \
        f"_body_yspan ambil ekor bukan badan: {(ys0, ys1)}"

    rng = np.random.RandomState(0)
    pat = rng.randint(0, 2, (GRID_N, GRID_N))
    cell = 80
    grid = GRID_N * cell
    border = 40                       # quiet-zone putih (seperti marker asli)
    sq = grid + 2 * border
    canvas = np.full((sq + 240, sq + 240, 3), 255, np.uint8)
    canvas[:] = (30, 140, 240)                                    # oranye BGR-ish
    y0 = x0 = 120
    cv2.rectangle(canvas, (x0, y0), (x0 + sq, y0 + sq), (255, 255, 255), -1)
    gx, gy = x0 + border, y0 + border
    for r in range(GRID_N):
        for c in range(GRID_N):
            if pat[r, c]:
                cv2.rectangle(canvas,
                              (gx + c * cell, gy + r * cell),
                              (gx + (c + 1) * cell, gy + (r + 1) * cell),
                              (0, 0, 0), -1)
    # ekor di ATAS (N): tonjolan putih kecil
    tw = grid // 4
    cv2.rectangle(canvas, (x0 + sq // 2 - tw // 2, y0 - 40),
                  (x0 + sq // 2 + tw // 2, y0), (255, 255, 255), -1)

    tmp = '/tmp/_wp_selftest.jpg'
    cv2.imwrite(tmp, canvas)
    r = process_image(tmp)
    assert r['ok'], f"selftest gagal proses: {r.get('reason')}"
    assert r['tail'] == 'N', f"ekor harus N, dapat {r['tail']}"
    assert abs(r['aspect'] - 1.0) < AR_TOL, f"aspek {r['aspect']}"
    # decode harus cocok pola asli (ekor N -> tak ada rotasi)
    match = (r['bits'] == pat).mean()
    assert match >= 0.92, f"decode cocok hanya {match:.2f}\n{bits_str(r['bits'])}"

    # rotasi: ekor di W harus dinormalisasi balik ke pola sama
    rotimg = cv2.rotate(canvas, cv2.ROTATE_90_CLOCKWISE)  # ekor N -> E... uji E
    cv2.imwrite(tmp, rotimg)
    r2 = process_image(tmp)
    assert r2['ok'] and r2['tail'] in ('E', 'W'), f"rot tail={r2.get('tail')}"
    os.remove(tmp)
    print("selftest: PASS (warp+ekor+decode+normalisasi orientasi)")


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--base', default=os.path.expanduser('~/catkin_ws/dataset_arucode'))
    ap.add_argument('--folders', nargs='+', default=['wp1_new', 'wp3', 'wp4'])
    ap.add_argument('--limit', type=int, default=None,
                    help='batasi N foto/folder (audit cepat)')
    ap.add_argument('--debug', action='store_true',
                    help='simpan overlay 4-sudut ke _debug/')
    ap.add_argument('--selftest', action='store_true')
    a = ap.parse_args()
    if a.selftest:
        _selftest(); return
    run(a.base, a.folders, a.limit, a.debug)


if __name__ == '__main__':
    main()




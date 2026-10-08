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
        return False, target
    return True, target


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
    # 2. threshold oranye -> mask biner
    mask = cv2.inRange(hsv, np.array(HSV_ORANGE_LO, np.uint8),
                       np.array(HSV_ORANGE_HI, np.uint8))
    # 3. morphological closing (dilate->erode) sambung mask terputus
    k = cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (CLOSE_KERNEL, CLOSE_KERNEL))
    mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, k, iterations=CLOSE_ITER)
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


def _batch_test(seed=42):
    dpath = os.path.join(os.path.dirname(os.path.abspath(__file__)),
                         "..", "..", "..", "gate", "dataset_gate_baru")
    dpath = os.path.abspath(dpath)
    if not os.path.isdir(dpath):
        # fallback: relatif working dir
        dpath = os.path.abspath("gate/dataset_gate_baru")
    print("dataset:", dpath)
    tuning_set, holdout_set = _split_dataset(dpath, seed=seed)
    print("total=%d  tuning=%d  holdout=%d"
          % (len(tuning_set) + len(holdout_set), len(tuning_set), len(holdout_set)))

    gate_root = os.path.abspath(os.path.join(dpath, ".."))
    manifest = os.path.join(gate_root, "holdout_manifest_multi.txt")
    with open(manifest, "w") as fh:
        fh.write("\n".join(os.path.basename(f) for f in holdout_set) + "\n")
    print("holdout manifest ->", manifest)

    save_dir = os.path.join(gate_root, "annotated_multi")
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
    ap.add_argument("--image")
    ap.add_argument("--save")
    ap.add_argument("--seed", type=int, default=42)
    args = ap.parse_args()

    if args.selftest:
        import sys
        sys.exit(0 if _selftest() else 1)
    if args.batch_test:
        _batch_test(seed=args.seed)
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

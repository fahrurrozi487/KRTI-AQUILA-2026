#!/usr/bin/env python3
"""
Tuning Langkah 1 utk gate_multi_detector.

ATURAN ANTI-OVERFIT: semua keputusan param diambil HANYA dari tuning_set.
holdout_set cuma diukur SEKALI dengan param terpilih (tak boleh dipakai memilih).

Dua sub-tujuan:
  A. solidity (lo,hi): selamatkan gate DEKAT yg sebelumnya kelewat (target align).
     metrik = detection rate + proxy false-positive (rata2 kandidat/frame).
  B. rel_area_frac: buang kandidat kecil (spanduk) tanpa buang gate jauh.
     metrik = berapa kandidat kecil terbuang vs total.
"""
import os
import sys
import time
import cv2
import numpy as np
import gate_multi_detector as G

DPATH = "/home/jetson/catkin_ws/gate/dataset_gate_baru"


def _load(files):
    imgs = []
    for f in files:
        im = cv2.imread(f)
        if im is not None:
            imgs.append(im)
    return imgs


def _stats(imgs, tag="", **kw):
    """Return (detect_rate%, avg_cand_per_detected, frac_frames_multi%)."""
    t0 = time.time()
    n_det = 0
    total_cand = 0
    n_multi = 0
    for im in imgs:
        g = G.detect_gates(im, **kw)
        if g:
            n_det += 1
            total_cand += len(g)
            if len(g) >= 2:
                n_multi += 1
    n = len(imgs)
    dr = 100.0 * n_det / n if n else 0.0
    avg = total_cand / n_det if n_det else 0.0
    multi = 100.0 * n_multi / n if n else 0.0
    if tag:
        print("    [%s done %.0fs]" % (tag, time.time() - t0)); sys.stdout.flush()
    return dr, avg, multi


def main():
    tun_f, hol_f = G._split_dataset(DPATH, seed=42)
    print("loading tuning=%d holdout=%d ..." % (len(tun_f), len(hol_f)))
    tun = _load(tun_f)
    hol = _load(hol_f)
    print("loaded.\n")

    # baseline (param file saat ini)
    print("=== BASELINE (solidity 0.30-0.60, rel_area=0) ===")
    dr, avg, multi = _stats(tun, tag="baseline")
    print("  tuning : det=%.1f%%  avg_cand=%.2f  multi=%.1f%%" % (dr, avg, multi))

    # ---- Sub-A: sweep solidity_lo (turunkan utk gate dekat), hi tetap dulu ----
    print("\n=== SUB-A: sweep solidity_lo (tuning_set saja) ===")
    print("  lo    det%%   avg_cand  multi%%")
    for lo in (0.30, 0.22, 0.18, 0.15, 0.12, 0.10):
        dr, avg, multi = _stats(tun, tag="loA=%.2f"%lo, solidity_lo=lo)
        print("  %.2f  %5.1f   %6.2f    %5.1f" % (lo, dr, avg, multi))

    # ---- Sub-A2: sweep solidity_hi (tangkap gate miring solidity tinggi) ----
    print("\n=== SUB-A2: sweep solidity_hi @ lo=0.15 (tuning_set saja) ===")
    print("  hi    det%%   avg_cand  multi%%")
    for hi in (0.60, 0.70, 0.80, 0.90):
        dr, avg, multi = _stats(tun, tag="hi=%.2f"%hi, solidity_lo=0.15, solidity_hi=hi)
        print("  %.2f  %5.1f   %6.2f    %5.1f" % (hi, dr, avg, multi))

    # ---- Sub-B: sweep rel_area_frac @ solidity terpilih ----
    print("\n=== SUB-B: sweep rel_area_frac @ lo=0.15 hi=0.90 (tuning_set saja) ===")
    print("  frac  det%%   avg_cand  multi%%")
    for fr in (0.0, 0.10, 0.20, 0.30, 0.40):
        dr, avg, multi = _stats(tun, tag="fr=%.2f"%fr, solidity_lo=0.15, solidity_hi=0.90,
                                rel_area_frac=fr)
        print("  %.2f  %5.1f   %6.2f    %5.1f" % (fr, dr, avg, multi))


if __name__ == "__main__":
    main()

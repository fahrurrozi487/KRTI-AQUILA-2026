#!/usr/bin/env python3
"""Test logika counter hysteresis (leaky-bucket) di MissionD._scan().

Standalone (TIDAK import mission_d.py / rospy) -- mereplika persis aturan
update di _scan() sbg fungsi murni, biar bisa dites tanpa node ROS/mavros.
Kalau _scan() diedit lagi, tetap SYNC-kan _step() di sini scr manual.
"""
import os
import sys


def _step(hits, hit, scan_frames, scan_leak):
    """Persis logika di MissionD._scan(): naik +1 (dipatok scan_frames)
    per hit, turun -scan_leak (dipatok 0) per miss."""
    if hit:
        return min(scan_frames, hits + 1)
    return max(0, hits - scan_leak)


def simulate(hit_seq, scan_frames=5, scan_leak=1):
    """-> (confirmed_at_frame_idx atau None, hits_history)."""
    hits = 0
    history = []
    for i, hit in enumerate(hit_seq):
        hits = _step(hits, hit, scan_frames, scan_leak)
        history.append(hits)
        if hits >= scan_frames:
            return i, history
    return None, history


def test_all_hits_confirms_at_scan_frames():
    idx, hist = simulate([1, 1, 1, 1, 1, 1, 1], scan_frames=5, scan_leak=1)
    assert idx == 4, hist  # 0-indexed: hit ke-5 di idx 4


def test_all_miss_never_confirms():
    idx, hist = simulate([0] * 100, scan_frames=5, scan_leak=1)
    assert idx is None, hist
    assert hist[-1] == 0, hist


def test_single_isolated_miss_does_not_erase_streak():
    """Ini justru inti perbaikannya: 1 miss di tengah streak bagus TIDAK
    membuang semua progress (beda dari hard-reset lama)."""
    seq = [1, 1, 1, 1, 0, 1]  # 4 hit, 1 miss, 1 hit -> harus confirm cepat
    idx, hist = simulate(seq, scan_frames=5, scan_leak=1)
    assert hist == [1, 2, 3, 4, 3, 4], hist   # miss cuma turun 1, bukan reset ke 0
    assert idx is None                          # blm nyampe 5 dlm 6 frame ini (butuh 1 lagi)
    idx2, hist2 = simulate(seq + [1], scan_frames=5, scan_leak=1)
    assert idx2 == 6, hist2                     # confirm di frame ke-7 (idx 6)


def test_hard_reset_baseline_would_be_slower():
    """Baseline lama (hard reset ke 0) buat pembanding -- sequence sama
    butuh JAUH lebih banyak frame drpd versi leaky-bucket di atas."""
    def simulate_hard_reset(hit_seq, scan_frames=5):
        hits = 0
        for i, hit in enumerate(hit_seq):
            hits = hits + 1 if hit else 0
            if hits >= scan_frames:
                return i
        return None
    seq = [1, 1, 1, 1, 0, 1, 1]  # leaky-bucket confirm di idx 6 (lih. test di atas)
    idx_hard = simulate_hard_reset(seq, scan_frames=5)
    assert idx_hard is None, idx_hard  # hard-reset BELUM confirm sama sekali di 7 frame ini


def test_cap_never_exceeds_scan_frames():
    hits = 0
    for _ in range(20):
        hits = _step(hits, True, scan_frames=5, scan_leak=1)
    assert hits == 5, hits


if __name__ == '__main__':
    tests = [v for k, v in list(globals().items()) if k.startswith('test_')]
    for t in tests:
        t()
        print(f"  OK  {t.__name__}")
    print(f"selftest: PASS ({len(tests)} test hysteresis counter _scan())")

#!/usr/bin/env python3
"""Self-check for the ambiguous_margin branch added to decode_frame().

Stubs out the OpenCV pipeline steps (mask/corners/warp/grid) so we can feed
decode_frame() a controlled 6x6 bit pattern and check it lands in 'ok',
'ambiguous', or 'no_match' as expected -- without needing a real camera frame.
"""
import os
import sys
import numpy as np

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
import wp_marker_detector as wmd

N = wmd.GRID_N  # 6

# P1 non-trivial (all-zero would hit the separate decode_failed guard).
P1 = np.array([[1, 0, 1, 0, 1, 0],
               [0, 1, 0, 1, 0, 1],
               [1, 0, 1, 0, 1, 0],
               [0, 1, 0, 1, 0, 1],
               [1, 0, 1, 0, 1, 0],
               [0, 1, 0, 1, 0, 1]], dtype=int)
S = [(0, 0), (0, 1), (0, 2), (0, 3), (0, 4),
     (1, 0), (1, 1), (1, 2), (1, 3)]       # 9 cells -> matches real WP1/WP4 separation
D = np.zeros((N, N), dtype=int)
for (r, c) in S:
    D[r, c] = 1
P4 = P1 ^ D                                 # differs from P1 in exactly 9 cells

REFS = {'1': {'pattern': P1.tolist(), 'reliable': False},
        '4': {'pattern': P4.tolist(), 'reliable': True}}


def _stub_pipeline(bits):
    wmd.marker_mask = lambda bgr: np.ones((10, 10), dtype=np.uint8)
    wmd.largest_component = lambda mask: np.ones((10, 10), dtype=np.uint8)
    wmd.detect_tail_side = lambda body: 'N'
    wmd.corners_of = lambda body: np.zeros((4, 2), dtype=np.float32)
    wmd.warp_marker = lambda bgr, corners: (None, 1.0)   # aspect=1.0 -> passes AR_TOL
    wmd.rotate_to_canonical = lambda warp, tail: warp
    wmd.decode_grid = lambda canonical: bits
    wmd._centroid_offset = lambda a, b: (0.0, 0.0)


def _run(bits):
    _stub_pipeline(bits)
    dummy_bgr = np.zeros((10, 10, 3), dtype=np.uint8)
    return wmd.decode_frame(dummy_bgr, REFS, hamming_threshold=12, ambiguous_margin=3)


def test_ok_when_clear_winner():
    dec = _run(P1)  # exact match to P1, dist(P1)=0, dist(P4)=9 -> margin 9
    assert dec['reason'] == 'ok', dec
    assert dec['wp_id'] == 1, dec


def test_ambiguous_when_close_to_two_refs():
    # error mask E: 5 flips inside S (the P1/P4 diff set), 1 flip outside S.
    # hamming(bits,P1)=popcount(E)=6; hamming(bits,P4)=popcount(E^D)=(9-5)+1=5.
    # best=P4 at 5, 2nd=P1 at 6 -> margin=1 < ambiguous_margin=3.
    E = np.zeros((N, N), dtype=int)
    for (r, c) in S[:5]:
        E[r, c] = 1
    E[N - 1, N - 1] = 1  # (5,5) is outside S
    bits = P1 ^ E
    dec = _run(bits)
    assert dec['reason'] == 'ambiguous', dec
    assert dec['wp_id'] == -1, dec


def test_no_match_when_far_from_everything():
    dec = _run(1 - P1)  # complement: dist(P1)=36 -> both refs way past threshold=12
    assert dec['reason'] == 'no_match', dec
    assert dec['wp_id'] == -1, dec


if __name__ == '__main__':
    test_ok_when_clear_winner()
    test_ambiguous_when_close_to_two_refs()
    test_no_match_when_far_from_everything()
    print('OK: ambiguous_margin branch behaves as expected')

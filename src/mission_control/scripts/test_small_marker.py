#!/usr/bin/env python3
"""Self-check for find_small_patch()/decode_small_marker() (aruco kecil).

Builds a synthetic frame (orange background + big white patch + small white
patch joined by a thin neck, mimicking the real marker sheets) and checks:
  1. find_small_patch() isolates the SMALL patch, not the big one.
  2. decode_small_marker() remaps off_x/off_y back to full-frame coordinates
     correctly (this is the part with real bug risk -- easy to get the
     crop-local -> full-frame math backwards).
No real bit pattern needed -- DUMMY_REFS means decode_frame() will never reach
'ok', we're only checking the isolation/remap geometry here.
"""
import os
import sys
import numpy as np
import cv2

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from wp_marker_detector import find_small_patch, decode_small_marker

W, H = 400, 300
ORANGE_BGR = (0, 165, 255)  # H~15, S high -> passes marker_mask()'s orange range

BIG = dict(x0=100, y0=150, x1=300, y1=280)     # 200x130
SMALL = dict(x0=180, y0=20, x1=220, y1=70)     # 40x50
NECK = dict(x0=197, y0=70, x1=203, y1=150)     # 6px thin link, severable by 9x9 opening
# decode_frame() assumes refs has >=1 entry (Hamming-match step); dummy id,
# never expected to match -- we're only checking off_x/off_y remap here.
DUMMY_REFS = {'1': {'pattern': [[0] * 6 for _ in range(6)], 'reliable': False}}


def _make_frame():
    img = np.zeros((H, W, 3), dtype=np.uint8)
    img[:] = ORANGE_BGR
    for r in (BIG, SMALL, NECK):
        img[r['y0']:r['y1'], r['x0']:r['x1']] = (255, 255, 255)
    return img


def test_isolates_small_not_big():
    crop, bbox = find_small_patch(_make_frame())
    assert crop is not None, "should find a separable small patch"
    ch, cw = crop.shape[:2]
    small_w, small_h = SMALL['x1'] - SMALL['x0'], SMALL['y1'] - SMALL['y0']
    big_w, big_h = BIG['x1'] - BIG['x0'], BIG['y1'] - BIG['y0']
    # crop harus sedekat ukuran small patch (+margin), BUKAN mendekati big patch
    assert abs(cw - small_w) < 20 and abs(ch - small_h) < 20, \
        f"crop {cw}x{ch} tidak dekat small {small_w}x{small_h}"
    assert not (abs(cw - big_w) < 20 and abs(ch - big_h) < 20), \
        "crop keliru narik badan besar, bukan marker kecil"


def test_offset_remapped_to_full_frame():
    dec = decode_small_marker(_make_frame(), DUMMY_REFS)
    assert dec['found_marker'], dec
    # posisi tengah small patch yang sebenarnya, dinormalisasi ke frame penuh
    scx = (SMALL['x0'] + SMALL['x1']) / 2.0
    scy = (SMALL['y0'] + SMALL['y1']) / 2.0
    expect_off_x = (scx - W / 2.0) / (W / 2.0)
    expect_off_y = (scy - H / 2.0) / (H / 2.0)
    assert abs(dec['off_x'] - expect_off_x) < 0.1, (dec['off_x'], expect_off_x)
    assert abs(dec['off_y'] - expect_off_y) < 0.1, (dec['off_y'], expect_off_y)


def test_no_marker_on_empty_orange():
    img = np.zeros((H, W, 3), dtype=np.uint8)
    img[:] = ORANGE_BGR
    dec = decode_small_marker(img, DUMMY_REFS)
    assert not dec['found_marker'], dec
    assert dec['reason'] == 'no_marker', dec


if __name__ == '__main__':
    test_isolates_small_not_big()
    test_offset_remapped_to_full_frame()
    test_no_marker_on_empty_orange()
    print('OK: find_small_patch/decode_small_marker berperilaku sesuai ekspektasi')

#!/usr/bin/env python3
"""Self-test: detect_wp2_presence() harus kunci ke tray MERAH, bukan
komplemen oranye/merah (lih. ponytail note di wp_marker_detector.py)."""
import os
import sys

import cv2
import numpy as np

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from wp_marker_detector import detect_wp2_presence  # noqa: E402


def _orange_canvas(h=480, w=640):
    img = np.zeros((h, w, 3), np.uint8)
    img[:] = (30, 140, 240)   # BGR oranye, sama seperti selftest wp_decode_audit
    return img


def test_empty_orange_not_present():
    img = _orange_canvas()
    r = detect_wp2_presence(img)
    assert r['present'] is False, r


def test_red_tray_present():
    img = _orange_canvas()
    cv2.rectangle(img, (250, 150), (450, 350), (0, 0, 220), -1)  # BGR merah
    r = detect_wp2_presence(img)
    assert r['present'] is True, r
    assert r['area_px'] > 3000, r


def test_hand_not_present():
    """Tangan/kulit (non-oranye, non-merah) TIDAK boleh trigger present."""
    img = _orange_canvas()
    cv2.rectangle(img, (250, 150), (450, 350), (140, 180, 210), -1)  # BGR skin-ish
    r = detect_wp2_presence(img)
    assert r['present'] is False, r


if __name__ == '__main__':
    test_empty_orange_not_present()
    test_red_tray_present()
    test_hand_not_present()
    print('test_wp2_presence: PASS')

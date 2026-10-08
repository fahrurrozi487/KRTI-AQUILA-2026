#!/usr/bin/env python3
"""wp_marker_detector.py — pure-OpenCV core for WP marker detection (no ROS).

Imports pipeline functions from wp_decode_audit (read-only; never modified).
Provides:
  decode_frame(bgr, refs, hamming_threshold) -> dict
  detect_wp2_presence(bgr, min_area_px)      -> dict
"""
import sys
import os
import json
import numpy as np
import cv2

# --- locate wp_decode_audit in the same scripts/ dir -------------------------
_SCRIPTS = os.path.dirname(os.path.abspath(__file__))
if _SCRIPTS not in sys.path:
    sys.path.insert(0, _SCRIPTS)

from wp_decode_audit import (
    marker_mask, largest_component, detect_tail_side,
    corners_of, warp_marker, rotate_to_canonical, decode_grid,
    GRID_N, AR_TOL,
)

# ponytail: GRID_N imported from audit, kept in sync automatically.


def _hamming(a, b):
    return int(np.sum(a != b))


def _centroid_offset(corners_or_body, img_shape):
    """Normalized center offset from image center: -1..+1 per axis.

    corners_or_body: Nx2 float32 corners array → uses corners centroid (tail excluded).
                     2D uint8 mask              → uses moments centroid.
    """
    h, w = img_shape[:2]
    arr = np.asarray(corners_or_body)
    if arr.ndim == 2 and arr.shape[1] == 2:
        # corners: shape (N, 2)
        cx, cy = arr.mean(axis=0)
    else:
        # body mask
        M = cv2.moments(arr)
        if M['m00'] == 0:
            return 0.0, 0.0
        cx, cy = M['m10'] / M['m00'], M['m01'] / M['m00']
    return float((cx - w / 2) / (w / 2)), float((cy - h / 2) / (h / 2))


def decode_frame(bgr, refs, hamming_threshold=12):
    """Detect and ID a WP1/WP3/WP4 marker in a BGR frame.

    refs: dict  wp_id(str) -> {"pattern": [[...]], "reliable": bool}
                Only include WP IDs meant for ID-decode (1, 3, 4).

    Returns dict:
      found_marker  bool
      wp_id         int   matched WP id, or -1
      hamming       int   min hamming distance, or -1
      reliable      bool  from JSON reliable field
      low_conf_tail bool  True if detect_tail_side returned None
      reason        str   "ok"|"no_match"|"decode_failed"|"no_marker"|"no_corners"|"warp_bad"
      aspect        float warp aspect ratio, 0 if no marker
      off_x         float marker center offset x, -1..+1 (0=centered), 0 if no marker
      off_y         float marker center offset y, -1..+1 (0=centered), 0 if no marker
    """
    mask = marker_mask(bgr)
    body = largest_component(mask)
    if body is None:
        return dict(found_marker=False, wp_id=-1, hamming=-1, reliable=False,
                    low_conf_tail=False, reason='no_marker', aspect=0.0,
                    off_x=0.0, off_y=0.0)

    # Tail detection with red exclusion (WP2 tray fix)
    hsv = cv2.cvtColor(bgr, cv2.COLOR_BGR2HSV)
    h, s = hsv[:, :, 0], hsv[:, :, 1]
    red = ((h < 7) | (h > 168)) & (s > 100)
    body_notail = body.copy()
    body_notail[red] = 0
    body_notail = cv2.morphologyEx(
        body_notail, cv2.MORPH_CLOSE,
        cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (15, 15)))
    tail = detect_tail_side(body_notail)
    low_conf_tail = (tail is None)

    corners = corners_of(body)
    if corners is None:
        ox, oy = _centroid_offset(body, bgr.shape)
        return dict(found_marker=True, wp_id=-1, hamming=-1, reliable=False,
                    low_conf_tail=low_conf_tail, reason='no_corners', aspect=0.0,
                    off_x=ox, off_y=oy)

    ox, oy = _centroid_offset(corners, bgr.shape)

    warp, aspect = warp_marker(bgr, corners)
    if abs(aspect - 1.0) > AR_TOL:
        return dict(found_marker=True, wp_id=-1, hamming=-1, reliable=False,
                    low_conf_tail=low_conf_tail, reason='warp_bad', aspect=float(aspect),
                    off_x=ox, off_y=oy)

    canonical = rotate_to_canonical(warp, tail)
    bits = decode_grid(canonical)

    if bits.sum() == 0:
        # all-zeros: decode failure (would Hamming-match WP3 ref with distance 8 < threshold)
        return dict(found_marker=True, wp_id=-1, hamming=-1, reliable=False,
                    low_conf_tail=low_conf_tail, reason='decode_failed', aspect=float(aspect),
                    off_x=ox, off_y=oy)

    # Hamming match against reference patterns
    distances = {}
    for wid, info in refs.items():
        ref = np.array(info['pattern'], dtype=int)
        distances[wid] = _hamming(bits, ref)

    best_id = min(distances, key=lambda k: (distances[k], not refs[k]['reliable']))
    best_dist = distances[best_id]

    if best_dist > hamming_threshold:
        return dict(found_marker=True, wp_id=-1, hamming=int(best_dist), reliable=False,
                    low_conf_tail=low_conf_tail, reason='no_match', aspect=float(aspect),
                    off_x=ox, off_y=oy)

    return dict(found_marker=True, wp_id=int(best_id), hamming=int(best_dist),
                reliable=bool(refs[best_id]['reliable']),
                low_conf_tail=low_conf_tail, reason='ok', aspect=float(aspect),
                off_x=ox, off_y=oy)


def detect_wp2_presence(bgr, min_area_px=3000):
    """Detect WP2 by presence: large non-orange AND non-red blob.

    Returns dict:
      present   bool
      area_px   int   area of largest qualifying blob
      off_x     float blob center offset x, -1..+1 (0 if not present)
      off_y     float blob center offset y, -1..+1 (0 if not present)
    """
    hsv = cv2.cvtColor(bgr, cv2.COLOR_BGR2HSV)
    h, s = hsv[:, :, 0], hsv[:, :, 1]
    orange = ((h >= 8) & (h <= 32) & (s > 40))
    red = ((h < 7) | (h > 168)) & (s > 100)
    exclude = (orange | red).astype(np.uint8) * 255
    candidate = cv2.bitwise_not(exclude)

    k = cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (15, 15))
    candidate = cv2.morphologyEx(candidate, cv2.MORPH_CLOSE, k)

    body = largest_component(candidate)
    if body is None:
        return dict(present=False, area_px=0, off_x=0.0, off_y=0.0)

    area = int((body > 0).sum())
    present = area >= min_area_px
    ox, oy = (_centroid_offset(body, bgr.shape) if present else (0.0, 0.0))
    return dict(present=present, area_px=area, off_x=ox, off_y=oy)


def load_refs(json_path, wp_ids=(1, 3, 4)):
    """Load reference patterns for the given wp_ids from JSON."""
    with open(json_path) as f:
        data = json.load(f)
    refs = {}
    for wid in wp_ids:
        entry = data[str(wid)]
        refs[str(wid)] = {
            'pattern': entry['pattern'],
            'reliable': entry['reliable'],
        }
    return refs


# --- self-test (python3 wp_marker_detector.py --selftest <image_path>) -------
if __name__ == '__main__':
    import argparse
    ap = argparse.ArgumentParser()
    ap.add_argument('image', nargs='?', help='test image path')
    ap.add_argument('--json', default=os.path.join(
        _SCRIPTS, '..', 'config', 'wp_marker_reference.json'))
    ap.add_argument('--wp2', action='store_true', help='run WP2 presence test instead')
    ap.add_argument('--min-area', type=int, default=3000)
    ap.add_argument('--threshold', type=int, default=12)
    args = ap.parse_args()

    if args.image is None:
        print('Usage: wp_marker_detector.py <image> [--wp2] [--json path]')
        sys.exit(0)

    bgr = cv2.imread(args.image)
    assert bgr is not None, f"Cannot read {args.image}"

    if args.wp2:
        r = detect_wp2_presence(bgr, args.min_area)
        print(f"WP2 present={r['present']}  area_px={r['area_px']}  threshold={args.min_area}"
              f"  off=({r['off_x']:.3f},{r['off_y']:.3f})")
    else:
        json_path = os.path.normpath(os.path.join(_SCRIPTS, args.json)) \
            if not os.path.isabs(args.json) else args.json
        refs = load_refs(json_path)
        r = decode_frame(bgr, refs, args.threshold)
        print(f"wp_id={r['wp_id']}  hamming={r['hamming']}  reliable={r['reliable']}"
              f"  reason={r['reason']}  aspect={r['aspect']:.3f}"
              f"  low_conf_tail={r['low_conf_tail']}"
              f"  off=({r['off_x']:.3f},{r['off_y']:.3f})")

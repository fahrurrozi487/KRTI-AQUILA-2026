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


def decode_frame(bgr, refs, hamming_threshold=12, ambiguous_margin=3):
    """Detect and ID a WP1/WP3/WP4 marker in a BGR frame.

    refs: dict  wp_id(str) -> {"pattern": [[...]], "reliable": bool}
                Only include WP IDs meant for ID-decode (1, 3, 4).
    ambiguous_margin: int  min hamming gap required between best and 2nd-best
                match. WP1/WP4 reference patterns are only 9 cells apart, so a
                noisy read can land closer to the wrong neighbor than to the
                true marker; below this margin we report 'ambiguous' instead
                of a confident (possibly wrong) id.

    Returns dict:
      found_marker  bool
      wp_id         int   matched WP id, or -1
      hamming       int   min hamming distance, or -1
      reliable      bool  from JSON reliable field
      low_conf_tail bool  True if detect_tail_side returned None
      tail_side     str   'N'/'S'/'E'/'W' sisi ekor (Task 4, arah navigasi), '' jika tak terdeteksi
      reason        str   "ok"|"no_match"|"ambiguous"|"decode_failed"|"no_marker"|"no_corners"|"warp_bad"
      aspect        float warp aspect ratio, 0 if no marker
      off_x         float marker center offset x, -1..+1 (0=centered), 0 if no marker
      off_y         float marker center offset y, -1..+1 (0=centered), 0 if no marker
    """
    mask = marker_mask(bgr)
    body = largest_component(mask)
    if body is None:
        return dict(found_marker=False, wp_id=-1, hamming=-1, reliable=False,
                    low_conf_tail=False, tail_side='', reason='no_marker', aspect=0.0,
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
    tail_side = tail or ''

    corners = corners_of(body)
    if corners is None:
        ox, oy = _centroid_offset(body, bgr.shape)
        return dict(found_marker=True, wp_id=-1, hamming=-1, reliable=False,
                    low_conf_tail=low_conf_tail, tail_side=tail_side, reason='no_corners', aspect=0.0,
                    off_x=ox, off_y=oy)

    ox, oy = _centroid_offset(corners, bgr.shape)

    warp, aspect = warp_marker(bgr, corners)
    if abs(aspect - 1.0) > AR_TOL:
        return dict(found_marker=True, wp_id=-1, hamming=-1, reliable=False,
                    low_conf_tail=low_conf_tail, tail_side=tail_side, reason='warp_bad', aspect=float(aspect),
                    off_x=ox, off_y=oy)

    canonical = rotate_to_canonical(warp, tail)
    bits = decode_grid(canonical)

    if bits.sum() == 0:
        # all-zeros: decode failure (would Hamming-match WP3 ref with distance 8 < threshold)
        return dict(found_marker=True, wp_id=-1, hamming=-1, reliable=False,
                    low_conf_tail=low_conf_tail, tail_side=tail_side, reason='decode_failed', aspect=float(aspect),
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
                    low_conf_tail=low_conf_tail, tail_side=tail_side, reason='no_match', aspect=float(aspect),
                    off_x=ox, off_y=oy)

    second_dist = min((d for wid, d in distances.items() if wid != best_id), default=None)
    if second_dist is not None and (second_dist - best_dist) < ambiguous_margin:
        return dict(found_marker=True, wp_id=-1, hamming=int(best_dist), reliable=False,
                    low_conf_tail=low_conf_tail, tail_side=tail_side, reason='ambiguous', aspect=float(aspect),
                    off_x=ox, off_y=oy)

    return dict(found_marker=True, wp_id=int(best_id), hamming=int(best_dist),
                reliable=bool(refs[best_id]['reliable']),
                low_conf_tail=low_conf_tail, tail_side=tail_side, reason='ok', aspect=float(aspect),
                off_x=ox, off_y=oy)


def find_small_patch(bgr, neck_kernel=9):
    """Isolate the small companion marker printed above the main body,
    joined to it by a thin paper neck (same physical sheet).

    Reuses marker_mask()/largest_component() (same segmentation as the big
    marker). The neck is thinner than both patches, so a morphological open
    severs it; whatever survives above the main body is the small-marker
    candidate.

    Returns (crop_bgr, (x0, y0, x1, y1)) in original-frame pixel coords,
    or (None, None) if no separable small patch is found.
    """
    mask = marker_mask(bgr)
    body = largest_component(mask)
    if body is None:
        return None, None
    total_area = int((body > 0).sum())
    k = cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (neck_kernel, neck_kernel))
    opened = cv2.morphologyEx(body, cv2.MORPH_OPEN, k)
    n, lbl, stats, _ = cv2.connectedComponentsWithStats(opened, 8)
    if n <= 2:
        return None, None  # neck not severed -> no distinguishable small patch
    # Opening bisa juga memecah badan besar sendiri (bentuknya gak beraturan),
    # bukan cuma motong leher. Kalau gak difilter, pecahan badan besar bisa
    # kepilih jadi "kecil" -> hasil kembar sama marker besar. Marker kecil
    # asli SELALU jauh lebih kecil dari badan besar (<15% luas total).
    cands = [(i, stats[i, cv2.CC_STAT_TOP]) for i in range(1, n)
             if 30 < stats[i, cv2.CC_STAT_AREA] < 0.15 * total_area]
    if not cands:
        return None, None
    cands.sort(key=lambda c: c[1])  # topmost component = the small patch
    i = cands[0][0]
    x, y, w, h = (stats[i, cv2.CC_STAT_LEFT], stats[i, cv2.CC_STAT_TOP],
                  stats[i, cv2.CC_STAT_WIDTH], stats[i, cv2.CC_STAT_HEIGHT])
    m = neck_kernel // 2 + 2
    y0, y1 = max(0, y - m), min(bgr.shape[0], y + h + m)
    x0, x1 = max(0, x - m), min(bgr.shape[1], x + w + m)
    return bgr[y0:y1, x0:x1], (x0, y0, x1, y1)


def decode_small_marker(bgr, refs, hamming_threshold=12, ambiguous_margin=3, neck_kernel=9):
    """Detect + ID the small companion marker (close-range nav reference).

    Reuses decode_frame()'s full pipeline on the isolated small-patch crop
    -- same reference JSON, same hamming/ambiguous logic, no separate
    decoder. off_x/off_y are remapped from the crop back to the FULL frame
    so callers can align on it exactly like the big marker's result.
    """
    crop, bbox = find_small_patch(bgr, neck_kernel=neck_kernel)
    if crop is None or crop.size == 0:
        return dict(found_marker=False, wp_id=-1, hamming=-1, reliable=False,
                    low_conf_tail=False, tail_side='', reason='no_marker', aspect=0.0,
                    off_x=0.0, off_y=0.0)
    dec = decode_frame(crop, refs, hamming_threshold=hamming_threshold,
                        ambiguous_margin=ambiguous_margin)
    if dec['found_marker']:
        x0, y0, x1, y1 = bbox
        h_full, w_full = bgr.shape[:2]
        cw, ch = (x1 - x0), (y1 - y0)
        cx_full = x0 + (dec['off_x'] + 1.0) * cw / 2.0
        cy_full = y0 + (dec['off_y'] + 1.0) * ch / 2.0
        dec = dict(dec)
        dec['off_x'] = float((cx_full - w_full / 2.0) / (w_full / 2.0))
        dec['off_y'] = float((cy_full - h_full / 2.0) / (h_full / 2.0))
    return dec


def detect_wp2_presence(bgr, min_area_px=3000):
    """Detect WP2 by presence: the RED drop-tray blob.

    WP2 fisik = kertas MMT dilipat dimasukkan ke box plastik MERAH, ditaruh
    di atas terpal oranye (bukan marker ID biasa, ini target drop/"ember").
    ponytail: sebelumnya mask = 'BUKAN oranye DAN BUKAN merah' (exclude
    merahnya!) -> justru buang fitur PALING khas WP2 (tray merah pekat,
    gampang dipisah dari oranye), dan match ke SEGALA objek non-oranye/
    non-merah lain di frame (tangan, tembok, dll) -> false-positive ~75%
    di frame kosong indoor (tervalidasi capture live 2026-08-05). Kertas
    di dalam tray sering terlipat/ketutup bayangan -> sinyal lemah & tak
    konsisten (lih. wp_reference_from_samples.py: cuma 13% foto WP2 lolos
    decode grid, vs 46-49% di WP1/3/4). Tray merah jauh lebih konsisten:
    deteksi warna merah itu sendiri, bukan komplemennya.
    Upgrade kalau perlu: cek bentuk (aspect rect) tray juga, bukan cuma warna.

    Returns dict:
      present   bool
      area_px   int   area of largest qualifying blob
      off_x     float blob center offset x, -1..+1 (0 if not present)
      off_y     float blob center offset y, -1..+1 (0 if not present)
    """
    hsv = cv2.cvtColor(bgr, cv2.COLOR_BGR2HSV)
    h, s = hsv[:, :, 0], hsv[:, :, 1]
    red = ((h < 7) | (h > 168)) & (s > 100)
    candidate = red.astype(np.uint8) * 255

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

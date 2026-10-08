#!/usr/bin/env python3
"""wp_decode_warpdump.py — dump full-res canonical warp + grid overlay for
named files, for close visual audit of grid-sampling alignment.

READ-ONLY. ponytail: one-off audit helper, no CLI polish.
"""
import os
import sys
import cv2
import numpy as np

sys.path.insert(0, os.path.dirname(__file__))
import wp_decode_audit as W

BASE = os.path.expanduser('~/catkin_ws/dataset_arucode')
OUT = os.path.join(BASE, '_diag', '_warps')


def dump(folder_name, filename):
    path = os.path.join(BASE, folder_name, filename)
    bgr = cv2.imread(path)
    r = W.process_image(path)
    if not r['ok']:
        print(f"{filename}: SKIP ({r['reason']})")
        return
    body = W.largest_component(W.marker_mask(bgr))
    corners = W.corners_of(body)
    warp, aspect = W.warp_marker(bgr, corners)
    warp = W.rotate_to_canonical(warp, r['tail'])
    big = cv2.resize(warp, (900, 900), interpolation=cv2.INTER_NEAREST)
    gh = gw = 900 / W.GRID_N
    bits = r['bits']
    for rr in range(W.GRID_N):
        for cc in range(W.GRID_N):
            y0, x0 = int(rr * gh), int(cc * gw)
            y1, x1 = int((rr + 1) * gh), int((cc + 1) * gw)
            cv2.rectangle(big, (x0, y0), (x1, y1), (0, 255, 0), 2)
            mark = '#' if bits[rr, cc] else '.'
            cv2.putText(big, mark, (x0 + 15, y0 + 40),
                        cv2.FONT_HERSHEY_SIMPLEX, 1.1, (0, 0, 255), 2)
    os.makedirs(OUT, exist_ok=True)
    outp = os.path.join(OUT, f"{folder_name}_{filename.rsplit('.',1)[0]}_warp.png")
    cv2.imwrite(outp, big)
    print(f"{filename}: tail={r['tail']} ar={aspect:.3f} -> {outp}")


if __name__ == '__main__':
    # argv: folder file [folder file ...]
    args = sys.argv[1:]
    for i in range(0, len(args), 2):
        dump(args[i], args[i + 1])

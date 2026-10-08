#!/usr/bin/env python3
"""
Konversi sensor_msgs/Image <-> numpy TANPA cv_bridge.

cv_bridge rusak di Jetson ini (OpenCV 4.8 vs cv_bridge 4.2). Modul ini dipakai
bersama oleh node vision (aruco_node, gate_node). Menghormati msg.step (row
stride) bila ada padding. Dukung: bgr8, rgb8, bgra8, rgba8, mono8/8UC1.
"""
import numpy as np
from sensor_msgs.msg import Image

_CH = {"bgr8": 3, "rgb8": 3, "bgra8": 4, "rgba8": 4, "mono8": 1, "8UC1": 1}


def imgmsg_to_np(msg):
    """sensor_msgs/Image -> numpy BGR (3ch) atau gray (mono)."""
    import cv2
    ch = _CH.get(msg.encoding)
    if ch is None:
        raise ValueError("encoding tak didukung: %s" % msg.encoding)
    buf = np.frombuffer(msg.data, dtype=np.uint8)
    row = msg.width * ch
    if msg.step and msg.step != row:                    # buang padding tiap baris
        buf = buf.reshape(msg.height, msg.step)[:, :row]
    img = buf.reshape(msg.height, msg.width, ch) if ch > 1 \
        else buf.reshape(msg.height, msg.width)
    if msg.encoding == "rgb8":
        img = cv2.cvtColor(img, cv2.COLOR_RGB2BGR)
    elif msg.encoding == "bgra8":
        img = cv2.cvtColor(img, cv2.COLOR_BGRA2BGR)
    elif msg.encoding == "rgba8":
        img = cv2.cvtColor(img, cv2.COLOR_RGBA2BGR)
    return img


def np_to_imgmsg(img, stamp, frame_id):
    """numpy BGR -> sensor_msgs/Image (bgr8)."""
    msg = Image()
    msg.header.stamp = stamp
    msg.header.frame_id = frame_id
    msg.height, msg.width = img.shape[:2]
    msg.encoding = "bgr8"
    msg.is_bigendian = 0
    msg.step = msg.width * 3
    msg.data = img.tobytes()
    return msg

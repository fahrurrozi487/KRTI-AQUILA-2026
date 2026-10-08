#!/bin/bash
# set_camera_exposure.sh (2026-08-09) -- OPT-IN v4l2 manual exposure buat kamera
# bawah. Sengaja TERPISAH dari usb_cam_node (bukan tebak param ROS internal
# node itu, gak ada dokumentasi lokal di Jetson ini) -- kontrol device V4L2
# LANGSUNG, jalan SESUDAH usb_cam_node buka device (delay 3s), override
# auto-exposure default-nya.
#
# KENAPA opt-in, BUKAN default aktif: dites indoor (2026-08-09) -- manual
# exposure=2000 naikin brightness 2x (111->233 mean grayscale), TAPI itu
# kondisi cahaya RUANGAN, bukan lapangan/outdoor asli. Nilai exposure yang
# pas itu BEDA jauh tergantung cahaya (siang terik vs mendung vs teduh) --
# hardcode 1 angka tanpa validasi lapangan BERISIKO overexposed di kondisi
# terang (lebih parah dari underexposed yg mau diperbaiki). WAJIB di-tuning
# ULANG di lokasi lomba asli sebelum dipercaya, lih. docstring camera_down.launch.
#
# Pakai: set_camera_exposure.sh <video_device> <exposure_absolute>
set -e
DEVICE="${1:-/dev/video1}"
EXPOSURE="${2:-1500}"
sleep 3   # kasih waktu usb_cam_node buka device duluan & apply default-nya
v4l2-ctl -d "$DEVICE" --set-ctrl=exposure_auto=1 --set-ctrl=exposure_absolute="$EXPOSURE"
echo "[set_camera_exposure] $DEVICE -> exposure_auto=1 (manual) exposure_absolute=$EXPOSURE"

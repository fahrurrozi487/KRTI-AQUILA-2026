#!/usr/bin/env bash
# run_mission_gps.sh — bring-up navigasi GPS+OptFlow (BUKAN T265) + jalankan misi GUIDED.
#
# Beda dari resep T265 (AGENTS.md / WP_T265_TO_GUIDED_FLOW.md):
#   - TIDAK ada T265 / vision_to_mavros / set_origin_home.launch -- GPS kasih EKF
#     origin otomatis begitu fix, node-node itu SPESIFIK non-GPS (baca docstring
#     set_origin_home.py sendiri: "Set EKF origin + home position untuk terbang
#     non-GPS (T265)").
#   - mission_d.py / phaseD_mission.launch TIDAK diubah sama sekali -- keduanya cuma
#     baca /mavros/local_position/pose, gak peduli sumbernya GPS atau VIO.
#
# ⚠️ WAJIB DIBACA: params/gps_optflow_ek3.param HARUS sudah di-load ke FC (MP ->
#    Full Parameter List -> Load -> Write -> Reboot) SEBELUM jalanin script ini.
#    Script ini TIDAK meng-upload param -- itu manual sekali per konfigurasi FC.
#
# ⚠️ WAJIB DIBACA: waypoints.yaml yang ada sekarang disurvei di bawah T265
#    (origin = titik takeoff manual via set_origin_home, selalu (0,0,0) di situ).
#    Di mode GPS, origin datang OTOMATIS dari GPS -- belum tentu sama presisi
#    dengan titik yang sama. SURVEI ULANG (record_waypoint.py) di mode GPS
#    sebelum percaya waypoints lama, atau titik referensi bisa meleset.
#
# Pakai:
#   ./run_mission_gps.sh <mission_name> [extra roslaunch args utk phaseD_mission.launch]
#   contoh: ./run_mission_gps.sh seleksi do_takeoff:=false use_aruco:=true
set -euo pipefail

MISSION="${1:-seleksi}"
shift || true
FCU_URL="${FCU_URL:-/dev/ttyTHS1:921600}"
GPS_TIMEOUT="${GPS_TIMEOUT:-30}"

cleanup() {
  echo "[run_mission_gps] cleanup: matiin roscore/mavros yang dinyalain script ini..."
  kill "${MAVROS_PID:-}" "${ROSCORE_PID:-}" 2>/dev/null || true
}
trap cleanup EXIT

echo "=============================================================="
echo " GPS+OptFlow bring-up -- BUKAN T265"
echo "=============================================================="
echo "Checklist manual SEBELUM lanjut (jawab TIDAK kalau ragu):"
echo "  [ ] params/gps_optflow_ek3.param sudah di-load ke FC & FC sudah reboot"
echo "  [ ] Drone di tempat terbuka, GPS lock (bukan indoor/dekat tembok)"
read -rp "Semua checklist di atas OK, lanjut? [y/N] " -n1 REPLY; echo
[[ "$REPLY" =~ ^[Yy]$ ]] || { echo "Dibatalkan."; trap - EXIT; exit 1; }

echo "[run_mission_gps] roscore..."
roscore >/tmp/run_mission_gps_roscore.log 2>&1 &
ROSCORE_PID=$!
sleep 2

echo "[run_mission_gps] mavros (fcu_url=$FCU_URL)..."
roslaunch mavros apm.launch fcu_url:="$FCU_URL" >/tmp/run_mission_gps_mavros.log 2>&1 &
MAVROS_PID=$!

echo "[run_mission_gps] menunggu mavros connected..."
t0=$(date +%s)
until rostopic echo -n1 /mavros/state 2>/dev/null | grep -q "connected: True"; do
  sleep 1
  if (( $(date +%s) - t0 > 30 )); then
    echo "[run_mission_gps] GAGAL: mavros belum connected setelah 30s. Cek $FCU_URL / kabel."
    exit 1
  fi
done
echo "[run_mission_gps] mavros connected."

echo "[run_mission_gps] menunggu GPS fix (3D, sats>=8, HDOP<1.5), timeout ${GPS_TIMEOUT}s..."
t0=$(date +%s)
gps_ok=false
while (( $(date +%s) - t0 < GPS_TIMEOUT )); do
  raw=$(timeout 2 rostopic echo -n1 /mavros/gpsstatus/gps1/raw 2>/dev/null || true)
  fix_type=$(echo "$raw" | grep -m1 "fix_type:" | awk '{print $2}')
  sats=$(echo "$raw" | grep -m1 "satellites_visible:" | awk '{print $2}')
  eph=$(echo "$raw" | grep -m1 "eph:" | awk '{print $2}')
  if [[ -n "$fix_type" && "$fix_type" -ge 3 && -n "$sats" && "$sats" -ge 8 \
        && -n "$eph" && "$eph" -lt 150 ]]; then
    gps_ok=true
    echo "[run_mission_gps] GPS OK: fix_type=$fix_type sats=$sats HDOP=$(echo "scale=2;$eph/100"|bc)"
    break
  fi
  echo "  ...fix_type=${fix_type:-?} sats=${sats:-?} HDOP=$( [[ -n "$eph" ]] && echo "scale=2;$eph/100"|bc || echo '?')"
  sleep 2
done
if [[ "$gps_ok" != true ]]; then
  echo "[run_mission_gps] GPS belum memenuhi syarat otomatis -- cek manual dulu:"
  rostopic echo -n1 /mavros/gpsstatus/gps1/raw 2>/dev/null || true
  read -rp "Tetap lanjut walau GPS belum ideal? [y/N] " -n1 REPLY; echo
  [[ "$REPLY" =~ ^[Yy]$ ]] || { echo "Dibatalkan."; exit 1; }
fi

echo "=============================================================="
echo " Jalankan misi: $MISSION  (args tambahan: $*)"
echo "=============================================================="
roslaunch mission_control phaseD_mission.launch mission:="$MISSION" "$@"

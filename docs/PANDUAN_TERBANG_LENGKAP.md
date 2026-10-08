# PANDUAN TERBANG LENGKAP — T265 (VIO) vs GPS+MTF01

Konsolidasi semua yang dibahas/dibangun sesi 2026-08-07/08. Dua jalur navigasi
independen, `mission_d.py` (state machine misi: scan/align/drop/traverse/land)
**identik** di keduanya — dia cuma baca `/mavros/local_position/pose`, gak
peduli sumbernya GPS atau VIO. Yang beda cuma bring-up navigasi + cara
waypoint didapat.

**Pilih mode:**
| | T265 (VIO) | GPS+MTF01 |
|---|---|---|
| Kapan pakai | GPS lemah/indoor, atau sengaja non-GPS | **Primary** (keputusan tim, lih. `PANDUAN_GPS_OPTFLOW_MTF01.md` §1.2) |
| Origin | Manual, titik takeoff = (0,0,0) | Otomatis dari GPS fix |
| Drift risk | Rumput = tekstur homogen, riskan (lih. §T265.6) | Rendah (GPS akurat di lapangan terbuka) |
| Setup fisik tambahan | Mounting T265 dimiringkan, dst | Tidak ada |

---

## PRASYARAT BERSAMA (kedua mode)

1. **Kamera perception** (terpisah dari kamera navigasi T265):
   - `camera_down` (C920, `/dev/video0`) → ArUco/WP marker (drop/scan)
   - `camera_front` → gate detection (traverse)
2. **Servo drop**: channel 9, PWM 1013=buka(drop) / 2015=tutup.
3. **File misi** (`config/mission_*.yaml`) — pilih sesuai babak: `seleksi`,
   `final`, `aruco_drop`, dst. Baca komentar header tiap file (isinya
   dokumentasi legs).
4. **Status decoder perception** (berlaku KE DUA mode navigasi, ini masalah
   kamera bukan masalah navigasi):
   - **WP3, WP4**: reliable (presisi 99-100%), **WP1**: TIDAK reliable
     (recall cuma 2.9% — kemungkinan besar scan bakal timeout). Lih.
     `WP_MARKER_DECODER_V2.md`.
   - **WP2 (red box drop trigger)**: reliable (99.9%).
   - **Gate detector**: 85.6%(Double)/100%(Triple) TAPI **overfitting
     dikonfirmasi** ke 1 sesi foto (lih. `GATE_DETECTOR_DOUBLE_TRIPLE.md`
     §keterbatasan) — belum tervalidasi gerbang/cahaya lain.
5. **`align_sign_x/y` (ArUco/WP) & `gate_sign_x/y/z` (gate) — BELUM
   dikalibrasi fisik sama sekali** (dicek 2026-08-08, semua masih default
   kode `+1.0`). Salah sign = P-control di `mission_d.py` jadi POSITIVE
   FEEDBACK (drone dikoreksi MENJAUH dari target, bukan mendekat). WAJIB
   kalibrasi darat dulu sebelum align/traverse pertama kali dipercaya:
   ```bash
   rosrun mission_control calibrate_align_sign.py _mode:=align _expected_id:=1
   rosrun mission_control calibrate_align_sign.py _mode:=gate
   ```
   Gerakin marker/gerbang di depan kamera, baca arah "koreksi" yang
   di-print, cocokkan akal sehat mounting. TIDAK kirim setpoint ke drone —
   aman dijalankan kapan saja tanpa resiko terbang.
6. **Belum pernah terbang fisik penuh** — baik T265 maupun GPS, seluruh
   rantai (takeoff→scan→drop→traverse→land) di lapangan asli. Semua
   komponen tervalidasi TERPISAH (SITL, dataset statis, unit test) tapi
   BUKAN rantai penuh yang terbukti di lapangan. Pilot WAJIB siaga ambil
   alih (Stabilize) di kedua mode.

---

## BAGIAN T265 (VIO, non-GPS)

### T265.1 Prasyarat & status params (per 2026-08-08)

| Item | Status |
|---|---|
| `params/t265_nongps_ekf3.param` di-load ke FC | Manual, cek sebelum terbang |
| `params/t265_from_TFLuna_delta.param` (delta landing/drift/EKF) | Manual, cek sebelum terbang |
| Mounting T265 dimiringkan ~15° | ✅ Fisik terpasang |
| `pitch_cam` di `t265_tf_to_mavros.launch` | ✅ `-0.2618` (kompensasi tilt), tervalidasi matematis, **belum verifikasi fisik** |
| `VISO_POS_X` | ✅ 0.06m terukur |
| `VISO_POS_Y/Z` | ❌ Belum diukur, masih asumsi 0 |
| Covariance T265→EKF (confidence-aware) | ✅ Fix kode `vision_to_mavros.cpp` selesai, live-test OK, **belum tes di rumput** |
| `VISO_POS_M_NSE`/`VEL_M_NSE`/`YAW_M_NSE` | Default ArduPilot, belum di-tuning (butuh log terbang) |
| Kalibrasi IMU T265 | Tidak applicable (T265 factory-calibrated tertutup, sudah dikonfirmasi) |

### T265.2 Bring-up (4 terminal)

```bash
# T1 — kamera T265 (initial_reset:=true WAJIB, T265 suka -nan sebelum lock)
roslaunch realsense2_camera rs_t265.launch initial_reset:=true

# T2 — bridge T265 -> mavros (pitch_cam sudah -0.2618 di launch file default)
roslaunch vision_to_mavros t265_tf_to_mavros.launch

# T3 — mavros (FC via UART)
roslaunch mavros apm.launch fcu_url:=/dev/ttyTHS1:921600

# T4 — set EKF origin + home (WAJIB tiap boot FC, non-GPS gak punya lat/lon absolut)
roslaunch mission_control set_origin_home.launch
```

**Verifikasi tiap tahap** (jangan lanjut kalau ada yang gagal):
```bash
rostopic echo -n1 /mavros/state                            # connected: True
rostopic echo -n1 /camera/odom/sample/pose/covariance       # [0] idealnya <=0.1 (confidence Medium+)
rostopic echo -n1 /mavros/vision_pose/pose_cov               # covariance HARUS ikut angka di atas, bukan 0.1/0.001 statis
rostopic echo -n1 /mavros/local_position/pose                # pose masuk akal, bukan NaN
```

### T265.3 Survei waypoint (kalau titik lapangan berubah dari `waypoints.yaml` sekarang)

```bash
rosrun mission_control record_waypoint.py _name:=wp1
rosrun mission_control record_waypoint.py _name:=wp2
rosrun mission_control record_waypoint.py _name:=wp3
rosrun mission_control record_waypoint.py _name:=wp4
# edit z manual di config/waypoints.yaml kalau perlu (ketinggian tiap WP)
```

### T265.4 Kamera perception + terbang

```bash
# T5 — kamera bawah (ArUco/WP) + kamera depan (gate), atau cameras.launch gabungan
roslaunch mission_control camera_down.launch
roslaunch mission_control camera_front.launch    # kalau ada leg traverse

# T6 — pilot: takeoff manual Stabilize -> hover di/dekat WP1 -> switch GUIDED

# T7 — jalankan misi (contoh: babak seleksi, WP marker decoder v2, gate ganda)
roslaunch mission_control phaseD_mission.launch \
  mission:=seleksi do_takeoff:=false sim_markers:=false \
  use_gate_multi:=true use_gate:=false \
  use_wp_marker:=true use_aruco:=false
```

`scan_timeout` sudah default 12s (naik dari 8s, kompensasi recall decoder
rendah — lih. `WP_MARKER_DECODER_V2.md`). WP1 kemungkinan besar tetap
timeout — itu bukan bug.

### T265.5 Setelah terbang (WAJIB, buat lanjutin tuning)

Catat: `EK3.INN` velPos/hgt dari log (buat tuning `VISO_POS_M_NSE`/`VEL_M_NSE`
nanti), dan bandingkan tracking confidence sebelum vs sesudah tilt 15°
(`rostopic echo /camera/odom/sample/pose/covariance` selama terbang).

### T265.6 Blocker yang JUJUR belum selesai

- Verifikasi fisik kompensasi tilt (`pitch_cam`) — matematika terbukti,
  belum dicoba di udara.
- `VISO_POS_Y/Z` belum diukur.
- `VISO_*_M_NSE` belum di-tuning dari data log terbang beneran.
- Drift di rumput belum diukur post-fix (confidence-aware covariance +
  tilt 15°) — cuma prediksi teoretis akan membaik.

---

## BAGIAN GPS + MTF01 (primary)

### GPS.1 Prasyarat & status

| Item | Status |
|---|---|
| `params/gps_optflow_ek3.param` di-load ke FC | Manual, cek sebelum terbang |
| `scripts/run_mission_gps.sh` | ✅ Dibuat, syntax valid, **belum tes fisik** (gak ada FC/GPS live saat dibuat) |
| `scripts/latlon_to_waypoints.py` | ✅ Dibuat, selftest+preview PASS, **belum tes fisik** |
| `config/waypoints_latlon.yaml` | ⚠️ **MASIH TEMPLATE** — lat/lon placeholder, WAJIB diisi koordinat asli sebelum dipakai |

### GPS.2 Bring-up + terbang (via script)

```bash
cd ~/catkin_ws/src/mission_control/scripts
./run_mission_gps.sh seleksi do_takeoff:=false use_gate_multi:=true use_gate:=false use_wp_marker:=true use_aruco:=false
```
Script ini nanya checklist manual (param sudah di-load? GPS lock di tempat
terbuka?), nyalain `roscore`+`mavros`, cek GPS fix otomatis
(`fix_type>=3`, `sats>=8`, `HDOP<1.5`), baru jalanin
`phaseD_mission.launch`. **Kamera perception (T5 di bagian T265) TETAP
perlu dinyalain manual terpisah** — script ini cuma urus navigasi+misi.

**Atau manual** (kalau mau kontrol tiap langkah):
```bash
roscore
roslaunch mavros apm.launch fcu_url:=/dev/ttyTHS1:921600
# tunggu GPS fix -- cek /mavros/gpsstatus/gps1/raw (fix_type>=3, sats>=8, eph<150)
roslaunch mission_control camera_down.launch
roslaunch mission_control camera_front.launch
# pilot: takeoff manual -> switch GUIDED
roslaunch mission_control phaseD_mission.launch mission:=seleksi do_takeoff:=false ...
```
Catatan: **T265, `vision_to_mavros`, `set_origin_home.launch` TIDAK
dinyalain di mode ini** — origin datang otomatis dari GPS.

### GPS.3 Waypoint

**Isi dulu `config/waypoints_latlon.yaml`** dengan lat/lon asli (sekali,
permanen) — manual edit YAML, atau rekam pakai drone (GUIDED, GPS fix bagus,
terbang ke tiap titik fisik):
```bash
rosrun mission_control record_waypoint_latlon.py _name:=wp1
rosrun mission_control record_waypoint_latlon.py _name:=gate_double
# dst
```
Lalu tiap sesi (sesudah mavros connect + GPS fix):
```bash
rosrun mission_control latlon_to_waypoints.py
# cek angka x/y/z yang di-print masuk akal SEBELUM lanjut misi
```
Atau survei fisik manual (`record_waypoint.py`, sama seperti T265.3) kalau
belum ada koordinat lat/lon presisi.

### GPS.4 Blocker yang JUJUR belum selesai

- `waypoints_latlon.yaml` masih placeholder — isi koordinat lapangan asli
  dulu, atau pakai `record_waypoint.py` fisik.
- `run_mission_gps.sh` dan `latlon_to_waypoints.py` belum pernah jalan
  dengan FC+GPS live — baru tervalidasi syntax/matematika/logic offline.
- GPS+MTF01 sendiri (param profil) belum pernah diverifikasi terbang
  (lih. checklist §3 `PANDUAN_GPS_OPTFLOW_MTF01.md` — belum dicentang).

---

## Troubleshooting cepat (dua mode)

| Gejala | Kemungkinan sebab |
|---|---|
| `mavros/state connected: False` | `fcu_url`/kabel UART salah, FC belum nyala |
| Drone gak gerak pas misi jalan | Mode bukan GUIDED (kalau `do_takeoff:=false`) |
| Scan WP1 selalu timeout | **Bukan bug** — WP1 decoder `reliable=false`, recall 2.9% |
| Drop meleset | Cek `align_sign_x/y` (T265) belum dikalibrasi fisik, lih `PANDUAN_AUTO_DAN_ARUCO_DROP.md` |
| (T265) Posisi ngedrift pas manuver | Kemungkinan `VISO_POS_Y/Z` belum benar (masih 0) |
| (GPS) `latlon_to_waypoints.py` timeout nunggu `gp_origin` | GPS belum fix, atau mavros belum connect |

## Referensi

- T265 detail: `PANDUAN_TERBANG_T265.md`, `WP_T265_TO_GUIDED_FLOW.md`,
  `PANDUAN_GPS_OPTFLOW_MTF01.md` §1.1 (diagnosis drift + remediasi)
- GPS detail: `PANDUAN_GPS_OPTFLOW_MTF01.md`, `PANDUAN_MISI_GPS.md`
- Misi & drop ArUco: `PANDUAN_AUTO_DAN_ARUCO_DROP.md`
- Decoder WP v2: `WP_MARKER_DECODER_V2.md`
- Gate detector: `GATE_DETECTOR_DOUBLE_TRIPLE.md`
- Kalibrasi sign align/gate: `scripts/calibrate_align_sign.py`
- Geometri misi/gate berlapis: `project-mission-geometry-status`
- Memory: `project-t265-ekf-vis-status`, `project-gps-mission-status`,
  `project-task3-status`, `project-gate-detector-status`,
  `project-mission-geometry-status`

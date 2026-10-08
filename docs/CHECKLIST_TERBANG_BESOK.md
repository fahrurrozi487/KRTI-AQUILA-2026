# CHECKLIST TERBANG BESOK (2026-08-10)

Satu dokumen runtut, dari nyala sampai selesai, buat 2 sumber navigasi (T265 /
GPS+MTF01) × 3 cakupan misi (full / ArUco-only / gate-only). Referensi detail
tetap di `PANDUAN_TERBANG_LENGKAP.md` dkk — dokumen ini urutan EKSEKUSI-nya.

⚠️ **Belum pernah terbang fisik penuh sama sekali** (baik T265 maupun GPS).
Pilot WAJIB siaga ambil alih (Stabilize) di semua percobaan.

---

## 0. SEBELUM APAPUN — kalibrasi darat (sekali, berlaku ke semua mode di bawah)

Ini **wajib** duluan, sebelum navigasi/mission apapun dites, karena kalau
sign align/gate salah, drone bisa dikoreksi MENJAUH dari target (positive
feedback) bukan mendekat:

```bash
roscore   # kalau belum jalan
# nyalain kamera_down + kamera_front dulu (lih. langkah 2/3 di tiap mode)
rosrun mission_control calibrate_align_sign.py _mode:=align _expected_id:=1
rosrun mission_control calibrate_align_sign.py _mode:=gate
```
Gerakin marker/gerbang di depan kamera manual, baca arah "koreksi" yang
di-print, cocokkan akal sehat mounting kamera fisik. **Gak kirim setpoint ke
drone** — aman dijalankan kapan saja.

**Cara set nilai sign yang ketemu** — sekarang udah jadi argumen resmi di
`phaseD_mission.launch` (2026-08-10), 2 cara:
```bash
# cara 1: tambahkan ke command launch (gak ubah file, gampang dicoba-coba)
roslaunch mission_control phaseD_mission.launch ... align_sign_x:=-1.0 gate_sign_y:=-1.0

# cara 2: ubah default permanen -- edit phaseD_mission.launch, cari baris
#   <arg name="align_sign_x" default="1.0" />
# ganti "1.0" jadi "-1.0" (atau argumen sign/swap lain yang perlu dibalik)
```
Argumen yang tersedia: `align_sign_x`, `align_sign_y`, `align_swap_xy`,
`gate_sign_x`, `gate_sign_y`, `gate_sign_z`, `gate_swap_xy`.

**Update 2026-08-10**: bug arsitektur ditemukan & DIPERBAIKI — koreksi
align/gate dulu gak dirotasi berdasar yaw (cuma valid di 1 heading, padahal
misi punya belokan 90° antara WP2 & gate_triple). Sekarang `mission_d.py`
otomatis rotasi ke ENU pakai yaw real-time (`MavrosHelper.get_yaw()` +
`_body_to_enu()`, sudah lolos self-test matematika `python3 mission_d.py
--selftest`). **Konsekuensi buat kalibrasi**: CUKUP tes SEKALI di sembarang
heading yang nyaman — TIDAK perlu ulang di 2 heading berbeda lagi.

⚠️ **Fix ini BERGANTUNG ke yaw yang dibaca FC akurat** — beda sumber per mode:
- **GPS**: yaw dari **compass** (`EK3_SRC1_YAW=1`, lih. `PANDUAN_GPS_OPTFLOW_MTF01.md`).
  Kalau compass belum dikalibrasi/miring krn medan magnet lokal (motor,
  logam), `get_yaw()` bisa salah, koreksi align/gate ikut salah arah walau
  sign udah bener. **Pastikan compass udah dikalibrasi ulang di LOKASI
  TERBANG** (bukan cuma sekali waktu beli), terutama kalau lokasi beda dari
  kalibrasi terakhir.
- **T265**: yaw dari T265 sendiri (`EK3_SRC1_YAW=6`, EXTNAV), compass
  dimatikan total (`COMPASS_ENABLE=0`) — gak kena isu kalibrasi compass,
  tapi tetap tergantung tracking confidence T265 gak jatuh.

**Cara ngecek fix ini beneran jalan pas tes fisik pertama** (gate-only dulu,
lih. urutan tes di bawah): perhatikan **gate_double** (sebelum yaw 90°) DAN
**gate_triple** (sesudahnya) SAMA-SAMA mengoreksi MENDEKAT ke tengah bukaan
(bukan salah satunya menjauh). Kalau salah satu gerbang mengoreksi ke arah
yang jelas salah padahal yang lain benar, itu tanda `get_yaw()` gak akurat
(cek compass) — BUKAN tanda sign_x/y salah (karena kalau sign yang salah,
DUA-DUANYA bakal salah, bukan cuma satu).

---

## Sebelum bring-up navigasi apapun — cek fisik dasar

Belum tercakup di bagian navigasi manapun karena berlaku ke SEMUA mode:

1. **Servo drop** — tes AKTUASI FISIK dulu (buka lalu tutup), TANPA nunggu
   sampai misi jalan sampai leg drop:
   ```bash
   rosservice call /mavros/cmd/command "{command: 183, param1: 9, param2: 1013}"  # buka (drop)
   rosservice call /mavros/cmd/command "{command: 183, param1: 9, param2: 2015}"  # tutup
   ```
   (`183`=`MAV_CMD_DO_SET_SERVO`, `param1`=channel 9, `param2`=PWM — persis
   urutan `MavrosHelper.set_servo()`, cocokkan sama `servo_channel`/
   `servo_open`/`servo_close` di `phaseD_mission.launch` kalau beda).
   Pastikan channel bener & payload beneran jatuh/tertahan sesuai posisi servo,
   BUKAN cuma asumsi dari kode.
2. **Gate detector — INGAT keterbatasannya**: akurasi 85.6%(Double)/100%(Triple)
   itu **overfitting terkonfirmasi ke 1 sesi foto tuning** (lih.
   `GATE_DETECTOR_DOUBLE_TRIPLE.md` §keterbatasan) — belum tervalidasi di
   gerbang/cahaya lain. Jangan kaget kalau akurasi di lapangan beda dari
   angka itu, terutama kalau cahaya/background beda dari sesi foto tuning.
3. **Kompas** (mode GPS aja, lih. catatan di atas) — kalibrasi ulang di lokasi
   terbang kalau belum/lama.

## Urutan tes yang disarankan (jangan langsung full misi)

Komponen yang berubah hari ini (yaw-rotation, sign args, takeoff_alt)
**belum pernah divalidasi fisik sama sekali** — uji terisolasi dulu sebelum
digabung, biar kalau ada yang gagal, gampang tau letaknya:

1. **ArUco/WP only** dulu (`mission:=aruco_only`) — validasi scan+align+drop
   tanpa gate sama sekali
2. **Gate only** (`mission:=gate_only`) — validasi traverse gate_double DAN
   gate_triple (dua heading beda, lih. catatan fix yaw di atas) tanpa
   scan/drop
3. **Baru full misi** (`mission:=seleksi`/`final`) sesudah 2 di atas lolos
   sendiri-sendiri

---

## 1. Pilih argumen dasar (dipakai di SEMUA kombinasi di bawah)

| Argumen | Nilai | Kapan dipakai |
|---|---|---|
| `do_takeoff` | `true` = node yang takeoff otomatis, `false` = pilot udah hover GUIDED manual | Pilih salah satu |
| `takeoff_alt` | `1.0` (baru bisa dari command line, sebelumnya hardcoded 2.0) | Cuma efektif kalau `do_takeoff:=true` |
| `use_wp_marker` | `true` | **SELALU true** kalau ada leg scan/drop ArUco/WP — marker fisik custom 6×6, BUKAN ArUco standar. `use_aruco:=true` yang disebut di beberapa dokumen lama itu **SUDAH USANG**, jangan dipakai |
| `use_gate_multi` | `true` | **SELALU true** kalau ada leg traverse gerbang — gerbang fisik berlapis (double/triple), `use_gate` (lama, single-gate) gak paham `gate_layers>1` |
| `sim_markers` | `false` | **SELALU false** di lapangan — `true` cuma buat SITL |

---

## 2A. NAVIGASI: T265 (VIO, non-GPS)

### Bring-up (4 terminal, urutan penting)

```bash
# T1
roslaunch realsense2_camera rs_t265.launch initial_reset:=true

# T2
roslaunch vision_to_mavros t265_tf_to_mavros.launch

# T3
roslaunch mavros apm.launch fcu_url:=/dev/ttyTHS1:921600

# T4 (WAJIB tiap boot FC — non-GPS gak punya lat/lon absolut)
roslaunch mission_control set_origin_home.launch
```

**Verifikasi sebelum lanjut** (jangan skip kalau ada yang gagal):
```bash
rostopic echo -n1 /mavros/state                          # connected: True
rostopic echo -n1 /camera/odom/sample/pose/covariance     # [0] idealnya <=0.1
rostopic echo -n1 /mavros/vision_pose/pose_cov            # covariance ikut angka di atas, BUKAN 0.1/0.001 statis
rostopic echo -n1 /mavros/local_position/pose             # masuk akal, bukan NaN
```

### Kamera perception (T5, T6)
```bash
roslaunch mission_control camera_down.launch
roslaunch mission_control camera_front.launch   # kalau ada leg traverse (gate-only & full)
```

### Pilot: takeoff manual Stabilize → hover dekat WP1 → switch GUIDED
(skip kalau pakai `do_takeoff:=true`)

### Jalankan misi — pilih SATU sesuai cakupan:

**Full misi (scan+drop+gate, s/d WP3/WP4):**
```bash
roslaunch mission_control phaseD_mission.launch \
  mission:=seleksi do_takeoff:=false sim_markers:=false \
  use_gate_multi:=true use_gate:=false \
  use_wp_marker:=true use_aruco:=false
# atau mission:=final (lanjut s/d WP4) kalau babak final
```

**ArUco/WP only (scan+drop, tanpa gate):**
```bash
roslaunch mission_control phaseD_mission.launch \
  mission:=aruco_only do_takeoff:=false sim_markers:=false \
  use_gate:=false use_gate_multi:=false \
  use_wp_marker:=true use_aruco:=false
```

**Gate only (traverse double+triple, tanpa scan/drop):**
```bash
roslaunch mission_control phaseD_mission.launch \
  mission:=gate_only do_takeoff:=false sim_markers:=false \
  use_gate_multi:=true use_gate:=false \
  use_wp_marker:=false use_aruco:=false
```

Tambahkan `takeoff_alt:=1.0` ke command manapun kalau `do_takeoff:=true`.

### Sesudah terbang (T265, wajib dicatat)
Log `EK3.INN` velPos/hgt (buat tuning `VISO_POS_M_NSE`/`VEL_M_NSE` nanti),
`rostopic echo /camera/odom/sample/pose/covariance` selama terbang (cek
confidence gak jatuh di rumput).

---

## 2B. NAVIGASI: GPS + MTF01 (primary, direkomendasikan tim)

### Bring-up
**Cara cepat (via script):**
```bash
cd ~/catkin_ws/src/mission_control/scripts
./run_mission_gps.sh seleksi do_takeoff:=false use_gate_multi:=true use_gate:=false use_wp_marker:=true use_aruco:=false
```
Script nanya checklist manual (param `gps_optflow_ek3.param` udah di-load?
GPS lock di tempat terbuka?), nyalain roscore+mavros, cek GPS fix otomatis
(`fix_type>=3`, `sats>=8`, `HDOP<1.5`).

**Atau manual (kontrol tiap langkah):**
```bash
roscore
roslaunch mavros apm.launch fcu_url:=/dev/ttyTHS1:921600
rostopic echo -n1 /mavros/gpsstatus/gps1/raw   # fix_type>=3 sats>=8 eph<150 SEBELUM lanjut
```

**T265, `vision_to_mavros`, `set_origin_home.launch` TIDAK dinyalain di mode
ini** — origin datang otomatis dari GPS.

### Kamera perception (sama seperti T265, terpisah dari navigasi)
```bash
roslaunch mission_control camera_down.launch
roslaunch mission_control camera_front.launch
```

### Waypoint — WAJIB dicek sebelum misi pertama hari ini
`config/waypoints_latlon.yaml` isinya masih placeholder di sesi terakhir yang
tercatat — cek dulu isinya beneran koordinat lapangan asli (bukan template):
```bash
cat ~/catkin_ws/src/mission_control/config/waypoints_latlon.yaml
```
Kalau masih placeholder, rekam dulu (GUIDED, GPS fix bagus, terbang ke tiap
titik fisik):
```bash
rosrun mission_control record_waypoint_latlon.py _name:=wp1
rosrun mission_control record_waypoint_latlon.py _name:=wp2
rosrun mission_control record_waypoint_latlon.py _name:=wp3
rosrun mission_control record_waypoint_latlon.py _name:=gate_double
rosrun mission_control record_waypoint_latlon.py _name:=gate_triple
# wp4 kalau babak final
```
(Sengaja rekam `gate_double`/`gate_triple` juga secara EKSPLISIT lewat GPS,
bukan andalin hitungan geometris lama di `waypoints.yaml` — itu dihitung
dgn asumsi origin T265 (=titik takeoff), yg gak persis sama dengan origin
GPS. Komentar di `waypoints_latlon.yaml` bilang gate points "tidak perlu"
direkam DI SITU kalau mau tetap pakai geometri lama — tapi cara paling
aman & gak ambigu buat mode GPS ya rekam langsung kayak di atas.)
Lalu **tiap sesi** (sesudah mavros connect + GPS fix baru):
```bash
rosrun mission_control latlon_to_waypoints.py
# CEK angka x/y/z yang di-print masuk akal (jarak antar-WP sesuai fisik) SEBELUM lanjut
```

### Pilot: takeoff manual Stabilize → hover dekat WP1 → switch GUIDED
(skip kalau `do_takeoff:=true`)

### Jalankan misi — sama persis 3 command di bagian 2A, TANPA ganti apapun
(`mission_d.py` gak peduli sumber posisi GPS atau T265 — command launch-nya
identik, cuma bring-up sebelumnya yang beda).

---

## 3. Troubleshooting cepat

| Gejala | Kemungkinan sebab |
|---|---|
| `mavros/state connected: False` | `fcu_url`/kabel UART salah, FC belum nyala |
| Drone gak gerak pas misi jalan | Mode bukan GUIDED (kalau `do_takeoff:=false` dan pilot belum switch) |
| Scan WP1 selalu timeout | Recall WP1 udah dinaikin (2.7%→8.7% per 9 Agustus) tapi masih paling lemah dari 3 WP — bisa aja tetap timeout, bukan otomatis bug |
| Drop meleset | `align_sign_x/y` belum tervalidasi FISIK (cuma ground-test) — cek langkah 0 |
| Traverse gerbang gak center | `gate_sign_x/y/z` sama, cek langkah 0 |
| (T265) Posisi ngedrift pas manuver | `VISO_POS_Y/Z` belum diukur, masih asumsi 0 |
| (GPS) `latlon_to_waypoints.py` timeout nunggu `gp_origin` | GPS belum fix, atau mavros belum connect |
| Takeoff berhenti di ketinggian ganjil (bukan pas `takeoff_alt`) | Normal — toleransi `takeoff_alt - 0.3m`, proporsinya lebih longgar di target rendah (1m) |

## Referensi lengkap (baca kalau troubleshooting lebih dalam)
`PANDUAN_TERBANG_LENGKAP.md` (master lama, masih valid buat detail param),
`PANDUAN_GPS_OPTFLOW_MTF01.md`, `PANDUAN_MISI_GPS.md`,
`PANDUAN_AUTO_DAN_ARUCO_DROP.md`, `WP_MARKER_DECODER_V2.md`,
`GATE_DETECTOR_DOUBLE_TRIPLE.md` (akurasi & keterbatasan overfitting).

# HANDOFF — Drone Non-GPS T265 (Jetson Nano)

Dokumen alih-tangan untuk coding agent lain (mis. OpenCode). Ringkasan konteks & keputusan
yang TIDAK tersimpan di kode. Semua klaim diverifikasi ke repo per 2026-07-16.
Tidak memuat API key / token / rahasia.

---

## 1. Tujuan proyek

Drone kompetisi **non-GPS** (indoor-style navigation di lapangan **rumput outdoor**).
Navigasi posisi dari **Intel RealSense T265** (VIO) sebagai pengganti GPS, di-fuse ke
ArduPilot lewat MAVROS. Misi: takeoff → terbang antar-waypoint → scan marker ArUco →
align & drop payload → lewati gate → land.

- **Babak seleksi:** s/d WP3. **Babak final:** s/d WP4.
- Keputusan tim: **T265-only, non-GPS**. GPS (kalau ada) hanya redundansi anti-flyaway,
  bukan sumber navigasi.

## 2. Hardware & lingkungan

- **Jetson Nano 4 GB**, Ubuntu 20.04, ROS Noetic (Python 3.8). Workspace: `~/catkin_ws`.
- **Pixhawk 6C**, ArduCopter (4.5+, EKF2 sudah dihapus firmware → **EKF3-only**).
- Koneksi FC↔Jetson: **UART `/dev/ttyTHS1` @921600** (TELEM2/SERIAL2). nvgetty disabled,
  udev rules + grup dialout sudah diset.
- **T265** front-facing (USB ke kanan) — VIO posisi.
- Kamera misi: USB cam bawah (scan/align ArUco) + USB cam depan (deteksi gate).
- Servo drop: **ch9**, PWM **1013=buka/drop**, **2015=tutup**.
- GCS: Mission Planner via **radio SiK 57600** langsung ke Pixhawk (BUKAN lewat Jetson;
  `gcs_url` mavros kosong).

## 3. Build & jalankan

> ⚠️ **Penting:** workspace ini dibangun dengan **`catkin build`** (catkin_tools),
> BUKAN `catkin_make`. Jangan campur keduanya (mereka pakai layout build berbeda dan
> saling merusak). `AGENTS.md` sudah dikoreksi ke `catkin build`.

```bash
cd ~/catkin_ws && catkin build          # build semua
source ~/catkin_ws/devel/setup.bash     # tiap shell baru / sesudah build
catkin build mission_control            # build satu package
python3 -m py_compile <script>.py       # cek syntax (tak ada linter)
```

Self-test detektor (tanpa ROS/kamera):
```bash
python3 src/mission_control/scripts/aruco_detector.py --selftest
python3 src/mission_control/scripts/gate_detector.py --selftest
```

## 4. Struktur package (`src/`)

- **mission_control/** — package utama: state machine misi, detektor, node.
  - `scripts/` — node rospy + core detektor OpenCV murni.
  - `launch/`, `config/` (waypoints + urutan misi), `msg/`.
- **vision_to_mavros/** — T265 pose → mavros (VIO). Ada guard NaN (lihat §8).
- **mavros/, mavlink/, realsense-ros/, iq_sim/** — upstream.

## 5. Node & skrip (mission_control/scripts)

| File | Fungsi | Status |
|---|---|---|
| `mavros_helper.py` | Pondasi: koneksi mavros, arm/mode/takeoff/setpoint/servo | ✅ |
| `mission_d.py` | State machine **GUIDED**: takeoff→goto→scan→align→drop→traverse→land | ✅ SITL |
| `mission_auto.py` | Generator+uploader misi **AUTO** (ENU→lat/lon, push ke FC) | ⚠️ belum uji live |
| `mission_node.py` | Fase B: takeoff + goto 1 koordinat | ✅ SITL |
| `set_origin_home.py` | Set EKF origin + home (WAJIB tiap boot, non-GPS) | ✅ teruji FC |
| `record_waypoint.py` | Rekam posisi T265 → waypoints.yaml (survei) | ✅ |
| `aruco_detector.py` / `aruco_node.py` | Deteksi ArUco (core + node) | ✅ |
| `gate_detector.py` / `gate_node.py` | Deteksi gate | ⏸️ DIPARKIR (tunggu footage head-on) |
| `image_convert.py` | Image↔numpy manual (cv_bridge rusak) | ✅ |
| `sim_markers.py` | Mock ArUco untuk uji logika misi di SITL | ✅ |

## 6. Config

- `config/waypoints.yaml` — koordinat ENU lokal (x=E, y=N, z=Up) relatif EKF origin.
  Saat ini PLACEHOLDER; ganti dgn survei `record_waypoint.py`.
- `config/mission_seleksi.yaml` / `mission_final.yaml` — urutan leg GUIDED (scan/drop/traverse/land).
- `config/mission_auto.yaml` — rute AUTO geometris (waypoint/drop-buta/land; TANPA kamera).
- `params/t265_nongps_ekf3.param` — param FC EKF3 (lihat §7).
- `params/sr1_telem_57600.param` — stream rate radio (lihat §9).
- `params/archive/t265_nongps_ekf2.param.OLD` — param EK2 usang, JANGAN dipakai.

## 7. Parameter ArduPilot (EKF3, non-GPS)

File: `params/t265_nongps_ekf3.param`. Kunci:
```
AHRS_EKF_TYPE=3, EK3_ENABLE=1, EK2_ENABLE=0
EK3_SRC1_POSXY=6 (ExternalNav), VELXY=6, POSZ=1 (Baro!), VELZ=6, YAW=6 (kamera)
VISO_TYPE=2 (IntelT265), GPS1_TYPE=0, COMPASS_USE/USE2/USE3=0
SERIAL2_PROTOCOL=2, SERIAL2_BAUD=921
```
- **`EK3_SRC1_POSZ=1` (Baro), bukan ExternalNav** — sengaja, altitude T265 rawan getaran.
- **`EK3_SRC1_YAW=6`** — kompas MATI, heading dari kamera. JEBAKAN: default `1` (Compass)
  bikin "stopped aiding" berulang. Wajib 6. Set via **Load file**, jangan ketik manual
  (sering ada param kelewat).

## 8. Fitur selesai & tervalidasi

- **Fase B/D di SITL** (resep: SITL + MAVProxy bridge + roscore terpisah + mavros ke MAVProxy).
- **set_origin_home.py** teruji ke FC nyata: set origin + home. Fix bug datum altitude —
  home pakai `current_gps=True` (BUKAN altitude manual), agar `rel_alt` di tanah ≈ 0.
- **Guard NaN** di `vision_to_mavros.cpp`: pose `-nan/inf` dari T265 dibuang sebelum publish
  (pakai `if(finite) publish`, bukan `continue` — agar rate.sleep tetap jalan). Build OK.
- **`initial_reset:=true`** di `t265_all_nodes.launch` — tracking mulai bersih.
- **`pitch_cam`** di `t265_tf_to_mavros.launch` — koreksi mount miring. NILAI TERGANTUNG
  MOUNT: `-0.2611` (kompensasi ~15° miring). **Kalau mount sudah lurus → set `0`.**
- **mission_auto.py** — mode `preview` (offline) & `push` (upload+readback). Konversi
  ENU→lat/lon terverifikasi. Belum diuji push live ke FC.

## 9. Masalah diketahui & keputusan teknis

- **Scale error T265 outdoor 20-30%** (rumput seragam, jarak jauh). BELUM ada koreksi
  scale (jalur ROS `vision_to_mavros` tak punya knob; ada di non-ROS `t265_to_mavlink.py`).
  Rencana: tambal `scale_factor` ~5 baris ke node ROS — HANYA setelah walk test membuktikan
  skala meleset konsisten. Jangan pindah ke non-ROS (merombak 673 baris misi demi 1 fitur).
- **GCS lag** = link radio 57600 kirim stream rendah (`SR1_*`), BUKAN Jetson/UART. Sudah
  dinaikkan (`params/sr1_telem_57600.param`). Ada indikasi lag memburuk saat vision 30Hz
  jalan (FC forward VISP ke radio) — belum tuntas dikonfirmasi.
- **Crash Loiter:** takeoff langsung Loiter dari darat → flyaway. Root: estimasi vision
  belum sehat (drift depan-kiri di AltHold = warning). Loiter memperbesar error jadi
  perintah motor agresif.

## 10. Bug/risiko

- **cv_bridge rusak** (OpenCV 4.8 vs 4.2) → pakai `image_convert.py`, bukan cv_bridge.
- **ArUco WP=ID:** WP1=id1, WP2=id2, WP3=id3, WP4=id4. DICT_7X7_50, allowlist 1-4.
  Marker lapangan putih-di-hitam → detektor `invert=true` (default di aruco_detect.launch).
- **Gate:** bidik BUKAAN (celah), bukan centroid oranye. Double-gate: oranye tengah =
  panel pembatas (bidik situ = nabrak).
- **Origin & home hilang tiap reboot FC** — non-GPS wajib set ulang tiap boot.
- **Takeoff GUIDED:** JANGAN stream setpoint saat takeoff (masuk sub-mode Position →
  tak naik → auto-disarm). Takeoff dulu tanpa setpoint, baru goto.

## 11. Keselamatan (WAJIB)

- Jangan arm / aktifkan motor / kirim perintah aktuator / firmware **tanpa izin eksplisit**.
- Urutan terbang: **Stabilize/AltHold dulu → hover stabil → baru Loiter/AUTO**, jari selalu
  siap balik Stabilize. Loiter/AUTO hanya aman kalau hover T265 sudah tenang.
- **Walk test darat** (gendong drone, cek jejak di MP searah & seskala) = gerbang mutlak
  sebelum terbang berbasis vision. BELUM dikonfirmasi lulus.

## 12. Alur operasi (tiap boot, non-GPS)

```bash
# Terminal 1: kamera
roslaunch realsense2_camera rs_t265.launch initial_reset:=true   # diamkan 3s
# Terminal 2: mavros
roslaunch mavros apm.launch fcu_url:=/dev/ttyTHS1:921600          # connected: True
# Terminal 3: jembatan pose
roslaunch vision_to_mavros t265_tf_to_mavros.launch enable_precland:=false  # ~30Hz
# Terminal 4: origin+home DI TITIK TAKEOFF
roslaunch mission_control set_origin_home.launch                 # origin+home OK
```
Verifikasi: `rostopic hz /mavros/vision_pose/pose` (~30), MP: `VISION_POSITION_ESTIMATE` masuk,
`rel_alt`≈0, tak ada "stopped aiding" berulang.

## 13. Langkah berikutnya (urut)

1. **Walk test darat** — verifikasi jejak vision (gerbang mutlak, belum lulus).
2. Ukur skala (gerak 5m terukur). Meleset konsisten? → tambal `scale_factor` di node ROS.
3. Cek jarak T265↔CG. >15cm → offset kompensasi perlu; <15cm → abaikan.
4. AltHold hover diam → Loiter (jari di Stabilize).
5. Survei WP (`record_waypoint.py`) → betulkan `z` → uji AUTO (dry-run push dulu).
6. Aktifkan gate detector saat footage head-on datang.

## 14. Catatan migrasi ke OpenCode

- **`AGENTS.md` sudah ada** di root (80 baris) → OpenCode langsung pakai itu; Langkah 6
  dokumen migrasi (soal `CLAUDE.md` fallback) TIDAK berlaku (tak ada `CLAUDE.md`).
- **`AGENTS.md` sudah dikoreksi** (`catkin_make` → `catkin build`). Isinya belum mencakup
  kerja terbaru (EKF3, set_origin_home, mission_auto, guard NaN, front-facing, GCS/radio) —
  detail itu ada di HANDOFF.md ini; boleh disalin balik ke `AGENTS.md` nanti bila perlu.
- Tak ada skills/commands/subagents Claude di proyek (tak perlu migrasi manual).
- `.claude/settings.local.json` = permission lokal Claude Code, TIDAK relevan untuk OpenCode.
- **Memori Claude** (`~/.claude/projects/.../memory/`) tidak otomatis terbawa. Fakta
  pentingnya sudah dirangkum di dokumen ini + `PANDUAN_TERBANG_T265.md` + `PLANNING_MISI_KOMPETISI.md`.

# Alur Data & Tutorial SITL: T265 Init → Misi GUIDED ke Pixhawk

Pelengkap `PANDUAN_TERBANG_T265.md` (command operasional lapangan asli) dan
`AGENTS.md` (gotcha). Dokumen ini berisi DUA bagian:

- **Bagian A (§1-4): arsitektur alur data** — node apa hidup di tiap tahap,
  topic/service apa mengalir, urutan dependency wajib. Berlaku untuk terbang
  FISIK non-GPS dengan T265 asli.
- **Bagian B (§5): tutorial SITL tervalidasi** — command persis + output yang
  terbukti muncul untuk menjalankan Misi Seleksi Wilayah (s/d WP3) di drone
  VIRTUAL (ArduCopter SITL, GPS simulasi — **bukan** T265/EKF3). Semua
  langkah di §5 sudah benar-benar dijalankan dan berhasil (2026-07-31), bukan
  teoretis.

Kedua bagian saling melengkapi, bukan duplikat: Bagian A untuk memahami/
menjalankan alur data FISIK (T265→EKF3→GUIDED), Bagian B untuk memvalidasi
LOGIKA state machine misi (`mission_d.py`) di SITL sebelum ke lapangan.

---

# BAGIAN A — Arsitektur Alur Data (Lapangan Asli, T265)

## 1. Diagram alur node (ringkas)

```
[T1] rs_t265.launch (kamera T265)
        │  /camera/odom/sample
        ▼
[T3] t265_tf_to_mavros.launch (vision_to_mavros_node)
        │  /mavros/vision_pose/pose  (~30Hz, remap dari topic internal "vision_pose")
        ▼
[T2] mavros apm.launch  ──MAVLink──▶  Pixhawk EKF3
        │  (harus SUDAH connected sebelum T3 bisa publish efektif)
        ▼
[T4] set_origin_home.launch (one-shot, DI titik takeoff)
        │  set origin + home via mavros service
        ▼
[T5] phaseD_mission.launch → mission_d.py (state machine GUIDED)
        │  /mavros/setpoint_position/local  (setpoint 20Hz, HANYA setelah takeoff)
        ▼
     Pixhawk (GUIDED mode, sub-mode TakeOff → Position)
```

T1/T2 tidak saling bergantung urutan start (bisa dinyalakan bersamaan),
tapi **T3 butuh T1 & T2 sudah hidup** untuk bisa publish pose yang berguna,
**T4 butuh T3 sudah publish pose ke EKF**, dan **T5 butuh T4 sudah selesai**
(origin/home ter-set). Lihat §3 untuk gate-check tiap panah.

---

## 2. Node per tahap

| Tahap | Launch/command | Node | Fungsi |
|---|---|---|---|
| T1 | `roslaunch realsense2_camera rs_t265.launch` | kamera T265 driver | Publish pose VIO mentah |
| T2 | `roslaunch mavros apm.launch fcu_url:=/dev/ttyTHS1:921600` | mavros | Bridge MAVLink ↔ ROS ke Pixhawk |
| T3 | `roslaunch vision_to_mavros t265_tf_to_mavros.launch enable_precland:=false` | `t265_to_mavros` (`vision_to_mavros_node`) | Transform frame T265 → ENU, buang NaN, publish ke mavros |
| T4 | `roslaunch mission_control set_origin_home.launch` | `set_origin_home` | Set EKF origin + home (one-shot, exit sendiri) |
| T5 | `roslaunch mission_control phaseD_mission.launch` | `mission_d` (+ `mavros_helper.py` sbg helper, bukan node terpisah) | State machine misi: takeoff → per-leg goto/scan/drop/traverse/land |

---

## 3. Topic & service per panah, plus gate-check

### T1 → T3
- **Topic:** `/camera/odom/sample` (internal realsense-ros, dikonsumsi `t265_to_mavros`).
- **Gate-check:** tidak ada perintah langsung; kalau T3 tidak publish `/mavros/vision_pose/pose` sama sekali, cek T1 dulu (lihat troubleshooting §7 PANDUAN_TERBANG_T265.md — kemungkinan node kamera belum jalan, atau `RS2_USB_STATUS_BUSY` karena proses lama masih pegang USB).

### T2 (mavros) — prasyarat paralel
- **Gate-check:** `rostopic echo -n1 /mavros/state` → harus `connected: True`.
  Gagal → cek wiring TX/RX (kemungkinan tertukar), baud (`SERIAL2_BAUD`), atau
  `BRD_SER2_RTSCTS` belum `0`.

### T3 → EKF3 (via mavros)
- **Topic:** `/mavros/vision_pose/pose` (`geometry_msgs/PoseStamped`, remap dari
  `vision_pose` internal node `t265_to_mavros`). Publish rate target ~30Hz
  (`output_rate` di launch di-set 15, tapi verifikasi lapangan pakai ~30Hz —
  cross-check kalau ada mismatch nilai).
- **MAVLink:** mavros forward pose ini sebagai `VISION_POSITION_ESTIMATE` ke FC.
- **Gate-check:**
  - `rostopic hz /mavros/vision_pose/pose` → harus ~30Hz. Kosong → T1 atau T3
    bermasalah (cek `/camera/odom/sample` jalan).
  - Mission Planner → Ctrl+F → Mavlink Inspector → `VISION_POSITION_ESTIMATE`
    harus masuk.
- **NaN guard (AGENTS.md):** `vision_to_mavros.cpp` skip publish kalau pose
  T265 masih `-nan` (sebelum tracking lock). Launch T265 dengan
  `initial_reset:=true` supaya tracking mulai bersih.
- **Orientasi kamera:** T265 **front-facing**, USB ke kanan (`r=0,p=0,y=0,
  gamma=-1.5708` — sudah default di `t265_tf_to_mavros.launch`). `pitch_cam`
  kompensasi mount miring; set `0` hanya kalau mount sudah fisik lurus.

### EKF3 (Pixhawk) — parameter kunci (bukan node, tapi gate wajib sebelum T4)
- `AHRS_EKF_TYPE=3`, `EK3_ENABLE=1`, `EK2_ENABLE=0`, `GPS1_TYPE=0`,
  `VISO_TYPE=2`, `EK3_SRC1_POSXY/VELXY/VELZ/YAW=6` (ExternalNav/kamera),
  `EK3_SRC1_POSZ=1` (Baro, sengaja bukan ExternalNav — altitude T265 rawan
  getaran), `COMPASS_ENABLE=0`.
- **Jebakan (AGENTS.md):** `EK3_SRC1_YAW` **wajib 6**. Default `1` (compass,
  yang sudah dimatikan) → EKF "stopped aiding" berulang.

### T4: set_origin_home
- **Service call** (dari `set_origin_home.py`, bukan lewat `mavros_helper.py`
  — node terpisah, one-shot) ke mavros untuk set origin GPS palsu + home.
- **Wajib:** dijalankan **DI titik takeoff** — origin = (0,0,0) untuk semua
  WP relatif. Home pakai `current_gps=True` (bukan altitude manual), supaya
  `rel_alt` di tanah ≈ 0.
- **Wajib diulang tiap boot FC** — origin & home hilang tiap reboot Pixhawk
  (non-GPS, FC tak simpan referensi lat/lon sendiri).
- **Gate-check:** muncul pesan **"GPS Glitch"** lalu **"GPS Glitch cleared"**
  di Mission Planner, ikon drone muncul di peta. Setelah itu `rel_alt`
  seharusnya ≈0 di tanah dan tidak ada "stopped aiding" berulang.

### T4 → T5
- **Dependency:** `phaseD_mission.launch` **tidak** meng-include T1-T4 sendiri
  (cek isi launch file — hanya include `gate_detect`/`aruco_detect`/
  `wp_marker_detect` kondisional). Artinya urutan T1→T2→T3→T4 harus SUDAH
  selesai manual sebelum `roslaunch mission_control phaseD_mission.launch`
  dijalankan.
- **Walk test (gerbang mutlak sebelum terbang berbasis vision):** gendong
  drone jalan kotak ~2×2m, bandingkan lintasan di Mission Planner/rviz vs
  gerakan nyata (bentuk & skala harus cocok, tanpa drift besar). ⚠️
  **Status walk test berbeda antar dokumen** — `PANDUAN_TERBANG_T265.md` §4e
  menempatkannya sebagai prosedur baku darat (checklist rutin), tapi
  `HANDOFF.md` §11/§13 (per 2026-07-16) masih menandai "BELUM dikonfirmasi
  lulus". Cek status terkini sebelum menganggap walk test sudah established
  routine.

### T5: mission_d.py internal — urutan service call GUIDED (kritis)

Ini bagian paling gampang salah kalau di-generalisasi tanpa baca kode. Urutan
persis di `_takeoff_seq()` (`mission_d.py`):

1. Loop `mavros/set_mode` service → `custom_mode="GUIDED"`, sampai
   `/mavros/state.mode == "GUIDED"`.
2. Loop `mavros/cmd/arming` service → `True`, sampai `/mavros/state.armed`.
3. Panggil `mavros/cmd/takeoff` service (`CommandTOL`, altitude target).
   **TIDAK ADA setpoint dikirim sebelum/selama langkah ini.**
4. Poll `/mavros/local_position/pose` sampai `z` mendekati target altitude
   (toleransi 0.3m, timeout 30s).
5. **Baru setelah itu** loop misi utama mulai publish
   `/mavros/setpoint_position/local` per-leg, di 20Hz.

**Kenapa urutan ini kaku (AGENTS.md + HANDOFF.md §10):** ArduCopter GUIDED
punya sub-mode TakeOff/Position. Kalau setpoint di-stream **sebelum** atau
**selama** takeoff (langkah 3), FC pindah ke sub-mode Position → tidak naik
→ auto-disarm. Root cause-nya di firmware, bukan bug kode — semua caller
yang mau takeoff GUIDED harus ikut urutan ini persis.

**Health-check berjalan (dipanggil di setiap primitive — goto/scan/align/
traverse/yaw):** `_healthy()` cek `state.armed` DAN umur pose lokal
(`pose_stale_sec`, default 2.0s). Gagal → `_abort()` → `set_mode("LAND")`.
Catatan: LAND butuh EKF sehat (HANDOFF §9) — kalau EKF sudah sangat buruk
saat abort, LAND sendiri bisa tidak stabil; ini titik rapuh yang belum
ada mitigasi selain kesiagaan pilot.

---

## 4. Tabel referensi cepat: topic & service

| Nama | Tipe | Arah | Dipakai oleh |
|---|---|---|---|
| `/camera/odom/sample` | internal realsense-ros | T1 → T3 | `t265_to_mavros` |
| `/mavros/vision_pose/pose` | `geometry_msgs/PoseStamped` | T3 → mavros → EKF3 | verifikasi `rostopic hz` |
| `/mavros/state` | `mavros_msgs/State` | mavros → semua | `mavros_helper.py`, gate-check T2 |
| `/mavros/local_position/pose` | `geometry_msgs/PoseStamped` | mavros → `mission_d.py` | `_pose_age()`, `_reached()`, `_get_yaw()` |
| `/mavros/setpoint_position/local` | `geometry_msgs/PoseStamped` | `mission_d.py` → mavros → FC | HANYA setelah takeoff selesai |
| `mavros/cmd/arming` | service `CommandBool` | `mission_d.py` → mavros | arm/disarm |
| `mavros/set_mode` | service `SetMode` | `mission_d.py` → mavros | GUIDED/LAND |
| `mavros/cmd/takeoff` | service `CommandTOL` | `mission_d.py` → mavros | takeoff altitude |
| `mavros/cmd/command` | service `CommandLong` | `mission_d.py` → mavros | servo drop (`DO_SET_SERVO`, ch9) |
| `/aruco_node/markers` atau `/wp_marker_node/markers` | `mission_control/ArucoMarkers` | detektor → `mission_d.py` | scan/align (pilih via `use_wp_marker` di `phaseD_mission.launch`) |
| `/gate_node/gate` | `mission_control/Gate` | detektor → `mission_d.py` | traverse |
| `/wp_marker_node/result` | `mission_control/WpMarkerResult` | detektor → `mission_d.py` | heading align (Task 4, opsional) |

---

# BAGIAN B — Tutorial SITL Tervalidasi: Misi Seleksi Wilayah (WP1→WP3)

Gaya cookbook — command persis, output yang diharapkan, cara verifikasi
sebelum lanjut ke langkah berikutnya. Target pembaca: belum pernah
menjalankan sistem ini sama sekali.

**Ini SITL** (drone virtual, ArduCopter default GPS-sim — **TIDAK** memakai
T265/EKF3 sama sekali, beda jalur dari Bagian A). Untuk validasi *logika
misi* sebelum ke lapangan. Semua langkah di bawah **sudah dijalankan dan
terbukti berhasil** (2026-07-31), bukan teoretis.

Scope: **Misi Seleksi Wilayah = takeoff → WP1 → gate_double → WP2 (drop)
→ yaw → gate_triple → WP3 (land)**, sesuai `mission_seleksi.yaml`.
BUKAN Misi Final (yang sampai WP4).

## 5.1 Prasyarat

```
[ ] ~/ardupilot/ArduCopter ada, sudah pernah di-build (--no-rebuild dipakai)
[ ] ~/catkin_ws sudah di-build:
      cd ~/catkin_ws && catkin build && source devel/setup.bash
[ ] mavproxy terpasang (mavproxy.py ada di PATH)
[ ] Tidak ada proses SITL/ROS lama nyangkut (lihat Langkah 0 verifikasi)
[ ] Paham: ini SITL, drone VIRTUAL — arm/takeoff/dll di bawah AMAN,
    TIDAK menyentuh Pixhawk/drone fisik apapun
```

### Langkah 0 — Verifikasi bersih sebelum mulai

```bash
pgrep -af "sim_vehicle|arducopter|mavproxy|roscore|rosmaster|mavros" || echo bersih
ss -ltn | grep -E ":5760|:14550|:11311"
```
**Output diharapkan:** `bersih` (atau tidak ada baris) — tidak ada proses/port lama.
Kalau ada sisa proses, `kill <PID>` satu-satu (bukan `pkill`, lihat
Troubleshooting §5.9 #3) sebelum lanjut.

## 5.2 Langkah 1: Nyalakan ArduCopter SITL

**Terminal 1:**
```bash
cd ~/ardupilot/ArduCopter
sim_vehicle.py -v ArduCopter -f quad --no-mavproxy --no-rebuild -I0
```

**Verifikasi (terminal lain):**
```bash
ss -ltn | grep 5760
```
**Output diharapkan:** `LISTEN ... 0.0.0.0:5760` — SITL siap menerima koneksi MAVLink.
Jangan lanjut kalau port belum listen (tunggu beberapa detik, build SITL besar).

## 5.3 Langkah 2: MAVProxy bridge

Koneksi mavros langsung ke SITL (`tcp:5760`) **tidak stabil** ("No satellites",
stream mati) — MAVProxy WAJIB sebagai jembatan (AGENTS.md, sudah terbukti).

**Terminal 2:**
```bash
mavproxy.py --master tcp:127.0.0.1:5760 --out tcpin:0.0.0.0:14550 --streamrate 10 --daemon
```

**Verifikasi:**
```bash
ss -ltn | grep 14550
```
**Output diharapkan:** `LISTEN ... 0.0.0.0:14550`. Log MAVProxy juga akan
menunjukkan:
```
online system 1
AP: ArduPilot Ready
AP: AHRS: EKF3 active
```

## 5.4 Langkah 3: roscore

**Terminal 3:**
```bash
source ~/catkin_ws/devel/setup.bash
roscore
```

**Verifikasi (terminal lain):**
```bash
source ~/catkin_ws/devel/setup.bash
rostopic list
```
**Output diharapkan:** minimal `/rosout` dan `/rosout_agg` muncul (tanpa error koneksi).

## 5.5 Langkah 4: mavros (connect ke MAVProxy, BUKAN langsung ke SITL)

**Terminal 4:**
```bash
source ~/catkin_ws/devel/setup.bash
roslaunch mavros apm.launch fcu_url:="tcp://127.0.0.1:14550"
```

**Verifikasi:**
```bash
source ~/catkin_ws/devel/setup.bash
rostopic echo -n1 /mavros/state
```
**Output diharapkan:**
```
connected: True
armed: False
mode: "STABILIZE"
```
Kalau `connected: False` → cek Terminal 2 (MAVProxy) masih hidup & port 14550 benar.

## 5.6 Langkah 5: set_origin_home (one-shot)

Meskipun SITL default sudah punya GPS sim (beda dari lapangan non-GPS asli
di Bagian A), `set_origin_home.launch` tetap dijalankan di sini — konsisten
dengan resep SITL yang sudah dipakai project ini sebelumnya (lihat
`WP_HEADING_ALIGN_NAVIGASI.md` §6), dan tidak mengganggu apapun kalau origin
GPS sudah ada.

**Terminal lain (one-shot, akan exit sendiri):**
```bash
source ~/catkin_ws/devel/setup.bash
roslaunch mission_control set_origin_home.launch
```

**Output diharapkan (lalu proses exit sendiri):**
```
[INFO] [set_origin_home] origin dikirim: -7.050111, 110.391339, 700.0
[INFO] [set_origin_home] home OK (result=0).
[set_origin_home-1] process has finished cleanly
```

## 5.7 Langkah 6: Jalankan Misi Seleksi Wilayah (SITL, marker di-mock)

Kamera fisik tidak ada di SITL — `sim_markers:=true` memakai `sim_markers.py`
sebagai pengganti deteksi ArUco/WP nyata. **Gate double/triple TIDAK ada
mock** (lihat Troubleshooting §5.9 #2) — traverse akan selalu jatuh ke
fallback "terbang lurus geometris" setelah timeout ~20 detik, ini **normal
untuk SITL**.

**Terminal lain:**
```bash
source ~/catkin_ws/devel/setup.bash
roslaunch mission_control phaseD_mission.launch \
  mission:=seleksi do_takeoff:=true sim_markers:=true \
  use_gate:=true use_aruco:=false use_wp_marker:=false
```

**Output diharapkan (urut, ~90 detik total), sudah terbukti persis ini:**
```
[D] GUIDED + arm + takeoff 2.0m
[D] takeoff selesai (z=1.76)
[D] === leg 1/6: wp1 (scan) ===
[D]  sampai wp1
[D] SCAN id=1 (hover)
[D]  id=1 TERKONFIRMASI
[D] === leg 2/6: gate_double (traverse) ===
[D]  sampai gate_double
[D] TRAVERSE gate (1 lapis, through 1.5m)
[D]  lapis 1/1: center bukaan
[WARN] [D]  gate TAK TERLIHAT -> through geometris WP        <- normal, lihat Troubleshooting #2
[D]  through -> (9.5,0.0,1.0)
[D]  sampai gate_through
[D] === leg 3/6: wp2 (drop) ===
[D]  sampai wp2
[D] ALIGN id=2 lalu DROP
[D]  terpusat (off<0.08) -> DROP
[D]  servo ch9 -> 1013 (buka/drop) success=True
[D] === leg 4/6: wp2 (yaw) ===
[D]  sampai wp2
[D] YAW left 90 deg (from -0.0 -> 90.0 deg)
[D]  yaw OK (err=1.1 deg)
[D] === leg 5/6: gate_triple (traverse) ===
[D]  sampai gate_triple
[D] TRAVERSE gate (1 lapis, through 1.5m)
[WARN] [D]  gate TAK TERLIHAT -> through geometris WP        <- normal
[D]  through -> (16.5,0.0,2.0)
[D]  sampai gate_through
[D] === leg 6/6: wp3 (land) ===
[D]  sampai wp3
[D] LAND
[D]  mendarat (armed=False)
[D] === MISI SELESAI ===
```

**Verifikasi sukses:** baris terakhir `=== MISI SELESAI ===` muncul, DAN
```bash
rostopic echo -n1 /mavros/state
```
menunjukkan `armed: False`.

### Variasi: dengan heading-align (Task 4) aktif

```bash
roslaunch mission_control phaseD_mission.launch \
  mission:=seleksi do_takeoff:=true sim_markers:=true \
  use_gate:=true use_aruco:=false use_wp_marker:=false \
  enable_heading_align:=true sim_tail_side:=E
```
**Verifikasi param benar-benar terpasang** (roslaunch kadang diam-diam
mengabaikan arg CLI, lihat Troubleshooting §5.9 #4):
```bash
rosparam get /mission_d/enable_heading_align   # harus: true
rosparam get /sim_markers/sim_tail_side        # harus: E
```
**Sudah teruji:** misi tetap selesai (`MISI SELESAI`) sama seperti tanpa
heading-align. Heading-align sendiri **kemungkinan besar akan ter-SKIP**
dengan log `heading align: WpMarkerResult wp_id=X != expected=Y -> skip`
— ini **normal**, lihat Troubleshooting §5.9 #5, BUKAN kegagalan.

## 5.8 Langkah 7: Matikan semua proses (bersih)

Jangan pakai `pkill` (lihat Troubleshooting §5.9 #3) — matikan satu per satu
dari `pgrep`:
```bash
pgrep -af "sim_vehicle|arducopter|mavproxy|rosmaster|roscore|mavros_node|mission_d.py|sim_markers.py|gate_node.py"
kill <PID1> <PID2> ...
```
**Verifikasi:**
```bash
pgrep -af "sim_vehicle|arducopter|mavproxy|rosmaster|roscore|mavros_node" || echo "bersih"
ss -ltn | grep -E ":5760|:14550|:11311" || echo "tidak ada listener tersisa"
```

## 5.9 Troubleshooting — HANYA yang benar-benar ditemukan saat validasi

### #1 — `_goto` timeout 60s ke waypoint `gate_double`
**Gejala:** `[D] ABORT: goto gate_double timeout 60s` → misi auto-LAND di leg 2.
**Root cause:** `waypoints.yaml` untuk `gate_double` masih **placeholder belum
disurvei** (`x=8.0, y=0.0, z=0.0`) — `z=0.0` = level tanah/origin EKF, target
altitude ini tidak pernah terpenuhi `reach_tol_z=0.3` selagi drone terbang.
**Solusi yang terbukti bekerja:** ubah `z` ke ketinggian terbang yang konsisten
dengan WP lain (`z=1.0`, sama seperti wp1/wp2/wp3). **PENTING:** ini baru
solusi **SITL-only** — koordinat asli (`x`, `y`) tetap placeholder belum
disurvei fisik, BUKAN siap pakai untuk lomba. Ganti dengan hasil
`record_waypoint.py` sebelum terbang sungguhan (lihat §5.10).
Backup sebelum edit: `config/waypoints.yaml.backup_20260731_pre_sitl_gate_z_fix`.

### #2 — Traverse gate selalu fallback "TAK TERLIHAT"
**Gejala:** tiap leg `traverse` menunggu ~20 detik (`gate_timeout`) lalu log
`gate TAK TERLIHAT -> through geometris WP`.
**Root cause:** `gate_node.py` subscribe ke `/camera_front/image_raw` (topik
kamera fisik) — **tidak ada mock/simulator untuk gate** di codebase ini
(`sim_markers.py` hanya mem-publish ArUco/WP marker, tidak menyentuh topic
gate). Ini **bukan bug**, memang belum ada yang membuatnya.
**Dampak:** jalur *visual-align* saat traverse **tidak tervalidasi** di SITL
— hanya jalur fallback timeout yang teruji. Fallback-nya sendiri bekerja
benar (misi tetap lanjut & selesai).
**Bukan solusi, tapi mitigasi kalau butuh tes traverse visual:** perlu buat
mock gate terpisah (di luar scope validasi ini, butuh instruksi eksplisit).

### #3 — `pkill -f "<pola>"` gagal (exit code 144, tanpa output)
**Gejala:** perintah `pkill -f nama_proses` langsung mati tanpa output apapun,
proses target tidak ikut mati.
**Ditemukan saat:** mematikan proses SITL/ROS lama sebelum retry misi, dan
saat shutdown akhir (Langkah 7).
**Solusi yang terbukti bekerja:** pakai `kill <PID>` dengan PID eksplisit dari
`pgrep -af`, bukan `pkill` dengan pola. (Kemungkinan pola `pkill` match ke
proses lain di luar target yang diblok kebijakan lingkungan — belum
ditelusuri lebih jauh karena di luar scope validasi misi.)

### #4 — roslaunch arg CLI diam-diam diabaikan
**Gejala (ditemukan di sesi Task 4 sebelumnya, relevan untuk Langkah 6 di atas):**
override `arg:=value` di CLI tidak berpengaruh kalau launch file belum punya
`<arg name="...">` yang sesuai — roslaunch tidak error, cuma diam-diam pakai
default.
**Solusi:** `phaseD_mission.launch` **sudah** punya `<arg name="enable_heading_align">`
dan `<arg name="sim_tail_side">` (diperbaiki sesi sebelumnya) — jadi Langkah 6
variasi heading-align di atas sudah aman dipakai apa adanya. **Selalu
verifikasi** dengan `rosparam get` (bukan cuma percaya command jalan tanpa
error), seperti dicontohkan di §5.7.

### #5 — Heading-align ter-skip di misi penuh (wp_id mismatch)
**Gejala:** `heading align: WpMarkerResult wp_id=3 != expected=1 -> skip`
(atau kombinasi id lain).
**Root cause:** `sim_markers.py` `~view_radius` default 3.0m membuat semua WP
(WP1/2/3 saling berdekatan ~1.2-2.6m di `waypoints.yaml`) "terlihat" sekaligus
oleh mock — `WpMarkerResult` termutakhirkan oleh WP lain sebelum
`_align_heading_to_tail()` sempat membaca punya WP yang baru selesai
di-scan/drop. **Ini bukan bug** — guard `wp_id` memang dirancang menolak data
yang tidak cocok (lihat `WP_HEADING_ALIGN_NAVIGASI.md` §2.4). **Perilaku
non-blocking terbukti benar**: misi tetap `MISI SELESAI` walau heading-align
sering skip.

## 5.10 Perbedaan SITL vs Lapangan Asli — apa yang TIDAK divalidasi di §5

| Aspek | Status di SITL (§5) | Status lapangan asli (Bagian A) |
|---|---|---|
| Posisi/navigasi | GPS simulasi ArduCopter (EKF default) | T265 VIO → EKF3 non-GPS (§1-4) — **jalur berbeda total**, SITL §5 TIDAK menguji T265/EKF3/vision_to_mavros sama sekali |
| Deteksi ArUco/WP marker | `sim_markers.py` (mock sempurna, off_x=off_y=0, selalu terdeteksi) | Kamera bawah asli + `aruco_node`/`wp_marker_node` — akurasi WP1 masih **belum reliable** (`WP_MARKER_DECODER.md`: agreement 0.671), verifikasi fisik pola marker WP1/WP3/WP4 **belum dilakukan** |
| Deteksi gate | **Tidak ada mock** (Troubleshooting #2) — traverse SELALU pakai fallback geometris di SITL | Kamera depan asli + `gate_node.py` (single gate, dipakai Misi Seleksi) — deteksi geometris `gate_multi_detector.py` (Task 1, untuk gate fisik double/triple asli) sudah 69.2% akurasi tapi **belum diintegrasikan** ke `phaseD_mission.launch` (`GATE_DETECTOR_DOUBLE_TRIPLE.md` §6) |
| Kalibrasi mounting kamera bawah (align lateral, heading tail) | Tidak relevan — offset selalu 0 dari mock | `align_sign_x/y`, `~tail_dir_rotate_steps`, `~tail_mirror` **belum dikalibrasi fisik** — walk-test-style di lapangan wajib sebelum lomba |
| Waypoint koordinat | `wp1/wp2/wp3` dari survei (tampak nyata), `gate_double/gate_triple` **placeholder diedit SITL-only** (Troubleshooting #1) | **WAJIB** disurvei ulang dengan `record_waypoint.py` di titik takeoff lapangan sesungguhnya — koordinat SITL TIDAK BOLEH dipakai langsung |
| Gate fisik double/triple | Belum ada bentuk fisiknya sama sekali (`mission_seleksi.yaml` komentar: "pakai 2 single gate sebagai pengganti") | Sama — ini bukan keterbatasan SITL, memang belum dibangun |
| Walk test (jejak T265 vs jejak nyata) | Tidak relevan (tidak pakai T265) | **Status tidak konsisten antar dokumen** — `PANDUAN_TERBANG_T265.md` §4e anggap ini prosedur baku, `HANDOFF.md` §11/§13 (per 2026-07-16) masih tandai "BELUM dikonfirmasi lulus". Cek status terkini sebelum asumsi. |

**Kesimpulan:** §5 memvalidasi **logika state machine misi** (`mission_d.py`:
urutan leg, guard kesehatan, servo drop, yaw, fallback traverse) — bukan
validasi hardware/vision/navigasi fisik (itu tugas Bagian A). Kedua jenis
validasi saling melengkapi, bukan saling menggantikan.

---

## 6. Referensi silang

- Command operasional terbang T265 asli (non-SITL): `PANDUAN_TERBANG_T265.md`
- Gotcha kritis kode & konvensi: `AGENTS.md`
- Status fitur & isu diketahui (arsitektur besar): `HANDOFF.md`
- Status detektor gate double/triple: `GATE_DETECTOR_DOUBLE_TRIPLE.md`
- Status decoder WP marker custom: `WP_MARKER_DECODER.md`
- Status fitur heading-align: `WP_HEADING_ALIGN_NAVIGASI.md`
- Urutan leg persis Misi Seleksi: `src/mission_control/config/mission_seleksi.yaml`
- Detail leg-by-leg misi (scan/drop/traverse/yaw), parameter tuning:
  komentar module-level di `src/mission_control/scripts/mission_d.py`

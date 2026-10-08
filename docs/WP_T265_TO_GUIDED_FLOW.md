# Alur Data & Tutorial SITL: T265 Init → Misi GUIDED ke Pixhawk

Pelengkap `PANDUAN_TERBANG_T265.md` (command operasional lapangan asli,
detail penuh) dan `AGENTS.md` (gotcha). Dokumen ini berisi TIGA bagian:

- **Bagian A (§1-5): arsitektur alur data + cookbook T265 fisik** — node apa
  hidup di tiap tahap, topic/service apa mengalir, urutan dependency wajib
  (§1-4), DAN (§5) command persis siap-jalan untuk menyalakan T265 s/d
  Loiter solid di drone fisik. Referensi detail penuh (wiring, param,
  troubleshooting): `PANDUAN_TERBANG_T265.md`.
- **Bagian B: tutorial SITL tervalidasi** — command persis + output yang
  terbukti muncul untuk menjalankan Misi Seleksi Wilayah (s/d WP3) di drone
  VIRTUAL (ArduCopter SITL, GPS simulasi — **bukan** T265/EKF3). Sudah
  dijalankan & berhasil **2× reproducible** (2026-07-31), bukan teoretis.
- **Bagian C: tutorial GUIDED misi penuh FISIK ASLI, end-to-end** — mengikat
  Bagian A (T265 fisik) + logika misi Bagian B (kombinasi node BARU
  `gate_multi_node`/`wp_marker_node`) jadi satu prosedur untuk terbang
  sungguhan. ⚠️ **Status: BELUM PERNAH diterbangkan sebagai satu misi utuh**
  — baca §C.0 sebelum mencoba.

Ketiga bagian saling melengkapi, bukan duplikat: Bagian A untuk alur data +
bring-up FISIK (T265→EKF3→GUIDED), Bagian B untuk memvalidasi LOGIKA state
machine misi (`mission_d.py`) di SITL, Bagian C untuk menjalankan keduanya
sekaligus di lapangan sungguhan.

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

## 4b. Tutorial Cookbook: Menyalakan T265 s/d Loiter Solid (Lapangan Fisik)

Versi ringkas-siap-pakai dari `PANDUAN_TERBANG_T265.md` — supaya dokumen INI
sendiri sudah "dari awal menyalakan T265 sampai akhir" tanpa harus lompat
antar file untuk urutan commandnya. **Untuk detail penuh** (diagram wiring
pin-per-pin, tabel parameter lengkap dgn arti tiap baris, tabel
troubleshooting) — itu tetap di `PANDUAN_TERBANG_T265.md`, dokumen ini tidak
menduplikasi semuanya.

⚠️ **Ini prosedur untuk drone FISIK ASLI — motor berputar, bisa terbang
sungguhan.** Beda total dari Bagian B (SITL, drone virtual, aman
eksperimen). Ikuti urutan APA ADANYA, jangan loncat tahap.

### 4b.1 Prasyarat (checklist, WAJIB semua tercentang)

```
[ ] Wiring Pixhawk TELEM2 <-> Jetson UART (ttyTHS1) sudah benar & terpasang
    (lihat PANDUAN_TERBANG_T265.md §1 utk diagram pin persis)
[ ] Parameter EKF3 non-GPS sudah di-Load ke FC + Write + REBOOT Pixhawk
    (params/t265_nongps_ekf3.param, lihat PANDUAN_TERBANG_T265.md §2)
[ ] T265 terpasang FRONT-facing, USB ke kanan, mount fisik LURUS atau
    pitch_cam sudah dikompensasi
[ ] Remote (RC) menyala, switch mode Stabilize/AltHold/Loiter terjangkau
[ ] Baterai drone + power Jetson terpisah, keduanya terisi
[ ] Area lapang, banyak tekstur visual di lantai/tanah (BUKAN polos/mengkilap)
[ ] Tidak ada orang di sekitar area uji
```

### 4b.2 Nyalakan software (3 terminal, urut)

Tiap terminal baru: `cd ~/catkin_ws && source devel/setup.bash` dulu.

**Terminal 1 — kamera T265:**
```bash
roslaunch realsense2_camera rs_t265.launch initial_reset:=true
```
Diamkan ~3 detik (tracking mulai bersih, guard NaN butuh lock awal).

**Terminal 2 — mavros (koneksi ke Pixhawk fisik, BUKAN SITL):**
```bash
roslaunch mavros apm.launch fcu_url:=/dev/ttyTHS1:921600
```
**Gate-check:** `rostopic echo -n1 /mavros/state` → `connected: True`. Gagal
→ cek wiring TX/RX (kemungkinan tertukar), `SERIAL2_BAUD`, atau
`BRD_SER2_RTSCTS` belum `0` (§2a PANDUAN).

**Terminal 3 — jembatan pose T265 → mavros (FRONT-facing):**
```bash
roslaunch vision_to_mavros t265_tf_to_mavros.launch enable_precland:=false
```
**Gate-check:** `rostopic hz /mavros/vision_pose/pose` → ~30Hz. Kosong →
Terminal 1 atau 3 bermasalah (cek `/camera/odom/sample` jalan dulu).

### 4b.3 Set origin + home (DI TITIK TAKEOFF, wajib tiap boot FC)

**Terminal 4:**
```bash
roslaunch mission_control set_origin_home.launch
```
**Gate-check:** pesan **"GPS Glitch"** lalu **"GPS Glitch cleared"** muncul
di Mission Planner, ikon drone muncul di peta. `rel_alt` ≈ 0 di tanah,
tidak ada "stopped aiding" berulang.

### 4b.4 Verifikasi darat WAJIB sebelum motor menyala

```bash
rostopic echo -n1 /mavros/state              # connected: True
rostopic hz /mavros/vision_pose/pose         # ~30 Hz
# Mission Planner: Ctrl+F -> Mavlink Inspector -> VISION_POSITION_ESTIMATE masuk
```

**Walk test (gerbang mutlak, JANGAN dilewati):** motor MATI, angkat/gendong
drone, jalan membentuk kotak ~2×2m, kembali ke titik awal. Lihat di Mission
Planner/rviz: lintasan harus **menyerupai bentuk & skala kotak nyata**, tanpa
drift/muter sendiri besar. Skala meleset atau muter sendiri → **JANGAN
lanjut ke §4b.5**, lihat `PANDUAN_TERBANG_T265.md` §7.

⚠️ **Status walk test belum konsisten antar dokumen milik proyek ini** —
`PANDUAN_TERBANG_T265.md` menempatkannya sebagai checklist rutin baku,
`HANDOFF.md` (per 2026-07-16) masih menandai belum ada satu run pun yang
tercatat lulus. **Perlakukan sebagai BELUM PERNAH lulus sampai Anda
sendiri melihatnya lulus di layar Mission Planner/rviz hari ini** — jangan
asumsikan sudah beres karena "sudah lama dikerjakan".

### 4b.5 Tes terbang bertahap (urut, jangan loncat — ini bukan opsional)

1. **Stabilize** — arm, throttle pelan, hover manual rendah (~0.5m), cek getaran/stabil.
2. **Alt-Hold** — hover, lepas throttle di tengah, ketinggian harus tertahan.
3. **Loiter (ujian utama T265)** — arm di Loiter (indikator EKF hijau di MP),
   naik pelan ke ~1m, **lepas semua stick**.
   - ✅ LULUS: drone diam menggantung (toleransi geser sedikit wajar).
   - ❌ GAGAL: drone kabur/menjauh sendiri → **segera switch Stabilize**,
     mendarat. JANGAN coba lagi tanpa diagnosis (`HANDOFF.md` §9: "Crash
     Loiter" — takeoff langsung Loiter dari darat pernah menyebabkan
     flyaway karena estimasi vision belum sehat).

➡️ **Ulangi Loiter sampai benar-benar stabil & berulang** (bukan sekali
kebetulan) sebelum mempertimbangkan Bagian C (GUIDED autonomous). Ini aturan
emas dari `PANDUAN_TERBANG_T265.md` — Bagian C TIDAK BOLEH dicoba sebelum
tahap ini solid.

---

# BAGIAN B — Tutorial SITL Tervalidasi: Misi Seleksi Wilayah (WP1→WP3)

Gaya cookbook — command persis, output yang diharapkan, cara verifikasi
sebelum lanjut ke langkah berikutnya. Target pembaca: belum pernah
menjalankan sistem ini sama sekali.

**Ini SITL** (drone virtual, ArduCopter default GPS-sim — **TIDAK** memakai
T265/EKF3 sama sekali, beda jalur dari Bagian A). Untuk validasi *logika
misi* sebelum ke lapangan. Semua langkah di bawah **sudah dijalankan dan
terbukti berhasil** (2026-07-31), bukan teoretis.

**Dua kombinasi node sudah tervalidasi terpisah** (sama tanggal, sesi
berbeda):
- **Kombinasi LAMA** (§5.7 command utama): `gate_node.py` (single gate) +
  `sim_markers.py` (mock ArUco/WP sempurna) — divalidasi sesi pertama.
- **Kombinasi BARU** (§5.7b, Task 1-3): `gate_multi_node.py` +
  `wp_marker_node.py` (node asli, TANPA mock) — divalidasi sesi kedua
  (23:xx), lihat "Verified via SITL run 2026-07-31 (sesi 2)" di §5.11.
  Ini kombinasi yang dipakai di lapangan sesungguhnya (Task 1-4 sudah
  selesai coding) — §5.7 (mock) tetap berguna untuk validasi cepat state
  machine tanpa peduli node detector mana yang dipakai.

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

## 5.7b Variasi tervalidasi: node BARU (`gate_multi_node` + `wp_marker_node`, Task 1-3)

Kombinasi ini menggantikan `gate_node.py`/`sim_markers.py` dengan node asli
yang dipakai di lapangan. **JANGAN nyalakan `gate_multi_detect.launch` atau
`wp_marker_detect.launch` secara TERPISAH sebelum `phaseD_mission.launch`**
— keduanya tidak punya arg `sim_markers` (silently diabaikan roslaunch kalau
dipaksa, lihat Troubleshooting #7), dan kalau sudah jalan standalone lalu
`phaseD_mission.launch` mencoba `<include>` node yang sama lagi, terjadi
tabrakan nama node. Nyalakan HANYA lewat flag `use_gate_multi`/
`use_wp_marker` di `phaseD_mission.launch` — node detector jadi bagian dari
satu proses `roslaunch` yang sama.

**Command persis yang terbukti jalan:**
```bash
source ~/catkin_ws/devel/setup.bash
roslaunch mission_control phaseD_mission.launch mission:=seleksi \
  sim_markers:=false use_gate_multi:=true use_gate:=false use_wp_marker:=true \
  enable_heading_align:=true \
  waypoints_file:=$(rospack find mission_control)/config/waypoints_sitl_test.yaml
```

**Flag WAJIB yang TIDAK ada di command §5.7 (mock) — kalau lupa salah satu, gagal:**
- `use_gate_multi:=true use_gate:=false` — dipasangkan eksplisit (default
  `use_gate` adalah `true`; kalau tidak di-`false`-kan, dua node gate jalan
  bersamaan, boros & membingungkan meski tak fatal).
- `use_wp_marker:=true` — sudah ada di §5.7 tapi WAJIB dipasangkan dengan
  `sim_markers:=false` (lihat poin berikut).
- **`sim_markers:=false`** — KEBALIKAN dari §5.7 (`sim_markers:=true`).
  Kalau `sim_markers:=true` dan `use_wp_marker:=true` dinyalakan bersamaan,
  `sim_markers.py` (mock) dan `wp_marker_node.py` (asli) **publish ke topic
  yang sama** `/wp_marker_node/result` → data tercampur acak. Lihat
  Troubleshooting #6.
- `waypoints_file:=.../waypoints_sitl_test.yaml` — WAJIB, override khusus
  SITL. `waypoints.yaml` asli **masih** punya `gate_double.z: 0.0` (dicek
  ulang di run ini, backup lama TIDAK pernah benar-benar diterapkan ke file
  aktif — lihat koreksi di Troubleshooting #1) — tanpa override ini, misi
  ABORT→LAND di leg 2 (`traverse gate_double`).

**Gate-check sebelum percaya command jalan benar** (bukan asumsi):
```bash
rosparam get /mission_d/gate_topic      # harus: /gate_multi_node/gate
rosparam get /mission_d/markers_topic   # harus: /wp_marker_node/markers
rostopic info /gate_multi_node/gate     # Publishers: /gate_multi_node
rostopic info /wp_marker_node/markers   # Publishers: /wp_marker_node
```
Catatan: kalau dicek SETELAH misi selesai, `mission_d` sudah exit (node
one-shot, bukan resident) — `rosparam get` tetap berhasil karena param
server tidak ikut hilang, tapi `rosnode info mission_d` sudah kosong. Itu
normal, bukan tanda gagal.

**Output nyata (urut, ~110 detik total, 2026-07-31):**
```
[D] GUIDED + arm + takeoff 2.0m
[D] takeoff selesai (z=1.75)
[D] === leg 1/6: wp1 (scan) ===
[D]  sampai wp1
[D] SCAN id=1 (hover)
[WARN] [D]  id=1 TIDAK terlihat (timeout) -> lanjut
[D] === leg 2/6: gate_double (traverse) ===
[D]  sampai gate_double
[D] TRAVERSE gate (1 lapis, through 1.5m)
[D]  lapis 1/1: center bukaan
[WARN] [D]  gate TAK TERLIHAT -> through geometris WP
[D]  through -> (9.5,0.0,1.0)
[D]  sampai gate_through
[D] === leg 3/6: wp2 (drop) ===
[D]  sampai wp2
[D] ALIGN id=2 lalu DROP
[WARN] [D]  align timeout -> drop apa adanya
[D]  servo ch9 -> 1013 (buka/drop) success=True
[D] === leg 4/6: wp2 (yaw) ===
[D]  sampai wp2
[D] YAW left 90 deg (from -0.0 -> 90.0 deg)
[D]  yaw OK (err=2.2 deg)
[WARN] [D]  heading align: belum ada WpMarkerResult -> skip
[D] === leg 5/6: gate_triple (traverse) ===
[D]  sampai gate_triple
[D] TRAVERSE gate (1 lapis, through 1.5m)
[WARN] [D]  gate TAK TERLIHAT -> through geometris WP
[D]  through -> (16.5,0.0,2.0)
[D]  sampai gate_through
[D] === leg 6/6: wp3 (land) ===
[D]  sampai wp3
[D] LAND
[D]  mendarat (armed=False)
[D] === MISI SELESAI ===
```
**Beda penting dari §5.7 (mock):** `SCAN id=1` dan `ALIGN id=2` di sini
**timeout** (`TIDAK terlihat` / `align timeout`) alih-alih `TERKONFIRMASI`/
`terpusat`, karena `wp_marker_node.py` asli butuh gambar kamera sungguhan
(`/camera_down/image_raw`) yang **tidak ada publisher-nya sama sekali** di
SITL murni ini (tidak ada Gazebo/`iq_sim`) — bukan kegagalan node, memang
tidak ada yang dikirim untuk dideteksi. Fallback graceful tetap jalan sama
persis seperti §5.7, misi tetap `MISI SELESAI`. Ini konsisten dengan §5.10:
validasi run ini adalah **jalur komunikasi & state machine**, BUKAN akurasi
deteksi visual — sama seperti §5.7, keterbatasan itu berlaku juga untuk node
BARU, bukan cuma node lama.

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
**Solusi yang terbukti bekerja — DIKOREKSI (dicek ulang 2026-07-31 sesi 2):**
paragraf lama di sini menyarankan edit langsung `z` di `waypoints.yaml`
asli ke `1.0`. **Itu TIDAK pernah benar-benar diterapkan** — `waypoints.yaml`
aktif masih `gate_double.z: 0.0` sampai saat ini (diverifikasi ulang, dan
file backup `.backup_20260731_pre_sitl_gate_z_fix` isinya identik dengan
file aktif, bukan versi sebelum-fix). Solusi yang SUNGGUH dipakai dan
terbukti jalan: **jangan sentuh `waypoints.yaml` asli sama sekali** (itu
data survei lapangan) — pakai file terpisah `config/waypoints_sitl_test.yaml`
(sudah ada, `gate_double.z: 1.0`) via arg `waypoints_file:=` saat
`roslaunch phaseD_mission.launch` (lihat §5.7b). **PENTING:** koordinat
`x`/`y` di file test ini tetap placeholder belum disurvei fisik — file ini
KHUSUS SITL, ganti dengan hasil `record_waypoint.py` sebelum terbang
sungguhan (lihat §5.10).

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

### #6 — Tabrakan publisher `/wp_marker_node/result` kalau `sim_markers:=true` + `use_wp_marker:=true`
**Gejala (potensial, dihindari dgn flag benar di §5.7b, dicatat sbg peringatan):**
`sim_markers.py` (mock, aktif tiap `sim_markers` default `true`) publish
`WpMarkerResult` mock ke `/wp_marker_node/result` — topic **default yang
SAMA PERSIS** dengan output asli `wp_marker_node.py`. Kalau `use_wp_marker:=true`
dinyalakan tanpa eksplisit `sim_markers:=false`, dua publisher rebutan satu
topic, `mission_d` (dan heading-align Task 4) menerima campuran data mock &
asli secara acak — tak error, tapi hasil tak bisa dipercaya.
**Solusi:** SELALU `sim_markers:=false` setiap kali `use_wp_marker:=true`.
Ini SATU-SATUNYA cara aman menjalankan node BARU (§5.7b) — jangan pernah
gabungkan `sim_markers:=true` dengan `use_wp_marker:=true`.

### #7 — `gate_multi_detect.launch`/`wp_marker_detect.launch` tidak punya arg `sim_markers`
**Gejala:** `roslaunch mission_control gate_multi_detect.launch sim_markers:=true`
(atau `wp_marker_detect.launch` serupa) jalan tanpa error, topic
`/gate_multi_node/gate`/`/wp_marker_node/result` muncul di `rostopic list`,
TAPI `rostopic hz` menunjukkan **nol pesan** terus-menerus.
**Root cause:** dua hal sekaligus — (1) kedua launch file itu HANYA punya
arg `image_topic` dan `publish_debug`, `sim_markers:=...` di CLI silently
diabaikan (pola sama dgn Troubleshooting #4); (2) SITL murni ini tidak
punya Gazebo/`iq_sim`, jadi `/camera_front/image_raw` dan
`/camera_down/image_raw` memang **tidak ada publisher-nya sama sekali** —
node detector menunggu gambar yang tak pernah datang.
**Solusi:** jangan nyalakan kedua launch file ini standalone untuk uji SITL.
Nyalakan lewat `phaseD_mission.launch` dengan `use_gate_multi:=true`/
`use_wp_marker:=true` (lihat §5.7b) — perilakunya tetap sama (tak ada
gambar → gate/marker tak pernah "terlihat" → fallback graceful), tapi
setidaknya tak ada risiko tabrakan nama node kalau nanti `phaseD_mission.launch`
juga mencoba `<include>` node yang sama.

## 5.10 Perbedaan SITL vs Lapangan Asli — apa yang TIDAK divalidasi di §5

| Aspek | Status di SITL (§5) | Status lapangan asli (Bagian A) |
|---|---|---|
| Posisi/navigasi | GPS simulasi ArduCopter (EKF default) | T265 VIO → EKF3 non-GPS (§1-4) — **jalur berbeda total**, SITL §5 TIDAK menguji T265/EKF3/vision_to_mavros sama sekali |
| Deteksi ArUco/WP marker | `sim_markers.py` (mock sempurna, off_x=off_y=0, selalu terdeteksi) | Kamera bawah asli (`camera_down.launch`, C920 `/dev/video1`) + `aruco_node`/`wp_marker_node` — akurasi WP1 masih **belum reliable** (`WP_MARKER_DECODER.md`: agreement 0.671), verifikasi fisik pola marker WP1/WP3/WP4 **belum dilakukan**. Heading-align (Task 4, `tail_side`): pemetaan arah `~tail_dir_rotate_steps`/`~tail_mirror` masih **asumsi, belum diverifikasi fisik** (`WP_HEADING_ALIGN_NAVIGASI.md` §4/§9) — JANGAN pakai `enable_heading_align:=true` di lapangan sebelum kalibrasi ini selesai. |
| Deteksi gate | **Tidak ada mock** (Troubleshooting #2) — traverse SELALU pakai fallback geometris di SITL | Kamera depan asli (`camera_front.launch`, C310 `/dev/video0`) + `gate_multi_node.py` (Task 1-2, gate ganda/tiga, **SUDAH terintegrasi** ke `phaseD_mission.launch` via `use_gate_multi:=true use_gate:=false`, lihat §5.7b/Bagian C) — akurasi deteksi geometris 69.2% (`GATE_DETECTOR_DOUBLE_TRIPLE.md`), **BELUM pernah diuji dengan gate fisik sungguhan/footage terbang** (§6 dokumen itu, per 2026-07-30/31) — koreksi dari versi tabel ini sebelumnya yang menyebut "belum diintegrasikan"; integrasi ROS-nya sudah selesai, yang belum adalah validasi visual di gate fisik |
| Kalibrasi mounting kamera bawah (align lateral, heading tail) | Tidak relevan — offset selalu 0 dari mock | `align_sign_x/y`, `~tail_dir_rotate_steps`, `~tail_mirror` **belum dikalibrasi fisik** — walk-test-style di lapangan wajib sebelum lomba |
| Waypoint koordinat | `wp1/wp2/wp3` dari survei (tampak nyata), `gate_double/gate_triple` **placeholder diedit SITL-only** (Troubleshooting #1) | **WAJIB** disurvei ulang dengan `record_waypoint.py` di titik takeoff lapangan sesungguhnya — koordinat SITL TIDAK BOLEH dipakai langsung |
| Gate fisik double/triple | Belum ada bentuk fisiknya sama sekali (`mission_seleksi.yaml` komentar: "pakai 2 single gate sebagai pengganti") | Sama — ini bukan keterbatasan SITL, memang belum dibangun |
| Walk test (jejak T265 vs jejak nyata) | Tidak relevan (tidak pakai T265) | **Status tidak konsisten antar dokumen** — `PANDUAN_TERBANG_T265.md` §4e anggap ini prosedur baku, `HANDOFF.md` §11/§13 (per 2026-07-16) masih tandai "BELUM dikonfirmasi lulus". Cek status terkini sebelum asumsi. |

**Kesimpulan:** §5 memvalidasi **logika state machine misi** (`mission_d.py`:
urutan leg, guard kesehatan, servo drop, yaw, fallback traverse) — bukan
validasi hardware/vision/navigasi fisik (itu tugas Bagian A). Kedua jenis
validasi saling melengkapi, bukan saling menggantikan.

## 5.11 Verified via SITL run 2026-07-31 (sesi 2)

Sesi ini menjalankan LANGKAH 1-7 penuh (§5.2-§5.8) dengan tujuan spesifik:
memvalidasi kombinasi node BARU (`gate_multi_node` + `wp_marker_node`,
§5.7b) end-to-end untuk PERTAMA KALI — sebelumnya §5.7 (kombinasi lama)
sudah tervalidasi tapi §5.7b belum pernah dijalankan sebagai satu misi utuh.

**Hasil: BERHASIL, `=== MISI SELESAI ===` dengan `armed: False`.** Semua
gate-check §5.2-§5.6 lulus persis seperti dokumentasi (tanpa deviasi).
Deviasi baru yang ditemukan & sudah didokumentasikan di atas:
- §5.7b (subsection baru) — command, flag wajib, dan output nyata kombinasi
  node BARU.
- Troubleshooting #6 (baru) — tabrakan publisher `sim_markers`+`use_wp_marker`.
- Troubleshooting #7 (baru) — `gate_multi_detect.launch`/`wp_marker_detect.launch`
  tak boleh distandalone-kan, dan tak punya arg `sim_markers`.
- Troubleshooting #1 — dikoreksi: solusi lama (edit `waypoints.yaml` asli)
  TERNYATA tak pernah benar-benar diterapkan; solusi sungguhan adalah file
  override terpisah.

**Reproducibility — diverifikasi ulang (run kedua, sama tanggal, ~1 jam
setelah run pertama):** seluruh LANGKAH 1-7 (§5.2-§5.8) diulang dari nol
(fresh restart semua proses, bukan lanjutan run pertama) memakai command
persis §5.7b tanpa perubahan apapun. Hasil: **identik** —
`=== MISI SELESAI ===`, `armed: False`, semua 6 leg lewat dengan pola
warning/timeout yang sama persis (hanya beda angka kecil run-to-run:
takeoff z=1.70 vs 1.75, yaw error 2.9° vs 2.2° — variasi timing normal,
BUKAN tanda ketidakstabilan). `rosparam get /mission_d/gate_topic` &
`markers_topic` dicek ulang, hasil sama. Tidak ada deviasi apapun dari
command/output yang tertulis di §5.7b — command di dokumen ini AMAN diikuti
apa adanya.

**Known issue (belum terselesaikan, bukan kegagalan run ini — batasan SITL
murni yang sudah berlaku sejak §5.7 lama):** tidak ada Gazebo/kamera di
lingkungan ini, jadi WP1 scan, WP2 align, heading-align, dan traverse gate
SEMUA gracefully timeout/fallback, tidak pernah benar-benar "melihat"
marker/gate — berlaku sama untuk node lama maupun node BARU. Validasi
akurasi deteksi visual `gate_multi_node`/`wp_marker_node` yang sesungguhnya
sudah dilakukan terpisah (gambar statis, bukan live SITL) di sesi Task 1/3
— lihat `GATE_DETECTOR_DOUBLE_TRIPLE.md`/`WP_MARKER_DECODER.md`.

---

# BAGIAN C — Tutorial GUIDED Misi Penuh Fisik Asli, End-to-End

> ⚠️ **Belum pernah diterbangkan sebagai satu misi utuh.** Tiap komponen
> teruji terpisah (arsitektur A, logika SITL B, akurasi detektor Task 1/3)
> — rantai penuhnya di fisik belum. Ikuti sebagai prosedur, bukan "sudah
> terbukti aman".

## C.0 Checklist Prasyarat (WAJIB semua tercentang)

```
[ ] Bagian A §4b (T265 s/d Loiter) sudah dijalankan HARI INI, Loiter solid berulang
[ ] Walk test lulus HARI INI, dilihat sendiri (bukan asumsi lama)
[ ] Servo drop (ch9) sudah dites darat: DO_SET_SERVO 1013 lalu 2015, mekanisme jalan
[ ] use_gate_multi/use_wp_marker sudah dites SITL (Bagian B §5.7b) di mesin ini
[ ] waypoints.yaml SUDAH disurvei ulang (§C.1) — bukan placeholder
[ ] Siap amati ALIGN pertama dari jarak aman (kalibrasi §C.3 belum tentu pas)
[ ] Safety pilot kedua siaga, remote di tangan, jari di switch Stabilize
```
Belum tercentang semua → **jangan lanjut**. Risiko tiap item: WP1 decode
belum reliable (0.671), gate detector 69.2% akurasi belum diuji fisik,
scale error T265 outdoor 20-30% belum dikoreksi, kalibrasi mounting kamera
belum diverifikasi — semua bisa bikin LAND lebih awal dari WP3, itu
perilaku AMAN by design, bukan bug.

## C.1 Survei waypoint (WAJIB, ganti placeholder)

`waypoints.yaml` sekarang placeholder, termasuk `gate_double.z: 0.0` (bug
terbukti ABORT di SITL, §5.9 #1). Bring-up dulu (§4b.2-§4b.3) di titik
takeoff, lalu gendong drone (motor mati) ke tiap titik:
```bash
rosrun mission_control record_waypoint.py _name:=wp1
rosrun mission_control record_waypoint.py _name:=gate_double
rosrun mission_control record_waypoint.py _name:=wp2
rosrun mission_control record_waypoint.py _name:=gate_triple
rosrun mission_control record_waypoint.py _name:=wp3
```
Satu sesi, jangan cabut daya (origin geser saat reboot). Betulkan `z` tiap
WP manual ke ketinggian terbang (rekam darat kasih `z≈0`) — **`gate_double.z`
JANGAN 0.0**. Backup file dulu (copy manual, bukan git).

## C.2 Bring-up kamera misi

Setelah §4b.2-§4b.3 (T265+mavros+pose+origin) lulus gate-check:

**Terminal 5 — kamera depan (gate) + bawah (marker):**
```bash
roslaunch mission_control cameras.launch
# device tertukar? v4l2-ctl --list-devices, lalu:
# roslaunch mission_control cameras.launch front_device:=/dev/videoX down_device:=/dev/videoY
```
**Gate-check:**
```bash
rostopic hz /camera_front/image_raw   # > 0
rostopic hz /camera_down/image_raw    # > 0
```
Jangan nyalakan `gate_multi_node`/`wp_marker_node` standalone — ikut lewat
command misi §C.4.

## C.3 Kalibrasi lapangan (kalau perlu)

`align_sign_x/y`, `align_swap_xy`, `tail_dir_rotate_steps`, `tail_mirror`
tak punya `<arg>` launch — set via `rosparam` sebelum start:
```bash
rosparam set /mission_d/align_sign_x -1.0        # ALIGN geser arah terbalik
rosparam set /mission_d/tail_dir_rotate_steps 1  # heading-align terputar 90 deg
```
Belum pernah dikalibrasi di rig ini → biarkan `enable_heading_align:=false`
(default), amati ALIGN pertama dari jarak aman.

## C.4 Jalankan misi

```bash
source ~/catkin_ws/devel/setup.bash
roslaunch mission_control phaseD_mission.launch mission:=seleksi \
  sim_markers:=false use_gate_multi:=true use_gate:=false use_wp_marker:=true
```
Beda dari SITL (§5.7b): TANPA `waypoints_file:=` override (pakai
`waypoints.yaml` hasil survei §C.1), TANPA `enable_heading_align` (default
`false` sampai §C.3 selesai).

**Gate-check sebelum arm:**
```bash
rosparam get /mission_d/gate_topic      # /gate_multi_node/gate
rosparam get /mission_d/markers_topic   # /wp_marker_node/markers
rostopic hz /gate_multi_node/gate       # > 0
rostopic hz /wp_marker_node/markers     # > 0
```

## C.5 Cara mulai aman

1. Uji GUIDED manual dulu (klik-peta MP jarak pendek dari Loiter) sebelum autonomous penuh.
2. Berdiri bisa lihat SELURUH lintasan (WP1→gate_double→WP2→gate_triple→WP3).
3. Safety pilot: jari di switch Stabilize sepanjang misi. Gejala aneh →
   flip Stabilize SEKETIKA, jangan tunggu timeout internal.
4. Ragu di leg manapun (mis. traverse menuju panel, bukan bukaan) → ambil
   alih manual.

## C.6 Ekspektasi per-leg

| Leg | Normal | Juga normal (bukan gagal) |
|---|---|---|
| SCAN wp1 | `TERKONFIRMASI` | timeout `TIDAK terlihat` (WP1 belum reliable) → lanjut |
| TRAVERSE gate | align visual sukses | fallback geometris (gate tak terdeteksi) |
| ALIGN+DROP | bergerak MENDEKATI marker | bergerak MENJAUH = `align_sign_x/y` salah → **HENTIKAN** |
| Heading-align | skip terus (default off) | — |
| LAND | di WP3 | lebih awal (ABORT/timeout) → cek log dulu |

## C.7 Setelah run pertama

Perlakukan sebagai pengumpulan data kalibrasi, bukan "selesai" — catat
hasil tiap leg, sesuaikan `align_sign_x/y`, `tail_dir_rotate_steps`/
`tail_mirror`, atau `waypoints.yaml`, lalu revisi dokumen ini (semangat §5.11).

## C.8 Troubleshooting fisik

| Gejala | Solusi |
|---|---|
| `/camera_front` atau `/camera_down` kosong | Device salah — `v4l2-ctl --list-devices`, override di §C.2 |
| ALIGN menjauh dari marker | `align_sign_x/y` terbalik (§C.3) — jangan lanjut DROP sampai benar |
| Heading-align putar arah salah | `tail_dir_rotate_steps`/`tail_mirror` belum sesuai — matikan `enable_heading_align` |
| Align menuju panel oranye, bukan bukaan | Bug lama (bidik bukaan, bukan centroid oranye) — regresi, laporkan |
| Lainnya | `PANDUAN_TERBANG_T265.md` §7, `HANDOFF.md` §10 |

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
- Kamera misi fisik (depan/bawah, USB): `src/mission_control/launch/cameras.launch`
  (+ `camera_front.launch`/`camera_down.launch` individual), dipakai Bagian C §C.2
- Rekam waypoint survei lapangan: `rosrun mission_control record_waypoint.py`,
  lihat Bagian C §C.1 dan `PANDUAN_TERBANG_T265.md` §6c

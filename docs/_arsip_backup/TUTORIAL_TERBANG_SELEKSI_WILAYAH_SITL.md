# Tutorial: Misi Seleksi Wilayah di SITL (dari Nol sampai LAND di WP3)

Gaya cookbook — command persis, output yang diharapkan, cara verifikasi
sebelum lanjut ke langkah berikutnya. Target pembaca: belum pernah
menjalankan sistem ini sama sekali.

**Bukan pengganti** `PANDUAN_TERBANG_T265.md` (itu untuk terbang FISIK
non-GPS dengan T265 asli) atau `WP_T265_TO_GUIDED_FLOW.md` (arsitektur
data). Dokumen ini murni **SITL** (drone virtual, ArduCopter default
GPS-sim — TIDAK memakai T265/EKF3 sama sekali) untuk validasi *logika
misi* sebelum ke lapangan. Semua langkah di bawah **sudah dijalankan
dan terbukti berhasil** (2026-07-31), bukan teoretis.

Scope: **Misi Seleksi Wilayah = takeoff → WP1 → gate_double → WP2 (drop)
→ yaw → gate_triple → WP3 (land)**, sesuai `mission_seleksi.yaml`.
BUKAN Misi Final (yang sampai WP4).

---

## 1. Prasyarat

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
Troubleshooting #3) sebelum lanjut.

---

## 2. Langkah 1: Nyalakan ArduCopter SITL

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

---

## 3. Langkah 2: MAVProxy bridge

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

---

## 4. Langkah 3: roscore

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

---

## 5. Langkah 4: mavros (connect ke MAVProxy, BUKAN langsung ke SITL)

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

---

## 6. Langkah 5: set_origin_home (one-shot)

Meskipun SITL default sudah punya GPS sim (beda dari lapangan non-GPS asli),
`set_origin_home.launch` tetap dijalankan di sini — konsisten dengan resep
SITL yang sudah dipakai project ini sebelumnya (lihat `WP_HEADING_ALIGN_NAVIGASI.md`
§6), dan tidak mengganggu apapun kalau origin GPS sudah ada.

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

---

## 7. Langkah 6: Jalankan Misi Seleksi Wilayah (SITL, marker di-mock)

Kamera fisik tidak ada di SITL — `sim_markers:=true` memakai `sim_markers.py`
sebagai pengganti deteksi ArUco/WP nyata. **Gate double/triple TIDAK ada
mock** (lihat Troubleshooting #2) — traverse akan selalu jatuh ke fallback
"terbang lurus geometris" setelah timeout ~20 detik, ini **normal untuk SITL**.

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
mengabaikan arg CLI, lihat Troubleshooting #4):
```bash
rosparam get /mission_d/enable_heading_align   # harus: true
rosparam get /sim_markers/sim_tail_side        # harus: E
```
**Sudah teruji:** misi tetap selesai (`MISI SELESAI`) sama seperti tanpa
heading-align. Heading-align sendiri **kemungkinan besar akan ter-SKIP**
dengan log `heading align: WpMarkerResult wp_id=X != expected=Y -> skip`
— ini **normal**, lihat Troubleshooting #5, BUKAN kegagalan.

---

## 8. Langkah 7: Matikan semua proses (bersih)

Jangan pakai `pkill` (lihat Troubleshooting #3) — matikan satu per satu
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

---

## 9. Troubleshooting — HANYA yang benar-benar ditemukan saat validasi

### #1 — `_goto` timeout 60s ke waypoint `gate_double`
**Gejala:** `[D] ABORT: goto gate_double timeout 60s` → misi auto-LAND di leg 2.
**Root cause:** `waypoints.yaml` untuk `gate_double` masih **placeholder belum
disurvei** (`x=8.0, y=0.0, z=0.0`) — `z=0.0` = level tanah/origin EKF, target
altitude ini tidak pernah terpenuhi `reach_tol_z=0.3` selagi drone terbang.
**Solusi yang terbukti bekerja:** ubah `z` ke ketinggian terbang yang konsisten
dengan WP lain (`z=1.0`, sama seperti wp1/wp2/wp3). **PENTING:** ini baru
solusi **SITL-only** — koordinat asli (`x`, `y`) tetap placeholder belum
disurvei fisik, BUKAN siap pakai untuk lomba. Ganti dengan hasil
`record_waypoint.py` sebelum terbang sungguhan (lihat §10).
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
error), seperti dicontohkan di §7.

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

---

## 10. Perbedaan SITL vs Lapangan Asli — apa yang TIDAK divalidasi di sini

| Aspek | Status di SITL (dokumen ini) | Status lapangan asli |
|---|---|---|
| Posisi/navigasi | GPS simulasi ArduCopter (EKF default) | T265 VIO → EKF3 non-GPS (lihat `PANDUAN_TERBANG_T265.md`) — **jalur berbeda total**, SITL ini TIDAK menguji T265/EKF3/vision_to_mavros sama sekali |
| Deteksi ArUco/WP marker | `sim_markers.py` (mock sempurna, off_x=off_y=0, selalu terdeteksi) | Kamera bawah asli + `aruco_node`/`wp_marker_node` — akurasi WP1 masih **belum reliable** (`WP_MARKER_DECODER.md`: agreement 0.671), verifikasi fisik pola marker WP1/WP3/WP4 **belum dilakukan** |
| Deteksi gate | **Tidak ada mock** (lihat Troubleshooting #2) — traverse SELALU pakai fallback geometris di SITL | Kamera depan asli + `gate_node.py` (single gate, dipakai Misi Seleksi) — deteksi geometris `gate_multi_detector.py` (Task 1, untuk gate fisik double/triple asli) sudah 69.2% akurasi tapi **belum diintegrasikan** ke `phaseD_mission.launch` (`GATE_DETECTOR_DOUBLE_TRIPLE.md` §6) |
| Kalibrasi mounting kamera bawah (align lateral, heading tail) | Tidak relevan — offset selalu 0 dari mock | `align_sign_x/y`, `~tail_dir_rotate_steps`, `~tail_mirror` **belum dikalibrasi fisik** — walk-test-style di lapangan wajib sebelum lomba |
| Waypoint koordinat | `wp1/wp2/wp3` dari survei (tampak nyata), `gate_double/gate_triple` **placeholder diedit SITL-only** (Troubleshooting #1) | **WAJIB** disurvei ulang dengan `record_waypoint.py` di titik takeoff lapangan sesungguhnya — koordinat SITL TIDAK BOLEH dipakai langsung |
| Gate fisik double/triple | Belum ada bentuk fisiknya sama sekali (`mission_seleksi.yaml` komentar: "pakai 2 single gate sebagai pengganti") | Sama — ini bukan keterbatasan SITL, memang belum dibangun |
| Walk test (jejak T265 vs jejak nyata) | Tidak relevan (tidak pakai T265) | **Status tidak konsisten antar dokumen** — `PANDUAN_TERBANG_T265.md` §4e anggap ini prosedur baku, `HANDOFF.md` §11/§13 (per 2026-07-16) masih tandai "BELUM dikonfirmasi lulus". Cek status terkini sebelum asumsi. |

**Kesimpulan:** dokumen ini memvalidasi **logika state machine misi**
(`mission_d.py`: urutan leg, guard kesehatan, servo drop, yaw, fallback
traverse) — bukan validasi hardware/vision/navigasi fisik. Kedua jenis
validasi ini saling melengkapi, bukan saling menggantikan.

---

## 11. Referensi silang

- Command operasional terbang T265 asli (non-SITL): `PANDUAN_TERBANG_T265.md`
- Arsitektur alur data node/topic (T265→GUIDED): `WP_T265_TO_GUIDED_FLOW.md`
- Gotcha kritis kode & konvensi: `AGENTS.md`
- Status fitur & isu diketahui (arsitektur besar): `HANDOFF.md`
- Status detektor gate double/triple: `GATE_DETECTOR_DOUBLE_TRIPLE.md`
- Status decoder WP marker custom: `WP_MARKER_DECODER.md`
- Status fitur heading-align: `WP_HEADING_ALIGN_NAVIGASI.md`
- Urutan leg persis Misi Seleksi: `src/mission_control/config/mission_seleksi.yaml`

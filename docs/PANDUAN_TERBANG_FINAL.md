# PANDUAN TERBANG FINAL — Misi Final KRTI 2026 (GPS + MTF01 OptFlow)

Konsolidasi end-to-end untuk `mission_final.yaml` (9 leg, WP1→WP5), arsitektur
GPS+OptFlow (bukan T265/VIO lagi — keputusan tim 2026-08-08, lih.
`ARSITEKTUR_LAMA_T265.md` untuk arsitektur lama & kenapa ditinggalkan, dan
`ARSITEKTUR_SISTEM.md` untuk arsitektur lengkap saat ini). Status per
**2026-09-05**.

---

## 0. STATUS RINGKAS — baca dulu sebelum apa pun

| # | Item | Status |
|---|---|---|
| 1 | Koordinat GPS di venue kompetisi | 🔴 **belum direkam** — semua 8 titik, bukan cuma WP4 |
| 2 | `gps_optflow_ek3.param` vs FC live | 🔴 **tidak sinkron** — jangan re-load file ke FC |
| 3 | `EK3_SRC1/2_POSXY/VELXY/YAW/VELZ` | 🟠 belum pernah dikonfirmasi manual |
| 4 | Servo channel drop (8 vs 9) | 🟠 belum diuji fisik |
| 5 | PreArm "Rangefinder: Not Detected" intermiten | 🟠 belum di-root-cause |
| 6 | Retry dari WP mana pun | 🟢 tervalidasi SITL (3 skenario, semua "MISI SELESAI") |
| 7 | Gate `gate_single_final` salah detektor | 🟢 **ditemukan & diperbaiki 2026-09-04** (lih. §2.7) — SITL ulang lulus |
| 8 | Mission gak berhenti kalau pilot ambil alih (mode != GUIDED) | 🟢 **ditemukan & diperbaiki 2026-09-04** (lih. §7) — SITL: berhenti <3s, tidak melawan pilot |

Detail tiap poin di §2. Urutkan pengerjaan sesuai nomor sebelum hari-H.
**Kalau mau langsung ke command per command terbang**, lompat ke §3.

---

## 1. Arsitektur Navigasi

Tim pindah dari T265/VIO ke **GPS sebagai sumber posisi utama**, dengan
MTF-01 (optical flow + rangefinder lidar via MAVLink) sebagai fallback EKF
saat GPS lemah. Semua titik misi sekarang koordinat GPS asli (lat/lon), bukan
jarak relatif ke home — supaya tidak bergeser tiap kali drone di-arm ulang.

| EKF3 source | SRC1 (utama) | SRC2 (fallback) |
|---|---|---|
| Posisi XY / Kecepatan XY | GPS (3) | OptFlow (5) |
| Posisi Z | Baro (1) | Baro (1) |
| Heading | Compass (1) | Compass (1) |

EKF **Origin** ditetapkan sekali otomatis oleh fix GPS pertama tiap boot FC —
beda dari **Home** yang di-reset tiap arm. Pipeline koordinat memanfaatkan
ini: titik WP disimpan permanen sebagai lat/lon, lalu dikonversi ke ENU lokal
terhadap origin yang sedang aktif setiap sesi terbang (lihat §4).

---

## 2. Blocker — wajib dibereskan sebelum terbang

### 2.1 🔴 Koordinat belum di venue kompetisi

Isi `config/waypoints_latlon.yaml` saat ini kemungkinan besar direkam di
lapangan latihan, bukan lokasi kompetisi — lat/lon itu absolut, jadi tidak
bisa dipakai di tempat lain. `wp4` malah kebukti belum pernah disurvei sama
sekali (nilainya identik dengan default hardcode di kode, bukan hasil rekam).
`wp5` dan `gate_single_final` masih placeholder salinan `wp3`.

→ Rekam ulang **ke-8 titik** di venue asli (§4).

### 2.2 🔴 `gps_optflow_ek3.param` sudah tidak sinkron dengan FC

File param menulis `FLOW_TYPE,3` (MTF-01 native protocol), tapi saat dicek
langsung di Mission Planner, FC yang sudah terbukti mengirim data flow beneran
justru berjalan di `FLOW_TYPE=10` / `RNGFND1_TYPE=10` (MAVLink) — sesuai skema
resmi ArduPilot untuk MTF-01 (protokol MAVLink penuh, bukan native).

**Jangan load ulang file param ini ke FC** — nilai di FC sekarang sudah
terbukti jalan, file-nya yang ketinggalan. Kalau mau dirapikan, update
angkanya di file mengikuti FC, bukan sebaliknya.

### 2.3 🟠 `EK3_SRC1/2_POSXY, VELXY, YAW, VELZ` belum pernah dikonfirmasi

Param-read via ROS/mavparam untuk field ini konsisten gagal (tidak reliable)
sepanjang sesi kerja. Belum pernah dicek manual di Mission Planner. Cek §5
sebelum terbang — kalau tidak sesuai tabel, EKF bisa fusi sumber yang salah
dan drift saat GPS lemah.

### 2.4 🟠 Servo channel drop: 8 atau 9?

`mission_auto.py` (jalur AUTO) default channel **8**. `phaseD_mission.launch`
(jalur GUIDED, dipakai `mission_d.py`) default channel **9** — sekarang sudah
bisa di-override lewat arg `servo_channel`, tapi belum pernah dites fisik
channel mana yang benar-benar terhubung ke mekanisme drop. Tes dulu di §6,
baru kunci nilainya di command misi (§3).

### 2.5 🟠 PreArm "Rangefinder: Not Detected" sesekali muncul

Muncul intermiten di log mavros saat FC live tersambung. Belum pernah dikejar
akar masalahnya — kemungkinan startup race (MTF-01 belum kirim data pertama
saat prearm check jalan). Kalau muncul saat arming: tunggu beberapa detik lalu
re-check; kalau menetap, curigai kabel SERIAL5 atau param `RNGFND1_TYPE`.

### 2.6 🟢 Sudah beres — retry dari WP mana pun

Kalau drone crash/keluar kendali, misi bisa di-restart dari WP mana pun (1-5)
termasuk auto-takeoff — tidak perlu ulang dari awal. Divalidasi SITL 3
skenario penuh (§9), termasuk 2 bug `line_follow` yang ditemukan & diperbaiki
lewat pengetesan itu sendiri.

### 2.7 🟢 Sudah beres — `gate_single_final` sempat terarah ke detektor yang salah

Ditemukan 2026-09-04 saat mengecek ulang semua `roslaunch` args di panduan ini:
ada **dua** detektor gate yang topologi fisiknya beda total —

| Detektor | Node | Struktur fisik |
|---|---|---|
| Single Gate (lama) | `gate_node.py` | terowongan oranye solid, bukaan dicari via profil kolom |
| Double/Triple Gate | `gate_multi_node.py` | bentuk-U (2 kaki + palang atas, **terbuka di bawah**) — dibangun dari nol khusus untuk ini, **tidak pernah divalidasi** untuk struktur Single Gate |

Sebelumnya `mission_d.py` cuma punya **satu** `gate_topic` global — dipilih via
arg `use_gate_multi` untuk **seluruh misi**. Itu cukup untuk `mission_seleksi`/
`mission_gate_only` (semua leg traverse-nya Double/Triple), tapi **salah**
untuk `mission_final.yaml`: `gate_double`/`gate_triple` butuh detektor
Double/Triple, sedangkan `gate_single_final` adalah Single Gate fisik —
dalam satu run, keduanya kebaca detektor yang sama (Double/Triple), yang
tidak pernah teruji di struktur Single Gate.

**Fix**: `mission_d.py` sekarang subscribe ke **dua** topic gate sekaligus
(`gate_topic` default `/gate_node/gate`, `gate_topic2` default
`/gate_multi_node/gate`), dan tiap leg `traverse` memilih sumbernya sendiri
lewat field `gate_source` di `mission_*.yaml` (`"1"`=Single Gate default,
`"2"`=Double/Triple). `mission_final.yaml` sudah diupdate: `gate_double`/
`gate_triple` = `gate_source: 2`, `gate_single_final` = default `"1"`.
`mission_seleksi.yaml`/`mission_gate_only.yaml` juga diupdate eksplisit ke
`gate_source: 2` (perilakunya tidak berubah, cuma jadi eksplisit). Kedua node
gate **boleh jalan bersamaan** sekarang (topic beda, tidak bentrok) —
divalidasi SITL, lih. §9.

---

## 3. ALUR TERBANG LENGKAP — Command per Command, Dari Awal Sampai Akhir

Ini urutan operasional penuh untuk **satu sesi terbang** (latihan maupun
hari-H), dari drone masih mati sampai selesai dan semua proses dimatikan
bersih. Asumsi: langkah **sekali-sebelum-kompetisi** (§4 survei GPS, §5 cek
param FC, §6 tes servo) **sudah selesai** — kalau belum, berhenti dulu dan
kerjakan itu, jangan lanjut ke sini.

Setiap blok command ditulis untuk dijalankan di terminal Jetson (langsung
atau via SSH). Command yang perlu tetap hidup di latar belakang selama sesi
ditandai — pakai terminal terpisah (tab baru / `tmux` / `screen`), **jangan**
`Ctrl+C` sebelum waktunya kecuali memang mau menghentikan sesi.

### 3.1 Persiapan Fisik (sebelum nyalakan apa pun)

1. Pasang baterai drone, **jangan** arm dulu.
2. Nyalakan remote (RC transmitter) — pastikan switch mode ada di
   **Stabilize**.
3. Cek fisik: kedua kamera (depan C310, bawah C920) terpasang & tidak
   longgar, MTF-01 terpasang mengarah bawah, servo drop bebas bergerak.
4. Nyalakan Jetson Nano, tunggu boot selesai (~1 menit).
5. Pastikan drone di **area terbuka**, GPS akan butuh pandangan langit yang
   jelas (bukan di dalam ruangan/dekat tembok tinggi).

### 3.2 Bring-Up Software (urutan pasti, jangan diloncat)

Buka terminal baru untuk tiap langkah yang ditandai **(latar belakang)** —
biarkan tetap jalan sepanjang sesi.

**Langkah 1 — roscore (latar belakang):**
```bash
roscore
```

**Langkah 2 — cek nomor device kamera** (bisa tertukar tiap dicolok ulang):
```bash
v4l2-ctl --list-devices
```
Catat mana `/dev/videoN` untuk kamera depan (gate) dan bawah (marker/line).

**Langkah 3 — nyalakan kedua kamera (latar belakang):**
```bash
roslaunch mission_control cameras.launch \
  front_device:=/dev/video0 down_device:=/dev/video1
```
(Sesuaikan nomor device dengan hasil Langkah 2 kalau beda dari default.)

**Langkah 4 — sambungkan mavros ke FC (latar belakang):**
```bash
roslaunch mavros apm.launch fcu_url:="/dev/ttyTHS1:921600"
```
Pakai `apm.launch` polos — **bukan** `apm_with_flow.launch` (sisa eksperimen
optical-flow-di-dashboard yang sudah dibatalkan, lihat §10).

**Langkah 5 — tunggu mavros connected** (terminal baru, sekali jalan):
```bash
rostopic echo -n1 /mavros/state
```
Ulangi sampai muncul `connected: True`. Kalau tidak kunjung connect: cek
kabel TELEM2↔`/dev/ttyTHS1`, baud 921600, FC sudah menyala.

**Langkah 6 — tunggu GPS fix layak** (terminal sama, sekali jalan, ulangi
sampai syarat terpenuhi):
```bash
rostopic echo -n1 /mavros/gpsstatus/gps1/raw
```
Syarat lanjut: `fix_type >= 3` (3D fix), `satellites_visible >= 8`,
`eph < 150` (HDOP < 1.5). Kalau lama tidak tercapai: cek drone benar-benar
di ruang terbuka, tunggu lebih lama (cold-start GPS bisa 1-2 menit).

**Langkah 7 — konversi koordinat GPS → ENU lokal** (sekali jalan, WAJIB
setiap sesi, setelah GPS fix, sebelum langkah 9):
```bash
rosrun mission_control latlon_to_waypoints.py
```
Ini membaca origin EKF yang baru saja ter-set dari fix GPS Langkah 6, dan
menulis ulang `config/waypoints.yaml` (yang dibaca `mission_d.py`). Kalau
langkah ini dilewat, misi akan pakai koordinat ENU dari sesi SEBELUMNYA
(origin beda → drone terbang ke tempat yang salah).

**Langkah 8 — nyalakan dashboard monitoring (latar belakang):**
```bash
roslaunch mission_control web_dashboard.launch
```
Buka `http://<ip-jetson>:5000/` dari laptop (satu jaringan/hotspot). Cek IP
Jetson dengan `hostname -I` kalau belum tahu.

**Langkah 9 — (kalau belum pernah dites hari ini) verifikasi servo channel**
— lihat §6 kalau `servo_channel` yang benar (8 atau 9) belum dikunci.

### 3.3 Jalankan Misi Final

Setelah Langkah 1-8 selesai (mavros connected, GPS fix OK, koordinat sudah
dikonversi, dashboard sudah bisa dibuka), drone siap arm & terbang. Command
ini sudah termasuk **auto-takeoff** — tidak perlu takeoff manual dulu:

```bash
roslaunch mission_control phaseD_mission.launch mission:=final \
  use_gate_multi:=true \
  use_wp_marker:=true  use_aruco:=false \
  use_line_follow:=true \
  servo_channel:=<8 atau 9, hasil §6>
```

Tiga hal yang WAJIB benar di command ini (kenapa nilainya begini, bukan yang
lama):

- **Gate — KEDUA node, bukan salah satu.** `mission_final.yaml` punya gate
  Double/Triple (`gate_double`, `gate_triple`) **dan** Single Gate
  (`gate_single_final`) dalam satu run — dua struktur fisik berbeda, dua
  detektor berbeda (lih. §2.7). `use_gate` **default sudah `true`** (jangan
  di-set `false`, itu pola lama sebelum fix §2.7) + tambahkan
  `use_gate_multi:=true`. Leg mana pakai detektor mana sudah ditentukan lewat
  `gate_source` di dalam `mission_final.yaml` sendiri, tidak perlu diatur dari
  command line.
- **Marker WP1/WP2/WP3 — decoder kustom Task 3, bukan ArUco polos.**
  `use_wp_marker:=true use_aruco:=false` → mengarahkan ke `wp_marker_node.py`
  (reference-pattern decoder + hamming-distance, dituning khusus dataset
  lapangan, presisi 95-100%). **Bukan** `aruco_node.py`/`use_aruco:=true` —
  itu decoder ArUco generik `DICT_7X7_50`, tidak pernah jadi jalur produksi
  untuk marker WP (cuma dipakai *secara internal* oleh `wp_marker_node.py`
  sendiri sebagai cross-check tambahan untuk WP3, dengan `valid_ids`
  ter-tuning khusus — bukan node terpisah yang dinyalakan dari launch).
- **Line-follower WP3→WP4** — `use_line_follow:=true`, resmi dikonfirmasi
  panitia 2026-09-03 (§0).

Alternatif — kalau mau pakai wrapper `run_mission_gps.sh` (menyalakan roscore
+ mavros + tunggu GPS fix otomatis, TAPI **tidak** menjalankan Langkah 3/7/8
di atas — kamera, konversi koordinat, dan dashboard tetap harus dijalankan
manual terpisah seperti biasa):
```bash
./run_mission_gps.sh final \
  use_gate_multi:=true \
  use_wp_marker:=true  use_aruco:=false \
  use_line_follow:=true \
  servo_channel:=<8 atau 9, hasil §6>
```

### 3.4 Selama Terbang — Monitoring

Buka `http://<ip-jetson>:5000/` (dari Langkah 8). Perhatikan:

| Yang dilihat | Normal | Tanda masalah |
|---|---|---|
| WP sekarang / progres leg | Naik berurutan wp1→...→wp5 | Diam lama di satu leg (bisa jadi `scan_timeout`/`gate_timeout` terpakai — masih aman, tapi kalau berkali-kali di leg yang sama, curiga vision gagal terus) |
| Mode & armed | `GUIDED`, `armed: True` | Mode berubah sendiri di luar kendali pilot |
| Baterai | Turun bertahap wajar | Drop tiba-tiba/tegangan rendah — siap ambil alih & landing |
| GPS fix/sats/HDOP | Tetap ≥3D, sats≥8 | Fix hilang di tengah terbang — EKF fallback ke OptFlow (SRC2), pantau kestabilan |
| Rangefinder (ToF) | Berubah wajar sesuai ketinggian | Statis/nol terus saat terbang — curiga sensor bermasalah |

Detail lengkap tiap field: lihat §8.

### 3.5 Kalau Terjadi Masalah — Retry / Abort

- **Pilot selalu bisa ambil alih kapan saja** lewat switch RC ke
  Stabilize/Loiter/Land — `mission_d.py` mendeteksi perubahan mode dalam
  <1 detik dan berhenti total, **tidak** melawan pilot (lihat §7 untuk
  detail fix ini).
- **Kalau drone perlu di-restart mid-mission** (habis di-recover manual,
  bukan mulai dari nol): ulangi §3.3 dengan tambahan `start_wp:=<nama>`.
  Tabel lengkap opsi restart: lihat §7.

### 3.6 Setelah Mendarat — Shutdown Bersih

1. Konfirmasi di dashboard/log: `armed: False`, mode akhir `LAND` (bukan
   masih `GUIDED` tergantung).
2. Matikan node misi (`Ctrl+C` di terminal Langkah 3.3) kalau belum otomatis
   berhenti sendiri (harusnya sudah, `mission_d.py` keluar sendiri setelah
   `=== MISI SELESAI ===`).
3. Matikan proses latar belakang satu-satu (`Ctrl+C` di tiap terminal):
   dashboard (Langkah 8) → mavros (Langkah 4) → kamera (Langkah 3) →
   roscore (Langkah 1) — urutan ini opsional, tidak wajib persis begini,
   tapi hindari mematikan roscore duluan selagi node lain masih coba
   connect ke situ.
4. Lepas baterai drone.
5. Matikan Jetson dengan benar (`sudo shutdown now`) sebelum cabut daya —
   jangan langsung cabut power tanpa shutdown software, risiko korup
   filesystem.

---

## 4. Survei Koordinat GPS di Venue

Dilakukan **sekali** saat tim tiba di lokasi kompetisi, sebelum sesi
latihan/misi pertama. Drone GUIDED, GPS fix bagus (3D, sats≥8), terbang manual
ke tiap titik lalu rekam.

Urutan rekam — samakan dengan urutan leg `mission_final.yaml`:

```bash
rosrun mission_control record_waypoint_latlon.py _name:=wp1
rosrun mission_control record_waypoint_latlon.py _name:=gate_double
rosrun mission_control record_waypoint_latlon.py _name:=wp2
rosrun mission_control record_waypoint_latlon.py _name:=gate_triple
rosrun mission_control record_waypoint_latlon.py _name:=wp3
rosrun mission_control record_waypoint_latlon.py _name:=wp4                # wajib -- sebelumnya placeholder
rosrun mission_control record_waypoint_latlon.py _name:=gate_single_final  # baru
rosrun mission_control record_waypoint_latlon.py _name:=wp5                # baru, landing pad
```

Tiap panggilan menulis/menimpa satu entri di `config/waypoints_latlon.yaml` —
file ini permanen, dipakai lintas sesi. **Setiap sesi terbang sesudahnya**
(bukan cuma sekali), jalankan konversi ke ENU lokal terhadap origin EKF yang
aktif saat itu — ini sudah jadi Langkah 7 di §3.2:

```bash
rosrun mission_control latlon_to_waypoints.py
```

Ini menulis ulang `config/waypoints.yaml` (dibaca `mission_d.py`).
`mission_d.py` sendiri tidak perlu diubah apa pun — dia selalu baca ENU lokal,
sumbernya GPS atau bukan.

---

## 5. Cek Parameter FC (Mission Planner)

Cek manual satu-satu di *Full Parameter List* — **jangan** bulk-load
`gps_optflow_ek3.param` mentah-mentah (lihat §2.2).

| Param | Nilai diharapkan | Status |
|---|---|---|
| GPS1_TYPE | 1 | ✅ confirmed |
| COMPASS_ENABLE | 1 | ✅ confirmed |
| AHRS_EKF_TYPE | 3 | ✅ confirmed |
| EK3_ENABLE | 1 | ✅ confirmed |
| VISO_TYPE | 0 | ✅ confirmed |
| FLOW_TYPE | 10 (MAVLink) | ✅ confirmed live — abaikan file param |
| RNGFND1_TYPE | 10 (MAVLink) | ✅ confirmed live — abaikan file param |
| SERIAL5_PROTOCOL | 1 (MAVLink1) | ✅ confirmed, MTF-01 di SERIAL5 |
| EK3_SRC1_POSXY / VELXY / YAW / VELZ | 3 / 3 / 1 / 3 | ❌ belum dicek — WAJIB |
| EK3_SRC2_POSXY / VELXY / YAW / VELZ | 5 / 5 / 1 / 1 | ❌ belum dicek — WAJIB |

Kolom "Status" adalah state per 2026-09-04 — verifikasi ulang kalau ada yang
mengubah param FC di antara sesi ini dan hari-H.

---

## 6. Tes Fisik Servo (sekali, di bengkel)

Selesaikan §2.4 sebelum hari-H — cukup dites sekali di darat, drone di stand,
tanpa terbang. Arm dulu (di darat, baling-baling dilepas / area aman), lalu
tes tiap channel:

```bash
rosservice call /mavros/cmd/command "{command: 183, param1: 8, param2: 1013}"   # coba channel 8
rosservice call /mavros/cmd/command "{command: 183, param1: 9, param2: 1013}"   # coba channel 9
```

Channel mana yang benar-benar menggerakkan mekanisme drop → itu nilai yang
dipakai sebagai `servo_channel:=<N>` di command misi (§3.3). PWM 1013 =
buka/drop, 2015 = tutup (default `mission_d.py`).

---

## 7. Retry dari WP Manapun / Abort

Kalau drone crash, keluar kendali, atau misi dibatalkan di tengah jalan —
tidak perlu ulang dari WP1. Tambahkan `start_wp:=<nama>` ke command yang sama
di §3.3. Leg sebelum WP itu di-skip, auto-takeoff tetap jalan dari titik itu.

| Restart dari | Efek | Flag |
|---|---|---|
| WP1 | mulai dari awal (default, tanpa `start_wp`) | — |
| WP2 | lewati scan WP1 + gate_double | `start_wp:=wp2` |
| WP3 | lewati sampai drop + gate_triple | `start_wp:=wp3` |
| WP4 | langsung line_follow → gate final → land | `start_wp:=wp4` |
| WP5 | langsung land di landing pad | `start_wp:=wp5` |

Kalau nama WP salah ketik / tidak ada di `mission_final.yaml`, `mission_d.py`
menolak fallback diam-diam — dia log error dan tetap jalan dari leg 0
(fallback aman), bukan macet.

**Abort darurat tetap tanggung jawab pilot keselamatan** — `mission_d.py`
tidak pernah menahan switch mode RC, LAND/RTL manual selalu bisa memutus alur
GUIDED kapan saja.

✅ **FIX 2026-09-04, divalidasi SITL**: sebelumnya `mission_d.py` cuma cek
armed+pose di tiap loop, **tidak** cek mode — kalau pilot switch RC keluar
GUIDED (mis. STABILIZE) buat ambil alih, mission baru berhenti lewat timeout
per-leg (bisa puluhan detik), dan saat itu terjadi dia malah **memaksa balik
ke LAND**, melawan pilot. Sekarang `mission_d.py` cek mode di setiap loop
tick (sama tempat cek armed/pose) — begitu mode berubah dari GUIDED, mission
langsung berhenti pada tick berikutnya (~50ms) dan **tidak** memanggil
`set_mode()` apa pun, mode yang dipilih pilot dibiarkan apa adanya. Diuji
SITL: paksa mode ke STABILIZE di tengah leg traverse → mission_d berhenti <3
detik (didominasi latensi round-trip SITL, bukan mission_d), FC tetap di
STABILIZE, tidak ada percobaan balik ke LAND/GUIDED.

---

## 8. Monitoring Saat Terbang (Dashboard)

`http://<ip-jetson>:5000/` — MJPEG dua kamera + telemetry, 1x refresh/detik,
jauh lebih ringan dari VNC di link radio terbatas.

| Ditampilkan | Sumber |
|---|---|
| Kamera bawah & depan (live MJPEG) | `/camera_down`, `/camera_front` |
| WP sekarang / progres leg | `/mission_d/status` |
| Mode & armed | `/mavros/state` |
| Baterai (V, %) | `/mavros/battery` |
| GPS fix / satelit / HDOP | `/mavros/gpsstatus/gps1/raw` |
| Rangefinder (ToF, MTF-01) | `/mavros/distance_sensor/rangefinder_pub` |
| Posisi lokal (ENU) | `/mavros/local_position/pose` |

Optical flow **sengaja tidak ditampilkan**: FC mengirim MAVLink
`OPTICAL_FLOW` (id 100), sedangkan plugin mavros hanya mendukung
`OPTICAL_FLOW_RAD` (id 106) — tidak ada topic yang bisa menerima format itu
tanpa bridge pymavlink custom. Kalau dibutuhkan nanti, itu pengembangan
terpisah, bukan konfigurasi.

---

## 9. Bukti Validasi SITL

**2026-09-03** — tiga skenario dijalankan penuh di ArduCopter SITL memakai
`mission_final.yaml` 9-leg (termasuk `line_follow` WP3→WP4 dan
`gate_single_final`):

| Skenario | Leg dijalankan | Hasil |
|---|---|---|
| Misi penuh dari WP1 | 9/9 | ✅ MISI SELESAI — landed, disarmed bersih |
| Retry `start_wp:=wp3` | 4/4 (skip 5 leg awal) | ✅ MISI SELESAI |
| Retry `start_wp:=wp4` | 3/3 (skip 6 leg awal) | ✅ MISI SELESAI |

Dua bug ditemukan & diperbaiki lewat pengetesan ini sendiri (bukan
sebelumnya):

1. `line_follow` sempat kena pre-goto generik yang langsung "teleport" ke WP
   tanpa sempat koreksi visual — sudah dikecualikan dari pre-goto.
2. Pengecekan "sudah sampai" pada `line_follow` sempat membandingkan ke titik
   WP asli, bukan titik terkoreksi yang benar-benar sedang dikejar —
   menyebabkan timeout 60 detik palsu saat garis melengkung.

**2026-09-04** — misi penuh diulang dengan `use_gate:=true use_gate_multi:=true`
(kedua node gate aktif bersamaan, fix §2.7): 9/9 leg, ✅ MISI SELESAI,
`gate_node` dan `gate_multi_node` sama-sama hidup sepanjang run tanpa
bentrok topic/port. Memvalidasi bahwa pemilihan detektor per-leg
(`gate_source`) berjalan sesuai rencana untuk ketiga leg traverse
(`gate_double`→sumber 2, `gate_triple`→sumber 2, `gate_single_final`→sumber 1).

Semua bug di atas sudah diverifikasi ulang setelah fix, termasuk di skenario
2026-09-04.

---

## 10. Referensi File

| File | Peran |
|---|---|
| `config/waypoints_latlon.yaml` | Sumber kebenaran GPS permanen (§4) |
| `config/waypoints.yaml` | ENU lokal, dibangkitkan ulang tiap sesi |
| `config/mission_final.yaml` | 9 leg misi final, resmi dikonfirmasi panitia |
| `scripts/mission_d.py` | State machine misi + retry-dari-WP + servo drop + pilih detektor gate per-leg (§2.7) |
| `scripts/gate_node.py` | Detektor Single Gate (`gate_source: 1`, default) — dipakai `gate_single_final` |
| `scripts/gate_multi_node.py` | Detektor Double/Triple Gate bentuk-U (`gate_source: 2`) — dipakai `gate_double`/`gate_triple` |
| `scripts/wp_marker_node.py` | Decoder WP1/WP2/WP3 produksi (`use_wp_marker:=true`) — **bukan** `aruco_node.py` |
| `scripts/aruco_node.py` | ArUco generik `DICT_7X7_50` — tidak dipakai langsung di `mission_final`, cuma dipanggil internal oleh `wp_marker_node.py` sbg cross-check |
| `scripts/record_waypoint_latlon.py` | Rekam 1 titik GPS (§4) |
| `scripts/latlon_to_waypoints.py` | Konversi lat/lon → ENU tiap sesi (§3.2 Langkah 7) |
| `scripts/web_dashboard.py` | Server Flask MJPEG + telemetry (§8) |
| `launch/phaseD_mission.launch` | Entry point misi GUIDED (§3.3) |
| `run_mission_gps.sh` | Wrapper roscore+mavros+tunggu GPS fix (§3.3, alternatif) |
| `params/gps_optflow_ek3.param` | ⚠️ tidak sinkron dengan FC, jangan re-load (§2.2) |
| `launch/apm_with_flow.launch` | Sisa eksperimen optical-flow-di-dashboard, dibatalkan — pakai `mavros apm.launch` polos untuk terbang |

---

## 11. Lampiran: Uji di Gazebo (2026-09-04)

Dicoba sebagai pelengkap SITL polos (§9) — SITL biasa `sim_markers.py` cuma
publish EVENT palsu (bukan gambar), jadi loop visual servoing beneran
(`_traverse()`/`_align_and_drop()`/`_line_follow()` mengoreksi posisi dari
`off_x/off_y` kamera REAL-TIME) **belum pernah** benar-benar tereksekusi di
sesi manapun sebelum ini — selalu lewat fallback "TAK TERLIHAT".

**Yang dibangun** (baru, di `mission_control/gazebo/`, terpisah dari `iq_sim`):
- `models/drone_dual_cam` — klon `drone_with_camera` (iq_sim) + kamera bawah,
  publish ke `/camera_front/image_raw` & `/camera_down/image_raw` (nama persis
  default `mission_d.py`, tanpa remap).
- `models/gate_leg` — 1 lapis Double/Triple Gate bentuk-U, warna oranye HSV
  mid `gate_multi_detector.py`. Diulang di world (2x double, 3x triple).
- `models/gate_single` — dinding solid + lubang, HSV mid `gate_detector.py`.
- `models/wp_marker_1/2/3` — bidang tanah bertekstur **gambar ArUco asli**
  (`aruco_image/Aruco_WP{1,2,3}.png`, bukan warna placeholder).
- `models/line_dash` — segmen garis warna HSV mid `line_detector.py`.

**Kalibrasi koordinat** (empiris, terbukti lewat goto uji): world Gazebo dan
ENU lokal mavros TIDAK 1:1 — plugin ArduPilot pakai rotasi roll 180°:
```
Gazebo_X =  ENU_y (utara)      Gazebo_yaw = ENU_yaw - 90°
Gazebo_Y = -ENU_x (timur)
Gazebo_Z =  ENU_z (naik)
```
Semua properti ditata pakai transform ini, cocok dgn layout
`waypoints_gazebo.yaml` (sama koordinat dgn uji SITL §9).

**Hasil (`mission:=final`, `sim_markers:=false`, semua detektor real: `use_gate`,
`use_gate_multi`, `use_wp_marker`, `use_line_follow` semua true)**:

| | |
|---|---|
| Status akhir | ✅ MISI SELESAI, 9/9 leg, landed bersih |
| **Bukti positif kunci** | `gate_double` lapis 1: **beneran terdeteksi** kamera depan lewat `gate_multi_node` — `"gate terpusat (off<0.12, n=1) -> through"` HANYA 0.4 detik, BUKAN fallback timeout. Ini pertama kalinya loop koreksi visual `_traverse()` tereksekusi dgn data nyata (bukan `None`), tervalidasi. |
| Sisanya (wp1/wp2/wp3 scan, gate_double lapis 2, gate_triple 3 lapis, gate_single_final) | ⚠️ semua fallback "TAK TERLIHAT/timeout" — TIDAK terdeteksi |

**Root cause sisanya gak terdeteksi**: BUKAN bug `mission_control`. Dicek
langsung ambil 1 frame `/camera_front` & `/camera_down` di tengah simulasi
(disimpan ke file, dilihat) — keduanya **abu-abu rata sempurna (std=0.0)**,
bukan render scene sungguhan. Bertepatan dgn `free -h` nunjukin RAM Jetson
kritis (84Mi free dari 3.9GB, sudah masuk swap) setelah ~7 menit
Gazebo+VNC+SITL+4 node detektor jalan bareng. Deteksi `gate_double` yang
berhasil terjadi ~1 menit sejak start (RAM masih longgar) -- render kamera
kemungkinan besar collapse ke abu-abu placeholder begitu OGRE/VNC framebuffer
kehabisan memori, BUKAN karena gate/marker gak kelihatan dari sudut kamera.
Jetson Nano B01 (4GB RAM) memang batas bawah utk Gazebo11 + rendering kamera
ganda + SITL + 4 detektor node sekaligus.

**Kesimpulan**: pipa penuh (Gazebo→kamera ROS→detector→`Gate`/`WpMarkerResult`
msg→`mission_d.py` koreksi visual→setpoint) TERBUKTI bekerja end-to-end saat
render kameranya hidup. Reliabilitas RENDER di Jetson ini yang jadi
bottleneck, bukan logika misi. **Tidak disarankan** jadi validasi akurasi
deteksi (itu tetap harus dari foto/data real, sudah pernah dilakukan
terpisah per detektor — lih. §10) — gunanya di sini murni buat mengonfirmasi
jalur ROS/servo-loop-nya nyambung benar.

Semua proses dibersihkan setelah uji (RAM pulih ke 2.3GB available). File
world/model TETAP ada di `mission_control/gazebo/` kalau mau diulang —
jalankan dgn VNC (`vncserver :1 -geometry 1280x800 -depth 24`, `DISPLAY=:1`)
sbg display buat render kamera headless, dan kalau mau coba lagi sebaiknya
matikan node yang gak lagi diuji (mis. cuma nyalakan 1 detektor per sesi)
biar gak kehabisan RAM lagi.

### 11.1 Uji ulang isolasi 1 detektor/sesi (2026-09-04, lanjutan)

Diulang khusus `wp_marker_node` sendirian (bukan 4 detektor sekaligus) — hasil
lebih bersih, dan ketemu 1 bug BARU di setup Gazebo-nya sendiri (bukan di
`mission_control`):

**RAM dikonfirmasi jadi penyebab §11 di atas**: dgn cuma 1 detektor jalan,
kamera render GAMBAR SUNGGUHAN (std piksel >0, bukan abu-abu rata std=0.0) —
tekstur marker ArUco kelihatan jelas di `/camera_down/image_raw`. Cocok dgn
teori RAM: makin sedikit node, makin stabil render-nya.

**Bug baru ditemukan & diperbaiki**: `wp_marker_1/2/3` awalnya SEMUA
menampilkan tekstur `wp_marker_1` (Aruco_WP1.png) — walau file tekstur
masing-masing beda persis (dicek md5sum). Root cause: ketiga model pakai
nama file relatif SAMA (`wp.png`) di `materials/textures/`; OGRE (rendering
engine Gazebo) meng-cache resource per nama file, jadi model kedua/ketiga yg
di-load ikut kebaca cache resource pertama. **Fix**: nama file tekstur
dibikin unik (`wp1.png`/`wp2.png`/`wp3.png`) + **restart gzserver penuh**
(respawn model SAJA tidak cukup — cache OGRE per-proses gzserver, bukan
per-model, jadi gak ke-reset walau model di-delete+spawn ulang).

**Hasil setelah fix** (drone terbang manual ke tiap titik, cek
`/wp_marker_node/debug_image`):

| WP | Tekstur render | Hasil decode |
|---|---|---|
| WP1 | ✅ benar (Aruco_WP1.png, beda dari WP3) | `WP-1 low_quality` |
| WP3 | ✅ benar (Aruco_WP3.png, beda dari WP1, dikonfirmasi visual) | `no_marker` (2 sampel frame) — `line_dash_0` (properti garis) sempat menutupi sudut marker di kedua sampel |

⚠️ **RALAT (ditemukan pas sesi lanjutan §11.4): interpretasi "cocok histori
WP1 recall rendah" di atas SALAH.** `"WP-1"` di teks debug BUKAN "marker
WP1 terbaca" — itu `f"WP{wp_id}"` dgn `wp_id=-1` (sentinel "gak ketemu ID
apapun"), jadi literal nyambung jadi string "WP" + "-1" = "WP-1". Kedua tes
WP1 maupun WP3 di atas SAMA-SAMA gagal dapet ID confident (`reason` bukan
`'ok'`), bukan salah satu berhasil salah satu belum cukup bukti. Root cause
sebenarnya (kegelapan tekstur sintetis) dijelaskan di §11.4.

Ketemu 1 lagi hal yg perlu dicatat buat pengulangan berikutnya: jarak antar
properti di world (`gate_single_final`, `line_dash`, `wp_marker_3` semua di
sekitar Gazebo Y=-12 s.d. -20) terlalu rapat dibanding FOV kamera — properti
tetangga bisa saling menutupi. Kalau mau uji `gate_single_final`/`line_follow`
terisolasi juga, longgarkan jarak ini dulu di `spawn_props.py`.

### 11.2 `gate_single_final` terisolasi (2026-09-04, lanjutan) — ✅ BERHASIL

Diulang khusus `gate_node.py` sendirian (Single Gate). Ketemu 2 bug LAGI di
model Gazebo-nya sendiri (bukan di `mission_control`), keduanya diperbaiki
sampai deteksi genuinely berhasil:

**Bug 1 — warna oranye kegelapan**: `gate_leg`/`gate_single`/`line_dash`
pakai warna `<ambient>/<diffuse>` polos tanpa `<script>` (beda dari tekstur
marker WP yg sudah pakai `lighting off`). Kena shading directional-light
Gazebo, Value jadi 67 (target render 167) — DI BAWAH threshold
`gate_detector.py` (butuh V>80). **Fix**: tambah `<emissive>` sama persis
warna ambient/diffuse di ketiga model (efeknya = `lighting off` versi
material polos, gak perlu material script terpisah). Setelah fix: HSV
terukur (19,167,234) — semua lolos threshold.

**Bug 2 — topologi model salah** (lebih penting): `gate_single` awalnya
dibangun sbg dinding TERTUTUP 4 sisi (ada slab bawah), berdasar baca
docstring "berbentuk terowongan". Ternyata SALAH — dicek langsung ke
`gate_detector.py::_make_synthetic()` (fungsi self-test RESMI milik node
ini sendiri, sumber kebenaran paling akurat drpd docstring): Single Gate
ASLI **terbuka di bawah** juga ("lubangi celah dari bawah balok atas ke
dasar") — topologinya SAMA kayak `gate_leg` (2 kaki + balok atas), cuma beda
algoritma deteksi (kolom-profil vs komponen-background). Kolom-profil
`detect()` cuma scan band 35%-100% tinggi bounding-box; slab bawah bikin
band itu selalu penuh oranye → `num_openings` selalu 0, GAK PERNAH bisa
`detected=True` walau bukaan tengahnya jelas keliatan. **Fix**: slab bawah
dihapus, kaki kiri/kanan diperpanjang sampai tanah (z=0).

**Hasil setelah KEDUA fix** — deteksi genuinely BERHASIL:
```
detected: True
off_x: 0.048   off_y: 0.329   num_openings: 1   opening_w_frac: 0.239
```
Debug image (`/gate_node/debug_image`) mengonfirmasi visual: bounding-box
oranye pas di gate, garis hijau tepat di tengah bukaan, titik TARGET merah —
persis seperti gate asli terdeteksi kamera sungguhan di lapangan.

**Kesimpulan §11.2**: sekarang ADA DUA bukti positif independen (gate_double
lapis-1 dari §11, gate_single_final penuh dari sesi ini) bahwa jalur
Gazebo→kamera ROS→detector→`Gate` msg→`mission_d.py` bekerja genuinely benar
utk KEDUA jenis gate (Double/Triple bentuk-U DAN Single Gate). Root cause
kegagalan sebelumnya utk `gate_single_final` (§11) bukan cuma RAM — model
Gazebo-nya sendiri emang cacat (2 bug di atas), sekarang sudah diperbaiki
permanen di `mission_control/gazebo/models/gate_single/`.

### 11.3 `line_follow` terisolasi (2026-09-04, lanjutan) — ✅ BERHASIL, tanpa bug baru

Diulang khusus `line_node.py` sendirian. Sebelum mulai, jarak `line_dash`
dilonggarkan di `spawn_props.py` (dulu nempel wp3/wp4 di §11.1, sekarang
0.8m dari wp3/wp4 di tiap ujung, pitch 0.8m antar dash).

**Langsung berhasil di percobaan pertama**, tanpa perlu bug fix tambahan —
fix `emissive` dari §11.2 (dulu ditambahkan ke `gate_leg`/`gate_single`/
`line_dash` sekaligus, karena pola bug-nya sama) sudah otomatis kepakai:

```
detected: True   off_x: 0.006   off_y: 0.456   angle_deg: 88.1   area_frac: 0.038
```

Debug image (`/line_node/debug_image`) mengonfirmasi visual: bounding-box
kuning pas di 1 segmen dash, garis hijau + titik target merah tepat di
tengah — drone diposisikan center di jalur, `off_x`≈0 seperti seharusnya.

**Tes tambahan — validasi loop koreksi (bukan cuma deteksi statis)**: drone
digeser manual 1m ke samping (lateral thd arah jalur) →
`off_x` melompat dari 0.005 ke **0.809**, `detected` tetap `True`. Ini
membuktikan sinyal koreksi VISUAL genuinely responsif terhadap posisi
sungguhan (bukan angka statis/kebetulan) — persis logika yang dipakai
`_line_follow()` di `mission_d.py` utk menghitung koreksi `off_x/off_y` ->
`_body_to_enu()` -> setpoint.

**Kesimpulan §11.3**: ketiga jenis leg visual-servo (traverse gate ganda,
traverse gate single, line_follow) sekarang SEMUA punya bukti positif
independen di Gazebo. Leg `scan`/`drop` (`wp_marker_node`) diulang lagi di
§11.4 dgn temuan lebih pasti.

### 11.4 `wp_marker_node` diulang lagi, jarak dilonggarkan lagi (2026-09-04) — root cause ketemu

Jarak `line_dash` ke `wp3` dilonggarkan lagi (1.2m, dari radius FOV kamera
bawah ~1.37m di altitude 2m — round sebelumnya 0.8m di §11.1 ternyata masih
belum cukup). Hasil: foto `wp3` sekarang BERSIH, tanpa gangguan visual apa
pun dari properti lain (dikonfirmasi visual).

**Koreksi penting thd §11.1**: ambil 5 sampel debug_image berurutan (bukan
cuma 1-2 kayak sebelumnya) — SEMUA nunjukin `WP-1 low_quality`, konsisten,
BUKAN acak. Ini membuka investigasi ke source code (`wp_decode_v2.py`, yang
ternyata dipakai `wp_marker_node.py` sbg decoder UTAMA — bukan
`wp_decode_audit.py` yg dipakai `wp_marker_detector.py` cuma utk fungsi
presence WP2) dan ketemu **root cause pasti, bukan tebakan**:

```python
MIN_CORE_BRIGHTNESS = 100   # kalibrasi dari distribusi confident-correct wp3
                             # (n=178, min=109, p10=114) ...
def _quality_gate(bgr, core):
    ...
    return float(vals.mean()) >= MIN_CORE_BRIGHTNESS, float(vals.mean())
```

`_quality_gate()` cek rata-rata grayscale seluruh area "core" marker (bukan
cuma sel putih) harus ≥100 — dikalibrasi dari FOTO ASLI, di mana sel
"hitam" marker gak pernah benar-benar RGB(0,0,0) (selalu ada cahaya
ambient/pantulan bikin "hitam" sebenarnya abu-abu gelap ~20-40). Tekstur
Gazebo saya pakai `lighting off` (fix §11.1) supaya TIDAK terpengaruh
shading — hasilnya PNG asli dirender APA ADANYA, termasuk hitamnya yang
BENERAN RGB(0,0,0). Dicek langsung mean grayscale file sumbernya:

| File | Mean grayscale | vs threshold (100) |
|---|---|---|
| Aruco_WP1.png | 80.5 | ❌ di bawah |
| Aruco_WP2.png | 92.7 | ❌ di bawah |
| Aruco_WP3.png | 89.1 | ❌ di bawah |

Ketiganya secara struktural di bawah threshold — BUKAN kebetulan, BUKAN bug
`mission_control`, BUKAN masalah render/occlusion/RAM (yg 3 hal itu semua
sudah dicek beres duluan di sesi ini). Murni krn tekstur sintetis saya
"terlalu bersih" (hitam sungguhan) dibanding foto lapangan asli (hitam yg
selalu sedikit terangkat cahaya ambient) — persis apa yang komentar kode
`MIN_CORE_BRIGHTNESS` bilang jadi dasar kalibrasinya.

**Koreksi ralat §11.1**: tulisan sebelumnya "`WP-1 low_quality` — cocok
histori nyata (WP1 recall rendah)" SALAH BACA. `"WP-1"` di debug text =
`f"WP{wp_id}"` dgn `wp_id=-1` (sentinel "gak ada ID cocok"), bukan "WP1
kebaca". Kedua tes WP1 maupun WP3 SAMA-SAMA gagal dpt ID confident
(`reason≠'ok'`) — bukan salah satu berhasil.

**Bukan dianggap kegagalan `mission_control`** — ini keterbatasan REALISME
tekstur sintetis Gazebo saya, bukan gap di kode/tuning proyek. Buat
benar-benar lolos quality-gate di Gazebo (kalau mau dicoba lagi), tekstur
perlu dimodifikasi: naikkan level "hitam" dari RGB(0,0,0) ke sekitar
RGB(30,30,30) biar rata-rata core mendekati kondisi foto asli — TAPI itu
berarti gambar Gazebo-nya BUKAN LAGI Aruco_WPx.png asli apa adanya
(trade-off fidelitas vs lolos gate sintetis). Karena akurasi decoder
WP1/WP2/WP3 sendiri SUDAH divalidasi ekstensif & terpisah pakai ratusan
foto lapangan asli (lih. §10, `WP_MARKER_DECODER_V2.md`), uji Gazebo ini
TIDAK menambah keyakinan soal akurasi decoder — cuma menegaskan ULANG bahwa
`_quality_gate()` bekerja SESUAI DESAIN (nolak gambar yang secara statistik
gak mirip foto lapangan asli, persis fungsinya).

---

Panduan ini mencerminkan status per 2026-09-05. Update §0/§2/§5 begitu ada
item yang tuntas dicek di lapangan.

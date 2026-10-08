# ARSITEKTUR SISTEM — VTOL KRTI 2026 (Misi Final)

Dokumen referensi arsitektur end-to-end: perangkat keras, alur data ROS,
seluruh node yang dipakai saat kompetisi, dan rincian teknis tiga subsistem
deteksi visual (marker WP/ArUco, gate, line-follower). Status per **2026-09-04**.
Untuk panduan operasional (checklist, command, urutan bring-up), lihat
`PANDUAN_TERBANG_FINAL.md` — dokumen ini fokus pada **bagaimana sistemnya
bekerja**, bukan **cara menjalankannya**.

---

## 1. Ringkasan

Drone VTOL (mode multirotor untuk segmen misi ini) terbang otonom lewat
rangkaian waypoint yang mengharuskan drone men-scan marker ID, drop payload,
menembus gerbang oranye berlapis, mengikuti garis putus-putus, dan mendarat
presisi — semuanya dengan verifikasi visual dari dua kamera, bukan cuma
terbang buta ke koordinat.

Prinsip arsitektur inti: **`mission_d.py` adalah satu-satunya "otak" yang
tahu urutan misi.** Ia tidak tahu cara mendeteksi gate oranye atau membaca
marker — itu tugas node-node detektor terpisah. `mission_d.py` cuma
mendengarkan topic ROS standar (`Gate`, `Line`, `ArucoMarkers`,
`WpMarkerResult`) dan mengonversi offset yang dilaporkan jadi koreksi
posisi. Pemisahan ini artinya detektor bisa diganti/dituning tanpa
menyentuh state machine, dan sebaliknya.

---

## 2. Perangkat Keras

| Komponen | Peran |
|---|---|
| Jetson Nano B01 (4GB RAM) | Companion computer — jalankan ROS Noetic, semua node deteksi & state machine |
| Pixhawk 6C | Flight controller (ArduCopter firmware 4.2.3+, EKF3) |
| GPS + kompas | Sumber posisi utama (EKF3 SRC1) |
| MTF-01 (Matek/MicoAir) | Optical flow + rangefinder lidar jarak pendek, via MAVLink native di SERIAL5 — fallback posisi EKF3 SRC2 saat GPS lemah |
| Kamera depan (C310, `/dev/video0`) | Deteksi gate (Single + Double/Triple) |
| Kamera bawah (C920, `/dev/video1`) | Deteksi marker WP1/WP2/WP3, line-follower |
| Servo drop payload | Channel PWM (8 di jalur AUTO, 9 di jalur GUIDED — belum dikonfirmasi fisik channel mana yang benar, lih. `PANDUAN_TERBANG_FINAL.md` §2.4) |
| Radio telemetri | TELEM2 = SERIAL2 Pixhawk, 921600 baud, ke Jetson via `/dev/ttyTHS1` |

Navigasi memakai **GPS + optical flow** (bukan T265/VIO — keputusan tim
2026-09-03). `VISO_TYPE=0`; T265 (kalau masih terpasang secara fisik) tidak
lagi ikut fusi EKF, murni non-aktif.

---

## 3. Arsitektur Perangkat Lunak — Alur Data

```
                          ┌─────────────────────┐
                          │   Pixhawk 6C (FC)    │
                          │  ArduCopter + EKF3    │
                          └──────────┬────────────┘
                                     │ MAVLink (/dev/ttyTHS1, 921600)
                          ┌──────────▼────────────┐
                          │   mavros (apm.launch)  │  <-- topic ROS standar:
                          │                         │      /mavros/state
                          └──────────┬──────────────┘      /mavros/local_position/pose
                                     │                      /mavros/setpoint_position/local
                                     │                      /mavros/cmd/*
              ┌──────────────────────┼───────────────────────────┐
              │                      │                           │
    ┌─────────▼─────────┐  ┌─────────▼──────────┐      ┌─────────▼─────────┐
    │  camera_front.launch│  │  camera_down.launch │      │   web_dashboard.py │
    │  (C310, gate)        │  │  (C920, marker/line) │      │  (monitoring, Flask)│
    └─────────┬─────────┘  └─────────┬──────────┘      └────────────────────┘
              │/camera_front/image_raw│/camera_down/image_raw
    ┌─────────┴──────────┐  ┌─────────┴──────────────────────┐
    │  gate_node.py        │  │  wp_marker_node.py               │
    │  (Single Gate)        │  │  (scan/drop WP1/WP2/WP3)          │
    │  gate_multi_node.py   │  │  line_node.py                     │
    │  (Double/Triple Gate) │  │  (line-follower WP3->WP4)          │
    └─────────┬──────────┘  └─────────┬──────────────────────┘
              │ Gate.msg               │ WpMarkerResult.msg / ArucoMarkers.msg / Line.msg
              └───────────┬────────────┘
                           ▼
                 ┌───────────────────┐
                 │   mission_d.py      │  <-- STATE MACHINE, satu-satunya
                 │   (Fase D)           │      yang tahu urutan leg misi
                 └──────────┬──────────┘
                             │ /mavros/setpoint_position/local, servo DO_SET_SERVO
                             ▼
                        (balik ke FC)
```

Semua node detektor **independen** satu sama lain — tidak saling
subscribe, semua cuma baca gambar kamera dan publish hasil ke topic
sendiri. `mission_d.py` yang menyatukan.

---

## 4. Node ROS — Inventaris Lengkap (misi final)

| Node | Script | Kamera | Publish (topic inti) | Dipakai untuk |
|---|---|---|---|---|
| `mission_d` | `mission_d.py` | — | `~status` (String), `/mavros/setpoint_position/local` | State machine misi, retry-dari-WP, kontrol servo |
| `mavros` | (paket `mavros`) | — | `/mavros/*` | Jembatan MAVLink ↔ ROS |
| `gate_node` | `gate_node.py` | depan | `~gate` (`Gate`) | Traverse **Single Gate** (`gate_single_final`) |
| `gate_multi_node` | `gate_multi_node.py` | depan | `~gate` (`Gate`) | Traverse **Double/Triple Gate** (`gate_double`, `gate_triple`) |
| `wp_marker_node` | `wp_marker_node.py` | bawah | `~result`, `~markers` (`ArucoMarkers`), `~wp2_present` | Scan WP1/WP3 (ID), drop WP2 (presence) |
| `line_node` | `line_node.py` | bawah | `~line` (`Line`) | Line-follower WP3→WP4 |
| `web_dashboard` | `web_dashboard.py` | depan+bawah (MJPEG) | HTTP `:5000` | Monitoring — bukan bagian alur kontrol |

**Tidak dipakai di misi final** (ada di repo, untuk mode lain/pengembangan):
`aruco_node.py` (ArUco generik `DICT_7X7_50` — hanya dipanggil *internal*
oleh `wp_marker_node.py` sendiri sebagai cross-check WP1/WP3, bukan node
terpisah), `sim_markers.py` (mock untuk SITL saja, `sim_markers:=false` di
lapangan), `mission_auto.py`/`mission_node.py` (jalur AUTO alternatif, tidak
dipakai jalur GUIDED/`mission_d.py`).

Semua node detektor mengikuti pola yang sama: **script inti OpenCV murni
tanpa ROS** (`*_detector.py`, bisa dites via `--selftest`/`--image` tanpa
roscore) + **wrapper ROS tipis** (`*_node.py`, subscribe gambar → panggil
detector → publish message). Ini sengaja — logika deteksi bisa diuji dan
dituning offline dengan dataset foto, tanpa perlu drone/simulator hidup.

---

## 5. Alur Misi (`mission_final.yaml`, 9 leg)

| # | WP | Aksi | Detektor |
|---|---|---|---|
| 1 | wp1 | `scan` id=1 | `wp_marker_node` (cv2.aruco) |
| 2 | gate_double | `traverse` 2 lapis | `gate_multi_node` |
| 3 | wp2 | `drop` id=2 | `wp_marker_node` (presence) |
| 4 | wp2 | `yaw` 90° kiri | — |
| 5 | gate_triple | `traverse` 3 lapis | `gate_multi_node` |
| 6 | wp3 | `scan` id=3 | `wp_marker_node` (cv2.aruco) |
| 7 | wp4 | `line_follow` | `line_node` |
| 8 | gate_single_final | `traverse` 1 lapis | `gate_node` |
| 9 | wp5 | `land` | — |

Dikonfirmasi resmi panitia (2026-09-03): line-follower ADA, di segmen
WP3→WP4 (bukan WP4→WP5 seperti dugaan awal). Setiap leg didahului `GOTO`
otomatis ke koordinat WP (kecuali `line_follow`, yang menempuh jaraknya
sendiri dengan koreksi visual — lihat §9).

**Retry dari WP manapun**: `mission_d.py` menerima `~start_wp` (rosparam)
— memotong daftar leg ke leg pertama yang cocok, termasuk auto-takeoff dari
titik itu. Divalidasi SITL berulang kali (lihat `PANDUAN_TERBANG_FINAL.md`
§10).

---

## 6. Pipeline Koordinat GPS

```
waypoints_latlon.yaml  (lat/lon PERMANEN, sumber kebenaran)
        │  latlon_to_waypoints.py (dijalankan tiap sesi, SETELAH GPS fix)
        │  baca /mavros/global_position/gp_origin (EKF origin aktif)
        ▼
waypoints.yaml  (ENU lokal meter, dibangkitkan ulang tiap sesi)
        │  dibaca langsung
        ▼
mission_d.py  (TIDAK PERNAH baca lat/lon — source-agnostic)
```

Kenapa dua lapis: EKF origin di-set otomatis oleh fix GPS pertama tiap boot
FC, TIDAK sama persis tiap sesi. Menyimpan lat/lon asli (bukan ENU) berarti
titik WP tidak perlu disurvei ulang tiap kali origin bergeser — cukup
konversi ulang di awal sesi. Detail lengkap + prosedur survei lapangan: lih.
`PANDUAN_TERBANG_FINAL.md` §3.

---

## 7. Detail: Deteksi Marker WP (ArUco/scan/drop)

**File**: `wp_marker_node.py` (wrapper ROS) → `wp_decode_v2.py` (decoder
utama) + `aruco_detector.py` (cross-check cv2.aruco) + `wp_marker_detector.py`
(presence WP2, dari `wp_decode_audit.py`).

### Arsitektur decoder ganda (per WP beda strategi)

| WP | Metode | Alasan |
|---|---|---|
| WP1 | `cv2.aruco` (`DICT_7X7_50`, invert=True) | Keputusan eksplisit user 2026-08-09: robust dari jarak jauh, meski recall lab HSV lebih tinggi (8.4% vs 4.2%) |
| WP2 | `detect_wp2_presence()` — deteksi *presence* blob non-oranye/non-merah, BUKAN baca ID | WP2 perannya konfirmasi lokasi drop, bukan identifikasi |
| WP3 | `cv2.aruco` juga (sama seperti WP1) — trade-off recall besar (31.9%→0% di tes HSV vs cv2.aruco) DISADARI & diterima user | Konsistensi dgn WP1, observasi lapangan: marker fisik kadang lebih mudah kebaca cv2.aruco dari jauh |
| WP4 | HSV robust (`decode_frame_v2_robust`) — TIDAK dipakai di jalur final (WP4 bukan scan leg lagi) | Dipertahankan untuk kompatibilitas mode lain |

Urutan coba per-frame: cv2.aruco (WP1/WP3) dulu → kalau nihil, HSV robust
(WP4 saja).

### Pipeline decode HSV/grid (WP4, dan basis historis WP1/WP3)

1. `marker_mask_v2()` — segmentasi HSV: cari **terpal oranye** (bukan
   marker langsung!) — marker = komplemen ("bukan-oranye") dari mask itu.
   Alasan: segmentasi warna putih langsung memecah badan marker jadi
   fragmen-fragmen kecil di sel hitam grid.
2. `largest_component()` — ambil blob "bukan-oranye" terbesar (badan
   marker + ekor arah).
3. `split_core_protrusion()` — pisahkan badan utama (grid ID) dari
   "ekor" (indikator arah, Task 4 heading-align).
4. `_quality_gate()` — tolak kalau area core < `MIN_CORE_AREA_PX` (5000px)
   atau rata-rata grayscale core < `MIN_CORE_BRIGHTNESS` (100) — dikalibrasi
   dari distribusi foto lapangan asli (marker "hitam" di foto nyata tidak
   pernah RGB murni 0 karena pantulan ambient).
5. `corners_small()` + warp perspektif → grid bit 6×6.
6. `match_bits_to_refs()` — rotation-search: cocokkan ke-4 rotasi bit
   terhadap tiap pola referensi (`wp_marker_reference_v2.json`), ambil
   hamming distance terkecil. `reason='ok'` kalau di bawah
   `hamming_threshold` (7) DAN kandidat kedua-terdekat cukup jauh
   (`ambiguous_margin`, 3) — mencegah salah pilih ID pas dua kandidat mepet.

### Agregasi temporal — LatchAggregator

Drone hover beberapa detik saat scan (`scan_timeout` di `mission_d.py`).
Alih-alih majority-vote per-frame (dicoba, dibatalkan — sinyal bersih
sering < mayoritas dari total frame, malah teredam noise), dipakai **OR
sederhana**: begitu 1 frame `reason='ok'`, ID itu **ditahan** (`~latch_sec`,
default 2.0s) supaya tidak "berkedip" balik ke -1 di antara dua bacaan ok
yang berdekatan. Tidak pernah lebih buruk dari deteksi single-frame terbaik.

### CLAHE fallback (opsional, mahal)

Kalau baseline gagal dengan alasan tertentu (`no_match`/`no_corners`/
`decode_failed`/`ambiguous` — BUKAN `low_quality`/`no_marker`, terbukti 0%
rescue), coba lagi dengan peningkatan kontras lokal (CLAHE). ~91% lebih
lambat/frame — hanya aktif selama `~expected_id` cocok whitelist
(`~clahe_fallback_wp_ids`, default cuma WP1; WP3/WP4 dikecualikan karena
pernah menyumbang salah-tebak di validasi skala penuh).

### Validasi akurasi

Divalidasi dataset foto lapangan asli (bukan cuma unit test): WP1 30.4%,
WP3 83.4% (presisi 100% di kedua angka itu — kalau `reason='ok'`, hampir
pasti benar; recall rendah ≠ salah baca, cuma kadang tidak konfiden).
WP1 secara struktural paling sulit (kontras rendah, sering perlu banyak
percobaan sebelum satu frame `ok`). Detail lengkap tuning: lih.
`WP_MARKER_DECODER_V2.md`.

---

## 8. Detail: Deteksi Gate

Dua detektor **independen total**, algoritma berbeda, dipilih per-leg lewat
field `gate_source` di `mission_*.yaml` (`"1"`=Single, `"2"`=Double/Triple —
lihat §2.7 `PANDUAN_TERBANG_FINAL.md` untuk kronologi kenapa ini penting:
sebelumnya cuma ada 1 pilihan global, salah satu leg akhirnya kebaca
detektor yang salah topologi).

### 8.1 Single Gate — `gate_node.py` / `gate_detector.py`

Struktur fisik: **dinding oranye solid dengan satu lubang, terbuka di
bawah** (bukan dinding tertutup 4 sisi — dikonfirmasi lewat kode self-test
`_make_synthetic()` node ini sendiri saat mengejar bug di sesi uji Gazebo).

Algoritma **profil kolom**:
1. Mask HSV oranye (H:8-30, S/V:80-255) → kontur terbesar → bounding box.
2. Ambil band 35%-100% tinggi bbox (skip bagian atas yang berisi palang).
3. Hitung rata-rata cakupan oranye per KOLOM piksel dalam band itu.
4. Kolom dengan cakupan rendah (`open_col_thresh`, 0.20) = kandidat bukaan.
5. Gabungkan kolom-kolom bersebelahan jadi "gap"; filter gap terlalu
   sempit (`min_gap_frac`/`min_open_img_frac`).
6. Ambil gap terlebar sebagai target; `off_x/off_y` = posisi tengah gap
   relatif pusat gambar.

### 8.2 Double/Triple Gate — `gate_multi_node.py` / `gate_multi_detector.py`

Struktur fisik: bentuk-**U** (2 kaki + palang atas, **tanpa** palang bawah
— topologi beda total dari Single Gate, ditulis dari nol, tidak berbagi
kode dengan `gate_detector.py`).

Algoritma **komponen background**:
1. Mask HSV oranye (H:5-25, S>40 — lebih toleran dari Single Gate untuk
   variasi overexposure) → kontur terluar terbesar.
2. Karena topologi-U terbuka bawah, bukaan **tersambung ke background**
   lewat celah bawah → tidak ada lubang tertutup untuk dicari via
   hierarchy child. Solusi: bukaan = komponen background (mask terbalik)
   **terhubung terbesar** di dalam bounding box kontur luar
   (`cv2.connectedComponentsWithStats`).
3. Solidity adaptif-skala: gate Triple (lebih dalam, dinding interior ikut
   oranye) punya solidity jauh lebih tinggi dari Double di jarak sama —
   threshold solidity dilonggarkan khusus kandidat besar+aspect lebar,
   supaya tidak salah tolak Triple tanpa meloloskan false-positive Double.
4. `off_x/off_y` dari pusat bounding-box komponen background (bukan
   centroid piksel mentah — bisa bias ke objek terang/gelap di background).

**Gate berlapis**: `gate_layers` (di `mission_*.yaml`) mengulang
"center bukaan → maju pelan lewati bidang" per lapis. Sinyal pindah lapis =
`area_frac` naik-lalu-turun (drone mendekat → oranye membesar → tembus →
mengecil) — BUKAN jarak/waktu tetap. Logika ini ada di `mission_d.py`
(`_traverse()`), sama untuk kedua jenis gate — cuma sumber `Gate` message-nya
yang beda (`gate_source`).

### Validasi akurasi

Dataset foto lapangan asli: Double 86.5%, Triple 100% (5 teknik berlapis:
adaptive solidity, skew-correction, dll — lih.
`GATE_DETECTOR_DOUBLE_TRIPLE.md`). ⚠️ Overfitting dikonfirmasi (baru 1 sesi
foto) — belum diuji dataset/gerbang fisik lain.

---

## 9. Detail: Line-Follower (WP3→WP4)

**File**: `line_node.py` (wrapper) → `line_detector.py` (`LineDetector`).

Struktur fisik: garis putus-putus dari tarp abu-abu **glossy** (bukan
matte-hitam). Riwayat penting: versi pertama pakai threshold Value (V)
saja, GAGAL — V tarp (median ~128-166) tumpang-tindih V rumput kering
(~120-125) karena tarp memantulkan cahaya matahari. Analisis foto
menemukan **Hue** jauh lebih diskriminatif: H tarp median 103 (rentang
87-107) vs H rumput median 26 (rentang 18-49) — margin bersih 38 poin.
Sekarang pakai HSV penuh (H:87-107, S:35-255, V:0-255), pola sama seperti
`gate_detector.py`.

⚠️ Kalibrasi HSV ini dari dataset rumput **kering/coklat** lokal — belum
tentu berlaku di venue kompetisi (rumput hijau bisa menggeser H rumput
mendekati tarp). `~hsv_lo2/hi2` disediakan sebagai slot fallback venue
kedua, kosong by default — WAJIB divalidasi dengan sample foto venue asli
sebelum lomba, jangan diisi tebakan.

Algoritma:
1. Mask HSV → kontur segmen dash terbesar yang cukup besar
   (`min_area_frac`, 0.01).
2. `off_x/off_y` = offset centroid segmen dari pusat gambar.
3. `angle_deg` = arah dash (fit garis, ambigu 180°) — dipakai
   `mission_d.py` untuk estimasi arah jalur, bukan cuma posisi lateral.

Kontrol di `mission_d.py` (`_line_follow()`): pola SAMA seperti align
ArUco/gate (`align_sign_x/y`, `swap_xy`, `gain`, `max_drift` — reuse, bukan
reinvent). **Bug penting yang pernah ditemukan & diperbaiki** (SITL,
2026-08-29): fungsi ini sebelumnya kena "pre-goto" generik yang langsung
menerbangkan drone buta ke target SEBELUM koreksi visual sempat jalan, dan
pengecekan "sudah sampai" sempat membandingkan ke titik WP asli alih-alih
titik terkoreksi yang sedang benar-benar dikejar — keduanya sudah
diverifikasi ulang via SITL.

---

## 10. Keamanan Penerbangan

- **`_healthy()`** (dicek tiap loop tick, ~20Hz, di semua primitive
  goto/scan/align/traverse/line_follow/yaw): mode harus GUIDED, armed,
  pose EKF harus segar (`pose_stale_sec`). Kalau mode berubah (pilot ambil
  alih manual), misi berhenti dalam <1 tick **tanpa** memaksa mode balik —
  tidak melawan keputusan pilot. Divalidasi SITL (§8 `PANDUAN_TERBANG_FINAL.md`).
- **`max_recede`**: abort kalau drone menjauh dari titik terdekat yang
  pernah dicapai selama `_goto()` (indikasi drift EKF/vision).
- **`goto_timeout`/`gate_timeout`/`scan_timeout`**: batas waktu tiap fase,
  dengan fallback ke geometri WP polos kalau visi tidak menemukan apa-apa
  (bukan macet selamanya).
- **Retry-dari-WP** (`~start_wp`): recovery cepat tanpa mengulang seluruh
  misi kalau terjadi crash/abort.

---

## 11. Monitoring — `web_dashboard.py`

Server Flask + MJPEG (ganti VNC — lebih ringan di link radio terbatas).
Subscribe: kedua kamera (stream MJPEG langsung, tanpa decode ulang di
browser), `~status` dari `mission_d.py` (WP sekarang), `/mavros/state`
(mode/armed), `/mavros/battery`, `/mavros/gpsstatus/gps1/raw`,
`/mavros/distance_sensor/rangefinder_pub` (ToF), `/mavros/local_position/pose`.
Optical flow sengaja tidak ditampilkan (FC kirim MAVLink `OPTICAL_FLOW`
id 100, mavros cuma dukung `OPTICAL_FLOW_RAD` id 106 — perlu bridge custom
yang belum dibangun).

---

## 12. Referensi File

| File | Peran |
|---|---|
| `scripts/mission_d.py` | State machine misi (Fase D) |
| `scripts/mavros_helper.py` | Wrapper koneksi mavros (arm/mode/takeoff/setpoint/servo) |
| `scripts/gate_detector.py` / `gate_node.py` | Single Gate |
| `scripts/gate_multi_detector.py` / `gate_multi_node.py` | Double/Triple Gate |
| `scripts/wp_decode_v2.py`, `aruco_detector.py`, `wp_marker_detector.py` / `wp_marker_node.py` | Decoder WP1/2/3 |
| `scripts/line_detector.py` / `line_node.py` | Line-follower |
| `scripts/web_dashboard.py` | Dashboard monitoring |
| `config/mission_final.yaml` | Definisi 9 leg misi final |
| `config/waypoints.yaml` / `waypoints_latlon.yaml` | Koordinat WP (ENU / lat-lon permanen) |
| `config/wp_marker_reference_v2.json` | Pola referensi grid ID marker |
| `launch/phaseD_mission.launch` | Entry point misi, merangkai semua node di atas |
| `docs/PANDUAN_TERBANG_FINAL.md` | Panduan operasional (checklist, command, blocker) |
| `docs/WP_MARKER_DECODER_V2.md`, `GATE_DETECTOR_DOUBLE_TRIPLE.md` | Riwayat tuning detail per detektor |

---

Dokumen ini mencerminkan status arsitektur per 2026-09-04. Update kalau ada
perubahan struktur node/pipeline (bukan sekadar tuning parameter — untuk itu
lihat dokumen riwayat masing-masing detektor).

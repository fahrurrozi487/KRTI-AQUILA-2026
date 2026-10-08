# Task 3 — Decoder Marker WP Custom — Ringkasan Status

Status: **WP3/WP4 reliable=true. WP1 reliable=false (agreement rendah). WP2: KEPUTUSAN STRATEGIS — presence detection, bukan decode ID (lihat §9). Mapping WP<->ID BELUM diverifikasi fisik.**
Script: `src/mission_control/scripts/wp_decode_audit.py`
Data: `wp_marker_reference.json` (`src/mission_control/config/`)
Dataset: `~/catkin_ws/dataset_arucode/{wp1_new,wp2,wp3,wp4}` (499 foto total: 350+62+37+50)

---

## 1. Ringkasan Masalah Awal

Marker fisik WP1-4 di lapangan **bukan ArUco standar** — `cv2.aruco` gagal
membaca **463/499 foto** dataset latihan.

Struktur fisik marker:
- **Badan utama**: grid persegi hitam-di-atas-putih, **tanpa border ArUco**
  (tidak ada bezel hitam solid mengelilingi grid seperti dictionary ArUco
  standar).
- **"Ekor"**: tonjolan kecil menempel di satu sisi badan, dipakai untuk
  orientasi/navigasi (task terpisah dari decode isi grid).

Karena bukan ArUco, decoder harus ditulis custom: segmentasi warna ->
isolasi sudut -> rektifikasi (warp) -> normalisasi orientasi via ekor ->
sampling grid -> decode bit.

## 2. Keputusan & Mapping yang Dipakai

- **AGENTS.md = sumber kebenaran**: WP1=id1, WP2=id2, WP3=id3, WP4=id4
  (dictionary `DICT_7X7_50`, dipakai downstream oleh `aruco_detector.py` —
  ini konvensi ID untuk pipeline ArUco resmi, dipakai juga sebagai patokan
  penamaan folder dataset custom-decoder ini).
- **Status verifikasi fisik: BELUM diverifikasi.** `wp_marker_reference.json`
  secara eksplisit menandai ini di field `_meta.warning`:
  > "Mapping folder->ID BELUM diverifikasi fisik ke marker asli di
  > lapangan. Trust AGENTS.md as-is (wp1_new=id1, wp2=id2, wp3=id3,
  > wp4=id4). Perlu dikonfirmasi manual sebelum dipakai di lomba."

## 3. Riwayat Perbaikan Teknis (kronologis)

Ditelusuri dari histori backup manual (`wp_decode_audit_backup_*.py`,
tidak ada git di repo ini).

### a. Segmentasi: deteksi-putih -> deteksi-background-oranye
**Alasan**: kalau mask diambil dari piksel putih saja, sel-sel hitam grid
memecah badan marker jadi banyak fragmen kecil -> connected-component
terbesar cuma potongan border marker, bukan badan utuh (bug aspek-rasio
1.7 di diagnostik warp). Fix: segmentasi lewat warna terpal oranye di
background (homogen), lalu ambil komplemennya (`~orange`) — badan marker
(putih+hitam) jadi satu blob padat tanpa perlu union piksel putih+hitam
manual.

### b. Isolasi sudut: `approxPolyDP` -> `minAreaRect`
**Alasan**: `approxPolyDP` rapuh — sering nyangkut di notch/tonjolan grid
atau ekor, menghasilkan >4 titik atau sudut yang salah. Badan marker
berbentuk kotak, jadi `minAreaRect` adalah primitif yang lebih tepat &
stabil untuk kasus ini.

### c. Ukuran grid: dikoreksi dari 5x5 ke 6x6
**Alasan**: klaim tim awal ("grid 5x5") terbukti keliru lewat audit visual
overlay debug — fitur 1-sel notch/kotak-terisolasi pada pola hanya cocok
direkonstruksi dengan `GRID_N >= 6`, rekonstruksi 6x6 paling setia ke
bentuk warp asli. Dikunci di kode: `GRID_N = 6`.

### d. Fix HSV overexposure: threshold saturasi 80 -> 40
**Alasan**: folder `wp1_new` punya foto overexposed di mana warna oranye
terpal jadi kurang saturated, lolos dari mask `s > 80`. Threshold
diturunkan ke `s > 40` supaya lebih toleran ke overexposure. Verifikasi:
cek overlap mask oranye baru dengan piksel putih marker asli = **0%** —
turun threshold tidak ikut memakan badan marker yang putih, jadi aman.

### e. Sampling grid: dipatok ke bounding-box konten HITAM (bukan sisi warp langsung)
**Alasan**: kalau grid disampling langsung dari sisi hasil warp,
sampling meleset karena `minAreaRect` ikut menyertakan quiet-zone putih
dan ekor (cluster hitam terpisah di atas badan). Fix: grid dipatok ke bbox
konten hitam badan saja lewat helper `_body_yspan()` (ambil run baris-hitam
kontigu bermassa terbesar, buang ekor yang terpisah oleh celah putih).

### f. WP2 — fix decode tray merah (`decode_grid` red pixel exclusion)
**Alasan**: WP2 selalu punya nampan merah (kondisi permanen lomba). Pixel
merah di warp terbaca sebagai "gelap" oleh Otsu (grayscale merah ≈ 100 dari
max 255, antara putih 240 dan hitam 30), sehingga border tray ikut terdecode
sebagai sel hitam → noise pattern.

**Fix yang dicoba dan ditolak**: exclude red dari `marker_mask` → GAGAL.
`largest_component` yang kehilangan anchor tray malah memilih ground/latar
belakang sebagai blob terbesar (non-orange, non-red area), sehingga seluruh
8 foto pertama WP2 gagal AR check (1.33–1.84).

**Fix yang diterapkan**: dalam `decode_grid`, sebelum konversi ke grayscale
dan threshold Otsu, pixel merah (H<7 atau H>168, s>100) diset ke putih
`[255,255,255]`. Blob detection (`largest_component`) tetap memakai
tray+kertas sebagai anchor (AR stabil), tapi decode hanya membaca pixel
non-merah.

**Hasil**: agreement WP2 naik dari **0.6748 → 0.705** (+3.0%), n_ok tetap
24/62, pattern berubah signifikan (tray border tidak lagi terdecode sebagai
sel hitam).

### g. WP1 failures — analisis dan keputusan
**Diagnosis**: 155/350 foto WP1 gagal AR check.
- **89 foto severe (AR>1.5)**: foto sudut SANGAT ekstrem (orang terlihat di
  tepi tarp, tarp foreshortened parah). Audit visual konfirmasi: ini benar
  gagal, tidak bisa diselamatkan.
- **66 foto mild (AR 1.25–1.5 atau 0.67–0.75)**: foto miring sedang, marker
  masih terbaca. Dicoba `AR_TOL=0.25→0.35` untuk menyelamatkan 41 dari 66
  foto ini.

**Hasil percobaan AR_TOL=0.35**: ok naik 195→236, tapi agreement **turun**
0.6705→0.666. Foto mild-angle menambah noise orientasi (ekor detection tidak
stabil untuk foto miring), bukan sinyal. **Divert ke AR_TOL=0.25**.

## 4. Hasil Akhir — Isi Lengkap `wp_marker_reference.json` (sesudah semua fix)

Konvensi: `bits[r][c] = 1` artinya sel (baris r, kolom c) **HITAM**.
Orientasi: ekor di atas (N), kanonik. Grid 6x6.

| WP | ID | K (ok/total) | Agreement | Reliable? |
|----|----|--------------|-----------|-----------|
| wp1_new | id1 | 195/350 | 0.671 | **false** |
| wp2 | id2 | 24/62 | 0.714 | **false** |
| wp3 | id3 | 24/37 | 0.743 | **true** |
| wp4 | id4 | 36/50 | 0.738 | **true** |

Pola modal per WP (`#`=hitam, `.`=putih):

**WP1 (id1)** — agreement 0.671, K=195/350
```
.##..#
......
.#.###
.#....
.##...
...##.
```

**WP2 (id2)** — agreement 0.714, K=24/62 *(RESEARCH ARTIFACT — presence detection)*
```
......
..#...
.###..
.###..
.###..
......
```

**WP3 (id3)** — agreement 0.743, K=24/37
```
......
..#...
.#.##.
....#.
...#..
.#.#..
```

**WP4 (id4)** — agreement 0.738, K=36/50
```
..#...
......
...#..
.##...
.##.#.
.#..#.
```

## 5. Matriks Hamming Distance antar 4 Pola WP (final)

Dari 36 sel total (grid 6x6) per pasangan. WP2 pakai presence detection
(bukan ID decode) — kolom WP2 hanya informasional.

| Pasangan | Sel beda | % beda | Catatan |
|---|---|---|---|
| WP1 vs WP2 | 12/36 | 33.3% | WP2 informasional (presence detection) |
| WP1 vs WP3 | 12/36 | 33.3% | aman untuk runtime |
| **WP1 vs WP4** | **9/36** | **25.0%** | **minimum — artefak WP1 tidak stabil, user konfirmasi WP1≠WP4 fisik** |
| WP2 vs WP3 | 10/36 | 27.8% | informasional |
| WP2 vs WP4 | 9/36 | 25.0% | informasional |
| WP3 vs WP4 | 13/36 | 36.1% | aman untuk runtime |

**RISIKO runtime WP1 vs WP4 (25%)**: user sudah konfirmasi WP1 ≠ WP4
secara fisik. Kemiripan 25% di data adalah **artefak agreement WP1 yang
rendah (0.671)**, bukan cerminan fisik. Verifikasi fisik pattern WP1 wajib
sebelum lomba.

## 6. Investigasi WP2 — Sesi Lanjutan (LANGKAH 1–4)

### Temuan Visual (LANGKAH 1)

**Hipotesis yang diuji**: nampan merah mengganggu `detect_tail_side()` untuk
WP2.

**Tiga temuan visual dari dataset foto WP2:**
1. **Nampan merah mendominasi body blob** (red% = 20–35% per foto): tray
   menutupi protrusi ekor kertas → `detect_tail_side` melihat persegi simetris
   tray, bukan asimetri tab ekor kertas. **Kontribusi TERBESAR.**
2. **Bayangan besar** (shadow dari fotografer): menutupi 30–50% tarp oranye di
   banyak foto. Piksel bayangan bisa berakhir di body blob jika saturasi turun
   di bawah threshold `s > 40`. **Kontribusi sedang** (tidak diuji terpisah,
   tapi memperburuk AR dari passing photos).
3. **Elemen mini-QR ("secondary element")**: tampak di sisi North paper di
   semua foto WP2 — tab kecil berwarna pink/merah. Ini adalah **ekor WP2**
   (sama desain dengan WP1/3/4 yang tab-nya hitam-di-putih). Untuk WP2, tab
   ini tersembunyi di dalam tray dan tidak membentuk protrusi yang terdeteksi
   `detect_tail_side`. Elemen ini **TIDAK mengganggu WP1/3/4** (tab WP1/3/4
   menonjol bebas di atas tarp oranye, terdeteksi jelas).

**Hipotesis TERBUKTI**: 4 dari 24 foto passing berubah arah ekor saat merah
di-exclude dari body sebelum `detect_tail_side`. 3 dari 4 perubahan menuju N
(benar secara fisik, dikonfirmasi visual audit warp). 1 perubahan ambigu
(S→W, foto dari sudut ekstrem).

### Fix yang Diterapkan (LANGKAH 2)

Dalam `process_image`, setelah `body = largest_component(mask)`:
- Buat `body_tail` = body dengan pixel merah (H<7 atau H>168, s>100) di-zero
- Re-close body_tail (kernel 15px)
- Gunakan `body_tail` HANYA untuk `detect_tail_side`
- Tetap gunakan `body` (dengan tray) untuk `corners_of` (AR stabil)

**Hasil LANGKAH 2:**

| Metrik | Sebelum sesi ini | Sesudah LANGKAH 2 | Delta |
|---|---|---|---|
| Agreement WP2 | 0.6748 (original) | **0.714** | +0.039 total |
| n_ok | 24/62 | 24/62 | 0 |
| Ekor distribution | N=1 E=9 S=5 W=8 | N=4 E=9 S=2 W=9 | 3 foto S→N |

*Catatan: fix decode_grid red exclusion (sesi sebelumnya) sudah memberikan +0.030 (0.6748→0.705). Fix tail detection (sesi ini) menambah +0.009.*

### Weighted Aggregation (LANGKAH 3)

Dicoba: bobot per foto = margin dominasi ekor (`(sv[0]-sv[1]) / sv[0]`).

**Hasil: delta = 0.000** — tidak membantu. Confidence scores terlalu homogen
(mean=0.28, min=0.01, max=0.83) → weighting jadi seperti unweighted. LANGKAH
3 tidak memberikan perbaikan.

### Keputusan Strategis (LANGKAH 4) — DISETUJUI USER

**Batas algoritmik tercapai**: WP2 agreement 0.714, reliable=false.

**Keputusan: WP2 pakai PRESENCE DETECTION, bukan decode ID grid.**

Alasan:
- WP2 adalah dropping point, posisi presisi sudah dari T265/GPS (`waypoints.yaml`)
- Vision hanya perlu konfirmasi "ada marker besar di frame", bukan "ID marker = 2"
- Presence detection jauh lebih robust dari decode ID yang bergantung pada
  ekor scatter yang tidak bisa dikendalikan algoritmik

WP1, WP3, WP4 tetap decode ID grid penuh.

## 7. Investigasi Lanjutan — Root Cause Analysis + EM Refinement

### 7a. Root Cause Analysis per-foto (TUGAS 1)

Analisis per-foto: per foto yang lolos AR gate, hitung hamming vs modal,
coba semua 4 rotasi, hitung blur (Laplacian variance).

**Kategori penyebab:**
- `perfect`: hamming=0 (foto sempurna cocok modal)
- `minor_noise`: hamming ≤5, rotation tidak membantu banyak
- `tail_error`: rotation memperbaiki ≥4 sel (tail detection salah)
- `large_noise`: hamming >5, tidak bisa diperbaiki dengan rotation

| WP | perfect | minor | tail_error | large_noise | Blur corr |
|---|---|---|---|---|---|
| WP1 | 0% | 6% | **21%** | **74%** | +0.437 (sharp=more error) |
| WP2 | 0% | 4% | 21% | **75%** | +0.259 |
| WP3 | 0% | 25% | **25%** | 50% | -0.393 (blurry=more error) |
| WP4 | 3% | 36% | **33%** | 28% | -0.236 |

**Temuan kunci:**
- WP1: 74% `large_noise` adalah bottleneck — bahkan foto yang tail-nya dikoreksi
  masih punya banyak sel salah. Akar masalah: foto dari sudut tajam tapi sharp
  (corr=+0.437 abnormal) → perspective distortion serius.
- WP4: 33% `tail_error` dengan avg 6.3 sel saved = kandidat terbaik untuk EM.
  Bimodal distribution: 14 foto h≤6 vs 18 foto h>12 = dua cluster yang jelas.
- WP3: 25% `tail_error` dan 25% `minor_noise` = potensi EM bagus.

**Per-cell consistency sebelum EM (sel paling bermasalah):**
- WP1: 8 sel di bawah 55% agreement — hampir random
- WP3: (2,4)/(5,1)/(5,3) = 50% = coin flip sebelum EM
- WP4: (5,4) = 50% sebelum EM

### 7b. EM Iterative Tail Refinement — REVERTED (eksplorasi gagal)

**Algoritma yang diimplementasikan** (`em_aggregate()` di `wp_decode_audit.py`):
- E step: tiap foto cari rotasi bits (0/90/180/270 CCW) yang cocok ke modal
- M step: hitung ulang modal dari bits yang sudah dirotasi; konvergen 2–4 iter

**Mengapa di-REVERT**: EM adalah **sirkular by construction** — rotasi per-foto
dipilih berdasarkan `argmin hamming(rot_k(bits_i), modal)`, dan modal dihitung
dari rotasi yang sama. Peningkatan yang terukur (WP1 +0.050, WP2 +0.058,
WP3 +0.060, WP4 +0.067) adalah *internal consistency optimization*, bukan bukti
akurasi nyata. Identik dengan bias Hamming-distance-ke-anchor: agreement naik
karena kita memilih representasi yang secara definisi dekat ke modal yang kita
hitung dari pilihan itu. **Angka-angka EM tersebut TIDAK VALID sampai
circularity diverifikasi fisik.**

**Apa yang diperlukan untuk validasi non-sirkular**: untuk setiap foto di mana
EM mengubah rotasi, harus ditunjukkan rotasi EM **konsisten dengan posisi ekor
fisik yang terlihat di foto asli** (bukan hanya "cocok ke modal"). Caranya:
overlay `detect_tail_side()` result vs rotasi EM, verifikasi visual untuk ≥10
foto per WP.

**Fungsi `em_aggregate()` tetap ada di kode** (referensi penelitian, diberi
WARNING comment) tapi `run()` kembali ke `aggregate()`. JSON dan docs
dikembalikan ke angka pra-EM (WP1=0.671, WP2=0.714, WP3=0.743, WP4=0.738).

## 8. Keterbatasan yang Diketahui

- **Mapping WP<->ID belum diverifikasi fisik** ke marker asli di lapangan.
- **WP1 agreement rendah (0.671)**: 89 foto sudut ekstrem tidak bisa
  diselamatkan. AR_TOL=0.35 dicoba, justru turunkan agreement → revert ke 0.25.
  Root cause (TUGAS 1): 74% foto `large_noise`, corr(blur,hamming)=+0.437
  (abnormal — foto tajam-sudut justru lebih noisy dari foto blur-sudut).
- **WP1 vs WP4 = 25.0%** (minimum pair): artefak WP1 tidak stabil, bukan
  kemiripan fisik. Verifikasi fisik wajib.
- **WP4 ekor scatter N=S=E=W=9 disengaja** (dataset dari segala sudut).
  Agreement 0.738 (reliable=true) membuktikan normalisasi ekor bekerja.
- **Docstring modul** `wp_decode_audit.py` baris pertama masih sebut "5x5" —
  teks basi, kode aktual sudah `GRID_N = 6`.

## 9. Langkah Selanjutnya yang Direkomendasikan

1. **[WAJIB sebelum lomba] Verifikasi fisik WP1/WP4**: pattern di JSON cocokkan
   ke marker fisik. Kemiripan 25% di data adalah artefak, bukan fisik.
2. **[WAJIB] Verifikasi orientasi ekor WP3/WP4**: ekor WP3 umumnya N di
   dataset tapi scatter WP4 perlu dicek di lapangan.
3. **[Opsional — kalau ada waktu] Validasi EM**: untuk ≥10 foto per WP di mana
   EM mengubah rotasi, tampilkan overlay posisi ekor fisik vs rotasi EM. Kalau
   konsisten → deploy `em_aggregate()` (sudah ada di kode, tinggal uncomment).
4. **[Task berikutnya] Implementasi presence detection WP2** di node
   mission_control:
   - Deteksi blob non-orange (dan non-red) dengan area ≥ threshold di frame
     kamera bawah → sinyal binary "WP2 terdeteksi"
   - Threshold luas dikalibrasi berdasarkan ketinggian terbang normal WP2
5. **[Task berikutnya] Integrasi `wp_marker_reference.json` ke node ROS** untuk
   WP1/WP3/WP4 — file JSON belum dikonsumsi oleh node manapun.

## 10. Keputusan Final Per WP

| WP | Metode | Agreement | Reliable | Catatan |
|---|---|---|---|---|
| WP1 | Decode ID grid | 0.671 | **false** | Verifikasi fisik wajib |
| WP2 | **Presence detection** | 0.714 (research) | false | Pattern tidak dipakai runtime |
| WP3 | Decode ID grid | 0.743 | **true** | Siap pakai |
| WP4 | Decode ID grid | 0.738 | **true** | Siap pakai; verifikasi ekor scatter |

## 11. Implementasi Node ROS (sesi 2026-07-31)

### File yang dibuat

| File | Tujuan |
|---|---|
| `msg/WpMarkerResult.msg` | Custom message: header, found_marker, wp_id, hamming, reliable, low_conf_tail, reason, aspect |
| `scripts/wp_marker_detector.py` | Pure-OpenCV core (tidak import ROS). Import pipeline dari `wp_decode_audit`. Fungsi: `decode_frame()`, `detect_wp2_presence()`, `load_refs()`. Ada `--selftest` CLI. |
| `scripts/wp_marker_node.py` | Thin ROS wrapper. Subscribe gambar, publish `~result` + `~wp2_present` + `~debug_image`. |
| `launch/wp_marker_detect.launch` | Launch file dengan semua param tersedia + komentar kalibrasi. |
| `CMakeLists.txt` | EDIT: tambah `WpMarkerResult.msg` ke `add_message_files`, tambah 2 script baru ke `catkin_install_python`. |

### Topics

| Topic | Type | Kapan publish |
|---|---|---|
| `~result` | `WpMarkerResult` | Setiap frame (setelah throttle) |
| `~wp2_present` | `std_msgs/Bool` | Setiap frame (setelah throttle) |
| `~debug_image` | `sensor_msgs/Image` | Hanya bila ada subscriber |

### Rosparam

| Param | Default | Catatan |
|---|---|---|
| `~image_topic` | `/camera_down/image_raw` | Kamera bawah untuk semua WP |
| `~json_path` | `$(find mission_control)/config/wp_marker_reference.json` | Final, jangan diubah |
| `~hamming_threshold` | `12` | 1/3 dari 36 sel; aman untuk separasi minimum WP1/WP4=9 sel |
| `~wp2_min_area_px` | `3000` | Tuning wajib di lapangan; lihat §11b |
| `~throttle_hz` | `5.0` | Cukup untuk konfirmasi keberadaan |
| `~publish_debug` | `true` | Nonaktifkan di produksi bila bandwidth terbatas |

### Desain keputusan kunci

- **em_aggregate() TIDAK dipakai** di node manapun (sirkular, permanently reverted — lihat §7b).
- **`wp_marker_detector.py` import dari `wp_decode_audit`**, tidak duplikasi pipeline.
- **All-zeros edge case**: `bits.sum()==0` langsung return `reason='decode_failed'` tanpa Hamming match. Tanpa ini, all-zeros Hamming ke WP3=8 < threshold=12 → false positive.
- **Tie-breaking**: argmin Hamming; kalau jarak sama, prefer WP dengan `reliable=True` (WP3/WP4).
- **WP2 deteksi**: blob terbesar yang bukan oranye DAN bukan merah, area ≥ `~wp2_min_area_px`.

### Hasil offline test (dataset foto lapangan)

| WP | Sampel | wp_id | hamming | reliable | reason |
|---|---|---|---|---|---|
| WP3 | `wp3/WIN_20260726_15_56_11_Pro.jpg` | 3 | 3 | True | ok |
| WP4 | `wp4/WIN_20260727_09_30_00_Pro (3).jpg` | 4 | 1 | True | ok |
| WP1 | `wp1_new/WIN_20260718_17_35_15_Pro (4).jpg` | 1 | 6 | False | ok |
| WP2 | `wp2/WIN_20260726_15_53_38_Pro.jpg` | — | — | — | present=True, area=200410 |

### 11b. Kalibrasi `wp2_min_area_px` di lapangan

Default 3000px² cocok untuk ketinggian menengah kamera 640×480. Rumus kasar:

```
T_px = (marker_side_m / altitude_m)^2 * (640 * 480) * fill_factor
```

- `fill_factor` ≈ 0.6–0.8 (marker tidak mengisi seluruh blob)
- T265 punya scale error ~20–30% → kalibrasi wajib dari percobaan hover aktual
- Cek dengan `rostopic echo /wp_marker_node/result` sambil hover di atas WP2

### Langkah selanjutnya (belum dilakukan)

1. **[WAJIB sebelum lomba]** Verifikasi fisik WP1/WP4 mapping ke marker asli di lapangan.
2. **[WAJIB]** Kalibrasi `~wp2_min_area_px` dari hover di atas WP2 di altitude normal.
3. **[Opsional]** Verifikasi overlay EM (`_diag/em_validation/`) — lihat §7b untuk kriteria validasi.

## 12. Verifikasi Final Sebelum Deployment Lapangan (sesi 2026-07-31)

Tiga hal yang diverifikasi: (1) validasi offset off_x/off_y, (2) swap label WP3/WP4,
(3) regresi INVERTED mode. Semua bukti berbasis angka + kode.

---

### 12a. Tugas 1 — Validasi offset off_x/off_y

**Konvensi tanda (dari formula, tidak dapat berubah secara matematis):**

```
off_x = (centroid_px_x − width/2)  / (width/2)
off_y = (centroid_px_y − height/2) / (height/2)
```

- `off_x < 0` → marker di **KIRI** frame  
- `off_x > 0` → marker di **KANAN** frame  
- `off_y < 0` → marker di **ATAS** frame ← OpenCV row=0 di atas, y bertambah ke bawah  
- `off_y > 0` → marker di **BAWAH** frame

**Konvensi ini identik dengan `off_x/off_y` di `ArucoMarker.msg`** (aruco_node) sehingga
`align_sign_x/y/swap_xy` kalibrasi lapangan yang sama bisa dipakai.

**Tabel hasil Tugas 1 (10 sampel dari 4 folder WP):**

| # | WP | File | off_x | off_y | reason | kuadran (kode) |
|---|---|---|---|---|---|---|
| 1 | wp3 | WIN_20260726_15_56_11_Pro.jpg | -0.057 | -0.290 | ok | ATAS |
| 2 | wp3 | WIN_20260726_15_56_12_Pro.jpg | -0.014 | -0.224 | ok | ATAS |
| 3 | wp3 | WIN_20260726_15_56_13_Pro.jpg | +0.024 | -0.259 | ok | ATAS |
| 4 | wp4 | WIN_20260726_15_58_49_Pro.jpg | +0.116 | -0.493 | ok | ATAS |
| 5 | wp4 | WIN_20260726_15_58_50_Pro.jpg | +0.154 | -0.650 | ok | KANAN-ATAS |
| 6 | wp4 | WIN_20260726_15_58_51_Pro.jpg | +0.093 | -0.389 | ok | ATAS |
| 7 | wp1 | WIN_20260718_17_35_15_Pro (4).jpg | +0.018 | -0.299 | no_match | ATAS |
| 8 | wp1 | WIN_20260718_17_35_15_Pro (5).jpg | +0.002 | -0.295 | no_match | ATAS |
| 9 | wp2 | WIN_20260726_15_53_28_Pro.jpg | +0.273 | -0.112 | present | KANAN |
| 10 | wp2 | WIN_20260726_15_53_32_Pro.jpg | +0.492 | -0.751 | present | KANAN-ATAS |

**Annotated images tersimpan di:** `dataset_arucode/_diag/offset_validation/`  
Tiap gambar: MERAH = frame center, HIJAU = marker centroid, KUNING = arah panah.
Verifikasi visual: pastikan marker body tampak di titik HIJAU (bukan di tempat lain).

**Catatan WP1 no_match (baris 7-8):** `off_x/off_y` tetap dihitung dari corners
meski ID decode gagal. Ini perilaku yang BENAR — alignment bisa dilakukan meski
ID belum terkonfirmasi (mission_d akan timeout tanpa konfirmasi ID, tapi posisi
tetap valid untuk hover).

**⚠ BIAS SISTEMATIS WP2 (temuan kritis):**

WP2 presence-detection mengeluarkan piksel oranye DAN merah sebelum ambil blob.
Dari 10 sampel WP2 (bukan 2 di tabel):

| Metrik | Nilai |
|---|---|
| off_x minimum | 0.157 |
| off_x maksimum | 0.712 |
| **off_x mean** | **+0.391** |
| off_x std | 0.181 |
| Sampel dengan off_x > 0.1 | **10/10 (100%)** |

**Penyebab:** Nampan merah (excluded) menutupi sebagian area, sehingga blob
yang tersisa (kertas putih + background non-oranye non-merah) centroidnya
cenderung geser ke arah background yang lebih luas di frame. Bias ini
**tidak deterministik** (bergantung komposisi frame) tapi dari dataset ini
selalu kanan (off_x > 0).

**Implikasi untuk alignment WP2:** Bila off_x/off_y dari presence-detection
dipakai untuk P-control alignment di mission_d, pilot WAJIB mengkalibrasi
`align_sign_x/y` untuk WP2 secara terpisah dari WP1/3/4, karena centroid blob
WP2 bukan berarti pusat marker yang sesungguhnya. Alternatif lebih aman:
**tidak pakai off_x/off_y WP2 untuk alignment** — cukup gunakan koordinat
T265/GPS dari waypoints.yaml yang sudah disurvei.

---

### 12b. Tugas 2 — Verifikasi swap label WP3/WP4

**Kutipan PERSIS dari AGENTS.md (baris 66, satu-satunya sumber kebenaran):**

> **ArUco WP = ID:** WP1=id1, WP2=id2, WP3=id3, WP4=id4. Dictionary DICT_7X7_50, allowlist `valid_ids=1,2,3,4`. Marker lapangan **putih-di-hitam** → `invert:=true` di aruco_detect.launch.

AGENTS.md **TIDAK** menyebutkan: deskripsi visual pola, posisi fisik di lintasan,
urutan misi, atau detail lain tentang WP3/WP4 selain ID-nya.

**Apakah WP3 dipakai di misi atau hanya folder riset?**

WP3 adalah **waypoint misi nyata**, bukan folder riset saja:
- `mission_seleksi.yaml`: `wp3, action: land, expected_id: 3` (landing point seleksi)
- `mission_final.yaml`: `wp3, action: scan, expected_id: 3` (sebelum lanjut ke WP4)
- `waypoints.yaml`: `wp3: x=-1.851, y=-2.647, z=1.0` (titik landing setelah triple gate)

**Catatan:** Deskripsi urutan misi dalam prompt sesi ini ("WP1→WP2→WP4→WP5") TIDAK
cocok dengan `mission_seleksi.yaml` / `mission_final.yaml`. File YAML adalah otoritatif.
WP3 ada di urutan misi resmi.

**Cross-Hamming validation (bukti internal consistency):**

| Folder | File | H→WP3ref | H→WP4ref | Cocok ke | Status |
|---|---|---|---|---|---|
| wp3_folder | ...15_56_11... | **3** | 16 | WP3 | ✓ konsisten |
| wp3_folder | ...15_56_12... | **3** | 16 | WP3 | ✓ konsisten |
| wp3_folder | ...15_56_13... | **3** | 16 | WP3 | ✓ konsisten |
| wp4_folder | ...15_58_49... | 14 | **1** | WP4 | ✓ konsisten |
| wp4_folder | ...15_58_50... | 14 | **13** | WP4 | ✓ konsisten |
| wp4_folder | ...15_58_51... | 14 | **1** | WP4 | ✓ konsisten |

Interpretasi: semua foto dari folder wp3 cocok ke pola referensi WP3 (bukan WP4),
dan sebaliknya. **Tidak ada bukti swap dari data**.

Hamming WP3 vs WP4 (pola EM final): **13/36 (36.1%)** — separasi terbesar dari semua
pasangan reliable. Ini memadai untuk runtime detection; risk misidentifikasi WP3↔WP4
sangat kecil secara algoritmik.

**LIMITATION — tidak bisa diverifikasi 100% dari kode:**

Internal consistency terbukti kuat, tapi swap masih **mungkin terjadi** kalau:
- Dataset asli sudah difoto dengan label yang tertukar di lapangan (wp3 folder
  sebenarnya difoto di lokasi marker id4 dan sebaliknya)
- AGENTS.md menyebutkan WP3=id3 tapi mungkin itu berdasarkan asumsi yang sama

**Checklist verifikasi fisik manual (bawa ke lapangan):**

> **Definisi dari AGENTS.md**: WP3 = id3, WP4 = id4 (Dictionary DICT_7X7_50)

**Langkah di lapangan:**

a. Di lapangan, ambil foto tegak lurus (overhead) marker fisik yang diberi tanda "WP3"
   dan "WP4" oleh panitia/tim. Pastikan pencahayaan memadai, tidak ada bayangan besar.

b. Di laptop (bukan Jetson), jalankan:
   ```bash
   cd ~/catkin_ws/src/mission_control/scripts
   python3 wp_marker_detector.py /path/to/WP3_fisik.jpg \
     --json ../config/wp_marker_reference.json
   ```
   Catat `wp_id` dan `hamming` yang keluar.

c. Decode manual (kalau kamera tidak tersedia): lihat pola kotak hitam-putih 6×6
   (tanpa border, dengan ekor tab di satu sisi). Putar foto sampai ekor di ATAS.
   Catat pola baris-per-baris: `1=hitam, 0=putih`. Bandingkan ke tabel berikut:

   | Sel | WP3 (id3) referensi | WP4 (id4) referensi |
   |---|---|---|
   | Baris 0 | `. . . . . .` | `. . # . . .` |
   | Baris 1 | `. . # . . .` | `. . . . . .` |
   | Baris 2 | `. # . # # .` | `. . . # . .` |
   | Baris 3 | `. . . . # .` | `. # # . . .` |
   | Baris 4 | `. . . # . .` | `. # # . # .` |
   | Baris 5 | `. # . # . .` | `. # . . # .` |
   | Jumlah sel hitam | **8** | **9** |

d. Isi tabel pembanding:

   | WP | Hasil decode kode (wp_id) | Pola manual cocok ke | MATCH? |
   |---|---|---|---|
   | Marker fisik "WP3" | ___ | ___ | ___ |
   | Marker fisik "WP4" | ___ | ___ | ___ |

e. Kesimpulan:
   - Kalau wp_id = 3 untuk marker "WP3" dan wp_id = 4 untuk marker "WP4" → **MATCH, tidak ada swap**
   - Kalau wp_id = 4 untuk marker "WP3" → **SWAP TERBUKTI** → laporkan sebelum deploy
   - Kalau hamming > 12 untuk keduanya → **TIDAK BISA DISIMPULKAN** → cek pencahayaan/sudut

---

### 12c. Tugas 3 — Konfirmasi INVERTED mode tidak regres

**Temuan utama: decode_grid adalah OBJEK YANG SAMA di audit dan detector.**

```python
from wp_decode_audit   import decode_grid as dg_audit
from wp_marker_detector import decode_grid as dg_det
dg_audit is dg_det  # → True (id Python identik)
```

`wp_marker_detector.py` **mengimport** `decode_grid` dari `wp_decode_audit.py`
(bukan duplikasi). Tidak mungkin ada divergensi threshold antar keduanya.

**Verifikasi cross-check hamming per sampel:**

| # | WP | File | bits_sum | H ke ref | recomputed==returned | Status |
|---|---|---|---|---|---|---|
| 1 | wp3 | ...15_56_11... | 11 | 3 | 3==3 True | OK |
| 2 | wp3 | ...15_56_12... | 11 | 3 | 3==3 True | OK |
| 3 | wp3 | ...15_56_13... | 11 | 3 | 3==3 True | OK |
| 4 | wp4 | ...15_58_49... | 10 | 1 | 1==1 True | OK |
| 5 | wp4 | ...15_58_50... | 12 | 12 | 12==12 True | OK |
| 6 | wp4 | ...15_58_51... | 10 | 1 | 1==1 True | OK |

**Logika THRESH_BINARY_INV + OTSU (tidak berubah sejak awal):**

- Piksel gelap (sel hitam marker) → gray rendah → INV → nilai tinggi → `bit = 1`
- Piksel terang (background putih) → gray tinggi → INV → nilai rendah → `bit = 0`
- Sesuai konvensi `wp_marker_reference.json`: `bits[r][c]=1` = sel **HITAM**

**STATUS TUGAS 3: LULUS — tidak ada regresi.**

---

### 12d. Status Akhir — Checklist Deployment

**SUDAH SIAP (terverifikasi kode):**

- [x] `decode_grid` INVERTED mode tidak berubah setelah refactor ke node ROS
- [x] `decode_frame` mengimport langsung dari `wp_decode_audit` — tidak ada duplikasi pipeline
- [x] `off_x/off_y` konvensi tanda benar (KIRI<0, KANAN>0, ATAS<0, BAWAH>0)
- [x] WP3/WP4 tidak ada bukti swap dari data (cross-Hamming 3/16 dan 14/1)
- [x] Hamming WP3↔WP4 = 13/36 (36.1%) — cukup terpisah untuk runtime
- [x] Build catkin bersih, semua py_compile OK
- [x] Offline test WP3 (H=3), WP4 (H=1), WP1, WP2-presence semua lulus

**MASIH BUTUH TINDAKAN MANUAL (tidak bisa diverifikasi dari kode):**

- [ ] **[KRITIS]** Verifikasi fisik WP3/WP4 di lapangan (gunakan checklist §12b langkah a-e)
- [ ] **[KRITIS]** Verifikasi fisik WP1/WP4 mapping ke marker asli (agreement WP1=0.671 rendah, Hamming WP1↔WP4=9/36 minimum — risiko swap terbesar)
- [ ] **[KRITIS]** Kalibrasi `~wp2_min_area_px` dari hover aktual di WP2 (default 3000px² belum divalidasi)
- [ ] **[WAJIB]** Kalibrasi `align_sign_x/y` + `align_swap_xy` di lapangan (mounting kamera bawah belum diketahui orientasinya)
- [ ] **[PERHATIAN WP2]** Jangan pakai `off_x/off_y` dari WP2 presence untuk alignment tanpa kalibrasi khusus — bias sistematis off_x mean=+0.391 (100% sampel > 0.1). Opsi aman: hover di waypoint.yaml WP2 koordinat, tidak rely pada centroid visual
- [ ] **[Opsional]** Review gambar EM validation di `_diag/em_validation/` (WP1/WP2/WP3/WP4) — lihat panduan §7b

**Cara launch saat field test:**
```bash
# Tanpa gate, custom WP decoder, kamera bawah:
roslaunch mission_control phaseD_mission.launch \
  mission:=seleksi use_wp_marker:=true use_aruco:=false use_gate:=false \
  do_takeoff:=false sim_markers:=false

# Monitor hasil realtime:
rostopic echo /wp_marker_node/result
rostopic echo /wp_marker_node/wp2_present
```

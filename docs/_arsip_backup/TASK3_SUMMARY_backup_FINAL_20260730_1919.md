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
| wp2 | id2 | 24/62 | 0.705 | **false** |
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

**WP2 (id2)** — agreement 0.705, K=24/62 *(setelah fix tray merah)*
```
....#.
.##...
.###..
..##..
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

## 7. Keterbatasan yang Diketahui

- **Mapping WP<->ID belum diverifikasi fisik** ke marker asli di lapangan.
- **WP1 agreement rendah (0.671)**: 89 foto sudut ekstrem tidak bisa
  diselamatkan. AR_TOL=0.35 dicoba, justru turunkan agreement → revert ke 0.25.
- **WP4 ekor scatter N=S=E=W=9 disengaja** (dataset dari segala sudut).
  Agreement 0.738 (reliable=true) membuktikan normalisasi ekor bekerja.
- **Docstring modul** `wp_decode_audit.py` baris pertama masih sebut "5x5" —
  teks basi, kode aktual sudah `GRID_N = 6`.

## 8. Langkah Selanjutnya yang Direkomendasikan

1. **[WAJIB sebelum lomba] Verifikasi fisik WP1/WP4**: pattern di JSON cocokkan
   ke marker fisik. Kemiripan 25% di data adalah artefak, bukan fisik.
2. **[WAJIB] Verifikasi orientasi ekor WP3/WP4**: ekor WP3 umumnya N di
   dataset tapi scatter WP4 perlu dicek di lapangan.
3. **[Task berikutnya] Implementasi presence detection WP2** di node
   mission_control:
   - Deteksi blob non-orange (dan non-red) dengan area ≥ threshold di frame
     kamera bawah → sinyal binary "WP2 terdeteksi"
   - Threshold luas harus dikalibrasi berdasarkan ketinggian terbang normal WP2
   - Tidak perlu decode grid, tidak perlu ekor detection
4. **[Task berikutnya] Integrasi `wp_marker_reference.json` ke node ROS** untuk
   WP1/WP3/WP4 — file JSON belum dikonsumsi oleh node manapun.

## 9. Keputusan Final Per WP

| WP | Metode | Agreement | Reliable | Catatan |
|---|---|---|---|---|
| WP1 | Decode ID grid | 0.671 | **false** | Verifikasi fisik wajib |
| WP2 | **Presence detection** | 0.714 (research) | false | Pattern tidak dipakai runtime |
| WP3 | Decode ID grid | 0.743 | **true** | Siap pakai |
| WP4 | Decode ID grid | 0.738 | **true** | Siap pakai; verifikasi ekor scatter |

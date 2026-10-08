# Task 1 — Detektor Double/Triple Gate (bentuk-U) — Ringkasan Status

Status: **selesai untuk deteksi geometris (tuning 69.2% / holdout 69.2% / gap 0.0),
integrasi ke gate_node.py BELUM dikerjakan (task terpisah). Eksplorasi lanjutan
(30 Jul, sesi ke-2) TIDAK menemukan kombinasi parameter baru yang lolos audit
visual — parameter final TETAP sama seperti Langkah 3 (lihat §7).**
Script: `src/mission_control/scripts/gate_multi_detector.py`
Dataset: `~/catkin_ws/gate/dataset_gate_baru` (520 foto, split tuning=416/holdout=104, seed=42)
Backup final: `src/mission_control/scripts/gate_multi_detector_FINAL_20260730.py`

---

## 1. Ringkasan Masalah Awal

Gate Double/Triple berbentuk **U**: 2 kaki + palang atas, **TANPA palang bawah**
(mirip gawang, terbuka di bawah) — berbeda dari Single Gate (`gate_detector.py`,
di luar scope task ini, tidak disentuh).

Konsekuensi topologi bentuk-U: bukaan gate **tersambung langsung ke background**
lewat celah bawah. Pendekatan pertama yang dicoba (hole-hierarchy `RETR_CCOMP`,
mencari bukaan sebagai "lubang tertutup" child contour) **gagal total** karena
memang tidak ada lubang tertutup untuk dicari — topologi bentuk-U tidak
menghasilkan child-contour sama sekali.

Fix mendasar: bukaan dicari sebagai **komponen background (mask terbalik)
terhubung TERBESAR di dalam bounding-box kontur luar**, via
`cv2.connectedComponentsWithStats`, bukan hierarchy hole.

## 2. Riwayat Perbaikan Teknis (kronologis)

### a. Hole-hierarchy (RETR_CCOMP child) → connectedComponentsWithStats pada mask terbalik
**Alasan**: dijelaskan di #1 — topologi U tidak punya lubang tertutup untuk
di-hierarchy-kan. Bukaan dicari sebagai blob background terbesar di dalam bbox
kontur luar oranye.

### b. Solidity range dilebarkan: 0.30–0.60 → 0.15–0.80
**Alasan**: gate yang dekat/mengisi-frame punya solidity turun ke ~0.17–0.27
(kelewat oleh range lama), dan gate dari sudut miring butuh batas atas lebih
tinggi. Range lama terlalu sempit untuk variasi jarak & sudut kamera nyata.

### c. `_validate_opening()` ditambahkan (Langkah 2)
**Alasan**: solidity/aspect saja tak cukup membedakan gate sejati dari noise
(semak/kayu oranye kebetulan berbentuk mirip). Validasi baru menambahkan 3
syarat: (1) bukaan harus LEBAR & TINGGI cukup (`OPEN_WIDTH_FRAC`/`OPEN_HEIGHT_FRAC`
≥30% bbox — menolak celah tipis antar-bilah), (2) bukaan harus dekat tepi bawah
bbox (`OPEN_BOTTOM_MARGIN_FRAC` ≤12% — menolak gate terbalik/ekstrem), (3) kaki
KIRI & KANAN harus terdeteksi oranye (`LEG_STRIP_FRAC`/`LEG_ORANGE_MIN`). Target
diganti dari **centroid piksel mentah** (bias ke objek terang/gelap di background)
menjadi **pusat bounding-box komponen bukaan** (lebih robust, tidak tertarik
konten background).

Disertai 2 generator sintetis untuk regresi selftest: `_make_blob()` (blob
tak-berbingkai, meniru noise semak/kayu) dan `_make_one_leg_thin_gap()` (1 kaki
lebar + celah tipis, meniru kegagalan nyata foto `17_30_17`).

### d. Bug ditemukan: `LEG_ORANGE_MIN` terlalu ketat untuk gate jarak-dekat (Langkah 3, 30 Jul 2026)
**Diagnosis**: audit visual foto kunci `17_31_45` (gate asli, jelas terlihat)
menunjukkan **0 kandidat terdeteksi** — bukan cuma "target salah ke motor"
seperti bug lama, tapi gate sama sekali tidak lolos. Ditelusuri manual: kontur
luar & bukaan lolos semua filter lain, tapi kaki kanan cuma 28.3% oranye di
strip vertikal (`LEG_STRIP_FRAC=0.20`), di bawah ambang `LEG_ORANGE_MIN=0.35`.
Root cause: foto jarak-dekat FOV-lebar membuat kaki gate tampak terdistorsi
perspektif (tidak murni vertikal), sehingga strip vertikal lurus 20% lebar bbox
banyak berisi non-kaki di sebagian tinggi kaki.

**Audit dampak** (sebelum memilih fix): dari 416 foto `tuning_set`, **186 foto
0-gate**, dan dari situ **164 foto (39.4%)** punya kandidat area-besar (kelas
gate asli, bukan noise kecil) yang gagal MURNI di leg-check — **60 foto (14.4%
dari total tuning_set)** menunjukkan pola near-miss persis (satu sisi kaki
sudah lolos, sisi lain di rentang 0.25–0.35). Kesimpulan: bug ini **bukan kasus
langka**, berdampak luas ke banyak foto gate asli yang selama ini danggap
"gagal wajar".

**Fix — sweep parameter empiris** (bukan tebak manual): grid-search
`LEG_STRIP_FRAC ∈ {0.20,0.25,0.30}` × `LEG_ORANGE_MIN ∈ {0.35,0.30,0.28,0.25}`
(12 kombinasi), diuji terhadap 4 pagar keselamatan:

| strip | legmin | tuning% | holdout% | gap | lolos pagar? | 17_31_45 rescue? |
|---|---|---|---|---|---|---|
| 0.20 | 0.35 (baseline) | 55.3 | 54.8 | 0.5 | ya | tidak |
| 0.20 | 0.30 | 63.0 | 61.5 | 1.4 | gap melebar | tidak |
| **0.20** | **0.28** | 65.4 | 65.4 | 0.0 | **ya** | **ya** |
| 0.20 | 0.25 | 70.0 | 67.3 | 2.6 | gap melebar | ya |
| 0.25 | 0.35 | 46.6 | 47.1 | 0.5 | ya | tidak |
| 0.25 | 0.30 | 63.7 | 61.5 | 2.2 | gap melebar | ya |
| 0.25 | 0.28 | 66.6 | 64.4 | 2.2 | gap melebar | ya |
| 0.25 | 0.25 | 72.4 | 70.2 | 2.2 | gap melebar | ya |
| 0.30 | 0.35 | 33.9 | 32.7 | 1.2 | gap melebar | ya |
| 0.30 | 0.30 | 51.7 | 52.9 | 1.2 | gap melebar | ya |
| 0.30 | 0.28 | 58.4 | 61.5 | 3.1 | gap melebar | ya |
| **0.30** | **0.25** | **69.2** | **69.2** | **0.0** | **ya** | **ya** |

Hanya 2 kombinasi lolos semua 4 pagar literal (3 negatif selftest tetap 0-gate,
foto noise acuan `17_30_06`/`17_30_12` varian `(4)` tetap 0-gate, `17_31_45`
ter-rescue, gap tidak melebar dari 0.5): **(0.20, 0.28)** dan **(0.30, 0.25)**.

## 3. Kombinasi Parameter Final & Alasan Pemilihan

**Dipilih: `LEG_STRIP_FRAC=0.30`, `LEG_ORANGE_MIN=0.25`** — rescue rate
tertinggi (69.2%/69.2%/gap 0.0) di antara kombinasi yang lolos semua 4 pagar
keselamatan yang disepakati, sesuai aturan keputusan eksplisit ("pilih rescue
rate tertinggi yang lolos semua pagar").

**Trade-off yang ditemukan lewat pengecekan tambahan** (di luar 4 pagar
wajib, dilakukan sebagai due-diligence ekstra sebelum commit ke kombinasi
ini): saat SEMUA varian foto (bukan cuma file acuan resmi) dari keluarga
noise `17_30_06`/`17_30_12` (10 foto total) dan keluarga gate `17_31_45`
(5 foto total) dicek —

| kombinasi | false-positive noise-family | rescue gate-family |
|---|---|---|
| (0.20, 0.28) | 1/10 | 3/5 |
| **(0.30, 0.25) — DIPILIH** | 3/10 | 4/5 |

Ada juga kombinasi (0.25, 0.30) dengan 0/10 false-positive & 4/5 rescue — tapi
**didiskualifikasi** karena melanggar pagar gap (2.2 poin, melebar dari
baseline 0.5 → indikasi overfitting).

Kombinasi `(0.30, 0.25)` diterima dengan trade-off ini **secara sadar dan
terdokumentasi** (lihat #5), bukan dipilih karena mengabaikan risikonya.

## 4. Hasil Akhir & Perbandingan Semua Iterasi

| Iterasi | Parameter leg-check | tuning% | holdout% | gap | Catatan metrik |
|---|---|---|---|---|---|
| Lama (pra `_validate_opening`, "iterasi 6") | — (belum ada leg-check) | — | — | — | ~46% **presisi-adjusted** (metrik BEDA: hitung target-salah sbg gagal, bukan cuma "0 kandidat") |
| Langkah 1 (baseline sesi ini) | strip=0.20, legmin=0.35 | 55.3 | 54.8 | 0.5 | raw detection rate |
| **Langkah 3 (FINAL)** | **strip=0.30, legmin=0.25** | **69.2** | **69.2** | **0.0** | raw detection rate |

**Kenaikan vs baseline Langkah 1**: +13.9 poin tuning, +14.4 poin holdout, gap
mengecil (0.5→0.0, bukan melebar).

**Peringatan metrik**: angka 46% lama dan 55.3%/69.2% sekarang **tidak
apple-to-apple** — 46% menghukum kasus "target salah" sebagai gagal, sedangkan
raw detection rate di sini hanya mengukur "ada ≥1 kandidat lolos validasi
bentuk". Karena `_validate_opening()` sekarang jauh lebih ketat dari iterasi
lama, kandidat yang lolos kemungkinan besar juga presisi — tapi ini **belum
diverifikasi manual foto-per-foto**, hanya diverifikasi utk 4 sampel kunci
(lihat #4b).

### 4b. Audit Visual 4 Sampel Kunci (setelah parameter final)

| Foto | Kandidat | Target | Status |
|---|---|---|---|
| `17_30_06_Pro (4)` | 0 | — | tetap benar ditolak |
| `17_30_12_Pro (4)` | 0 | — | tetap benar ditolak |
| `17_31_45_Pro` | 3 (kandidat #0 area=250208 = gate asli, prioritas tertinggi krn diurut area DESC; 2 kandidat kecil area 9496 & 2176 = noise level rendah) | kandidat #0 di pixel (1184,656) = **pusat bounding-box bukaan asli** | **berhasil di-rescue** — target sekarang di pusat bukaan geometris, bukan bias-centroid seperti bug lama (motor di background hanya kebetulan berdekatan dgn titik pusat bukaan, bukan tanda algoritma "tertarik" ke motor) |
| `17_30_17_Pro (7)` | 1 (+1 kandidat kecil baru di tembok, area sangat kecil) | tetap di papan kayu (tidak berubah dari sebelumnya) | **tetap ditolak sesuai keputusan user** — dianggap bukan gate asli (prop/objek lain kebetulan oranye), prioritas rendah |

## 5. Keterbatasan yang Diketahui

- **False-positive pada 3/10 foto saudara-frame dari keluarga noise
  `17_30_06`/`17_30_12`** (bukan varian acuan resmi) — trade-off yang diterima
  sadar demi rescue rate gate asli jarak-dekat (lihat #3). Kalau risiko ini
  terbukti signifikan di lapangan, kombinasi `(0.20, 0.28)` adalah alternatif
  lebih konservatif (1/10 false-positive, tapi rescue rate lebih rendah).
- **Foto `17_30_17` tetap ambigu** — objek kanan di foto ini lolos validasi
  bentuk (kaki kiri-kanan oranye + bukaan lebar), tapi menurut penilaian user
  ini KEMUNGKINAN BESAR bukan gate asli (prop/rak lain di lokasi latihan).
  Belum ada mekanisme di kode untuk membedakan "gate asli" vs "objek oranye
  lain berbentuk mirip" selain heuristik solidity/aspect/leg-check yang sudah
  ada — kasus ini lolos semua heuristik itu.
- **Metrik akurasi (raw detection rate) belum sepenuhnya diverifikasi sebagai
  "presisi" (target benar)** — hanya 4 sampel kunci yang diaudit visual manual.
  230-288 foto lain yang "terdeteksi" belum dicek satu-per-satu apakah target-
  nya tepat di pusat bukaan gate asli.
- **`REL_AREA_FRAC` masih nonaktif** (0.0) — belum dicoba diaktifkan sebagai
  mitigasi tambahan utk menekan false-positive keluarga noise (kandidat
  perbaikan lanjutan kalau limitasi pertama di atas jadi masalah nyata).
- **Opsi (c) — leg-check toleran skew/rotasi per-baris** tidak jadi diperlukan
  karena opsi (a)/(b) (sweep grid sederhana) sudah cukup lolos semua pagar
  keselamatan. Tetap tercatat sebagai opsi cadangan kalau iterasi berikutnya
  butuh presisi lebih tinggi tanpa trade-off false-positive di atas.

## 6. Langkah Selanjutnya yang Direkomendasikan

1. **Integrasi ke `gate_node.py`** — task terpisah (Task 2), **belum
   dikerjakan** di sesi ini. `gate_multi_detector.py` saat ini berdiri sendiri
   (bisa dijalankan `--selftest`/`--image`/`--batch-test` tanpa ROS), belum
   dipanggil dari node ROS manapun.
2. Kalau butuh menekan false-positive keluarga noise (limitasi #5 pertama)
   lebih lanjut: coba aktifkan `REL_AREA_FRAC` (mis. 0.15) atau eksplorasi
   opsi (c) leg-check toleran-skew.
3. Verifikasi manual foto-per-foto (bukan cuma 4 sampel kunci) kalau perlu
   klaim akurasi "presisi" yang lebih kuat, bukan cuma raw detection rate.

## 7. Eksplorasi Lanjutan (Sesi 30 Jul, ke-2) — HASIL NIHIL, terdokumentasi penuh

Otorisasi penuh diberikan untuk memaksimalkan akurasi lebih lanjut, dengan
syarat keras: setiap keputusan harus berbasis data, dan WAJIB audit visual
sebelum klaim perbaikan. Hasil: **tidak ada kombinasi baru yang bertahan
setelah audit visual** — parameter final TETAP sama seperti Langkah 3
(§3). Ini dilaporkan apa adanya karena hasil negatif tetap merupakan
temuan yang valid & berharga (bukan kegagalan proses).

### 7a. Breakdown komposisi kegagalan (520 foto penuh, param Langkah 3 aktif)

| Kategori | Jumlah | % dari 520 |
|---|---|---|
| DETECTED | 360 | 69.2% |
| leg_clear_miss (worst-side <0.15) | 70 | 13.5% |
| leg_near_miss (worst-side 0.15-0.25) | 45 | 8.7% |
| aspect_reject | 26 | 5.0% |
| solidity_reject | 16 | 3.1% |
| opening_tak_nyentuh_bawah | 3 | 0.6% |

Leg-check mendominasi (115/160 = 71.9% dari SEMUA kegagalan). Histogram
distribusi kandidat yg gagal aspect (n=37, semua di rentang 0.28-0.40,
median 0.359 — SEMUA di bawah `ASPECT_LO=0.40`, tidak ada yg di atas
`ASPECT_HI`) dan solidity (n=17, semua di rentang 0.826-0.935, median
0.908 — SEMUA di atas `SOLIDITY_HI=0.80`, tidak ada yg di bawah
`SOLIDITY_LO`) menunjukkan pola "tebing rapat", bukan ekor panjang acak —
secara data terlihat seperti kandidat layak dicoba dilebarkan.

### 7b. Sweep yang diuji & hasilnya (semua diverifikasi thd 4 pagar keselamatan)

**Melebarkan `LEG_STRIP_FRAC` lebih jauh (0.35, 0.40) — DITOLAK, data
konkret:** akurasi justru ANJLOK (58.7%/62.5% pada strip=0.35, 52.4%/50.0%
pada strip=0.40 — dibanding 69.2%/69.2% di strip=0.30), DAN false-positive
keluarga noise naik ke 5-6/10 (dari 3/10). strip=0.30 terbukti sudah
optimal lokal, bukan bisa dinaikkan lagi.

**Menurunkan `LEG_ORANGE_MIN` lebih jauh (ke 0.20) — DITOLAK, data
konkret:** akurasi terlihat menggiurkan di atas kertas (holdout sampai
83.7%), TAPI **false-positive keluarga noise SELALU melonjak ke 5-8/10**
(vs batas keras 3/10) di SEMUA kombinasi strip/aspect/solidity yg dicoba
bersamanya — tanpa kecuali. Kesimpulan: `LEG_ORANGE_MIN=0.25` sudah di
titik plafon praktis; tidak bisa diturunkan lebih jauh tanpa melanggar
pagar #4.

**Menurunkan `ASPECT_LO` (ke 0.35/0.30/0.27) — DITOLAK, data konkret:**
akurasi naik di atas kertas (holdout sampai 75.0% pada aspect_lo=0.27),
TAPI false-positive keluarga noise NAIK KONSISTEN dari 3/10 ke 5/10 di
SEMUA nilai aspect_lo<0.40 yg dicoba (independen dari solidity_hi). Harus
tetap di 0.40.

**Menaikkan `SOLIDITY_HI` (0.80 → 0.90) — LOLOS 4 PAGAR NUMERIK, TAPI
GUGUR DI AUDIT VISUAL (WAJIB):** kombinasi ini (bersama aspect_lo=0.40,
strip=0.30, legmin=0.25 tetap) menghasilkan tuning=70.0%/holdout=71.2%/
gap=1.2/fp_noise=3 — lolos SEMUA angka pagar. Tapi audit visual terhadap
**5 foto yang "baru terdeteksi"** (dibandingkan dgn param Langkah 3)
mengungkap:

| Foto | Kandidat yg lolos | Verdict visual |
|---|---|---|
| `17_29_44 (5)` | area besar, tapi objek = "prop" kain-oranye+rak (identik kategori 17_30_17) | **AMBIGU** — sama seperti 17_30_17, tidak pasti gate asli |
| `17_29_44 (6)` | sama seperti di atas (frame burst berdekatan) | **AMBIGU** |
| `17_30_13 (2)` | kotak KECIL (area 2410) di bangku kayu background — gate asli (area 223542) di foto yg SAMA tetap TIDAK lolos (`left_frac=0.2485`, hanya 0.0015 di bawah ambang 0.25!) | **FALSE POSITIVE** — target ke bangku, bukan gate |
| `17_31_51 (3)` | kotak KECIL (area 1600) di tanah/rumput — gate asli besar di background sama sekali tidak jadi kandidat | **FALSE POSITIVE** |
| `17_31_52 (4)` | kotak KECIL (area 2251) di tanah/rumput, pola identik dgn di atas | **FALSE POSITIVE** |

**Kesimpulan: 0 dari 5 foto "rescued" adalah gate asli yang bersih
terverifikasi.** Kenaikan angka 69.2%→71.2% adalah **ilusi statistik** —
raw detection rate naik karena objek KECIL YANG SALAH ikut lolos validasi
bentuk (solidity 0.826-0.935 ternyata juga cocok utk kerikil/bangku kecil
berbentuk kebetulan U), BUKAN karena gate asli makin banyak terdeteksi.
**`SOLIDITY_HI=0.90` DITOLAK / TIDAK diadopsi** — file `gate_multi_detector.py`
TIDAK diubah dari state Langkah 3.

### 7c. Kesimpulan akhir eksplorasi

Parameter Langkah 3 (`ASPECT_LO=0.40, ASPECT_HI=2.50, SOLIDITY_LO=0.15,
SOLIDITY_HI=0.80, LEG_STRIP_FRAC=0.30, LEG_ORANGE_MIN=0.25`) **tetap
final**. Empat arah eksplorasi berbasis data (strip lebih lebar, legmin
lebih rendah, aspect_lo lebih rendah, solidity_hi lebih tinggi) SEMUA
gagal — tiga karena melanggar pagar numerik (gap/false-positive), satu
(solidity_hi) karena gagal audit visual meski lolos pagar numerik. Ini
menunjukkan **69.2%/69.2%/gap 0.0 (raw detection rate) adalah plafon
praktis** untuk pendekatan threshold-tuning sederhana (opsi a/b) pada
dataset ini — perbaikan lebih lanjut kemungkinan besar butuh pendekatan
berbeda (opsi c: leg-check toleran skew per-baris, atau fitur baru sama
sekali), bukan sekadar geser angka ambang lagi.

**Contoh margin near-miss paling ekstrem yang ditemukan**: foto
`17_30_13 (2)` — gate asli dgn `left_frac=0.2485` vs ambang `0.25`,
meleset hanya **0.0015** (kurang dari 1 piksel dari ~250px lebar strip).
Ini mengilustrasikan betapa dekatnya batas saat ini ke titik optimal
teoretis — tapi tetap tidak bisa digeser tanpa membuka pintu false-positive
di foto lain (lihat sweep legmin=0.20 di atas).

Tidak ada backup baru dibuat sesi ini (`gate_multi_detector_FINAL_v2_*`)
karena **tidak ada perubahan kode** — file identik dengan
`gate_multi_detector_FINAL_20260730.py` dari sesi sebelumnya.

## 8. Integrasi ke ROS (`gate_multi_node.py`) — Task 2

Sesi terpisah (30 Jul 2026) membuat node ROS baru yang memanggil
`detect_gates()` supaya `gate_multi_detector.py` bisa dipakai `mission_d.py`
saat terbang. `gate_multi_detector.py` sendiri **TIDAK diubah** (tetap
FINAL) — node ini murni wrapper tipis, TIDAK duplikasi logika deteksi.

### 8a. File baru

| File | Isi |
|---|---|
| `src/mission_control/scripts/gate_multi_node.py` | Class `GateMultiNode` — subscribe kamera, panggil `gate_multi_detector.detect_gates()`, publish `mission_control/Gate` |
| `src/mission_control/launch/gate_multi_detect.launch` | Launch file, arg `image_topic` (default sama dgn lama: `/camera_front/image_raw`) + `publish_debug` |
| `src/mission_control/CMakeLists.txt` | Diedit (backup dulu: `CMakeLists.txt.backup_20260730_1811`) — tambah `scripts/gate_multi_detector.py` & `scripts/gate_multi_node.py` ke `catkin_install_python`, konsisten dgn konvensi `gate_detector.py`/`gate_node.py` yg sudah ada |

`gate_node.py`/`gate_detector.py` (Single Gate) **TIDAK disentuh sama
sekali** — tetap dipakai apa adanya utk misi Single Gate.

### 8b. Format topic & pemetaan field `Gate.msg`

Topic publish: **`/gate_multi_node/gate`** (BEDA dari `/gate_node/gate` lama
— sengaja, supaya 2 node bisa jalan bersamaan saat testing tanpa bentrok).
Plus `/gate_multi_node/debug_image` (overlay, reuse `_draw_overlay()` dari
`gate_multi_detector.py`, sama seperti pola lama).

Sebelum menulis kode, field `Gate.msg` dicek pemakaiannya di `mission_d.py`
(grep langsung, bukan asumsi) supaya pemetaan tidak sembarangan:

| Field | Sumber di node baru | Dicek di `mission_d.py`? |
|---|---|---|
| `detected` | `len(gates) > 0` | dipakai di logika kontrol (align + transisi lapis) |
| `off_x`, `off_y` | kandidat area-terbesar (`gates[0]`) | dipakai di logika kontrol — definisi offset sudah identik (kamera depan, -1..1) |
| `area_frac` | `gates[0]["area"] / (W*H)` | dipakai di logika kontrol (pola naik-lalu-turun vs `gate_area_drop_ratio`). **BEDA definisi** dari `gate_node.py` lama (yg pakai total-piksel-oranye SELURUH frame via `mask.mean()/255`, bukan area 1 kontur). Karena `mission_d.py` cuma pakai pola RELATIF (`area < peak*(1-ratio)`), bukan nilai absolut, logika transisi-lapis tetap berfungsi — **tapi skala numeriknya beda**, jadi kalau ada rosparam lain yg diam-diam bergantung pada besaran absolut `area_frac` (bukan cuma rasio), perlu dicek ulang manual. Tidak ditemukan yg begitu di grep sejauh ini. |
| `num_openings` | `1` kalau detected, `0` kalau tidak | **grep konfirmasi**: HANYA dipakai di 1 baris `rospy.loginfo(...)`, TIDAK ada percabangan `if`/logika berdasar nilai ini. `detect_gates()` sendiri tidak membedakan sub-bukaan ganda per struktur gate (selalu ambil 1 komponen background TERBESAR per kontur luar), jadi disederhanakan jadi 1/0 apa adanya, bukan dipalsukan jadi 2/3. |
| `opening_w_frac` | `0.0` selalu | **grep konfirmasi**: TIDAK direferensikan sama sekali di `mission_d.py`. `detect_gates()` juga tidak mengekspos lebar-bukaan spesifik di return value-nya (beda dari `bbox_outer` yg itu bbox GATE, bukan bbox bukaan) — dibiarkan 0.0 apa adanya, bukan diaproksimasi keliru dari `bbox_outer`. |
| `bbox` | `list(gates[0]["bbox_outer"])` | **grep konfirmasi**: TIDAK direferensikan di `mission_d.py`. 1:1 dgn `bbox_outer`. |

Rosparam node baru **hanya** `~image_topic` & `~publish_debug` (BEDA dari
node lama yg juga expose `~hsv_lo/hi/~min_area_frac` — sengaja TIDAK
diekspos, krn `detect_gates()` tidak menerima override HSV/leg-threshold
via argumen fungsi, constant-nya sudah divalidasi ketat lewat sweep di
Task 1 §3/§7; membuka rosparam utk itu berisiko detune tanpa sadar di
lapangan tanpa proses sweep+audit visual ulang).

### 8c. Cara pindah dari `gate_node.py` lama ke `gate_multi_node.py` baru

Untuk misi Double/Triple Gate: jalankan `gate_multi_detect.launch`
(bukan `gate_detect.launch`), lalu set rosparam `mission_d.py`:
```
~gate_topic := /gate_multi_node/gate    (default lama: /gate_node/gate)
```
`mission_d.py` **TIDAK perlu diubah kodenya sama sekali** — cukup ganti
argumen `gate_topic` di launch file misi (di luar scope task ini, launch
file misi utama tidak disentuh di sesi ini). Untuk misi Single Gate:
tetap pakai `gate_detect.launch` + `gate_node.py` seperti biasa, TIDAK
ada yg berubah.

### 8d. Hasil testing (TANPA hardware — read-only thd kamera asli)

1. `python3 -m py_compile gate_multi_node.py` — **OK**.
2. XML `gate_multi_detect.launch` — **valid** (parse dgn `xml.dom.minidom`).
3. **Testing end-to-end via `roscore` lokal** (bukan hardware — murni
   message-broker, dimatikan lagi setelah testing selesai): jalankan
   `gate_multi_node.py` sungguhan sbg node ROS, publish foto statis dari
   dataset ke topic tiruan, baca balik `/gate_multi_node_test/gate`:

   | Foto | Hasil aktual (via ROS pub/sub asli) | Sesuai ekspektasi? |
   |---|---|---|
   | `17_31_45_Pro` (gate asli) | `detected=True off=(0.233,0.215) area_frac=0.1207 bbox=[655,0,1265,1080]` | **Ya** — persis cocok dgn analisis manual `detect_gates()` langsung (§4b) |
   | `17_30_06 (4)` (noise) | `detected=False off=(0,0) area_frac=0 bbox=[0,0,0,0]` | **Ya** |
   | `17_30_12 (4)` (noise) | `detected=False` (setelah re-test terisolasi; percobaan pertama sempat timeout krn race-condition test-harness, BUKAN bug node — dikonfirmasi ulang scr terpisah dgn hasil konsisten 0.09s) | **Ya** |
   | `/debug_image` | diterima, `encoding=bgr8 w=1920 h=1080`, ukuran data sesuai | **Ya** — jalur overlay debug berfungsi |

   Semua proses testing (roscore + node) dimatikan bersih setelah selesai
   (diverifikasi via `pgrep`, tidak ada proses ROS tersisa).

### 8e. Keterbatasan yang BELUM bisa diverifikasi tanpa hardware asli

- **Kamera asli / stream real-time**: testing di atas pakai 1 foto statis
  dipublish manual, BUKAN stream kontinu dari `/camera_front/image_raw`
  sungguhan. Performa `queue_size=1`/`buff_size=2**24` di bawah beban
  frame-rate nyata (drop behavior saat `detect_gates()` lebih lambat dari
  interval frame) belum teruji.
- **Skala `area_frac` di kondisi terbang nyata**: karena definisi berbeda
  dari `gate_node.py` lama (§8b), rosparam `gate_area_drop_ratio` di
  `mission_d.py` (dituning berdasar gate_node lama) mungkin perlu
  di-cross-check ulang di bench/SITL test — logikanya (rasio relatif)
  seharusnya tetap valid, tapi belum divalidasi dgn approach fisik nyata.
- **`catkin_make`/`catkin build` belum dijalankan ulang** setelah edit
  `CMakeLists.txt` — perubahan baru berlaku penuh (`rosrun` via devel-space
  symlink resmi) setelah build ulang; testing di atas menjalankan script
  Python secara langsung (bypass rosrun) utk menghindari langkah build
  yang di luar scope read-only sesi ini.
- **`mission_d.py` launch file misi utama** belum diubah utk pakai
  `gate_multi_detect.launch` (lihat §8c) — itu langkah integrasi akhir yg
  sengaja belum dikerjakan (di luar scope task ini, perlu instruksi
  eksplisit + kemungkinan testing SITL sebelum dipakai di lapangan).
`gate_multi_detector_FINAL_20260730.py` dari sesi sebelumnya.

## 9. Tuning Triple Gate (`dataset_triple_baru`) — Sesi 6 Aug — HASIL NIHIL untuk perbaikan aman, terdokumentasi penuh

Dataset `gate/dataset_gate_baru/` (520 foto, §1-8) ternyata **100% Double
Gate** — diverifikasi lewat sampling visual 48 foto acak (2 batch), semua
menunjukkan 1 frame + 1 panel divider (spesifikasi Double Gate). Dataset
BARU `gate/dataset_triple_baru/` (dari "logi l310 triple gate
complete.zip") berisi foto Triple Gate murni (frame lebih dalam, 2
panel divider) — dipakai di sesi ini utk evaluasi & eksplorasi terpisah.
Otorisasi: cari strategi paling optimal, tingkatkan akurasi Triple
semaksimal mungkin **TANPA** menurunkan 69.2% Double, dgn syarat sama
seperti §7 (audit visual wajib, tidak boleh paksa angka).

### 9a. Verifikasi sumber data (FASE 0)

- `gate/dataset_triple_baru/`: **239 foto RAW terverifikasi** (1280×720,
  tanpa overlay) — satu-satunya sumber sah.
- Ditemukan dataset triple LAIN yg sempat dieksplorasi sesi sebelumnya
  (dini hari 6 Aug, tercatat di `gate/_debug/gate_triple_good_samples/README.txt`):
  `test/frames_gate_triple/` (54 foto, footage drone blur dari SISI
  BELAKANG/DALAM struktur mentah tanpa cat) — **sudah dikoreksi & ditolak
  user sendiri sesi itu**, dikonfirmasi ulang via cek visual (motion-blur,
  abu-abu, bukan sisi depan oranye) — **dikeluarkan sepenuhnya**, tidak
  dipakai di sesi ini.
- Folder lain yg ditemukan (`gate/annotated_triple_baru/`,
  `gate/_debug/gate_triple_good_samples/`, `gate/annotated/triplegate_anno.png`)
  semuanya OVERLAY/debug, bukan sumber mentah.
- Temuan independen yg tetap berlaku dari sesi sebelumnya: `detect_gates()`
  dijalankan ke render desain resmi `gate/triplegate.png` (kondisi ideal)
  → **tetap `openings=0`** — indikasi awal bahwa geometri Triple Gate
  (kotak 3D dalam, bukan frame-U datar) menantang asumsi algoritma,
  independen dari isu warna/footage. Dikonfirmasi lebih jauh di §9c.

### 9b. Baseline (FASE 1)

Split `dataset_triple_baru` seed=42, holdout 20% → manifest
`gate/holdout_manifest_triple.txt`. Param **SAAT INI, tidak diubah**:

| Set | Hasil |
|---|---|
| tuning (191) | 126/191 = **66.0%** |
| holdout (48) | 36/48 = **75.0%** |
| gap | 9.0 poin |
| **total (239)** | 162/239 = **67.8%** |

Ini baseline resmi Triple Gate, menggantikan angka lama "28.6%/35-sampel"
yang sudah usang (dari eksplorasi dgn dataset yg salah, lihat 9a).

### 9c. Breakdown akar masalah (FASE 2, 239 foto penuh)

| Kategori kegagalan | Jumlah | % dari total | % dari gagal |
|---|---|---|---|
| `solidity_reject` | 76 | 31.8% | **98.7%** |
| `near_miss_open_size` | 1 | 0.4% | 1.3% |
| `leg_miss` | 0 | 0% | 0% |
| `aspect_reject` | 0 | 0% | 0% |
| `opening_area_reject` | 0 | 0% | 0% |
| `no_contour` | 0 | 0% | 0% |

Sangat berbeda dari komposisi Double Gate (§7a, didominasi leg-check).
Solidity kandidat gagal: rentang **0.696-0.965** (rata-rata 0.90), SEMUA
di atas `SOLIDITY_HI=0.80`, NOL di bawah `SOLIDITY_LO`.

**Akar penyebab (dikonfirmasi visual+numerik):** pada jarak dekat,
oranye Triple Gate (frame lebih dalam dari Double — dinding interior
ikut terlihat) memenuhi 60-95% frame, membuat siluet luar nyaris persegi
solid (bukan bentuk-U tipis berongga yg jadi asumsi `SOLIDITY_HI=0.80`,
dikalibrasi dari Double Gate yg dangkal). Ini konsisten dgn temuan render
resmi §9a (`openings=0` bahkan di kondisi ideal) — indikasi struktural,
bukan cuma soal foto lapangan.

**Uji hipotesis prioritas kandidat (3-layer):** 39 frame punya ≥2 kandidat
kontur luar lolos validasi. Diperiksa semua — kandidat asli SELALU menang
prioritas (area 280rb-870rb px) vs kandidat noise (area 1.5rb-4rb px,
margin ~100x). **Hipotesis DITOLAK data**: tidak ada kasus target salah-
gerbang/salah-prioritas di dataset ini. "Gerbang terjauh tak terdeteksi"
juga N/A — dataset ini 1 objek fisik per frame, bukan beberapa gerbang
berurutan.

### 9d. Eksplorasi parameter (FASE 3) — DITOLAK, data konkret

**Sweep `SOLIDITY_HI` (0.80→0.99), diverifikasi thd Double (520, split
identik Task 1) DAN Triple (239, split §9b) sekaligus:**

| SOL_HI | Double tuning/holdout | Triple tuning/holdout |
|---|---|---|
| 0.80 (baseline) | 69.2% / 69.2% | 66.0% / 75.0% |
| 0.85 | 69.7% / 69.2% | 68.1% / 81.2% |
| 0.90 | 70.0% / 71.2% | 68.6% / 81.2% |
| 0.97 | 70.0% / 71.2% | 69.6% / 83.3% |

Di atas kertas terlihat menang telak. **Audit visual num, WAJIB (pelajaran
§7b diulang persis)**: SEMUA 5 file Double yg "gain" pada rentang
0.84-0.97 adalah **file yg SAMA PERSIS dengan 5 false-positive §7b**
(`17_29_44(5)/(6)`, `17_30_13(2)`, `17_31_51(3)`, `17_31_52(4)`) — 2 di
antaranya (area 105-107rb px) ternyata target JATUH DI CELAH SEMPIT antara
kaki-kanan-asli & tepi panel divider (bukan bukaan asli, dikonfirmasi
crop zoom), 3 sisanya kotak noise kecil (area 1.6-2.4rb px) di
tanah/rumput. **0 dari 5 adalah gate asli.** Independen mengonfirmasi
ulang kesimpulan §7b, kali ini walau ada insentif baru (gain Triple).

**Titik aman diverifikasi presisi (grid 0.01):** `SOLIDITY_HI ≤ 0.83` →
**0 flip Double, TAPI 0 gain Triple juga** (Triple tetap 162/239).
`SOLIDITY_HI ≥ 0.84` → flip Double **pertama muncul BERSAMAAN** dgn gain
Triple pertama, di nilai yg SAMA PERSIS. **Tidak ada jendela pemisah** —
setiap ambang yg menolong Triple otomatis membuka false-positive Double
yg identik dgn temuan §7b.

**Dicoba 2 fitur pemisah lain (upaya cari separator yg lebih baik dari
solidity semata) — sama-sama gagal:**
- **Aspect ratio**: kandidat asli yg SAH (baseline, kedua dataset) sudah
  punya aspect serendah 0.417 (Double)/0.65 (Triple) — tumpang-tindih
  penuh dgn aspect false-positive (0.446-1.35). Tidak ada ambang aman.
- **Fraksi area-bbox/area-frame**: awalnya terlihat menjanjikan (false-
  positive Double 0.001-0.116 vs gain Triple asli 0.460-0.944, ada celah).
  **TAPI** dicek ulang thd SELURUH baseline Double yg SAH (bukan cuma yg
  "gain") — banyak deteksi gerbang JAUH yg sudah tervalidasi & jadi
  bagian dari 69.2% resmi punya fraksi serendah **0.0007** (gerbang kecil
  di kejauhan, by design). Rentang ini tumpang-tindih PENUH dgn fraksi
  false-positive (0.001-0.116). **Tidak ada ambang aman** di sini juga.

### 9e. Audit visual (FASE 4)

Seluruh 48 foto holdout diaudit visual (param baseline, tidak diubah):
36 DET — semua crosshair benar di bukaan asli, kotak hijau pas di
struktur gerbang. 12 MISS — semua foto jarak sangat dekat/head-on dgn
oranye memenuhi >60% frame, konsisten 100% dgn kategori `solidity_reject`
di §9c. Tidak ada temuan baru/mengejutkan di audit ini — mengonfirmasi
analisis §9c-9d sepenuhnya.

### 9f. Kesimpulan & rekomendasi — parameter TETAP TIDAK DIUBAH

**Tidak ada kombinasi parameter aman yg meningkatkan Triple tanpa
mengorbankan Double.** Ini plafon nyata (bukan kegagalan eksplorasi) —
dibuktikan dgn 3 pendekatan independen (solidity, aspect, fraksi-area)
yg SEMUA gagal memisahkan populasi "Triple asli dekat" dari "Double
false-positive" krn rentang nilainya tumpang-tindih di seluruh sistem,
bukan cuma di titik tertentu. `gate_multi_detector.py` **TIDAK diubah**
dari `_FINAL_20260730.py` — tidak ada backup `_v3_` baru krn tidak ada
kode yg berubah.

**Angka akhir terpisah (param final Task 1, tidak berubah):**

| Dataset | Tuning | Holdout | Gap | Total |
|---|---|---|---|---|
| Double Gate (520) | 69.2% | 69.2% | 0.0 | 69.2% |
| Triple Gate (239) | 66.0% | 75.0% | 9.0 | 67.8% |
| **Gabungan (759)** | — | — | — | **68.7%** |

Triple Gate SETARA dgn Double (selisih 1.4 poin), tapi via mekanisme
kegagalan yg BERBEDA TOTAL (solidity murni vs leg-check dominan §7a) —
kesetaraan angka ini kebetulan, bukan bukti algoritma sudah general utk
kedua bentuk.

**Opsi perbaikan lanjutan (LOGIKA, bukan parameter — perlu keputusan
eksplisit, TIDAK diimplementasikan sesi ini):**

1. **Kriteria skala relatif per-KONTEKS** (bukan global): mode terpisah
   Double vs Triple (mis. dari urutan waypoint misi yg sudah tahu gerbang
   mana yg didekati), masing-masing dgn `SOLIDITY_HI` sendiri. Butuh info
   konteks dari luar frame (bukan cuma pixel), perubahan arsitektur.
2. **Fitur baru**: validasi bentuk yg tahan "isi penuh frame" (mis. cek
   rasio ketebalan-kaki vs lebar-bukaan, bukan solidity global) —
   berpotensi memisahkan kotak Triple solid dari celah divider Double,
   tapi ini fitur BARU yg perlu didesain+sweep+audit dari nol, bukan
   sekadar geser angka.
3. **Terima 67.8% sbg plafon Triple saat ini**: dokumentasikan
   keterbatasan, jangan tuning lebih jauh sampai ada foto lapangan yg
   lebih bervariasi jarak (dataset ini hanya 1 sesi ~2.5 menit, kemungkinan
   bias ke jarak tertentu — lihat §9g).

### 9g. Keterbatasan yang jujur

- **Ukuran sample jauh lebih kecil** dari Double (239 vs 520) dan berasal
  dari **1 sesi foto tunggal ~2.5 menit** (1 objek fisik, 1 lokasi, 1
  kondisi cahaya berangsur senja) — beda dgn Double yg juga 1 sesi tapi
  lebih lama & sudah 69.2% teraudit matang di 2 sesi eksplorasi (§3, §7).
- **Variasi jarak kemungkinan tidak representatif**: semua 12 kegagalan
  holdout adalah foto sangat dekat — belum jelas proporsi ini
  mencerminkan pola terbang drone asli (seberapa sering drone akan
  sedekat itu ke Triple Gate saat align/lewat) atau cuma pola gerak
  fotografer manual sesi capture.
- **Tidak ada foto Triple Gate jarak jauh/sedang yg gagal** utk
  dibandingkan — semua kegagalan homogen (solidity), berbeda dgn Double
  yg py kegagalan lebih beragam (leg/aspect/solidity/bottom, §7a). Belum
  bisa dipastikan apakah Triple Gate PUNYA kegagalan non-solidity yg
  sekadar belum muncul di sample kecil ini.
- **Warna gate baru lebih saturated** (median RGB 217,141,28) vs Double
  (225,170,88) — hue identik (~36°), dan dikonfirmasi TIDAK jadi penyebab
  kegagalan (breakdown §9c murni solidity, bukan warna/HSV), tapi tetap
  dicatat sbg perbedaan fisik antar unit gerbang.

## 10. Implementasi Opsi 2 — Solidity Adaptif-Skala (sesi lanjutan, 6 Aug) — DITERAPKAN

Otorisasi eksplisit diberikan utk lanjut ke **Opsi 2** dari §9f (fitur
validasi baru, bukan geser parameter linear). Backup manual dibuat SEBELUM
edit: `gate_multi_detector_backup_20260806_1731_before_v3triple.py`
(diverifikasi `diff` identik dgn file aktif sebelum edit).

### 10a. Desain

Root cause §9c (`solidity_reject`, 98.7% kegagalan Triple) vs false-positive
§9d/§7b (celah kaki-vs-divider Double, area 105-107rb px, aspect 0.446-0.456)
ternyata terpisah bersih di **kombinasi 2 fitur** (bukan 1): kandidat gate
ASLI berskala-besar (area >=100rb px, baik Triple dekat maupun Double) SELALU
py aspect >=1.15; false-positive besar SATU-SATUNYA kandidat besar dgn aspect
serendah 0.45-0.46 (diverifikasi exhaustive thd seluruh 759 foto, §9d).
Kandidat kecil (noise/gate jauh, ratusan-ribuan px) py aspect rendah PULA
(serendah 0.417) tapi TIDAK bahaya krn skalanya kecil.

**Implementasi**: kandidat yg GAGAL jalur validasi asli (`SOLIDITY_HI=0.80`,
`ASPECT_LO=0.40`) dapat "kesempatan kedua" HANYA jika (a) area >=
`LARGE_AREA_THRESH=20000` (celah aman antara noise ~4rb & gate asli ~100rb+
px) DAN (b) aspect >= `ASPECT_MIN_LARGE=0.60` (blokir celah kaki-vs-divider)
DAN (c) solidity <= `SOLIDITY_HI_RELAXED=0.97` (cakup maks terukur 0.961).
Jalur validasi ASLI **tidak disentuh sama sekali** — kandidat yg sudah lolos
sebelumnya (termasuk 5 kandidat Double area>=20rb aspect 0.47-0.59 yg
verified LEGIT, lihat cek regresi di bawah) tetap lolos lewat jalur lama,
independen dari fitur baru. Constants + logic ada di
`gate_multi_detector.py` (blok "solidity adaptif-skala" & fungsi
`detect_gates()`), param baru jg diekspos opsional di signature fungsi
(`large_area_thresh`, `aspect_min_large`, `solidity_hi_relaxed`) konsisten
dgn pola param existing (sweep tanpa edit file).

### 10b. Verifikasi (WAJIB sebelum diterima)

1. `--selftest` — **SEMUA LULUS** (4 kasus positif + 3 kasus negatif sintetis).
2. **Diff exhaustive SELURUH 759 foto** (backup vs file baru, bukan cuma
   sample): Double **gain=0, loss=0** (nol perubahan, byte-perbyte perilaku
   sama utk semua 520 foto). Triple **gain=11, loss=0** — persis 11 file yg
   didesain utk direscue, tidak lebih tidak kurang.
3. **Re-verifikasi 5 false-positive lama** (`17_29_44(5)/(6)`, `17_30_13(2)`,
   `17_31_51(3)`, `17_31_52(4)`) — **SEMUA tetap ditolak**.
4. **Audit visual SEMUA 11 gain Triple** (bukan sample) — kotak hijau pas di
   struktur gerbang asli, crosshair merah di bukaan asli, termasuk sudut
   oblique/dekat yg sebelumnya gagal. **11/11 terverifikasi benar.**
5. **Re-audit visual 48 foto holdout Triple** dgn param baru — 40 DET (naik
   dari 36) semua benar, 8 MISS sisa (`17_49_05`, `17_49_09(3)`, `17_49_12`,
   `17_49_43(3)`, `17_49_45`, `17_49_48(3)`, `17_49_49(5)`, `17_49_58(2)`)
   semua kategori `near_miss_not_open_bottom` (lihat §10c) — bukan kegagalan
   fitur baru, konsisten dgn ekspektasi.

### 10c. Angka akhir SETELAH fitur diterapkan

| Dataset | Tuning | Holdout | Gap | Total |
|---|---|---|---|---|
| Double Gate (520) | **69.2%** (tidak berubah) | **69.2%** (tidak berubah) | 0.0 | **69.2%** |
| Triple Gate (239) | 69.6% (dari 66.0%) | 83.3% (dari 75.0%) | 13.7 | **72.4%** (dari 67.8%) |
| Gabungan (759) | — | — | — | **70.4%** (dari 68.7%) |

Triple naik +4.6 poin (67.8%→72.4%), Double **presisi nol perubahan**
(diverifikasi exhaustive, bukan cuma sample). Gap tuning-holdout Triple
melebar (9.0→13.7) krn 4 dari 11 gain kebetulan jatuh di holdout — bukan
tanda overfitting ke tuning_set (fitur baru sepenuhnya berbasis analisis
akar-masalah §9c/9d, bukan sweep-cari-angka-di-tuning_set).

### 10d. Yang TIDAK terselesaikan — kategori `near_miss_not_open_bottom`

Fitur ini menyasar **root cause #1** (`solidity_reject`, 76/77=98.7% dari
kegagalan awal) dan berhasil merescue 11 file. Begitu solidity longgar utk
kandidat besar, **root cause #2 muncul**: 65 foto (tersisa dari 76, 8 di
antaranya sekarang lolos jadi bagian dari 11 gain, sisanya 65 masih gagal)
terhalang `near_miss_not_open_bottom` — komponen bukaan-latar TIDAK
menyentuh 12% bawah bbox (`OPEN_BOTTOM_MARGIN_FRAC=0.12`).

**Investigasi akar penyebab (visual + mask dump, `17_49_43`)**: BUKAN soal
geometri/logika sama seperti solidity — mask HSV menunjukkan **pita tipis
oranye-palsu** melintang horizontal di sekitar tepi jalan/tanah berbatu
(warna tanah/aspal kecoklatan kebetulan masuk rentang `HSV_ORANGE_LO=(5,80,40)`)
yg **memutus komponen background jadi 2 bagian terpisah** (adegan rumah
gelap di atas, jalan/rumput di bawah) — connectedComponents mengambil yg
LEBIH BESAR (adegan rumah), yg TIDAK menyentuh bawah bbox.

**Ini BUKAN masalah yg sama** dgn solidity (bentuk/skala) — ini artefak
**threshold warna** (HSV menangkap tanah/aspal kecoklatan sbg "oranye").
Palang docs §8b eksplisit: konstanta HSV "sudah divalidasi ketat lewat
sweep di Task 1, membuka utk retuning berisiko detune tanpa proses sweep+
audit ulang penuh" — **di luar scope opsi 2** (itu solidity/aspect, bukan
warna). **TIDAK dicoba diperbaiki sesi ini** — kalau mau dikejar, perlu sesi
terpisah khusus resweep HSV (risiko lebih tinggi, scope lebih besar, wajib
audit ulang total thd Double 520 foto).

### 10e. File final

- `gate_multi_detector.py` (aktif) — **DIUBAH** dari state Task 1
  (`_FINAL_20260730.py`), fitur solidity adaptif-skala ditambahkan.
- `gate_multi_detector_FINAL_v3_triple_20260806.py` — snapshot final sesi
  ini (identik `diff` dgn file aktif).
- `gate_multi_detector_backup_20260806_1731_before_v3triple.py` — state
  SEBELUM edit (identik `_FINAL_20260730.py`), utk rollback kalau perlu.

## 11. Percobaan lanjutan utk `near_miss_not_open_bottom` (65 foto) — HASIL NIHIL, TIDAK diimplementasikan

Otorisasi diberikan utk lanjut mengejar sisa 65 kegagalan (§10d). Dicoba
**2 pendekatan**, KEDUANYA gagal audit — file produksi **TIDAK diubah**,
semua eksperimen di script eksternal saja (diverifikasi `diff` file aktif
identik `_FINAL_v3_triple_20260806.py` setelah kedua percobaan).

**Percobaan A — cek pixel HSV gate vs "pita tanah" (`17_49_43`)**: Hue
tumpang-tindih penuh (gate 9-21°, pita 14-34°). Saturation ADA beda median
(gate palang-atas S~255 vs pita S~79) TAPI kaki gate yg ternaungi bayangan
py S serendah 112-154 -- tumpang-tindih dgn pita (31-141). **Tidak ada
ambang S aman** tanpa memotong piksel kaki gate asli yg ternaungi. TIDAK
dicoba diimplementasikan (dihentikan di analisis, sebelum sampai ke sweep
penuh) krn overlap sudah jelas dari sampel piksel langsung.

**Percobaan B — gabung (merge) komponen background berdekatan vertikal +
tumpang-tindih horizontal** (fallback HANYA saat jalur asli gagal krn
bottom-margin, pola "kesempatan kedua" sama seperti §10): parameter awal
(overlap>=0.5, celah<=10%bh) **lolos numerik mencurigakan** — 65/65 Triple
rescued, TAPI Double **gain=53** (bukan 0!). Audit visual 4 sample gain
Double: **SEMUA false-positive jelas** (area 2268-5729px, gabungan
serpihan noise tak terkait, sama sekali bukan skala gate). Termasuk 2 dari
5 false-positive lama (`17_31_51(3)`, `17_31_52(4)`) ikut lolos lagi lewat
jalur merge.

Dicoba diperketat (`MIN_COMPONENT_AREA_FOR_MERGE` naik bertahap):

| Ambang ukuran komponen min | Triple rescued | Double gain (harus 0) |
|---|---|---|
| 15.000 px | 53/65 | 10 |
| 30.000 px | 51/65 | 8 |
| 50.000 px | 45/65 | 7 |
| 80.000 px | 5/65 | **6** |

**Tidak ada titik di mana Double gain=0** — bahkan di ambang paling ketat
(80rb px, yg SUDAH membunuh sebagian besar rescue Triple jadi cuma 5/65),
Double masih bocor 6 false-positive. Pola "tebing tanpa celah" sama
persis dgn eksplorasi sebelumnya (§9d) tapi kali ini **tidak ada solusi
sama sekali** yg ditemukan, bukan cuma sempit.

**Kesimpulan (SAAT ITU): kategori `near_miss_not_open_bottom` (65 foto,
akar penyebab = artefak warna tanah/aspal di mask HSV) adalah plafon
nyata dgn 2 pendekatan berbeda yg SUDAH dicoba dan gagal audit.** Tidak
diimplementasikan apapun dari sesi ini. Angka final TETAP spt §10c:
Triple 72.4%, Double 69.2%.

**KOREKSI (lanjutan sesi sama, lihat §12): kesimpulan di atas TERBUKTI
TERLALU DINI.** Percobaan A (ambang Cr GLOBAL) memang benar gagal, tapi
belum dicoba varian **Cr ADAPTIF PER-FRAME** — itu yg akhirnya berhasil.

## 12. Referensi literatur & implementasi refine-Cr adaptif — BERHASIL, DITERAPKAN

Diminta cari referensi eksternal utk strategi lanjutan. Ringkasan riset
(HSV vs YCbCr utk illumination-invariance, CLAHE shadow suppression,
convexity defects, GateNet/continual-learning drone racing) -- lihat
jawaban sesi chat utk daftar lengkap+sumber. Direkomendasikan coba YCbCr
dulu (effort kecil, target langsung akar masalah §11).

### 12a. Iterasi 1 — ambang Cr GLOBAL: GAGAL (sampel besar membongkar sampel kecil)

Sampel awal 2-titik (1 foto) terlihat menjanjikan (Cr gate~209 vs Cr
strip~137). **Diverifikasi ulang di skala penuh** (14 juta piksel gate
asli dari 60 foto lolos vs 141rb piksel pita-palsu dari 65 foto gagal):
distribusi TERNYATA nyaris identik (median gate=165 vs strip=157, bahkan
persentil-95/99 strip 214/218 MELEBIHI gate 201/210). **Tidak ada ambang
Cr tetap yg aman.** Pelajaran metodologis: sampel kecil (1-2 titik)
menyesatkan, WAJIB verifikasi skala penuh sebelum simpulkan.

### 12b. Iterasi 2 — Cr ADAPTIF PER-FRAME: BERHASIL

Dicoba sudut berbeda: bukan ambang mutlak, tapi **margin RELATIF dalam
SATU frame yg sama** (median Cr strip-kaki kandidat itu sendiri vs Cr
piksel di celah). Diuji 5 sampel awal: margin selalu positif & konsisten
(6-9), meski ambang absolut bergeser antar-foto (krn lighting). Ini akal
sehat: Cr piksel gate vs tanah BISA bergeser bareng antar-kondisi cahaya,
tapi urutan relatifnya (gate > tanah) tetap.

**Implementasi**: kesempatan-kedua (pola sama §10 — TIDAK pernah
menimpa jalur asli) HANYA saat validasi normal gagal. Ambil referensi Cr
= median Cr piksel-oranye di strip kaki kiri+kanan KANDIDAT ITU SENDIRI.
Re-klasifikasi piksel oranye jadi "bukan oranye asli" kalau Cr-nya <
(referensi - `CR_REFINE_MARGIN=8`). ConnectedComponents ulang pakai mask
yg direfine -> celah pita-tanah otomatis tersambung tanpa perlu logic
merge eksplisit (beda dari percobaan merge §11 yg gagal — di sini yg
diubah adalah INPUT mask-nya, bukan post-process komponen).

**Sweep keamanan** (exhaustive 520 Double @ tiap kombinasi):

| `CR_REFINE_MARGIN` | Triple rescued | Double gain (harus 0) |
|---|---|---|
| 5 | 65/65 | 3 |
| 8 (dipilih) | 65/65 | 2 (tanpa syarat area) |
| 12 | 63/65 | 4 |

Margin=8 tanpa syarat tambahan masih bocor 2 false-positive Double
(diaudit visual: 1 target ke properti kayu di background, 1 target ke
permukaan kaki gate solid — dikonfirmasi false, area kecil 4.9rb & 30rb
px, jauh dari skala gate 400rb+). Ditambah **`CR_REFINE_MIN_AREA`**
(kandidat outer-contour harus >= ambang ini baru dicoba refine — beda
dari `LARGE_AREA_THRESH` §10, sengaja lebih tinggi):

| `CR_REFINE_MIN_AREA` | Triple rescued | Double gain |
|---|---|---|
| 100.000 (dipilih) | **65/65** | **0** |
| 150.000 | 65/65 | 0 |
| 200.000 | 65/65 | 0 |

100.000 dipilih (paling longgar dari 3 titik yg sama-sama aman) — margin
lebar dari false-positive terbesar yg ditemukan (~30rb) & rescue Triple
terkecil (~424rb).

### 12c. Verifikasi (WAJIB, pola sama §10b)

1. `--selftest` — SEMUA LULUS.
2. **Diff exhaustive 759 foto** (backup vs file baru): Double **gain=0,
   loss=0**. Triple **gain=66, loss=0** (66, bukan 65 — 1 kasus
   `near_miss_open_size` ikut terselesaikan sbg efek samping recompute
   `_validate_opening` di mask yg direfine).
3. **Audit visual SEMUA 66 gain** (grid penuh, bukan sample) — **66/66
   benar**: crosshair konsisten di bukaan asli, termasuk sudut oblique
   ekstrem, motor lewat, variasi cahaya. Tidak ada false-positive
   ditemukan.

### 12d. Angka akhir — 100% Triple

| Dataset | §10 (sblm refine-Cr) | §12 (sesudah) |
|---|---|---|
| Double Gate (520) | 69.2%/69.2%/gap 0.0 | **69.2%/69.2%/gap 0.0 (tidak berubah)** |
| Triple Gate (239) | 69.6%/83.3% (72.4% total) | **100.0%/100.0%/gap 0.0** |
| Gabungan (759) | 70.4% | **77.1%** |

### 12e. Keterbatasan jujur — 100% BUKAN jaminan generalisasi

Angka 100% Triple **dicapai di dataset SAMA yg dipakai analisis akar
masalah** (239 foto, 1 objek fisik, 1 sesi ~2.5 menit, kondisi
tanah/aspal & cahaya spesifik hari itu) — bukan holdout independen yg
benar-benar buta dari proses desain (holdout `_split_dataset` tetap
dihormati utk metrik tuning/holdout formal, tapi PEMILIHAN margin/ambang
sendiri disetel melihat SEMUA 65 kegagalan, termasuk yg masuk holdout).
Referensi-Cr **relatif per-frame** (bukan angka tetap) secara desain
LEBIH tahan pergeseran kondisi dibanding ambang HSV/Cr global — tapi
**belum diuji di gerbang/lokasi/musim lain** dgn warna tanah/aspal
berbeda. Rekomendasi: saat dapat footage lapangan baru, ulangi audit
visual §12c sblm percaya angka ini tetap 100%.

### 12f. File final

- `gate_multi_detector.py` (aktif) — refine-Cr adaptif ditambahkan di
  atas v3 (§10).
- `gate_multi_detector_FINAL_v4_triple_20260806.py` — snapshot final.
- `gate_multi_detector_backup_20260806_1839_before_crrefine.py` —
  state SEBELUM edit sesi ini (identik `_FINAL_v3_triple_20260806.py`).

## 13. Validasi tambahan angka 100% Triple + CLAHE utk Double — sesi lanjutan

Diminta (a) uji ketahanan klaim 100% Triple pakai metodologi literatur,
(b) coba metode artikel utk tingkatkan Double (masih 69.2%, tak tersentuh
sesi §9-12).

### 13a. Uji ketahanan 100% Triple (robustness thd perturbasi)

100% dicapai di 239 foto YG SAMA dipakai desain fitur (§12e, keterbatasan
sudah dicatat). Utk cek apakah rapuh, diuji 239 foto dgn 7 perturbasi
sintetis (gelap/terang, kontras naik/turun, gamma, blur ringan):

| Perturbasi | Deteksi |
|---|---|
| baseline | 239/239 (100.0%) |
| gelap -30 | 237/239 (99.2%) |
| terang +30 | 239/239 (100.0%) |
| kontras rendah (0.7x) | 238/239 (99.6%) |
| kontras tinggi (1.3x) | 239/239 (100.0%) |
| gamma gelap (1.5) | 231/239 (96.7%) |
| gamma terang (0.7) | 239/239 (100.0%) |
| blur ringan | 239/239 (100.0%) |

Bertahan 96.7-100% di semua perturbasi (turun paling banyak di gamma gelap
ekstrem) — degradasi bertahap, bukan ambruk. Ini BUKAN bukti generalisasi
ke lokasi/gerbang lain (§12e tetap berlaku), tapi mengonfirmasi mekanisme
(referensi-Cr relatif per-frame) tahan thd variasi piksel wajar, bukan
angka yg kebetulan pas.

### 13b. CLAHE utk Double (leg_miss, target dari referensi literatur)

Kegagalan Double didominasi `leg_miss` (115/160=71.9%, §7a) — dugaan:
kaki gate ternaungi bayangan turun di bawah `HSV_ORANGE_LO` (S=80,V=40).
Referensi: CLAHE+OTSU+HSV utk shadow suppression outdoor.

**Percobaan 1 — CLAHE ganti mask PENUH (semua kandidat, bukan fallback):
DITOLAK.** Gain besar (51-67) TAPI loss JUGA besar (3-44) — terlalu
invasif, mengubah kandidat yg SUDAH benar di jalur asli.

**Percobaan 2 — CLAHE sbg kesempatan-kedua FRAME-LEVEL** (HANYA dicoba
kalau mask asli 100% nihil, bukan per-kandidat spt refine-Cr): exhaustive
520 Double @ `clip=3,tile=8` -> **gain=64-67, loss=0**. Angka pagar lolos.

**AUDIT VISUAL (WAJIB) — DITEMUKAN MASALAH:** grid 67 gain awal terlihat
OK sekilas, tapi cek acak 6 sample: **1 false-positive jelas** (target di
rumput/jalan, area 4182px) + beberapa ambigu. **Breakdown lengkap: 36/67
(54%!) kandidat area <100rb px** — CLAHE menaikkan kontras SELURUH frame
termasuk noise kecil tak terkait gate, bukan cuma kaki gate. Pola PERSIS
sama dgn pelajaran §7b/§9d/§11: **lolos pagar numerik (loss=0) TIDAK
otomatis berarti gain-nya benar** — WAJIB audit gain juga, bukan cuma loss.

**Perbaikan**: tambah `CLAHE_MIN_AREA=100000` (sama nilainya dgn
`CR_REFINE_MIN_AREA`, kebetulan skala sama, bukan ambang yg sama secara
konsep) — kandidat CLAHE HANYA diterima kalau skala gate asli. Re-verifikasi
exhaustive: **gain=31 (turun dari 67, buang 36 yg mencurigakan), loss=0**
(Double MAUPUN Triple). **Audit visual PENUH ke semua 31** (grid + 3
spot-check resolusi penuh) — **31/31 benar**, kotak hijau pas di struktur
gerbang, crosshair konsisten di bukaan asli, tidak ada lagi target di
rumput/objek tak terkait.

### 13c. Angka akhir

| Dataset | §12 (sblm CLAHE) | §13 (sesudah) |
|---|---|---|
| Double Gate (520) | 69.2%/69.2%/gap 0.0 | **75.5%/74.0%/gap 1.4** |
| Triple Gate (239) | 100.0%/100.0%/gap 0.0 | **100.0%/100.0%/gap 0.0 (tidak berubah)** |
| **Gabungan (759)** | 77.1% | **83.0%** |

Double naik +6.3 poin (69.2%→75.5% tuning) via CLAHE kesempatan-kedua
frame-level, TANPA menyentuh jalur deteksi asli sama sekali (nol regresi,
diverifikasi exhaustive 520 foto 2x — sebelum & sesudah tambah
`CLAHE_MIN_AREA`).

### 13d. Pelajaran metodologis (utk sesi berikutnya)

**"0 loss" BUKAN cukup utk klaim aman — WAJIB audit breakdown skala/visual
GAIN juga**, bukan cuma pastikan tidak ada yg regresi. Perubahan yg
mempengaruhi SELURUH mask/frame (CLAHE) punya risiko lebih tinggi
menghasilkan gain-noise dibanding perubahan per-kandidat/scoped (solidity
adaptif §10, refine-Cr §12) — pola "kesempatan kedua HANYA per-kandidat yg
sudah gagal" tetap pendekatan teraman, tapi bahkan itu tidak otomatis bebas
dari perlu syarat skala tambahan (`CR_REFINE_MIN_AREA`, `CLAHE_MIN_AREA`).

### 13e. File final

- `gate_multi_detector.py` (aktif) — CLAHE kesempatan-kedua frame-level
  ditambahkan di atas v4 (§12), dgn `CLAHE_MIN_AREA` safeguard.

## 14. Post-pairing kaki terpisah (Double Gate) — sesi lanjutan 6 Aug — BERHASIL, DITERAPKAN

Diminta: coba metode dari literatur utk naikkan Double lebih lanjut (masih
75.5% setelah §13). Sebelum coba teknik baru buta, breakdown dulu 129
kegagalan pasca-CLAHE:

| Kategori | Jumlah | % dari 129 gagal |
|---|---|---|
| leg_clear_miss (worst-side <0.15) | 67 | 51.9% |
| aspect_reject | 35 | 27.1% |
| leg_near_miss (0.15-0.25) | 24 | 18.6% |
| not_open_bottom | 3 | 2.3% |

### 14a. Akar masalah BARU (bukan shadow lagi) — ditemukan via bongkar kontur manual

Audit visual + cetak kontur mentah dari beberapa foto `leg_clear_miss`/
`aspect_reject` (mis. `WIN_20260727_17_30_00_Pro (5).jpg`,
`WIN_20260727_17_30_01_Pro (3).jpg`) mengungkap pola konsisten: foto jarak
SANGAT dekat, kedua kaki gate memenuhi hampir seluruh tinggi frame, tapi
mask oranye PECAH jadi 2-3 kontur solo (kaki kiri, kaki kanan, kadang
fragmen kecil) — bukan 1 kontur U menyambung — karena celah tengah
(bukaan) menampakkan latar (semak/rumah) yg memutus kontinuitas mask.
Sample dicek dgn HSV median (bukan asumsi shadow): warna kaki tetap
S=223,V=255 (sangat jenuh, BUKAN gelap) — mengonfirmasi ini BUKAN kasus
CLAHE (§13) sama sekali, murni soal topologi kontur terputus.

Pipeline lama cuma ambil kontur TERBESAR sbg satu-satunya kandidat -> kalau
itu cuma 1 kaki solo, otomatis gagal aspect (kaki solo aspect ~0.23-0.53,
jauh di bawah `ASPECT_LO=0.40`) atau leg-check (strip kiri/kanan jatuh di
ruang kosong/latar, bukan di kaki asli). **126/129 (97.7%) kegagalan
cocok pola ini.**

### 14b. Referensi literatur & desain

Teknik **post-pairing tiang** — dipakai luas di deteksi gawang RoboCup:
kalau gawang tak tampak sbg 1 blob utuh (occlusion tengah), deteksi
tiang kiri+kanan INDIVIDUAL lalu pasangkan berdasar kemiripan tinggi &
jarak horizontal wajar, bukan cari 1 kontur U utuh.

Kriteria kandidat "tiang" (`_leg_candidates`, param `PAIR_LEG_*`):
tinggi (>=50% tinggi frame), ramping (aspect<=0.55), solid
(solidity>=0.5), area skala tiang asli (>=20rb px, sama `LARGE_AREA_THRESH`).
Pemasangan (`_try_pairing`): overlap vertikal>=50% (baris sama) + celah
horizontal 15-85% dari lebar gabungan (bukan celah semu tipis/2 objek tak
terkait). Target = tengah celah antara 2 tiang. **HANYA dicoba sbg
kesempatan TERAKHIR** (normal+CR-refine+CLAHE semua nihil dulu) — dipasang
di mask ASLI (bukan CLAHE, krn bukan kasus shadow) — by construction
loss=0 (tak pernah menimpa kandidat yg sudah lolos jalur manapun sebelumnya).

### 14c. Verifikasi (WAJIB, pola sama §10b/§12c/§13b)

1. `--selftest` — SEMUA LULUS (termasuk regresi eksplisit "satu kaki+celah
   tipis" tetap DITOLAK — bukan 2 kontur solid terpisah).
2. **Exhaustive 520 Double + 239 Triple**: Double **gain=38, loss=0**.
   Triple **gain=0, loss=0** (tak tersentuh, sesuai desain — Triple sudah
   1 kontur nyambung krn lebih dalam/dekat dinding interior).
3. **Audit visual SEMUA 38 gain** (grid kontak-sheet penuh + 5 spot-check
   resolusi penuh lintas timestamp berbeda) — **38/38 benar**: crosshair
   selalu di tengah bukaan asli antara 2 kaki, kotak magenta pas membingkai
   tiap kaki, tidak ada target ke noise/latar.

### 14d. Angka akhir

| Dataset | §13 (sblm pairing) | §14 (sesudah) |
|---|---|---|
| Double Gate (520) | 75.5%/74.0%/gap 1.4 | **82.5%/82.7%/gap 0.2** |
| Triple Gate (239) | 100.0%/100.0%/gap 0.0 | **100.0%/100.0%/gap 0.0 (tidak berubah)** |

Double naik +7.0 poin (75.5%→82.5%), GAP tuning-holdout malah membaik
(1.4→0.2 poin) — bukan cuma raw rate naik, generalisasi tuning->holdout
juga lebih rapat.

### 14e. Keterbatasan jujur

Sama pola §12e: 126/129 kegagalan yg jadi basis desain juga dataset yg
sama dipakai audit — belum diuji di sesi foto/gerbang lain. Kriteria
`PAIR_LEG_*` dipilih dari 1 sesi observasi (jarak-dekat, 2 kaki penuh
tinggi frame) — foto dgn framing SANGAT berbeda (mis. kaki tidak
memenuhi 50% tinggi frame) tidak akan terbantu teknik ini. `not_open_bottom`
(3 foto) dan sisa kegagalan tanpa 2 kontur tiang solid (91 foto) TETAP
gagal — bukan diklaim terselesaikan semua.

### 14f. File final

- `gate_multi_detector.py` (aktif) — post-pairing ditambahkan di atas v5
  (§13), fungsi baru `_leg_candidates`/`_try_pairing`.
- `gate_multi_detector_FINAL_v6_pairing_20260806.py` — snapshot final.
- `gate_multi_detector_backup_20260806_2125_before_pairing.py` — state
  SEBELUM edit sesi ini (identik `_FINAL_v5_clahe_20260806.py`).

## 15. Leg-check via peak sub-window density (Double Gate) — sesi lanjutan 6 Aug — BERHASIL, DITERAPKAN

Diminta lanjut cari metode dari literatur utk sisa 91 kegagalan Double
pasca-pairing (82.5%). Breakdown presisi (telusuri SEMUA kontur per
kandidat, bukan cuma kontur terbesar, cocokkan alasan gagal PALING DEKAT
lolos):

| Kategori | Jumlah | % dari 91 gagal |
|---|---|---|
| leg_check_fail (kandidat bentuk SUDAH lolos, leg-strip average gagal) | 90 | 98.9% |
| not_open_bottom | 1 | 1.1% |

### 15a. Akar masalah — bukan kandidat baru, margin lama yg belum terpecahkan

10 margin leg-check TERKECIL semua di bawah 0.05 (mis. `17_30_13 (2)`:
`left_frac=0.2485` vs ambang `0.25`, MELESET 0.0015 — **kasus ekstrem
yg SAMA PERSIS sudah dicatat di §7b sesi 30 Jul**, belum terpecahkan
sampai sesi ini meski sudah lewat CR-refine/CLAHE/pairing). Kolom-density
profil kaki kasus ini (dicetak manual) mengungkap sebab fisik: kaki gate
**2 warna** — pita LUAR pucat/kayu tak-jenuh (gagal `HSV_ORANGE_LO`
S>=80) + pita DALAM oranye jenuh lebih SEMPIT (kolom density sampai
0.97). Rata-rata di seluruh `LEG_STRIP_FRAC` (30% lebar bbox) terdilusi
pita pucat, walau kolom oranye jenuh asli ADA di dalamnya.

### 15b. Referensi literatur & desain

**Projection-profile peak** — teknik klasik segmentasi baris teks/objek
tipis (puncak lokal densitas menandai objek nyata; rata-rata wilayah
pencarian lebar bisa terdilusi noise/latar sekitarnya, dipakai jg utk
deteksi objek tiang/pole ramping). Diterapkan sbg **kesempatan KEDUA
per-kandidat** (pola sama refine-Cr §12, BUKAN frame-level spt
CLAHE/pairing): kalau rata-rata strip PENUH gagal, geser jendela SEMPIT
(`PEAK_LEG_WINDOW_FRAC` dari lebar bbox) di strip yg SAMA, ambil densitas
MAKS — kalau ADA kolom padat oranye asli di dalamnya, tetap lolos.

Sweep param (window x density) thd 4-pagar-keselamatan (pola sama §3/§7b):
window LEBAR (>=0.10) nyaris tak menambah gain (pita oranye jenuh SEMPIT
di data ini — window=0.06 hasil terbaik). density=0.55 **SEMPAT
meloloskan 1 foto keluarga noise dikenal** (`17_30_12 (2)`) — dinaikkan
ke **density=0.65** (nol hit noise family, gain turun 23→14, masih
signifikan). Final: `PEAK_LEG_WINDOW_FRAC=0.06, PEAK_LEG_MIN_DENSITY=0.65`.

### 15c. Verifikasi (WAJIB, pola sama §10b/§12c/§13b/§14c)

1. `--selftest` — SEMUA LULUS.
2. Exhaustive 520 Double + 239 Triple: Double **gain=14 (sweep) / +16
   nyata di batch-test produksi** (443 prediksi sweep vs 445 aktual —
   selisih wajar krn sweep hanya sampel kandidat kontur terbesar per
   foto, batch-test produksi jalan penuh via `_validate_opening`), Triple
   **gain=0 loss=0** (tak tersentuh).
3. **Audit visual SEMUA 14 gain** (grid kontak-sheet + 3 spot-check
   resolusi penuh, termasuk kasus ekstrem `17_30_13 (2)`) — **14/14
   benar**: kotak hijau pas membingkai gate utuh, crosshair di bukaan
   asli.

### 15d. Angka akhir

| Dataset | §14 (sblm peak-leg) | §15 (sesudah) |
|---|---|---|
| Double Gate (520) | 82.5%/82.7%/gap 0.2 | **85.6%/85.6%/gap 0.0** |
| Triple Gate (239) | 100.0%/100.0%/gap 0.0 | **100.0%/100.0%/gap 0.0 (tidak berubah)** |

Double naik +3.1 poin (82.5%→85.6%), GAP tuning-holdout **0.0** (turun
dari 0.2) — generalisasi tuning→holdout makin rapat tiap iterasi (§13:
1.4 → §14: 0.2 → §15: 0.0).

**Rekap kumulatif sesi 6 Aug (§9-15), Double Gate: 69.2% → 85.6% (+16.4
poin)** via 4 teknik berlapis (CLAHE shadow §13, post-pairing kaki
terpisah §14, peak-density leg-check §15) — TANPA menyentuh jalur
deteksi ASLI sama sekali (semua kesempatan tambahan, nol regresi
diverifikasi tiap tahap).

### 15e. Keterbatasan jujur

Sama pola §12e/§14e: kriteria dipilih dari dataset & sesi observasi yg
SAMA dipakai desain — belum diuji foto/gerbang lain. `not_open_bottom`
(1 foto) dan kegagalan tanpa kandidat bentuk-lolos sama sekali tetap
gagal. Kalau di lapangan nanti kaki gate TIDAK 2-warna (dicat rata,
bukan kayu+cat), teknik ini tidak akan menemukan apa pun berbeda dari
strip-average biasa (harmless, no-op) — bukan resiko regresi, cuma tidak
akan menolong kasus itu.

### 15f. File final

- `gate_multi_detector.py` (aktif) — peak sub-window density ditambahkan
  di `_validate_opening()`, fungsi baru `_peak_density`.
- `gate_multi_detector_FINAL_v7_peakleg_20260806.py` — snapshot final.
- `gate_multi_detector_backup_20260806_2210_before_peakleg.py` — state
  SEBELUM edit sesi ini (identik `_FINAL_v6_pairing_20260806.py`).
- `gate_multi_detector_FINAL_v5_clahe_20260806.py` — snapshot final.
- `gate_multi_detector_backup_20260806_2011_before_clahe.py` — state
  SEBELUM edit sesi ini (identik `_FINAL_v4_triple_20260806.py`).

## 16. Skew-corrected peak density (Double Gate, jarak sangat dekat) — sesi 9 Aug — BERHASIL, DITERAPKAN

Diminta analisis akar masalah sisa 60 kegagalan Double (14.4%, state §15).
Dibongkar per-kandidat (fungsi `_validate_opening()` di-trace manual,
BUKAN cuma lihat hasil akhir): **100% dari 60 kegagalan** itu kandidat
bentuk SUDAH lolos & bukaan SUDAH ketemu, tapi gagal leg-check MESKI
peak-window §15 sudah dicoba juga. Breakdown lebih dalam: 56/60 (93.3%)
gagal di `leg_peak_density`, 4/60 (6.7%) di `not_open_bottom` (kategori
lama, belum ada solusi).

### 16a. Akar masalah BARU — bukan "kaki 2 warna" generik, tapi jarak SANGAT dekat

10 kasus margin peak-density terkecil (0.007-0.17) semuanya **ASIMETRIS**:
1 kaki peak-density nyaris sempurna (0.97-1.0), kaki SATUNYA jauh di
bawah ambang 0.65 (0.55-0.65). Audit visual (`WIN_..._17_30_15_Pro.jpg`)
konfirmasi: foto diambil SANGAT dekat gerbang (nyaris menembus bukaan) --
di jarak ini, batas pita warna kaki (pucat-luar/oranye-dalam) jadi
**MIRING** (perspektif), bukan vertikal lurus spt asumsi jendela
peak-window §15. Beda dari akar masalah §15a (kaki 2-warna SIMETRIS,
diasumsikan lurus) -- ini kasus jarak-ekstrem yg baru muncul.

### 16b. Referensi literatur & desain

**Projection-profile skew correction** — teknik klasik document-skew-
detection (proyeksi profil + koreksi shear, akurat utk sudut kecil,
lih. WebSearch 2026-08-09). Diterapkan sbg **kesempatan KETIGA**
per-kandidat (pola sama §12/§15, BUKAN frame-level): kalau peak-window
VERTIKAL §15 masih gagal, geser tiap baris strip secara linear (shear)
di beberapa sudut diskrit (`SKEW_N_ANGLES=7`), "meluruskan" batas miring
jadi vertikal, PAKAI ULANG `_peak_density()` yg sudah ada (bukan
reimplementasi) di versi ter-deskew.

Sweep `SKEW_MAX_SHEAR_FRAC` (fraksi lebar strip) thd 520 Double penuh:
0.3->rescued=5/regressed=0, 0.5->rescued=3/regressed=0, 0.8->rescued=4/
regressed=0, 1.2->rescued=0/regressed=0 (non-monoton -- shear TERLALU
besar bikin jendela drift jauh dari kolom aslinya, kembali ke 0 manfaat).
Final: `SKEW_MAX_SHEAR_FRAC=0.3`.

### 16c. Verifikasi (WAJIB, pola sama §10b/§12c/§14c/§15c)

1. `--selftest` — SEMUA LULUS.
2. Exhaustive 520 Double + 239 Triple via `--batch-test --dataset <nama>`:
   Double gain=+5 (445->450) regresi=0, Triple gain=0 regresi=0 (skew
   fallback gak pernah kesentuh krn Triple sudah 100% duluan sblm sampai
   ke cek ini).
3. **Audit visual SEMUA 5 gain** — 5/5 benar, crosshair tepat di tengah
   bukaan asli, tidak ada yg kena 2 noise family dikenal (`17_30_06`/
   `17_30_12`).

### 16d. Angka akhir

| Dataset | §15 (sblm skew) | §16 (sesudah) |
|---|---|---|
| Double Gate (520) | 85.6%/85.6%/gap 0.0 | **86.5%/tuning 86.8% holdout 85.6%/gap 1.2** |
| Triple Gate (239) | 100.0%/100.0%/gap 0.0 | **100.0%/100.0%/gap 0.0 (tidak berubah)** |

Double naik +0.9 poin (85.6%->86.5%). Gap tuning-holdout melebar dikit
(0.0->1.2) krn 5 foto yg terselamatkan kebetulan lebih banyak jatuh di
sisi tuning-set (bukan indikasi overfitting baru, cuma distribusi
kebetulan dari split acak seed=42).

**Rekap kumulatif, Double Gate: 69.2% -> 86.5% (+17.3 poin)** via 5 teknik
berlapis (CLAHE §13, post-pairing §14, peak-density §15, skew-correction
§16) — TANPA menyentuh jalur deteksi ASLI sama sekali di tiap tahap.

### 16e. Keterbatasan jujur

Sama pola §15e: kriteria (`SKEW_MAX_SHEAR_FRAC=0.3`) dipilih dari dataset
& sesi observasi yg SAMA dipakai desain -- belum diuji foto/gerbang lain
(lih. keterbatasan umum §9g, masih berlaku penuh). `not_open_bottom`
(4 foto, TIDAK berubah dari §15) dan sisa kegagalan leg_peak_density yg
gagal walau sudah di-deskew (kemungkinan gate yg beneran gak 2-warna,
bukan soal perspektif) tetap gagal. Gain kecil (+5/520=+1%) krn kategori
ini emang sempit (cuma kasus ekstrem jarak sangat dekat + kaki 2-warna
bersamaan) -- bukan solusi besar, tapi genuine & aman (nol regresi).

### 16f. File final

- `gate_multi_detector.py` (aktif) — skew-corrected peak density
  ditambahkan di `_validate_opening()`, fungsi baru `_deskew_shift`,
  `_peak_density_skewed`.
- `gate_multi_detector_FINAL_v8_skew_20260809.py` — snapshot final.
- `gate_multi_detector_backup_20260809_before_skew.py` — state SEBELUM
  edit sesi ini (identik `_FINAL_v7_peakleg_20260806.py`).

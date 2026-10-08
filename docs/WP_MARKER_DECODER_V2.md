# Task 3 v2 — Rebuild Decoder Marker WP (dataset lapangan baru, 2026-08-07)

Status: **WP3/WP4 reliable=true (presisi 99-100%, recall 13-23%). WP1 reliable=false
(presisi 96%, recall cuma 2.9%). WP2 red-box (drop trigger) reliable=true (99.9%, di
912 foto). WP2-approach (ID-decode tanpa box) reliable=false.**

Script: `scripts/wp_decode_v2.py` (pipeline baru) + `scripts/wp_marker_detector.py`
(`detect_wp2_presence` dipakai apa adanya, tidak berubah dari v1).
Data: `wp_marker_reference_v2.json` (`config/`)
Dataset: `~/catkin_ws/dataset_arucode/Aruco_yang_baru/{wp1,wp2,wp3,wp4}` (~4000 foto,
foto lapangan diambil BERDIRI dari jarak jauh — beda kondisi dari dataset v1 lama
yang jarak dekat/rapi, lih. `wp_marker_reference_v2.json` di ~499 foto).

## Kenapa rebuild (bukan reuse v1 langsung)

Pipeline v1 (`wp_decode_audit.py`, `wp_marker_reference.json` lama) gagal TOTAL kalau
dipoint ke dataset baru — bukan karena desain marker berubah drastis, tapi karena
**cara foto diambil berubah**: dataset v1 jarak dekat (tarp penuh 1 frame), dataset v2
lapangan asli jarak jauh + berdiri (background rumput/paving ikut, marker cuma
~250-400px dalam frame 1920x1080, bukan ~1000px+).

## 3 masalah yang ditemukan & diperbaiki

1. **`marker_mask()` v1 nangkep background** — asumsi "tarp = seluruh frame" gak
   berlaku lagi. Fix: `marker_mask_v2()` batasi pencarian "bukan-oranye" ke DALAM
   tarp terbesar dulu (bukan seluruh frame).
2. **Marker kecil (navigasi arah, fitur baru) nempel LANGSUNG ke badan besar** —
   bukan "ekor" tipis (desain lama) yang gampang diputus `MORPH_OPEN`. Fix:
   `split_core_protrusion()` — generalisasi `detect_tail_side()` v1, pisah via
   profil proyeksi baris/kolom (bukan morfologi). **Keputusan user (2026-08-07):
   posisi nempelnya marker kecil = arah navigasi, isi pola marker kecil TIDAK
   didecode.**
3. **`corners_of()` v1 di-tune utk marker ~1000px+** (downscale 0.25 + kernel besar)
   — di marker v2 (~250-400px) kernel yang sama overerode, kuadran kepotong. Fix:
   `corners_small()` tanpa downscale, + fallback `minAreaRect` kalau kuadran hasil
   `approxPolyDP` < 75% luas mask asli (tepi mask gak mulus krn lipatan kain/bayangan).

Efek samping ditemukan sekalian: foto BERDIRI/miring (bukan nadir kayak kamera drone)
bikin `aspect` warp secara SISTEMATIS jauh dari 1.0 (foreshortening perspektif) —
divalidasi visual (warp tetap rapi meski aspect 1.7-2.6). `AR_TOL` v1 (0.25) gak
relevan lagi utk kondisi foto ini.

## Deteksi ID: rotation-search (bukan gantung ke arah)

`decode_frame_v2()` TIDAK pakai `tail_side` hasil `split_core_protrusion()` buat
nentuin rotasi kanonik sebelum decode (beda dari v1's `rotate_to_canonical`) — karena
deteksi arah kadang salah pilih sisi (~30-40% foto, root cause regresi awal saat scale
ke 300 sampel). Sebagai gantinya: decode grid MENTAH (tanpa rotasi), lalu cocokkan
KE-4 rotasi ke tiap referensi (WP1/3/4), ambil (wp_id, rotasi) dgn Hamming distance
minimum. `tail_side` tetap dilaporkan terpisah (buat heading-align), independen dari
ID-decode.

## Hasil akhir (full-dataset, semua foto per WP)

| WP | Metode | Total foto | decoded_ok | Presisi (saat ok) | Reliable |
|---|---|---|---|---|---|
| WP1 | Rotation-search Hamming | 789 | 23 (2.9%) | 95.7% | **false** |
| WP3 | Rotation-search Hamming | 766 | 178 (23.2%) | 100.0% | **true** |
| WP4 | Rotation-search Hamming | 837 | 107 (12.8%) | 99.1% | **true** |
| WP2 (in_box) | Red-box presence (unchanged v1) | 912 | — | 99.9% present | **true** |
| WP2 (outside_box, approach) | Rotation-search Hamming | 550 | 25 (4.5%) | ~90% (n kecil) | **false** |

Separasi antar pola referensi (Hamming, dari 36 sel): WP1↔WP3=19, WP1↔WP4=17,
WP3↔WP4=22 — **jauh lebih aman** dari v1 (min 9/36).

**Baca hati-hati**: recall rendah (WP3/WP4 cuma 13-23%) BUKAN berarti sistem sering
salah — presisi 99-100% berarti begitu sistem MEMUTUSKAN suatu ID, itu hampir selalu
benar. Recall rendah berarti sistem sering bilang "belum yakin" (`no_match`) dan itu
AMAN (drone tetap hover/nunggu, bukan gerak salah arah) — tapi berarti `scan_timeout`/
`align_timeout` default (`phaseD_mission.launch`, 8-20s) mungkin PERLU DINAIKKAN biar
cukup attempt (`throttle_hz`=5Hz × timeout) buat ngumpulin `scan_frames`/`align_frames`
konfirmasi. WP1 (2.9% recall) kemungkinan besar TIDAK akan clear scan_timeout default
sama sekali — cocok dgn `reliable=false`.

## Yang BELUM dikerjakan / next steps

1. **Recall rendah** (77-97% foto gagal di-decode) — pipeline corner-finding masih
   belum general ke seluruh variasi kondisi lapangan (cahaya, blur, sudut ekstrem).
   Ini target perbaikan paling bernilai buat sesi lanjutan (mirip riwayat tuning v1
   yang makan berhari-hari).
2. **WP2 red-box**: hanya divalidasi sbg presence (ada/tidak) + false-positive check
   di foto non-WP2. BELUM dikalibrasi `wp2_min_area_px` di ketinggian terbang asli
   (masih pakai default 3000px dari v1, cocok kamera 640x480 ketinggian menengah).
3. **WP3 = landing pad**: TIDAK ada perubahan kode — ini murni konfigurasi mission
   yaml (`action: land` di leg WP3), `mission_d.py` sudah punya `_land()`.
4. **Verifikasi fisik mapping WP↔ID** tetap WAJIB sebelum lomba (sama kayak v1) —
   dataset ini foto training/kalibrasi, BELUM PERNAH divalidasi terbang.
5. `~result_small` topic dipertahankan (backward-compat) tapi sekarang cuma cerminan
   `~result` — TIDAK ada decode kedua terpisah lagi (beda perilaku dari v1, cek kalau
   ada consumer yang gantung ke perbedaan itu).

## File terkait

| File | Isi |
|---|---|
| `scripts/wp_decode_v2.py` | Pipeline inti v2: `marker_mask_v2`, `split_core_protrusion`, `corners_small`, `decode_frame_v2`, `load_refs_v2` |
| `scripts/wp_marker_node.py` | Node ROS, pakai `decode_frame_v2` (ganti dari `wp_marker_detector.decode_frame`) |
| `config/wp_marker_reference_v2.json` | Reference pattern WP1/3/4 (`_meta` berisi statistik build) |
| `launch/wp_marker_detect.launch` | `json_path` & `hamming_threshold` di-update ke v2 (7, dari 10) |

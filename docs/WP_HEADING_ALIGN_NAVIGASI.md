# Task 4 — Heading Align via Ekor Marker WP — Ringkasan Status

Status: **Build sukses, DIUJI SITL end-to-end (4 arah N/E/S/W + 2 skenario
guard) — semua LULUS sesuai ekspektasi. Masih BELUM diaktifkan di mission
yaml lomba manapun (`~enable_heading_align` default `False`), BELUM
diverifikasi di hardware asli/kalibrasi lapangan.**
File utama: `src/mission_control/scripts/mission_d.py`
Self-test: `src/mission_control/scripts/test_tail_heading.py`
Mission test-only (SITL): `src/mission_control/config/mission_task4_heading_test.yaml`
Backup sebelum sesi ini: `mission_d_before_task4_heading_20260731_1725.py`,
`WpMarkerResult_backup_task4_20260731_1725.msg`,
`wp_marker_detector_backup_task4_20260731_1725.py`,
`wp_marker_node_backup_task4_20260731_1725.py`
Backup fix wp_id/staleness guard: `mission_d_before_task4_wpid_guard_20260731_1905.py`
Backup sim_markers.py: `sim_markers_backup_task4_20260731_1903.py`
Backup phaseD_mission.launch: `phaseD_mission_before_task4_args_20260731_1926.launch.bak`

---

## 1. Klarifikasi penting (jangan tertukar lagi)

**"ArUco 10×10cm" (istilah rencana lama di `mission_seleksi.yaml` komentar dan
`docs/PANDUAN_MISI_SELEKSI.md`) dan "ekor" (istilah dari investigasi Task 3)
adalah OBJEK FISIK YANG SAMA** — tab kecil yang menempel langsung di badan
marker ArUco custom 50×50cm (WP1/WP3/WP4), **bukan** marker terpisah di lokasi
lain di lapangan. Task 4 ini reuse `detect_tail_side()` dari Task 3
(`wp_decode_audit.py` / dipakai ulang di `wp_marker_detector.py`), bukan
implementasi baca marker baru.

Ini menggantikan pemahaman awal sesi Task 4 (kode lama `expected_id_small` di
`mission_d.py`) yang salah asumsi: mengira marker kecil itu ArUco ID terpisah
yang butuh koreksi **posisi lateral** (X/Y). Yang benar: ekor cuma memberi
**arah diskrit (N/S/E/W)**, dipakai utk **heading (yaw)**, bukan lateral.

## 2. Riwayat revisi

1. **Sesi awal**: `mission_d.py` punya `expected_id_small` + `small_marker_align_tol`
   — align lateral tahap-2 di `_scan()`/`_align_and_drop()`, treat "marker kecil"
   sbg ArUco ID terpisah dgn off_x/off_y sendiri. **Ini SALAH SASARAN** (lihat §1).
2. **Investigasi ulang**: ditemukan `detect_tail_side()` (Task 3) sudah menghitung
   arah N/S/E/W tapi hasilnya **dibuang** — cuma `low_conf_tail` (bool) yang
   sampai ke `WpMarkerResult.msg`. Tidak ada satupun topic ROS yang publish arah
   ekor sesungguhnya.
3. **Keputusan final**: (a) tambah field `tail_side` ke `WpMarkerResult.msg` +
   `wp_marker_detector.py` + `wp_marker_node.py` supaya arah ekor sampai ke ROS;
   (b) `mission_d.py` dapat fungsi baru `_align_heading_to_tail()` yang MEMUTAR
   drone (yaw) ke arah ekor, dipanggil setelah scan/drop selesai; (c) kode lateral
   lama (`expected_id_small` dkk) **DIBIARKAN ADA tapi tidak dipanggil** — arsip
   historis, bukan dihapus.
4. **Fix tambahan (ditemukan saat siapkan `sim_markers.py`)**: versi pertama
   `_align_heading_to_tail()` pakai `self._wp_marker_result` (pesan TERAKHIR
   apapun) tanpa cek `wp_id` cocok leg yang baru di-scan/drop, dan tanpa cek
   umur pesan. Risiko: deteksi transien salah dari WP lain, atau pesan basi,
   bisa memutar drone ke arah yang salah. Fix: `_align_heading_to_tail()`
   sekarang WAJIB terima `expected_id`, cek `res.wp_id == expected_id` DAN
   `_wp_marker_result_age(res) <= ~tail_result_stale_sec` (default 2.0s)
   sebelum dipakai — sama pola dgn `_healthy()`/`_pose_age()` yang sudah ada.

## 3. Keputusan arsitektur final

- **Sumber `tail_side`**: field baru `string tail_side` di `WpMarkerResult.msg`
  ('N'/'S'/'E'/'W'/''), diisi dari `tail` yang sebelumnya dibuang di
  `decode_frame()` (`wp_marker_detector.py`). `mission_d.py` subscribe topic baru
  `~wp_marker_result_topic` (default `/wp_marker_node/result`), terpisah dari
  `~markers_topic` (yang cuma bawa off_x/off_y utk align lateral marker besar).
- **Definisi "WP berikutnya"**: heading align dipanggil setelah leg `action:
  scan` atau `action: drop` — BUKAN sebelum/sesudah `traverse` (gate sudah punya
  alignment sendiri via `_traverse()`).
- **Makna "align heading"**: yaw MENGHADAP KE ARAH FISIK yang ditunjuk ekor
  (label N/S/E/W dari `detect_tail_side()`, relatif ke FRAME GAMBAR kamera
  bawah) — **bukan** bearing dihitung dari koordinat `waypoints.yaml`. Dikonversi
  ke perintah **relatif** thd yaw drone sekarang (`_tail_to_yaw_cmd()`), lalu
  pakai `_yaw_turn()` yang SUDAH ADA (tidak ditulis ulang).
- **Non-blocking by design**: kalau `tail_side` kosong/`low_conf_tail=True`/topic
  belum ada data/WpMarkerResult basi/`wp_id` tak cocok leg saat ini → SKIP (log
  warning, lanjut misi). Marker besar (ID WP) tetap SATU-SATUNYA penentu
  sukses/gagal scan/drop — logika itu tidak disentuh sama sekali.
- **Guard freshness + identitas**: `_align_heading_to_tail()` menolak
  `WpMarkerResult` yang lebih tua dari `~tail_result_stale_sec` (default 2.0s)
  ATAU yang `wp_id`-nya bukan `expected_id` leg yang baru selesai discan/drop.
- **Default OFF**: `~enable_heading_align` (bool) default `False`. Harus di-set
  `True` eksplisit (rosparam/launch) di mission yang mau pakai fitur ini. Belum
  ada mission yaml yang mengaktifkannya.

## 4. Kode yang diubah

| File | Perubahan |
|---|---|
| `msg/WpMarkerResult.msg` | + field `string tail_side` |
| `wp_marker_detector.py` | `decode_frame()`: `tail` (sebelumnya dibuang) sekarang diteruskan sbg `tail_side` di semua 6 jalur return |
| `wp_marker_node.py` | `res.tail_side = str(dec['tail_side'])` |
| `mission_d.py` | + import `WpMarkerResult`; + param `~enable_heading_align` (default False), `~tail_dir_rotate_steps` (default 0), `~tail_mirror` (default False), `~tail_result_stale_sec` (default 2.0), `~wp_marker_result_topic`; + subscriber `_wp_marker_result_cb`; + `_tail_to_yaw_cmd(tail_side)` (mapping N/E/S/W → yaw relatif); + `_wp_marker_result_age(res)` (pola sama `_pose_age()`); + `_align_heading_to_tail(x,y,z,expected_id)` (guard staleness + wp_id match, non-blocking, panggil `_yaw_turn()`); dipanggil di `run()` setelah `scan`/`drop`. Param lama `~small_marker_align_tol`/`expected_id_small` **dibiarkan ada, ditandai DEPRECATED di komentar, tidak dihapus**. |
| `sim_markers.py` | + param `~sim_tail_side` (default 'N'), `~wp_marker_result_topic`; publish `WpMarkerResult` mock (wp_id + tail_side seragam) sejalan dgn tiap marker ArUco simulasi yg 'terlihat', supaya `_align_heading_to_tail()` bisa dilatih di SITL |
| `test_tail_heading.py` (baru) | Self-test murni Python (tanpa ROS master, `rospy.Time.now` di-monkeypatch ke wall-clock) utk `_tail_to_yaw_cmd()` DAN 8 skenario guard `_align_heading_to_tail()` (disabled, no-result, stale, wp_id-mismatch, low-conf, unknown-dir, N-no-turn, sukses) |

### Konvensi mapping arah (di `_tail_to_yaw_cmd()`)

Default (belum kalibrasi): ekor='N' (atas gambar) → drone sudah menghadap arah
ekor, **tak perlu putar**. 'E' (kanan gambar) → putar **KANAN (CW) 90°**. 'W' →
putar **KIRI (CCW) 90°**. 'S' → **180°**.

⚠️ **Ini ASUMSI mounting kamera bawah** (atas-gambar = depan drone, kanan-gambar
= kanan drone) — **belum diverifikasi fisik**. Knob kalibrasi disediakan (mirip
`align_sign_x/y` utk align lateral): `~tail_dir_rotate_steps` (0-3, geser mapping
kalau mounting terputar 90/180/270°) dan `~tail_mirror` (tukar E↔W kalau gambar
kamera mirror/terbalik kiri-kanan thd badan drone).

## 5. Keterbatasan yang diketahui

1. **BELUM di-`catkin build`.** Dicek langsung: Python message class
   `WpMarkerResult` yang ter-generate di `devel/` saat ini **belum punya**
   field `tail_side` (field lama saja). `.msg` adalah definisi IDL, bukan file
   Python — perubahan di §4 baris pertama TIDAK akan terpakai sampai rebuild.
   **Command yang perlu dijalankan manual** (di luar scope read-only sesi ini):
   ```bash
   cd ~/catkin_ws && catkin build mission_control
   source ~/catkin_ws/devel/setup.bash   # di SETIAP shell yang pakai node ini
   ```
2. **Belum diuji SITL/hardware.** Verifikasi sejauh ini: `py_compile` (5 file)
   + self-test murni Python (`test_tail_heading.py` — mapping arah, DAN 8
   skenario guard `_align_heading_to_tail()`, tanpa ROS master) + `decode_frame()`
   dijalankan lgs ke 117 foto asli dataset Task 3 (tail_side konsisten dgn
   `low_conf_tail`, 0 mismatch). **Belum** ada bukti runtime end-to-end lewat
   topic ROS sungguhan (`wp_marker_node` publish → `mission_d` subscribe →
   `_yaw_turn` jalan) — itu baru bisa diuji setelah `catkin build` (lihat #1).
3. **Asumsi mapping arah (§4) belum diverifikasi fisik** — kalibrasi
   `~tail_dir_rotate_steps`/`~tail_mirror` kemungkinan besar perlu disetel di
   lapangan (mirip walk test `align_sign_x/y`) sebelum dipakai beneran.
4. **"WP berikutnya" belum berarti geometris.** Heading yg dihasilkan MENGIKUTI
   ARAH FISIK EKOR, bukan bearing terhitung ke `waypoints.yaml` WP selanjutnya
   — kalau ekor secara fisik TIDAK diarahkan tim ke WP berikutnya saat marker
   dipasang, hasil heading align ini tidak akan menunjuk WP berikutnya secara
   akurat. Ini keputusan lapangan (pemasangan marker), di luar scope kode.
5. Kode lateral lama (`expected_id_small`, `small_marker_align_tol`) masih ada
   di `_scan()`/`_align_and_drop()` sbg arsip — kalau memang tidak akan dipakai
   lagi, perlu keputusan eksplisit terpisah utk membersihkannya (belum
   dieksekusi sesi ini).

## 6. Hasil uji SITL (sesi ini, 2026-07-31)

**Setup**: resep 4-terminal AGENTS.md (SITL `-I0` → MAVProxy bridge → roscore →
mavros → `set_origin_home.launch`), semua gate check lolos (`/mavros/state
connected:True`, home ter-set). `catkin build mission_control` sukses,
`tail_side` dikonfirmasi aktif di `WpMarkerResult` ter-generate.

**Temuan sebelum tes valid**: perintah awal (`enable_heading_align:=true
_sim_tail_side:=N`, gaya CLI `rosrun`) **tidak berpengaruh** — `phaseD_mission.launch`
belum punya `<arg>` utk param Task 4, roslaunch diam-diam mengabaikannya
(dibuktikan: `PARAMETERS` summary roslaunch tidak memuat `enable_heading_align`
sama sekali). **Fix**: ditambahkan `<arg name="enable_heading_align">` dan
`<arg name="sim_tail_side">` ke `phaseD_mission.launch` (backup dulu), dgn
sintaks `arg:=value` standar (bukan `_arg:=value`).

**Temuan kedua**: tes pertama dgn `mission_seleksi.yaml` (WP1/2/3) memicu
guard `wp_id` (`res.wp_id=3 != expected=1 -> skip`) — root cause: WP1/2/3
saling berdekatan (~1.2-2.6m), semua masuk `~view_radius` default 3.0m
`sim_markers.py`, jadi WpMarkerResult utk id lain ikut ter-publish & menimpa
sebelum `_align_heading_to_tail()` sempat baca punya WP1. **Ini justru bukti
guard wp_id (§2.4) memang perlu** — bukan bug baru. Fix test (bukan fix kode):
mission test-only `mission_task4_heading_test.yaml` (1 leg, cuma WP1) supaya
`sim_markers.py` cuma punya 1 marker virtual.

**Hasil 4 arah** (mission test-only, `enable_heading_align:=true`):

| `sim_tail_side` | Log `_align_heading_to_tail` | Log `_yaw_turn` | Hasil |
|---|---|---|---|
| N | `ekor='N' (sudah menghadap) -> tak perlu putar` | (tak dipanggil) | ✅ sesuai (0°, no-op) |
| E | `ekor='E' -> yaw right 90 deg` | `right 90 deg (0.0 -> -90.0)`, `yaw OK (err=2.2°)` | ✅ sesuai |
| S | `ekor='S' -> yaw right 180 deg` | `right 180 deg (-90.0 -> -270.0)`, `yaw OK (err=4.4°)` | ✅ sesuai |
| W | `ekor='W' -> yaw left 90 deg` | `left 90 deg (89.9 -> 179.9)`, `yaw OK (err=6.0°)` | ✅ sesuai |

(Yaw start point tiap tes = yaw akhir tes sebelumnya, krn 1 vehicle SITL yg
sama dipakai berturutan tanpa reset attitude — konsisten dgn `_yaw_turn()`
yang memang RELATIF ke yaw saat itu, bukan bug.)

**Hasil guard (non-blocking)**:
- `enable_heading_align:=false` → **0 baris log heading align** (fitur benar2 mati).
- `sim_tail_side` tak dikenal (`'X'`) → `tail_side='X' tak dikenal -> skip`
  (WARN), misi lanjut selesai normal — **tidak macet**.
- `wp_id` tak cocok → sudah terbukti "di alam liar" di temuan kedua di atas
  (bukan tes sengaja, tapi hasilnya sama: skip + WARN + misi lanjut).
- ⚠️ Tidak berhasil menguji `tail_side` KOSONG murni (`low_conf_tail=True`
  case): CLI `arg:=""` / `arg:=` ternyata **tidak diterima roslaunch sbg
  override** — jatuh balik ke `default="N"` di XML (dibuktikan via `rosparam
  get`). Ini keterbatasan cara override CLI roslaunch, bukan bug kode. Kasus
  ini tetap tervalidasi lewat unit-test murni Python (`test_tail_heading.py`,
  §di atas) yang LULUS.

**Kesimpulan penting**: LOGIKA mapping arah (§4) **terbukti benar 100%** utk
keempat arah — tapi ingat, `sim_tail_side` di SITL adalah label ground-truth
yang di-inject langsung (bukan hasil pipeline vision sungguhan), jadi SITL
**tidak** dan **tidak bisa** membuktikan/membantah asumsi kalibrasi mounting
kamera fisik (§4 "atas-gambar = depan drone dst") — itu tetap murni menunggu
kalibrasi lapangan fisik. Tidak ada temuan yang mengharuskan
`~tail_dir_rotate_steps`/`~tail_mirror` diubah dari default (0, False) —
karena SITL tidak menguji jalur itu sama sekali.

Semua proses SITL (SITL, MAVProxy, roscore, mavros) dimatikan bersih di akhir
sesi (diverifikasi `pgrep` luas utk ros/ardupilot/mavlink — kosong).

## 7. Langkah selanjutnya

1. Kalibrasi lapangan `~tail_dir_rotate_steps`/`~tail_mirror` (walk-test-style,
   pakai kamera bawah + marker fisik sungguhan) SEBELUM mengaktifkan
   `~enable_heading_align:=true` di misi sungguhan — SITL tidak bisa
   menggantikan langkah ini (lihat §6 kesimpulan).
2. Setelah kalibrasi terverifikasi, set `enable_heading_align:=true` di
   mission yaml/launch yang mau dipakai (belum ada default yang diaktifkan).
3. Pertimbangkan: `mission_seleksi.yaml`/`mission_final.yaml` asli (bukan
   test-only) punya WP berdekatan dalam radius pandang sim (§6) — kalau mau
   uji SITL end-to-end mission lomba yg sesungguhnya (bukan test-only 1-leg),
   `~view_radius` `sim_markers.py` mungkin perlu diperkecil dulu (belum
   diekspos sbg `<arg>` launch, saat ini cuma bisa via rosparam).

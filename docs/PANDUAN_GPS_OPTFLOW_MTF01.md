# PANDUAN: Profil Terbang GPS + Optical Flow (MTF01) & Catatan Optimasi T265

Tanggal: 2026-08-05. Pelengkap `PANDUAN_TERBANG_T265.md` (profil VIO non-GPS) dan
`AGENTS.md` (gotcha). Dokumen ini mencakup:

- **Bagian 1: keputusan & diagnosis** — kenapa beralih dari VIO T265 ke GPS+OptFlow
  untuk manual & autonomous mission planner, plus catatan drift T265 di lapangan rumput.
- **Bagian 2: profil param baru** — `~/catkin_ws/params/gps_optflow_ek3.param`,
  apa isinya, cara load, verifikasi.
- **Bagian 3: checklist lapangan** — urutan verifikasi sebelum terbang sungguhan.

---

## 1. Latar Belakang & Diagnosis

### 1.1 Kenapa T265 drift di lapangan rumput (menghadap depan)

T265 forward-facing di lapangan terbuka berumput punya kelemahan struktural VIO:

| Penyebab | Efek |
|---|---|
| **Setengah FOV = langit** (fisheye ±170°) | Langit = **nol fitur** — VIO kehilangan anchor, fallback ke IMU murni, drift |
| **Rumput = tekstur homogen** | Sedikit fitur yang bisa ditrack, kualitas rendah |
| **Offset fisik belum lengkap** (`VISO_POS_Y/Z=0` asumsi) | Saat roll/pitch, EKF3 menerjemahkan offset salah → translasi fiktif |
| **IMU T265 belum terkalibrasi** | Gyro bias kecil → drift posisi saat lama terbang |
| **EK3_VIS_DELAY/VERR belum di-tune** | Tuning akhir, bukan penyebab utama |

Rencana perbaikan drift (status per 2026-08-07, dicek langsung — bukan asumsi):

1. Mounting miring 15–25° ke bawah (rumput dominan, langit keluar) — **fisik
   TERPASANG 2026-08-07, ~15°**. KOREKSI dari catatan sebelumnya di sini
   (sempat salah tulis "`pitch_cam` tetap 0/preset diskrit" — itu KELIRU,
   dicek ulang ke kode: `pitch_cam` di `vision_to_mavros.cpp` itu rotasi
   kontinu beneran, `tf::createQuaternionFromRPY(0, pitch_cam, 0)`, bukan
   dibatasi ke preset). Sudah diupdate: `pitch_cam = -0.2618` rad (-15°,
   negatif = kamera nunduk, interpolasi linear dari preset Downfacing
   -90°=-1.5707963 di komentar launch file yg sama) di
   `t265_tf_to_mavros.launch`. **Tervalidasi matematis** (simulasi quaternion
   kamera fisik miring 15° → lewat rantai rotasi persis kayak di kode → hasil
   pitch body = 0.000°, level) — TAPI **belum diverifikasi fisik** (nyalain
   T265 beneran, cek `/mavros/vision_pose/pose_cov` pas drone level harus
   kebaca level, dan bandingkan tracking confidence sebelum/sesudah tilt di
   rumput beneran). Gak perlu rebuild (param launch file doang, bukan kode
   C++) — tinggal relaunch.
2. ~~Kalibrasi IMU T265~~ — **TIDAK APPLICABLE, dikonfirmasi 2026-08-07**.
   `rs-imu-calibration` (ada di source `librealsense/tools/`) eksplisit cuma
   utk D435i/L515 (readme: "intended to calibrate the IMU built in D435i and
   L515 cameras") — arsitektur beda, T265 gak punya jalur EEPROM writable yg
   dipakai tool itu. Firmware T265 aktif (0.2.0.951) juga sudah versi
   terakhir yang tersedia (cocok dgn satu-satunya `target-0.2.0.951.mvcmd`
   di build lokal; T265 sendiri sudah discontinued Intel sejak 2021) — tidak
   ada firmware lebih baru utk dicoba. T265 = factory-calibrated tertutup,
   tidak ada jalur kalibrasi ulang dari sisi user. Coret dari rencana.
3. Ukur `VISO_POS_X/Y/Z` fisik — **X sudah terukur** (0.06m, T265 di depan
   CoG, lih. `params/viso_pos_offset.param`). **Y/Z belum** (fisik, perlu
   tangan user; Z negatif kalau sensor di atas CoG).
4. Cek tracking confidence — **tooling siap & baseline didapat 2026-08-07**:
   `rostopic echo /camera/odom/sample/pose/covariance` (indeks [0]), formula
   dari source `realsense-ros` (`base_realsense_node.cpp`): `cov_pose =
   linear_accel_cov(default 0.01) * 10^(3 - tracker_confidence)`. Baca:
   0.01→confidence 3 (High), 0.1→2 (Medium), 1.0→1 (Low), 10.0→0 (Failed).
   Baseline indoor diam: **confidence=2 (Medium)** — bandingkan ke bacaan di
   rumput saat test lapangan nanti (langkah 1/3 di atas).
5. ~~Tune `EK3_VIS_DELAY` + `EK3_VIS_VERR_MIN/MAX`~~ — **nama param SALAH,
   dikoreksi 2026-08-07**. Ditrace ke source ArduPilot
   (`AP_NavEKF3_Measurements.cpp`): `EK3_VIS_VERR_MIN/MAX` cuma dipakai jalur
   MAVLink `VISION_POSITION_DELTA` (body-odometry, field `quality` 0-100) —
   BUKAN jalur T265 kita (`VISION_POSITION_ESTIMATE`, pose absolut, dikirim
   `mavros` vision_pose_estimate plugin). Param itu ada di
   `t265_from_TFLuna_delta.param` (sempat di-tuning 0.1→0.15 / 0.9→1.0) tapi
   **efeknya nol** ke T265 — dibiarkan ke-set (harmless) tapi diberi
   komentar koreksi di file itu.

   **Temuan lebih besar dari ini**: sebelum 2026-08-07, `vision_to_mavros`
   SELALU kirim covariance NOL ke mavros (baca TF doang, `PoseStamped` polos
   gak punya field covariance) → `posErr` yang nyampe EKF SELALU ke-clamp ke
   floor tetap `VISO_POS_M_NSE` (default 0.2m, belum pernah di-set eksplisit)
   — **EKF gak pernah tau tracker_confidence T265 turun**, cuma NaN total yang
   ketangkep (guard di `vision_to_mavros.cpp`). **SUDAH DIPERBAIKI** (kode,
   2026-08-07): `vision_to_mavros.cpp` sekarang subscribe
   `/camera/odom/sample`, forward covariance asli (mencerminkan
   `tracker_confidence` real-time) via topic baru
   `/mavros/vision_pose/pose_cov` (`PoseWithCovarianceStamped`, bukan
   `PoseStamped`) + failsafe kalau covariance basi >1s (dianggap
   confidence=Failed, bukan diam pakai data lama). Build sukses, live-test
   OK (covariance terkirim match `/camera/odom/sample`).

   **Yang masih murni "belum dikerjakan"**: nilai eksplisit `VISO_POS_M_NSE`/
   `VISO_VEL_M_NSE`/`VISO_YAW_M_NSE` (floor noise saat confidence tertinggi)
   — masih default ArduPilot, belum ada dasar data buat ubah (butuh log
   terbang, sama seperti rencana awal, tapi param-nya beda dari yang ditulis
   semula).

### 1.2 Keputusan: profil GPS + OptFlow

Untuk **manual flight** dan **autonomous mission planner (AUTO/GUIDED)** dipilih
profil **GPS primary + MTF01 optical flow**:

- GPS = primary XY pos/vel (ENU di lapangan terbuka, HDOP bagus).
- MTF01 + TF-Luna = fallback otomatis (SRC2) saat GPS lost/degraded + membantu
  low-altitude hold.
- Kompas diaktifkan kembali (kalibrasi OFS/MOT dari dump lama masih dipakai).

**KOREKSI 2026-08-08** (baris ini sebelumnya salah, dibiarkan sampai
dikonfirmasi lewat kode): T265 **TIDAK perlu jalan sama sekali** di mode GPS —
bukan cuma `VISO_TYPE=0` (dilepas dari EKF), tapi memang gak ada alasan
software buat nyalain node-nya sama sekali. Dicek langsung: kamera perception
(ArUco/WP marker = C920 `camera_down`, gate = C310 `camera_front`) itu
**hardware USB terpisah dari T265**, nol referensi ke topic fisheye T265 di
seluruh `mission_control` (`grep -rln fisheye` kosong). T265 di codebase ini
MURNI sensor navigasi (VIO) — di mode GPS, boleh dicabut/dimatikan total.

---

## 2. Profil Param: `gps_optflow_ek3.param`

Basis: `TF-Luna_GPS_18.13_17-7-26.param` (dump GPS asli yang sudah terbang).
Delta dari profil `t265_nongps_ek3.param`. Konvensi EK3 source dikonfirmasi dari
source ArduPilot lokal (`libraries/AP_NavEKF/AP_NavEKF_Source.h`):

```
SourceXY : NONE=0 GPS=3 BEACON=4 OPTFLOW=5 EXTNAV=6
SourceZ  : NONE=0 BARO=1 RANGEFINDER=2 GPS=3 EXTNAV=6
SourceYaw: NONE=0 COMPASS=1 GPS=2 EXTNAV=6 GSF=8
```

### 2.1 Isi file (ringkas)

| Param | Nilai | Alasan |
|---|---|---|
| `GPS1_TYPE` | 1 | Nyalakan GPS |
| `AHRS_GPS_USE` | 1 | EKF pakai GPS |
| `COMPASS_ENABLE/USE/USE2` | 1 | Heading mission planner |
| `EK3_SRC1_POSXY/VELXY` | 3 (GPS) | Primary XY |
| `EK3_SRC1_POSZ` | 1 (Baro) | Primary Z |
| `EK3_SRC1_VELZ` | 3 (GPS) | Primary vel Z |
| `EK3_SRC1_YAW` | 1 (Compass) | Heading |
| `EK3_SRC2_POSXY/VELXY` | 5 (OptFlow) | Fallback GPS lost |
| `EK3_SRC2_POSZ/VELZ` | 1 (Baro) | Fallback Z |
| `EK3_SRC2_YAW` | 1 (Compass) | Heading fallback |
| `FLOW_TYPE` | 3 | Mateksys MTF01 |
| `FLOW_ORIENT_YAW` | 0 | Mount depan (verifikasi arah saat test) |
| `FLOW_POS_X/Y/Z` | 0 | ⚠️ Belum diukur — mulai 0, isi saat ada indikasi meleset |
| `EK3_FLOW_USE` | 1 | Fusion flow aktif |
| `EK3_SRC_OPTIONS` | 1 | Fuse GPS + flow velocity bareng |
| `EK3_RNG_USE_SPD` | 2 | TF-Luna untuk scaling speed flow |
| `EK3_RNG_USE_HGT` | -1 | Rangefinder bukan primary height |
| `VISO_TYPE` | 0 | VIO T265 mati dari EKF |

### 2.2 Cara load

1. MP → Config/Tuning → **Full Parameter List**
2. Load `gps_optflow_ek3.param` → **Write** → **REBOOT** (⚠️ jangan saat armed)
3. Confirm: `EK3_SRC1_POSXY` = 3, `FLOW_TYPE` = 3, `VISO_TYPE` = 0

---

## 3. Checklist Verifikasi (sebelum terbang sungguhan)

### 3.1 Ground / bench (tanpa motor)

1. MP → **MAVLink Inspector**:
   - `OPTICAL_FLOW_RAD.quality` > 100 (siang hari), flow_x/flow_y bergerak saat sensor digeser
   - `DISTANCE_SENSOR.current_distance` valid (TF-Luna, bukan 0/65535)
   - `GPS_RAW_INT`: fix 3D, HDOP < 1.5, sats ≥ 8
2. **Test arah flow**: angkat drone, geser maju/kanan → cek arah flow di MP.
   Terbalik? `FLOW_ORIENT_YAW` 0 → 180 (atau sebaliknya), tulis ulang.
3. Kompas: heading di MP berubah saat drone diputar (kalibrasi lama harus valid).

### 3.2 Terbang singkat (SITL dulu kalau bisa; fisik: konfirmasi user)

1. Mode manual (Stabilize → AltHold) → Loiter di 1–2 m
2. Graph MP: inovasi `EKF3.INN` (velPos < 0.3 m, hgt < 0.3 m)
3. Gerakkan manual maju/mundur/kiri/kanan ±1 m → inovasi tetap kecil
4. Matikan GPS sesaat (simulasi di SITL) → EKF3 harus switch ke SRC2 (flow),
   posisi tetap terjaga tanpa jump
5. Mission planner: upload misi latihan singkat, jalankan AUTO, amati tracking WP

### 3.3 Catatan log

- `NKF4.SP/SV` = source aktif pos/vel (1 = GPS normal, jadi 5 saat fallback flow)
- Kalau flow tidak pernah terpakai di lapangan terbuka = normal (GPS lebih bagus),
  bukan bug.

---

## 4. Referensi

- File param: `~/catkin_ws/params/gps_optflow_ek3.param`
- Dump asal: `~/catkin_ws/params/TF-Luna_GPS_18.13_17-7-26.param`
- Profil T265 non-GPS: `~/catkin_ws/params/t265_nongps_ekf3.param`
- Delta T265 (EKF vision noise, koreksi EK3_VIS_VERR 2026-08-07):
  `~/catkin_ws/params/t265_from_TFLuna_delta.param`
- Fix covariance T265 real-time (2026-08-07): `vision_to_mavros/src/vision_to_mavros.cpp`
  (subscribe `/camera/odom/sample`, publish `PoseWithCovarianceStamped` ke
  `/mavros/vision_pose/pose_cov`) + `vision_to_mavros/launch/t265_tf_to_mavros.launch`
- Arsitektur T265 + misi GUIDED: `WP_T265_TO_GUIDED_FLOW.md`
- Source enum EKF3: `~/ardupilot/libraries/AP_NavEKF/AP_NavEKF_Source.h`

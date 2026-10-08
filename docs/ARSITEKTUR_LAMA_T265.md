# ARSITEKTUR LAMA — Navigasi T265 (VIO) & Kenapa Ditinggalkan

Dokumen histori: arsitektur navigasi yang dipakai **sebelum** keputusan tim
pindah ke GPS + MTF-01 optical flow (2026-08-08). T265 **tidak lagi
dipakai** di arsitektur final (lihat `ARSITEKTUR_SISTEM.md`) — dokumen ini
disimpan supaya keputusan itu punya jejak alasan teknis yang jelas, dan
sebagai referensi kalau suatu saat GPS hilang total di lapangan dan T265
perlu jadi cadangan darurat. Status per **2026-09-04**.

---

## 1. Ringkasan

Intel RealSense T265 adalah kamera **visual-inertial odometry (VIO)** —
menggabungkan 2 kamera fisheye + IMU internal untuk menghasilkan estimasi
posisi 6-DOF relatif terhadap titik awal, tanpa GPS. Ide awalnya: drone
tetap bisa terbang otonom di lokasi tanpa sinyal GPS (indoor, atau sebagai
cadangan). Peran T265 di arsitektur ini **murni sensor navigasi (posisi)**
— sama sekali tidak terhubung ke kamera-kamera perception (ArUco/gate),
yang selalu memakai C920/C310 terpisah di kedua arsitektur.

**Kenapa ditinggalkan**: bukan satu alasan tunggal, tapi akumulasi
kelemahan struktural VIO di kondisi lapangan kompetisi (lapangan
terbuka berumput) plus satu bug perangkat lunak yang cukup serius
(§4.2) yang baru ketahuan setelah ditelusuri sampai ke source code. Tim
memutuskan GPS + optical flow (MTF-01) jauh lebih cocok untuk kondisi
outdoor terbuka — lihat §5 untuk perbandingan langsung.

---

## 2. Arsitektur — Alur Data

```
T265 (2x kamera fisheye + IMU)
      │  librealsense
      ▼
realsense2_camera (rs_t265.launch)
      │  /camera/odom/sample (nav_msgs/Odometry, ~200Hz, POSISI + COVARIANCE)
      ▼
vision_to_mavros (t265_tf_to_mavros.launch)
      │  transform mounting (forward-facing, tilt fisik dikompensasi pitch_cam)
      │  /mavros/vision_pose/pose_cov (geometry_msgs/PoseWithCovarianceStamped)
      ▼
mavros (vision_pose_estimate plugin)
      │  MAVLink VISION_POSITION_ESTIMATE
      ▼
Pixhawk 6C — EKF3 (VISO_TYPE=2, EK3_SRC1_POSXY/VELXY/YAW=6 "ExternalNav")
```

Berbeda dari arsitektur GPS: **tidak ada sumber posisi otomatis**. EKF3
harus diberi tahu titik nol secara eksplisit tiap boot FC lewat
`set_origin_home` (lihat §3.3) — origin **tidak persisten**, beda mendasar
dari GPS yang otomatis dapat origin dari fix satelit pertama.

---

## 3. Detail Konfigurasi

### 3.1 Pemasangan fisik

Forward-facing (kamera menghadap arah depan drone), USB ke kanan — dipilih
alih-alih down-facing karena lebih tahan propwash/angin dan tidak
tergantung tekstur rumput yang seragam persis di bawah drone. Belakangan
(2026-08-07) ditambah **tilt fisik ~15° menunduk** untuk mengurangi porsi
langit (tanpa fitur visual sama sekali) di FOV fisheye, dikompensasi lewat
parameter `pitch_cam` di launch file (rotasi kontinu, bukan preset diskrit
seperti sempat disalahpahami di awal — lihat §4.3).

### 3.2 Parameter EKF3 non-GPS (`t265_nongps_ekf3.param`)

| Parameter | Nilai | Arti |
|---|---|---|
| `GPS1_TYPE` | 0 | GPS **dimatikan total** |
| `VISO_TYPE` | 2 | Visual odometry = Intel T265 |
| `EK3_SRC1_POSXY/VELXY/VELZ/YAW` | 6 | Semua dari ExternalNav (T265) |
| `EK3_SRC1_POSZ` | 1 | Tetap Baro (T265 Z kurang stabil) |
| `COMPASS_ENABLE` | 0 | Kompas **dimatikan** — heading 100% dari kamera |

Implikasi penting: kalau T265 kehilangan tracking, drone kehilangan
**posisi DAN heading sekaligus** (beda dari GPS+OptFlow yang tetap punya
kompas independen sebagai sumber heading).

### 3.3 Origin harus di-set ulang tiap boot

`set_origin_home` (script atau Mission Planner) **wajib** dijalankan di
titik takeoff, setiap kali FC baru boot/reboot — origin = (0,0,0) di titik
itu. Implikasi operasional: **tidak boleh cabut daya** di tengah sesi
survei waypoint (origin geser saat reboot → semua WP yang sudah direkam
jadi tidak valid), dan protokol lapangan jadi lebih rawan kesalahan
prosedur dibanding GPS (yang originnya otomatis & konsisten selama GPS fix
tidak hilang total).

### 3.4 Walk test wajib sebelum tiap sesi terbang

Karena tidak ada ground-truth independen untuk memverifikasi akurasi VIO,
prosedurnya: angkat drone (motor mati), jalan membentuk kotak ~2m×2m
manual, bandingkan lintasan yang terbaca di Mission Planner/rviz dengan
bentuk & skala kotak nyata. Kalau skala meleset atau lintasan muter sendiri
→ tidak boleh terbang. Ini langkah manual tambahan yang tidak diperlukan
di arsitektur GPS.

---

## 4. Kekurangan & Masalah yang Ditemukan

### 4.1 Kelemahan struktural VIO di lapangan rumput terbuka

| Penyebab | Efek |
|---|---|
| Setengah FOV = langit (fisheye ±170°, forward-facing) | Langit = nol fitur visual → VIO kehilangan anchor, jatuh ke IMU murni, drift |
| Rumput = tekstur homogen | Sedikit fitur yang bisa di-track, kualitas rendah |
| Permukaan mengkilap/licin | Tracking makin buruk (disebutkan eksplisit di checklist prasyarat sebagai kondisi yang harus dihindari) |

Ini bukan bug perangkat lunak — ini keterbatasan fundamental VIO monocular/
stereo di lingkungan low-texture, yang justru adalah kondisi khas lapangan
kompetisi outdoor berumput.

### 4.2 Bug serius: covariance T265 dibuang total sebelum sampai EKF (ditemukan & diperbaiki 2026-08-07)

Temuan paling signifikan dari investigasi arsitektur ini. Ditelusuri
end-to-end lewat source code (realsense-ros → vision_to_mavros → mavros →
ArduPilot):

```
T265 tracker_confidence (0-3, real-time)
  → realsense2_camera: /camera/odom/sample — covariance BENAR
    (formula: cov = 0.01 * 10^(3 - confidence))
  → vision_to_mavros.cpp (versi LAMA): baca TF SAJA, publish
    geometry_msgs::PoseStamped — message ini TIDAK PUNYA field
    covariance. Confidence DIBUANG di titik ini.
  → mavros vision_pose_estimate plugin: covariance yang dikirim ke
    FC SELALU NOL
  → ArduPilot: posErr = cbrtf(0) = 0 → di-clamp ke floor tetap
    VISO_POS_M_NSE (default 0.2m, tidak pernah di-set eksplisit)
```

**Akibatnya: EKF3 tidak pernah benar-benar tahu kapan tracking confidence
T265 menurun** — satu-satunya proteksi adalah guard NaN total (kalau
tracking hilang SEPENUHNYA), bukan degradasi bertahap. Drone bisa terbang
dengan estimasi posisi yang diam-diam memburuk tanpa EKF "curiga".

Ditemukan juga kesalahpahaman lama soal parameter tuning: `EK3_VIS_DELAY`/
`EK3_VIS_VERR_MIN/MAX` (sempat di-tuning di `t265_from_TFLuna_delta.param`)
ternyata **hanya berlaku untuk jalur MAVLink `VISION_POSITION_DELTA`**
(body-odometry), **bukan** `VISION_POSITION_ESTIMATE` (pose absolut) yang
dipakai T265 — efeknya nol ke setup ini. Parameter yang benar-benar
menentukan seberapa EKF percaya T265 adalah `VISO_POS_M_NSE`/
`VISO_VEL_M_NSE`/`VISO_YAW_M_NSE`, yang saat itu masih nilai default
(belum pernah di-tuning berbasis data log terbang sungguhan).

**Fix yang sempat diimplementasikan** (`vision_to_mavros.cpp`): subscribe
langsung `/camera/odom/sample`, publish `PoseWithCovarianceStamped` (bukan
`PoseStamped` polos) ke topic baru, isi covariance dari data odom asli
kalau masih segar (<1 detik) — kalau basi, failsafe ke covariance besar
(EKF dipaksa skeptis, bukan diam-diam pakai data lama). Divalidasi: build
sukses, live-test covariance yang terkirim match `/camera/odom/sample`.
**Belum pernah divalidasi dengan terbang sungguhan di rumput** — proyek
sudah keburu pindah arsitektur sebelum sampai tahap itu.

### 4.3 Kalibrasi mounting tidak lengkap

- Offset fisik `VISO_POS_X` sudah terukur (0.06m), **`VISO_POS_Y/Z` belum
  pernah diukur fisik** — kalau drone roll/pitch, EKF3 bisa menerjemahkan
  offset kamera-ke-CoG yang salah jadi translasi posisi fiktif.
- Kompensasi tilt 15° (`pitch_cam`) tervalidasi **matematis** (simulasi
  rantai rotasi quaternion persis kode aslinya) tapi **tidak pernah
  diverifikasi fisik** (nyalakan T265 sungguhan, cek drone level terbaca
  level di `/mavros/vision_pose/pose_cov`).

### 4.4 Kalibrasi IMU internal tidak memungkinkan

Sempat direncanakan sebagai langkah remediasi drift, ternyata **tidak
applicable**: tool kalibrasi resmi (`rs-imu-calibration` dari librealsense)
eksplisit hanya untuk D435i/L515 — T265 pakai arsitektur closed/tidak
punya jalur EEPROM writable yang dipakai tool itu. T265 sendiri sudah
*discontinued* oleh Intel sejak 2021, firmware yang terpasang sudah versi
terakhir yang tersedia. Artinya bias gyro internal T265 (kontributor drift
saat terbang lama) **tidak bisa dikoreksi dari sisi user sama sekali** —
keterbatasan permanen perangkat, bukan sesuatu yang bisa "diperbaiki
nanti".

### 4.5 Ketergantungan USB & isu operasional

- Butuh USB3 untuk bandwidth data penuh; port/kabel yang salah bisa
  menyebabkan tracking berkualitas rendah tanpa error yang jelas.
- Gejala umum: `RS2_USB_STATUS_BUSY` — proses lama masih menahan device,
  perlu `pkill -9 -f realsense2_camera` manual sebelum start ulang.
- Origin non-persisten (§3.3) menambah risiko human-error prosedural di
  lapangan yang tidak ada padanannya di arsitektur GPS.

### 4.6 Belum pernah diterbangkan sebagai satu misi utuh

Per dokumentasi terakhir (`WP_T265_TO_GUIDED_FLOW.md`): tiap komponen
(Loiter, walk test, GUIDED per-leg) sempat divalidasi terpisah, tapi
**tidak pernah** ada satu sesi terbang fisik yang menjalankan seluruh
rangkaian misi dari takeoff sampai landing dengan T265 sebagai satu-satunya
sumber posisi. SITL yang divalidasi ekstensif (`§5` dokumen yang sama)
memakai EKF default ArduCopter (GPS simulasi) — **tidak pernah benar-benar
menguji jalur T265/EKF3/vision_to_mavros sama sekali**, jadi validasi SITL
yang ada tidak bisa dipakai sebagai bukti kesiapan arsitektur T265.

---

## 5. Perbandingan Langsung dengan Arsitektur GPS (Final)

| | T265 (VIO, lama) | GPS + MTF-01 (final, sekarang) |
|---|---|---|
| Sumber posisi | Kamera + IMU, relatif titik takeoff | Satelit, absolut (lat/lon) |
| Origin | Manual, di-set ulang tiap boot, **tidak boleh cabut daya** di tengah sesi | Otomatis dari fix GPS pertama, persisten per-boot |
| Ketahanan lapangan terbuka berumput | **Lemah** — tekstur homogen + langit di FOV = drift | Kuat — GPS tidak bergantung tekstur visual sama sekali |
| Sumber heading | Kamera (kompas dimatikan) — hilang bareng saat tracking loss | Kompas independen, tidak terpengaruh kualitas posisi |
| Verifikasi sebelum terbang | Walk test manual tiap sesi | Cek fix_type/sats/HDOP (otomatis via `run_mission_gps.sh`) |
| Bug diketahui | Covariance-ke-EKF pernah selalu nol (§4.2, sudah di-patch tapi belum divalidasi fisik) | Tidak ada isu setara yang ditemukan |
| Kalibrasi sensor | IMU internal tidak bisa dikalibrasi ulang (perangkat closed) | Kompas standar, prosedur kalibrasi baku tersedia |
| Cadangan saat sumber utama lemah | — | MTF-01 optical flow (SRC2, otomatis) |
| Status validasi lapangan | Belum pernah 1 misi utuh | (lihat `PANDUAN_TERBANG_FINAL.md` untuk status terkini) |

Kolom kanan bukan berarti arsitektur GPS otomatis "sempurna" — dia punya
blocker sendiri yang berbeda (lihat `PANDUAN_TERBANG_FINAL.md` §0/§2,
terutama soal parameter EK3_SRC yang belum dikonfirmasi dan koordinat GPS
yang belum disurvei di venue asli). Tabel ini murni membandingkan
**karakteristik struktural** kedua pendekatan navigasi, bukan status
kesiapan hari-ini.

---

## 6. Peran T265 Setelah Keputusan Pindah ke GPS

Dikonfirmasi eksplisit dari kode (2026-08-08): **T265 tidak perlu
menyala sama sekali** di arsitektur GPS — bukan cuma `VISO_TYPE=0`
(dilepas dari fusi EKF), tapi memang tidak ada node/topic apa pun di
`mission_control` yang mereferensikan data fisheye T265
(`grep -rln fisheye` di seluruh package kosong). Kamera perception
(ArUco/WP marker via C920 `camera_down`, gate via C310 `camera_front`)
adalah hardware USB yang sama sekali terpisah dari T265. Artinya secara
fisik, T265 boleh dicabut total dari drone tanpa memengaruhi apa pun di
misi final.

---

## 7. Referensi File (histori, bukan dipakai di misi final)

| File | Isi |
|---|---|
| `PANDUAN_TERBANG_T265.md` | Panduan operasional lengkap (wiring, param, tes bertahap Stabilize→Loiter→Auto) |
| `docs/WP_T265_TO_GUIDED_FLOW.md` | Alur data detail + tutorial SITL/lapangan, termasuk troubleshooting yang pernah ditemukan |
| `docs/PANDUAN_GPS_OPTFLOW_MTF01.md` §1 | Diagnosis drift T265 lengkap + keputusan resmi pindah ke GPS+OptFlow |
| `params/t265_nongps_ekf3.param` | Profil parameter EKF3 non-GPS untuk T265 |
| `params/t265_from_TFLuna_delta.param` | Tuning `EK3_VIS_VERR_*` (terbukti tidak berefek, lih. §4.2) |
| `src/vision_to_mavros/` | Package jembatan T265 pose → mavros (termasuk fix covariance §4.2) |

---

Dokumen ini murni histori/referensi — tidak ada tindak lanjut yang
direncanakan kecuali GPS gagal total di lapangan dan tim butuh fallback
darurat. Kalau itu terjadi: baca §4 dulu sebelum terbang, terutama status
fix covariance (sudah di-patch tapi belum divalidasi fisik) dan
`VISO_POS_Y/Z` yang belum pernah diukur.

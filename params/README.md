# Parameter ArduPilot — T265 non-GPS (EKF3)

File `t265_nongps_ekf3.param` = konfigurasi FC untuk navigasi non-GPS pakai T265,
sesuai PANDUAN_TERBANG_T265.md Bab 2. Format `NAMA,NILAI` (loadable).

> ⚠️ Firmware FC ini (ArduCopter 4.5+) **hanya punya EKF3** — EKF2 sudah dihapus.
> File EK2 lama diarsipkan di `archive/t265_nongps_ekf2.param.OLD`, jangan dipakai.

## Cara load
- **Mission Planner**: Config → Full Parameter List → **Load** → pilih file ini →
  **Write Params** → **REBOOT Pixhawk**.
- **MAVProxy/mavros alternatif**: set satu per satu `param set NAMA NILAI` lalu reboot.

## Isi & arti
| Param | Nilai | Arti |
|---|---|---|
| AHRS_EKF_TYPE | 3 | Pakai EKF3 |
| EK3_ENABLE | 1 | Aktifkan EKF3 |
| EK2_ENABLE | 0 | Matikan EKF2 |
| GPS1_TYPE | 0 | Matikan driver GPS |
| VISO_TYPE | 2 | Sumber visual odometry = Intel T265 |
| EK3_SRC1_POSXY | 6 | Posisi horizontal = ExternalNav (T265) |
| EK3_SRC1_VELXY | 6 | Kecepatan horizontal = ExternalNav |
| EK3_SRC1_POSZ | 1 | Posisi vertikal = Baro (tahan getaran) |
| EK3_SRC1_VELZ | 6 | Kecepatan vertikal = ExternalNav |
| EK3_SRC1_YAW | 6 | Heading = ExternalNav (heading kamera) |
| COMPASS_ENABLE / USE / USE2 / USE3 | 0 | Matikan kompas (heading dari T265) |
| SERIAL2_PROTOCOL | 2 | TELEM2 = MAVLink 2 (port ke Jetson) |
| SERIAL2_BAUD | 921 | 921600 baud |
| BRD_SER2_RTSCTS | 0 | Matikan flow control (CTS/RTS tak dikabel) |

## File parameter

| File | Isi |
|---|---|
| `t265_nongps_ekf3.param` | EKF3 non-GPS (T265) — wajib tiap setup awal |
| `flight_tuning.param` | Kecepatan, kehalusan, landing smooth — baca bawah |
| `sr1_telem_57600.param` | Stream rate radio SiK 57600 |
| `archive/t265_nongps_ekf2.param.OLD` | EK2 usang, JANGAN dipakai |

## `flight_tuning.param` — kecepatan / kehalusan / landing

Nilai **konservatif** untuk drone non-GPS T265 (lapangan kecil). Default ArduPilot
ditandai sebagai komentar. Hanya load **setelah walk test lulus** + uji SITL.

**Cara load:** Mission Planner → Config → Full Parameter List → **Load** → pilih file
→ **Write Params** → **REBOOT Pixhawk**. Jangan load saat armed.

### Kelompok param
- **Kecepatan AUTO/RTL/Guided:** `WPNAV_SPEED` (3 m/s), `WPNAV_SPEED_UP` (1.5),
  `WPNAV_SPEED_DN` (1.0). T265 rawan drop tracking di kecepatan tinggi →保守.
- **Kecepatan pilot (Stabilize/AltHold):** `PILOT_MAX_VEL_XY`, `PILOT_SPEED_UP/DN` —
  di-match ke 3 m/s biar drone gak loncat saat switch mode.
- **Yaw halus:** `ATC_SLEW_YAW` 30 deg/s (default 60) → putar tenang, kurangi
  gangguan tracking T265.
- **Loiter halus:** `LOIT_BRK_ACCEL` 50, `LOIT_BRK_JERK` 500, `LOIT_SPEED` 200.
- **Landing smooth:** `LAND_SPD_HIGH_MS` 1.0 (turun awal), `LAND_SPD_MS` 0.35 (final),
  `LAND_ALT_LOW_M` 10 (alt switch ke final). Jangan `LAND_SPD_MS` < 0.3 (bounce).

### Tuning lanjutan (TIDAK ada di file ini — perlu test flight)
Param di file ini hanya navigasi. Untuk tune **attitude controller** (roll/pitch rate P,
notch filter, AutoTune) butuh flight test fisik — lihat wiki ArduPilot "Tuning Process".
PSC_POSXY/VELXY gains dibiarkan default; ubah hanya bila Loiter/AUTO oscillate.

## Catatan penting EKF3
- **Orientasi T265 = FRONT-facing** (keputusan utk lapangan rumput). Saat bring-up,
  Terminal 3 pakai `t265_tf_to_mavros.launch` (varian forward, USB ke kanan).
  Param FC di atas TIDAK berubah krn orientasi (ditangani di transform sisi ROS).
- `EK3_SRC1_POSZ = 1` (Baro) sengaja dipilih, bukan ExternalNav: altitude T265 rawan
  drift/getaran. Kalau Loiter altitude buruk, baru pertimbangkan ganti ke 6.
- Param precision-landing (`PLND_*`) sengaja BELUM diaktifkan — tahap nanti setelah
  Loiter sukses.
- TELEM2 Pixhawk 6C = SERIAL2. Kalau wiring ke TELEM lain, ganti nomor SERIALx.

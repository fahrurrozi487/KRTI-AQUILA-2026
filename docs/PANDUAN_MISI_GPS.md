# PANDUAN: Jalankan Misi GUIDED pakai GPS+OptFlow (bukan T265)

Pelengkap `PANDUAN_GPS_OPTFLOW_MTF01.md` (diagnosis + param profil GPS+OptFlow) —
dokumen ini fokus ke **cara jalanin misi** (`mission_d.py`) pakai profil itu.

## Kenapa gak perlu ubah kode misi

`mission_d.py`/`phaseD_mission.launch` cuma baca `/mavros/local_position/pose` dan
kirim setpoint ke `/mavros/setpoint_position/local` — **gak peduli** posisi itu
asalnya dari GPS atau T265 (VIO). Semua logic scan/align/drop/land/traverse tetap
sama. Yang beda cuma **bring-up navigasinya**: T265 butuh node kamera + bridge +
set origin manual; GPS udah dapet origin otomatis dari satelit begitu fix.

Node yang **TIDAK dipakai** di mode GPS (khusus non-GPS/T265):
`realsense2_camera` (T265), `vision_to_mavros` (bridge T265→mavros),
`set_origin_home.launch` (docstring-nya sendiri bilang "untuk terbang non-GPS").

## ⚠️ WAJIB dicek sebelum terbang

1. **`params/gps_optflow_ek3.param` sudah di-load ke FC** (MP → Full Parameter List
   → Load → Write → **Reboot**, jangan saat armed). Sekali per konfigurasi FC,
   bukan tiap boot.
2. **`waypoints.yaml` harus valid utk origin EKF yang aktif di mode GPS.** File
   yang ada sekarang direkam pakai `record_waypoint.py` di bawah T265 — origin-nya
   = titik manual dari `set_origin_home.py`, selalu (0,0,0) tepat di titik itu.
   Di mode GPS, origin EKF datang **otomatis** dari GPS fix, gak dijamin persis
   sama dengan titik manual itu. Dua cara benerin, pilih salah satu:

   **Cara A — survei ulang fisik** (sama seperti alur T265, lih.
   `PANDUAN_AUTO_DAN_ARUCO_DROP.md`), dilakukan SAAT mavros sudah connect ke FC
   yang sudah pakai profil GPS:
   ```bash
   rosrun mission_control record_waypoint.py _name:=wp1
   rosrun mission_control record_waypoint.py _name:=wp2
   # dst
   ```

   **Cara B — definisikan lat/lon ASLI sekali, convert otomatis tiap sesi
   (Direkomendasikan, gak perlu jalan fisik ulang tiap kali origin beda)**:
   isi `config/waypoints_latlon.yaml` (template sudah ada) dengan lat/lon hasil
   survei. Dua cara isi:
   - **Manual**: edit YAML langsung (dari GPS/RTK genggam, peta presisi, dst)
   - **Rekam pakai drone**: terbang GUIDED ke tiap titik fisik (GPS fix bagus),
     lalu di titik itu:
     ```bash
     rosrun mission_control record_waypoint_latlon.py _name:=wp1
     rosrun mission_control record_waypoint_latlon.py _name:=gate_double
     # dst tiap titik
     ```
     Baca `/mavros/global_position/global` (lat/lon, fix ter-filter EKF — MENOLAK
     rekam kalau GPS belum fix, bukan diam-diam simpan koordinat ngaco) +
     `/mavros/local_position/pose` (z/AGL, konsisten sama `waypoints.yaml`).
     Companion `record_waypoint.py` (ENU lokal) — pola sama, cuma beda yang
     disimpan. Entri lain di file gak kesentuh (merge, bukan overwrite total).

   Lalu SEBELUM tiap misi (sesudah mavros connect + GPS fix / origin established):
   ```bash
   rosrun mission_control latlon_to_waypoints.py
   ```
   Baca origin EKF yang LAGI AKTIF (`/mavros/global_position/gp_origin`),
   convert ke ENU, tulis `waypoints.yaml` — otomatis benar brapa pun origin
   GPS hari itu beda dari sesi sebelumnya. Entri lain (mis. `gate_double`/
   `gate_triple` yang direkam fisik) tidak disentuh. Preview dulu tanpa nulis
   file (offline, gak perlu mavros):
   ```bash
   rosrun mission_control latlon_to_waypoints.py _preview:=true _lat0:=<lat> _lon0:=<lon>
   ```
   Detail lengkap & alasan desain: docstring `scripts/latlon_to_waypoints.py`.

## Cara jalanin (script)

```bash
cd ~/catkin_ws/src/mission_control/scripts
./run_mission_gps.sh <mission_name> [args tambahan phaseD_mission.launch]

# contoh
./run_mission_gps.sh seleksi do_takeoff:=false use_aruco:=true sim_markers:=false
```

Script ini (`run_mission_gps.sh`):
1. Checklist manual (param sudah di-load? GPS lock di tempat terbuka?) — jawab
   `n` kalau ragu, script berhenti.
2. Nyalain `roscore` + `mavros` (`fcu_url` default `/dev/ttyTHS1:921600`, override
   lewat env var `FCU_URL=... ./run_mission_gps.sh ...`).
3. Tunggu `mavros` connected (timeout 30s).
4. Cek GPS otomatis via `/mavros/gpsstatus/gps1/raw` (`fix_type>=3` / 3D,
   `satellites_visible>=8`, `HDOP<1.5`) — timeout `GPS_TIMEOUT` detik (default 30,
   override via env var). Kalau belum memenuhi syarat, kamu ditanya manual mau
   tetap lanjut atau tidak (bukan auto-block, karena kadang syarat ideal gak
   selalu perlu buat sekadar tes darat).
5. `roslaunch mission_control phaseD_mission.launch mission:=<mission_name> ...`
   — semua arg tambahan setelah nama misi diteruskan apa adanya (`use_aruco`,
   `use_wp_marker`, `do_takeoff`, dst — sama seperti pakai `phaseD_mission.launch`
   langsung, lih. `PANDUAN_AUTO_DAN_ARUCO_DROP.md`).
6. `Ctrl-C` di mission matiin `roscore`/`mavros` juga (trap cleanup).

## Verifikasi manual (opsional, lebih detail dari auto-check script)

Sama seperti checklist §3.1 `PANDUAN_GPS_OPTFLOW_MTF01.md`: MAVLink Inspector →
`OPTICAL_FLOW_RAD.quality` > 100, `DISTANCE_SENSOR` valid (TF-Luna), `GPS_RAW_INT`
fix 3D / HDOP < 1.5 / sats ≥ 8.

## Referensi

- Diagnosis & param profil: `PANDUAN_GPS_OPTFLOW_MTF01.md`
- Resep T265 (perbandingan): `AGENTS.md` §SITL test recipe, `WP_T265_TO_GUIDED_FLOW.md`
- Script bring-up + misi: `scripts/run_mission_gps.sh`
- Script konversi lat/lon → ENU: `scripts/latlon_to_waypoints.py`,
  template input: `config/waypoints_latlon.yaml`
- Script rekam lat/lon dari drone: `scripts/record_waypoint_latlon.py`
  (companion `scripts/record_waypoint.py`, versi ENU lokal)
- Survei WP & jalanin misi ArUco: `PANDUAN_AUTO_DAN_ARUCO_DROP.md`

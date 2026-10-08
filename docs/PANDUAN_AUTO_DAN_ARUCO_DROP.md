# Panduan: AUTO + Drop Servo & GUIDED Scan ArUco + Drop + Land WP3

Dua mode uji/terbang terpisah. **Jari siap Stabilize.**

> **14 Sep 2026 — MIGRASI T265 → GPS+OptFlow.** Bring-up di bawah dulu pakai
> T265 (`vision_to_mavros`, `set_origin_home.launch`, `ttyTHS1`). Proyek
> sekarang terbang GPS+OptFlow (lih. `scripts/run_mission_gps.sh`, `mav.parm`:
> `EK3_SRC1_POSXY/VELXY=3` GPS; Pixhawk di `/dev/ttyACM0` USB). Sudah
> disesuaikan; kalau memang mau setup T265 lama, konfirmasi dulu.

| Mode | File rute | Kamera | Drop |
|---|---|---|---|
| **A. AUTO** | `mission_auto_drop.yaml` | Tidak | **Buta** (koordinat survei) |
| **B. GUIDED** | `mission_aruco_drop.yaml` | ArUco bawah | **Align** ke marker id=2 |

Record WP **sama** untuk keduanya: `record_waypoint.py` → `waypoints.yaml`.

---

## Prasyarat bersama

### Bring-up navigasi (tiap boot, di titik takeoff)
```bash
cd ~/catkin_ws && source devel/setup.bash

roslaunch mavros apm.launch fcu_url:=/dev/ttyACM0:921600

# tunggu GPS fix sebelum lanjut survei/terbang:
rostopic echo -n1 /mavros/gpsstatus/gps1/raw
# lanjut kalau fix_type>=3, satellites_visible>=8, eph<150
```

Cek: `connected: True`, `rostopic echo -n1 /mavros/global_position/rel_alt` ~0
di tanah (kalau minus besar / ngelilir terus, GPS/baro belum siap — jangan
lanjut), walk test/Loiter sudah OK.

### Survei WP (satu sesi origin GPS — jangan reboot FC di tengah)
```bash
rosrun mission_control record_waypoint.py _name:=wp1
rosrun mission_control record_waypoint.py _name:=wp2   # atas wadah drop
rosrun mission_control record_waypoint.py _name:=wp3   # land
```
`z` yang tersimpan sekarang AGL asli (`/mavros/global_position/rel_alt`, 14
Sep 2026 — bukan lagi `local_position/pose.z`). Edit `config/waypoints.yaml`:
set **`z`** ke ketinggian terbang dalam meter AGL (bukan z≈0 dari gendong).

### Servo drop
- Channel **8**, open(drop)=**1013**, close=**2015** (default di kode,
  dikonfirmasi fisik: `SERVO8_FUNCTION=RCIN7`).
- Uji di darat sebelum terbang.

---

# A. AUTO biasa + dropping servo

FC terbang sendiri. **Tidak** scan/align kamera. Drop = tiba di WP2 lalu buka servo.

### Rute (`mission_auto_drop.yaml`)
```
wp1 (hold 2s) → wp2 DROP → wp3 LAND
```

### 1) Dry-run (AMAN)
```bash
# preview offline (tanpa FC)
rosrun mission_control mission_auto.py \
  _mode:=preview \
  _route_file:=$(rospack find mission_control)/config/mission_auto_drop.yaml

# push + readback (disarmed, mavros hidup) -- origin GPS sesi ini diambil OTOMATIS
roslaunch mission_control phaseD_auto.launch \
  route_file:=$(find mission_control)/config/mission_auto_drop.yaml mode:=push
```
Cek log `[AUTO] PUSH sukses` + `[AUTO] origin dari live gp_origin (...)` +
Mission Planner Plan → Read.

### 2) Terbang
1. Takeoff **Stabilize/AltHold**, hover stabil (~z target).  
2. **Jangan** AUTO dari darat.  
3. Switch **AUTO**.  
4. Jari Stabilize siap.  
5. Abort → Stabilize → land manual.

### Catatan AUTO drop
- Drop **buta** di koordinat `wp2` — presisi = kualitas survei + GPS.  
- Tidak ada center ke ArUco.  
- Untuk drop di tengah wadah pakai ArUco → mode **B**.

### Troubleshooting AUTO
| Gejala | Perbaikan |
|---|---|
| Jalur meleset | Origin GPS beda sesi (reboot FC antara survei & terbang); survei ulang |
| Push gagal | mavros connected? cek log ambil "origin dari live gp_origin" bukan fallback |
| Servo tidak buka | ch8, PWM, cek item DO_SET_SERVO di misi FC |
| `rel_alt` aneh / EKF failsafe | GPS fix belum cukup (sats/HDOP); Stabilize dulu |

---

# B. GUIDED: scan ArUco + drop + land WP3

ROS stream setpoint. Kamera bawah align ke marker sebelum drop.

### Rute (`mission_aruco_drop.yaml`)
```
scan WP1 (id=1) → drop WP2 (id=2, align) → land WP3
```

### 1) Kamera + ArUco
```bash
roslaunch mission_control camera_down.launch    # C920 /dev/video1
# atau cameras.launch jika depan juga hidup

# cek deteksi (terpal inverted)
roslaunch mission_control aruco_detect.launch invert:=true
rostopic echo /aruco_node/markers
```

### 2) Urutan terbang
1. Bring-up navigasi + origin (bagian prasyarat).  
2. Kamera bawah hidup.  
3. **Opsi A (disarankan):** pilot takeoff Stabilize → hover di/near WP1 → switch **GUIDED**.  
4. Jalankan misi:

```bash
roslaunch mission_control phaseD_mission.launch \
  mission:=aruco_drop \
  do_takeoff:=false \
  sim_markers:=false \
  use_gate:=false \
  use_aruco:=true \
  aruco_invert:=true
```

**Opsi B:** takeoff dari node (darat, mode akan di-set GUIDED oleh kode):
	```bash
roslaunch mission_control phaseD_mission.launch \
  mission:=aruco_drop \
  do_takeoff:=true \
  sim_markers:=false \
  use_gate:=false \
  use_aruco:=true \
  aruco_invert:=true
```

5. Abort → Stabilize.

### Yang terjadi di node
1. Goto WP1 → hover → **scan** id=1 (beberapa frame).  
2. Goto WP2 → **align** `off_x/off_y` → **servo drop** → hold ~2 s.  
3. Goto WP3 → **LAND**.

### Invert
| Marker | Arg |
|---|---|
| Putih di hitam (terpal) | `aruco_invert:=true` (default) |
| Hitam di putih (cetak normal) | `aruco_invert:=false` |

### Align melenceng arah
Param di `mission_d` (default 1.0):
```bash
# contoh flip horizontal
# tambah di launch node atau:
# _align_sign_x:=-1.0  _align_sign_y:=1.0
```
Letak di kode: `scripts/mission_d.py` → `~align_sign_x`, `~align_sign_y`, `~align_swap_xy`.

### Troubleshooting GUIDED ArUco
| Gejala | Perbaikan |
|---|---|
| Tidak gerak | Harus mode **GUIDED** jika `do_takeoff:=false` |
| Marker tidak kebaca | `invert`, topik `/camera_down/image_raw`, jarak/hover |
| Scan timeout | Lanjut best-effort; pastikan di atas id=1 |
| Drop meleset | Align sign; marker besar 50×50; `align_tol` (default 0.08) |
| Double node ArUco | Jangan jalankan `aruco_detect` terpisah jika `use_aruco:=true` |

---

## Cheatsheet cepat

### AUTO + drop
```bash
# dry-run -- origin GPS sesi ini diambil OTOMATIS (live gp_origin)
roslaunch mission_control phaseD_auto.launch \
  route_file:=$(find mission_control)/config/mission_auto_drop.yaml mode:=push

# terbang: Stabilize hover → switch AUTO
```

### GUIDED scan + drop + land WP3
```bash
roslaunch mission_control camera_down.launch

# hover GUIDED di WP1, lalu:
roslaunch mission_control phaseD_mission.launch \
  mission:=aruco_drop do_takeoff:=false sim_markers:=false \
  use_gate:=false use_aruco:=true aruco_invert:=true
```

### Record WP (sama untuk A & B)
```bash
rosrun mission_control record_waypoint.py _name:=wp1
rosrun mission_control record_waypoint.py _name:=wp2
rosrun mission_control record_waypoint.py _name:=wp3
# edit z di config/waypoints.yaml
```

---

## File terkait

| File | Isi |
|---|---|
| `config/mission_auto_drop.yaml` | AUTO: wp1 → drop wp2 → land wp3 |
| `config/mission_aruco_drop.yaml` | GUIDED: scan → drop → land |
| `config/mission_auto.yaml` | AUTO navigasi saja (tanpa drop) |
| `config/mission_aruco_only.yaml` | GUIDED scan+drop (tanpa land eksplisit di wp3) |
| `launch/phaseD_auto.launch` | Upload misi AUTO |
| `launch/phaseD_mission.launch` | GUIDED mission_d |
| `docs/PANDUAN_MISI_SELEKSI.md` | Misi seleksi penuh (gate + yaw) |
| `docs/TUTORIAL_TERBANG.md` | Stabilize / walk test / Loiter |

---

*AUTO = geometris + drop buta. GUIDED ArUco = scan + align drop + land. Keduanya butuh WP survei + origin yang sama.*

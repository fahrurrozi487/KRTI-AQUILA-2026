# Panduan Lengkap: Terbang AUTO (GPS+OptFlow) + Drop Servo

Dari **record waypoint** sampai **switch AUTO** di udara.  
FC yang terbang sendiri (misi di-upload). **Tanpa** kamera ArUco/gate.

**Jari selalu siap Stabilize.**

> **14 Sep 2026 — MIGRASI T265 → GPS+OptFlow.** Panduan ini dulu ditulis buat
> setup non-GPS (T265, `vision_to_mavros`, `set_origin_home.launch`,
> `ttyTHS1`). Proyek sekarang terbang pakai **GPS+OptFlow** (lih.
> `scripts/run_mission_gps.sh`, dan `mav.parm`: `EK3_SRC1_POSXY/VELXY=3` GPS,
> Pixhawk konek via `/dev/ttyACM0` USB — bukan `ttyTHS1` UART). Bagian
> bring-up di bawah sudah disesuaikan; node T265-only (`set_origin_home.*`,
> `vision_to_mavros`) TIDAK dipakai lagi di jalur ini. Kalau kamu memang masih
> mau terbang non-GPS T265, itu setup LAMA — konfirmasi dulu sebelum ngikutin
> command apa pun di bawah.

---

## 0. Apa itu AUTO di setup ini?

| Item | Isi |
|---|---|
| Eksekutor | Pixhawk (mode **AUTO**) |
| Navigasi | GPS + OptFlow → EKF3 (bukan T265/vision) |
| Rute | Di-upload dari Jetson lewat `mission_auto.py` |
| Drop | **Buta** di koordinat WP2 (`DO_SET_SERVO` ch**8**, dikonfirmasi fisik: `SERVO8_FUNCTION=RCIN7`) — bukan align kamera |
| File rute | `config/mission_auto_drop.yaml` |
| File koordinat | `config/waypoints.yaml` |

```
Survei WP (record) → preview/push misi ke FC → takeoff Stabilize → switch AUTO
```

Rute default drop:
```
wp1 (hold 2s) → wp2 DROP → wp3 LAND
```

---

## 1. Prasyarat

### Hardware / skill
- [ ] Loiter hover stabil  
- [ ] Servo drop ch8 diuji di darat (PWM open **1013** = drop)  
- [ ] RC: Stabilize + AUTO mudah dijangkau  
- [ ] Drone di tempat terbuka (GPS lock butuh sky view, BUKAN indoor)  

### Parameter FC (sekali)
- GPS+OptFlow: `params/gps_optflow_ek3.param` — load ke FC (MP → Full Parameter
  List → Load → Write → **Reboot**) SEBELUM bring-up. Script ini TIDAK
  meng-upload param, itu manual sekali per konfigurasi FC.  
- Kunci: `EK3_SRC1_POSXY/VELXY = 3` (GPS), GPS aktif (bukan `VISO_TYPE`/ExternalNav)  

### Software tiap shell
```bash
cd ~/catkin_ws && source devel/setup.bash
```

---

## 2. Bring-up navigasi (tiap boot)

Lakukan di **titik takeoff** (origin = titik di mana GPS pertama kali fix
setelah FC boot — BUKAN lagi di-set manual seperti T265; jangan reboot FC
setelah ini sebelum survei selesai, origin ikut reset).

**Terminal 1 — mavros**
```bash
roslaunch mavros apm.launch fcu_url:=/dev/ttyACM0:921600
```
(Pixhawk sekarang konek via USB `/dev/ttyACM0` — bukan `/dev/ttyTHS1`. Cek
`ls /dev/ttyACM*` kalau beda di device kamu.)

**Terminal 2 — tunggu GPS fix**, baru lanjut ke bagian survei:
```bash
watch -n1 'rostopic echo -n1 /mavros/gpsstatus/gps1/raw 2>/dev/null | grep -E "fix_type|satellites|eph"'
# lanjut kalau: fix_type >= 3, satellites_visible >= 8, eph < 150 (HDOP < 1.5)
```
(Bisa juga pakai `run_mission_gps.sh` — script itu ngecek ini otomatis + ada
konfirmasi checklist, tapi dia langsung ke `phaseD_mission.launch`/GUIDED,
bukan `phaseD_auto.launch` — buat AUTO tetap lanjut manual pakai step di
bawah setelah GPS OK.)

### Verifikasi
```bash
rostopic echo -n1 /mavros/state
# connected: True

rostopic echo -n1 /mavros/global_position/rel_alt
# harus ~0 di tanah (kalau minus besar / ngelilir terus, JANGAN lanjut --
# ada masalah EKF/baro/GPS, lih. bagian 8. Troubleshooting)
```

---

## 3. Record waypoint (survei)

### Aturan
1. **Satu sesi origin** — jangan reboot FC di tengah survei (origin GPS reset tiap boot).  
2. Rekam posisi **drone** di titik yang diinginkan (gendong atau terbang pelan Stabilize).  
   `x,y` = ENU lokal (relatif origin GPS), `z` = AGL asli dari `/mavros/global_position/rel_alt`
   (14 Sep 2026: bukan lagi `local_position/pose.z` — lih. `record_waypoint.py`).  
3. Setelah selesai, **edit `z`** di YAML ke ketinggian terbang dalam meter AGL (survei darat sering `z≈0`).  
4. Nama WP harus sama dengan di `mission_auto_drop.yaml`: **`wp1`**, **`wp2`**, **`wp3`**.

### Titik yang direkam

| Nama | Di mana |
|---|---|
| `wp1` | Titik lewat pertama (setelah takeoff / area start otonom) |
| `wp2` | **Tepat di atas pusat wadah drop** (penting untuk drop buta) |
| `wp3` | Titik landing |

### Command
```bash
# pastikan bring-up (bagian 2) + GPS fix sudah OK
rosrun mission_control record_waypoint.py _name:=wp1
rosrun mission_control record_waypoint.py _name:=wp2
rosrun mission_control record_waypoint.py _name:=wp3
```

File tersimpan (default):
```text
~/catkin_ws/src/mission_control/config/waypoints.yaml
```

Contoh isi setelah survei:
```yaml
wp1: {x: 5.0, y: 0.0, z: 2.0, qz: 0.0, qw: 1.0}
wp2: {x: 12.0, y: 0.0, z: 2.0, qz: 0.0, qw: 1.0}
wp3: {x: 18.0, y: 0.0, z: 2.0, qz: 0.0, qw: 1.0}
```

### Edit z (wajib dicek)
Buka `config/waypoints.yaml`, pastikan `z` = ketinggian yang aman untuk lewat / drop / land (mis. **2.0**).

### Tips survei
- WP2: usahakan **center wadah** + ketinggian drop yang diinginkan.  
- Jangan reboot FC di tempat lain setelah survei — origin GPS ke-reset, WP jadi bergeser.  
- Kalau FC harus reboot: **survei ulang semua WP**.

---

## 4. Rute AUTO + drop

File: `config/mission_auto_drop.yaml`
```yaml
legs:
  - {wp: wp1, action: waypoint, hold: 2.0}
  - {wp: wp2, action: drop,     hold: 2.0}
  - {wp: wp3, action: land}
```

| action | Arti di FC |
|---|---|
| `waypoint` | NAV_WAYPOINT (+ hold detik) |
| `drop` | Waypoint lalu **DO_SET_SERVO** ch8 open (1013) |
| `land` | NAV_LAND |

Servo: ch**8**, open=**1013** (default di `phaseD_auto.launch` / `mission_auto.py`,
dikonfirmasi fisik: `SERVO8_FUNCTION=RCIN7`).

---

## 5. Dry-run (wajib sebelum terbang)

### 5a. Preview offline (tanpa upload)
```bash
rosrun mission_control mission_auto.py \
  _mode:=preview \
  _route_file:=$(rospack find mission_control)/config/mission_auto_drop.yaml
```
Cek log: daftar item, konversi lat/lon masuk akal, tidak error file/WP.

### 5b. Push ke FC (disarmed)
```bash
# mavros connected, drone DISARMED
roslaunch mission_control phaseD_auto.launch \
  route_file:=$(find mission_control)/config/mission_auto_drop.yaml \
  mode:=push
```

Cek:
- Log: `[AUTO] PUSH sukses` + readback item  
- Mission Planner → **Plan** → **Read**  
- Harus ada: takeoff/home, waypoint wp1, waypoint+servo di wp2, land di wp3  

### Origin di launch AUTO — sekarang otomatis
14 Sep 2026 (v2): `mission_auto.py` **ambil origin sendiri** dari live
`/mavros/global_position/gp_origin` sesi ini (`use_live_origin`, default
true) — gak perlu lagi `rostopic echo` manual + ketik ulang ke `lat:=/lon:=`.
`lat`/`lon` di `phaseD_auto.launch` sekarang cuma **fallback** kalau live
origin gak kepakai (mis. gp_origin belum ke-set/timeout 5s) — di situ baru
jatuh ke angka hardcode lama, dan itu WAJIB dicek manual dulu.

```bash
roslaunch mission_control phaseD_auto.launch \
  route_file:=$(find mission_control)/config/mission_auto_drop.yaml \
  mode:=push takeoff_alt:=2.0
```
Cek log `[AUTO] origin dari live gp_origin (...)`. Kalau yang muncul
`origin dari param ~lat/~lon (fallback)` — artinya live origin GAGAL diambil
(cek GPS fix dulu), dan angka yang kepakai adalah fallback hardcode, **cek
manual apakah itu benar** sebelum lanjut push.

Kalau mau paksa pakai angka manual (skip live fetch, mis. T265/testing):
```bash
roslaunch mission_control phaseD_auto.launch \
  route_file:=$(find mission_control)/config/mission_auto_drop.yaml \
  mode:=push use_live_origin:=false \
  lat:=<latitude> lon:=<longitude> takeoff_alt:=2.0
```

---

## 6. Prosedur terbang AUTO

1. Bring-up + origin (bagian 2).  
2. Misi sudah di-push (bagian 5b) di sesi yang sama.  
3. Arm, takeoff **Stabilize** (atau AltHold).  
4. Naik ke ketinggian aman, hover stabil.  
5. **Switch AUTO** (bukan dari darat).  
6. Amati: wp1 → hold → ke wp2 → drop → ke wp3 → land.  
7. **Abort:** switch **Stabilize** → land manual.

### Urutan visual
```
[Darat] push misi (disarmed)
    ↓
[Stabilize] takeoff + hover
    ↓
[AUTO] FC jalankan rute
    ↓
wp1 → wp2 (DROP) → wp3 (LAND)
    ↓
Selesai / Stabilize jika bermasalah
```

---

## 7. Checklist hari H (AUTO + drop)

```
[ ] Param GPS+OptFlow (params/gps_optflow_ek3.param) di FC, sudah reboot
[ ] Loiter hover sudah lulus
[ ] Drone di tempat terbuka, GPS fix_type>=3 sats>=8 eph<150
[ ] Bring-up mavros (ttyACM0) + GPS fix OK (bagian 2)
[ ] WP wp1, wp2, wp3 tersurvei DI SESI GPS INI; z (AGL) sudah benar
[ ] wp2 = pusat wadah drop
[ ] mission_auto_drop.yaml legs sesuai
[ ] preview OK
[ ] push: log bilang "origin dari live gp_origin" (bukan fallback param)
[ ] push OK + Plan→Read di MP
[ ] Servo ch8 diuji di darat
[ ] Takeoff Stabilize → hover → AUTO
[ ] Jari Stabilize siap
```

---

## 8. Troubleshooting

| Gejala | Perbaikan |
|---|---|
| Timeout pose saat record | mavros belum connected / GPS belum fix |
| WP meleset total | Origin GPS beda sesi (reboot FC di antara survei & terbang); survei ulang |
| `rel_alt` minus besar / ngelilir terus di darat | GPS belum fix (`fix_type<3`) atau baro belum settle — tunggu, jangan lanjut |
| Push gagal | `connected: True`? FC disarmed? |
| AUTO tidak start | Switch di udara setelah hover; mode list RC |
| Drop tidak terjadi | Item DO_SET_SERVO di misi? ch8? PWM? |
| Drop meleset dari wadah | Survei ulang wp2 di sesi GPS yang sama dgn terbang; jangan andalkan drop buta untuk presisi tinggi |
| Land kasar / bounce | `LAND_SPEED` di flight_tuning; TF-Luna jika ada |
| EKF failsafe / GPS glitch | Stabilize; hentikan AUTO; cek GPS sats/HDOP |

---

## 9. Cheatsheet satu halaman

```bash
# --- Bring-up ---
roslaunch mavros apm.launch fcu_url:=/dev/ttyACM0:921600
# tunggu GPS fix (fix_type>=3, sats>=8, eph<150) sebelum lanjut:
rostopic echo -n1 /mavros/gpsstatus/gps1/raw

# --- Record (di sesi GPS yang sama) ---
rosrun mission_control record_waypoint.py _name:=wp1
rosrun mission_control record_waypoint.py _name:=wp2
rosrun mission_control record_waypoint.py _name:=wp3
# edit z (AGL, meter) di: src/mission_control/config/waypoints.yaml

# --- Dry-run ---
rosrun mission_control mission_auto.py _mode:=preview \
  _route_file:=$(rospack find mission_control)/config/mission_auto_drop.yaml

roslaunch mission_control phaseD_auto.launch \
  route_file:=$(find mission_control)/config/mission_auto_drop.yaml mode:=push
# origin diambil OTOMATIS dari gp_origin -- cek log "origin dari live gp_origin"

# --- Terbang ---
# Stabilize takeoff → hover → switch AUTO
```

---

## 10. File terkait

| File | Fungsi |
|---|---|
| `scripts/record_waypoint.py` | Rekam posisi ke YAML |
| `config/waypoints.yaml` | Koordinat WP |
| `config/mission_auto_drop.yaml` | Rute AUTO + drop |
| `config/mission_auto.yaml` | AUTO navigasi saja (tanpa drop) |
| `scripts/mission_auto.py` | Preview / push misi |
| `launch/phaseD_auto.launch` | Launch uploader |
| `launch/set_origin_home.launch`, `scripts/set_origin_home.py` | **T265 non-GPS SAJA** — origin GPS otomatis, TIDAK dipakai di jalur ini |
| `scripts/run_mission_gps.sh` | Referensi bring-up GPS (buat jalur GUIDED, tapi checklist GPS-nya sama) |
| `docs/PANDUAN_AUTO_DAN_ARUCO_DROP.md` | Banding AUTO vs GUIDED ArUco |
| `docs/TUTORIAL_TERBANG.md` | Stabilize / walk test / Loiter |
| `docs/PANDUAN_MISI_SELEKSI.md` | GUIDED seleksi (gate + yaw) |

---

## 11. AUTO vs GUIDED (ingat)

| Butuh | Mode |
|---|---|
| Jalur + drop kasar di wadah (tanpa kamera) | **AUTO** (panduan ini) |
| Drop harus di tengah ArUco wadah | **GUIDED** `mission_aruco_drop` |
| Center gate + yaw + drop | **GUIDED** `mission_seleksi` |

---

*Drop AUTO = buta di WP2. Presisi = survei + GPS. Origin & survei harus satu sesi.*

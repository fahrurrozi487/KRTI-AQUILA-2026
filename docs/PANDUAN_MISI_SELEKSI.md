# Panduan Terbang Misi Seleksi Wilayah

GUIDED autonomous setelah pilot sampai **WP1**.  
Non-GPS (T265) + gate cam (C310) + ArUco cam (C920).

**Jari selalu siap switch Stabilize.**

---

## 0. Ringkasan alur kompetisi

| Segmen | Siapa | Isi |
|---|---|---|
| Takeoff → 2 single gate → **WP1** | **Pilot (manual)** | Autonom belum boleh |
| **WP1 → gate → WP2 → drop → yaw kiri → gate → WP3 land** | **GUIDED (Jetson)** | Manual dilarang |

### Legs otonom (`mission_seleksi.yaml`)

```
scan WP1 (ArUco id=1)
→ traverse gate_double     # sementara: single gate #1 (pengganti double)
→ drop WP2 (ArUco id=2 di wadah)
→ yaw 90° left di WP2
→ traverse gate_triple     # sementara: single gate #2 (pengganti triple)
→ land WP3 (ArUco id=3)
```

### Catatan lapangan saat ini
- Double/triple gate **belum ada** → pakai **2 single gate** (nama WP tetap `gate_double` / `gate_triple`).
- ArUco **50×50 cm** = scan/align; **10×10 cm** = arah (nanti, belum di kode).
- Marker terpal **putih di hitam (inverted)** → detektor `invert:=true` (**default sudah aktif**).
- WP = ID: **WP1=1, WP2=2, WP3=3, WP4=4**.

---

## 1. ArUco invert — sudah di-set?

**Ya.** Default:

| File | Setting |
|---|---|
| `launch/aruco_detect.launch` | `invert` default **`true`** |
| `aruco_node.py` | `~invert` default **`True`** |

Artinya: launch biasa **sudah** mode inverted (cocok terpal).

```bash
# normal (default) — inverted lapangan
roslaunch mission_control aruco_detect.launch

# hanya jika cetakan hitam-di-putih (PNG resmi / cetak normal)
roslaunch mission_control aruco_detect.launch invert:=false
```

Cek di log node: `[aruco] siap. ... invert=True ...`

---

## 2. Prasyarat (sebelum hari lomba / tes)

### Hardware
- [ ] Walk test T265 lulus (drift kecil)
- [ ] Loiter hover stabil
- [ ] C310 depan → `/dev/video0` (gate), default **1280×720 MJPEG**
- [ ] C920 bawah → `/dev/video1` (ArUco), default **640×480 YUYV**
- [ ] Servo drop ch9 terpasang & diuji di darat (PWM open/close)
- [ ] RC: Stabilize + GUIDED mudah dijangkau

### Survei WP (setelah `set_origin_home` di takeoff)

Rekam **di sesi yang sama** dengan origin (jangan reboot FC di tengah):

| Nama WP | Di mana merekam |
|---|---|
| `wp1` | Di / atas ArUco WP1 (setelah 2 single gate manual) |
| `gate_double` | **Di depan** plane gate #1 (~1.5–2.5 m), hadap bukaan |
| `wp2` | Di atas wadah drop (pusat wadah) |
| `gate_triple` | **Di depan** plane gate #2 (~1.5–2.5 m), hadap bukaan |
| `wp3` | Titik land |

```bash
rosrun mission_control record_waypoint.py _name:=wp1
# ... ulangi untuk gate_double, wp2, gate_triple, wp3
```

Lalu edit `config/waypoints.yaml`:
- `z` = ketinggian terbang (tengah bukaan / hover), **bukan** z≈0 dari gendong darat.

### `traverse_through_m`
Harus **lebih besar** dari jarak WP-gate → plane, supaya tembus gate:

```
jarak_akhir = jarak_WP_ke_plane − traverse_through_m
# butuh hasil negatif (sudah lewat plane)
```

Contoh: WP 2 m di depan plane → `traverse_through_m:=3.0` atau lebih.

---

## 3. Bring-up hari H (urutan terminal)

Tiap shell: `cd ~/catkin_ws && source devel/setup.bash`

### A. Navigasi non-GPS (wajib)

**T1 — T265**
```bash
roslaunch realsense2_camera rs_t265.launch initial_reset:=true
```

**T2 — mavros**
```bash
roslaunch mavros apm.launch fcu_url:=/dev/ttyTHS1:921600
# rostopic echo -n1 /mavros/state  → connected: True
```

**T3 — vision bridge**
```bash
roslaunch vision_to_mavros t265_tf_to_mavros.launch enable_precland:=false
# rostopic hz /mavros/vision_pose/pose  → ~15–30
```

**T4 — origin + home di titik takeoff**
```bash
roslaunch mission_control set_origin_home.launch
```

### B. Kamera + detektor

**T5 — kedua kamera**
```bash
roslaunch mission_control cameras.launch
# C310=/dev/video0 depan → 1280x720 MJPEG (default camera_front.launch)
# C920=/dev/video1 bawah → 640x480 YUYV
```

> **Log C310 720p MJPEG:** pesan berulang  
> `No accelerated colorspace conversion found from yuv422p to rgb24`  
> = **WARNING normal di Jetson** (konversi warna di CPU), **bukan error fatal**.  
> Stream tetap jalan. Abaikan selama `rostopic hz` hidup.  
> Kalau CPU/lag:  
> `roslaunch mission_control camera_front.launch image_width:=640 image_height:=480 pixel_format:=yuyv`

**T6 — ArUco (invert default true)**
```bash
roslaunch mission_control aruco_detect.launch
# log: invert=True
# cek: rostopic echo /aruco_node/markers
```

**T7 — Gate** (boleh digabung lewat phaseD; atau terpisah dulu untuk cek)
```bash
roslaunch mission_control gate_detect.launch
# cek: rostopic echo /gate_node/gate
```

### Verifikasi cepat
```bash
rostopic echo -n1 /mavros/state
rostopic hz /mavros/vision_pose/pose
rostopic hz /camera_front/image_raw          # ~15–30 Hz
rostopic echo -n1 /camera_front/image_raw | grep -E "width|height"  # 1280 x 720
rostopic hz /camera_down/image_raw
rostopic echo -n3 /aruco_node/markers
rostopic echo -n3 /gate_node/gate
```

---

## 4. Prosedur terbang misi seleksi

### Fase pilot (manual)
1. Mode **Stabilize** (atau AltHold setelah stabil).
2. Arm, takeoff.
3. Lewati **2 single gate** manual sampai area **WP1**.
4. Hover stabil di / dekat ArUco WP1.
5. Switch mode RC ke **GUIDED**.
6. Pastikan masih hover tenang (posisi T265 OK).

### Fase otonom (Jetson)
**Jangan** jalankan misi sebelum GUIDED + hover di WP1.

```bash
roslaunch mission_control phaseD_mission.launch \
  mission:=seleksi \
  do_takeoff:=false \
  sim_markers:=false \
  use_gate:=true \
  use_aruco:=true \
  aruco_invert:=true \
  traverse_through_m:=3.0
```

`phaseD_mission` start `mission_d` + (opsional) `gate_node` + (opsional) `aruco_node`.

**Invert ArUco dari phaseD (satu command):**
```bash
# terpal putih-di-hitam (default lapangan)
... aruco_invert:=true

# cetakan normal hitam-di-putih
... aruco_invert:=false

# tanpa ArUco (gate-only / SITL mock)
... use_aruco:=false sim_markers:=true   # atau false di lapangan tanpa marker
```

Kalau ArUco sudah dijalankan terpisah di T6, set `use_aruco:=false` agar tidak double node.

### Yang terjadi di otonom
1. **Goto WP1** → **scan** id=1 (konfirmasi marker).  
2. **Goto depan gate #1** → **center** bukaan → **through** lurus.  
3. **Goto WP2** → **align** ArUco id=2 → **drop** servo ch9.  
4. **Yaw 90° kiri** di tempat (di WP2).  
5. **Goto depan gate #2** → center → through.  
6. **Goto WP3** → **LAND**.

### Abort
- Switch **Stabilize** segera.  
- Land manual.  
- Matikan node misi (Ctrl+C di terminal phaseD).

---

## 5. Checklist singkat hari H

```
[ ] Param EKF T265 non-GPS sudah di FC (YAW=6, VISO=2, GPS off)
[ ] Walk test & Loiter sudah lulus di sesi sebelumnya
[ ] WP disurvei di origin yang sama dengan takeoff hari ini
[ ] z WP sudah dikoreksi ke ketinggian terbang
[ ] traverse_through_m disesuaikan lapangan
[ ] T265 + mavros + vision + set_origin_home
[ ] cameras + aruco_detect (invert=True) + gate
[ ] Servo drop diuji di darat
[ ] Pilot: Stabil → 2 gate → WP1 hover → GUIDED
[ ] phaseD mission:=seleksi do_takeoff:=false sim_markers:=false
[ ] Jari Stabilize siap sepanjang otonom
```

---

## 6. Troubleshooting misi seleksi

| Gejala | Cek / perbaikan |
|---|---|
| Tidak gerak setelah launch misi | Mode masih Stabilize? Harus **GUIDED**. `rostopic echo /mavros/state` |
| ArUco tidak kebaca | `invert=True`? Kamera bawah? Marker inverted? `rostopic echo /aruco_node/markers` |
| Scan timeout lanjut saja | Normal (best-effort); pastikan hover di atas marker |
| Drop meleset | Align tol / sign; pastikan id=2 & marker besar di wadah |
| Yaw ke arah salah | `direction: left` di yaml; cek heading T265 vs layout lapangan |
| Gate tidak center / nabrak | `gate_tol=0.12`; `gate_sign_*`; WP di depan plane; through cukup panjang |
| Through belum lewat gate | Naikkan `traverse_through_m` |
| BAD VISION / drift | Stabilize abort; jangan paksa; cek T265 rate |
| Node mati / XML error | `source devel/setup.bash`; komentar launch tanpa `--` |
| Log swscaler yuv422p→rgb24 | **Bukan error** di C310 720p MJPEG; cek `rostopic hz /camera_front/image_raw` |
| Camera calibration yaml not found | Warning normal; tidak wajib untuk off_x/off_y gate/ArUco |
| Front cam bukan 720p | Default di `camera_front.launch` = 1280×720 mjpeg; cek width/height di image msg |

---

## 7. Command cheatsheet

```bash
# Bring-up navigasi
roslaunch realsense2_camera rs_t265.launch initial_reset:=true
roslaunch mavros apm.launch fcu_url:=/dev/ttyTHS1:921600
roslaunch vision_to_mavros t265_tf_to_mavros.launch enable_precland:=false
roslaunch mission_control set_origin_home.launch

# Kamera + detektor
roslaunch mission_control cameras.launch
roslaunch mission_control aruco_detect.launch          # invert true default
roslaunch mission_control gate_detect.launch

# Survei (sebelum lomba / di sesi origin yang sama)
rosrun mission_control record_waypoint.py _name:=wp1

# Misi otonom (setelah pilot di WP1 + GUIDED)
# aruco_invert:=true  → terpal inverted; false → cetak normal
roslaunch mission_control phaseD_mission.launch \
  mission:=seleksi do_takeoff:=false sim_markers:=false \
  use_gate:=true use_aruco:=true aruco_invert:=true \
  traverse_through_m:=3.0
```

---

## 8. File terkait

| File | Fungsi |
|---|---|
| `config/mission_seleksi.yaml` | Urutan legs otonom |
| `config/waypoints.yaml` | Koordinat WP (hasil survei) |
| `scripts/mission_d.py` | State machine GUIDED |
| `launch/phaseD_mission.launch` | Launch misi |
| `launch/aruco_detect.launch` | ArUco, **invert default true** |
| `launch/gate_detect.launch` | Gate |
| `launch/cameras.launch` | C310 + C920 |
| `launch/camera_front.launch` | C310 gate: **1280×720 MJPEG** (default) |
| `launch/camera_down.launch` | C920 ArUco: 640×480 YUYV |
| `docs/TUTORIAL_TERBANG.md` | Stabilize / walk test / Loiter / AUTO umum |
| `docs/HANDOFF.md` | Keputusan teknis & gotcha |

---

## 9. Belum di misi (nanti)

- ArUco **10×10** sebagai heading (sekarang yaw fixed 90° kiri).  
- Double / triple gate asli (sekarang 2 single).  
- Otonom dari takeoff (tidak sesuai aturan seleksi).

---

*Seleksi wilayah: pilot sampai WP1, lalu GUIDED `mission_seleksi` sampai land WP3.*

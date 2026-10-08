# Tutorial Terbang — Non-GPS T265 (Jetson + Pixhawk)

Panduan urut dari bring-up hingga misi GUIDED gate-only.  
**Aturan emas:** Stabilize/AltHold stabil dulu → walk test lulus → baru Loiter → baru AUTO/GUIDED.  
**Jari selalu siap switch Stabilize.**

---

## 0. Checklist pra-terbang

### Hardware
- [ ] Pixhawk 6C + Jetson Nano, UART `/dev/ttyTHS1` @921600 (TELEM2)
- [ ] T265 front-facing (USB ke kanan), mount kencang
- [ ] Remote RC: mode Stabilize / AltHold / Loiter / GUIDED / AUTO / LAND
- [ ] Area lapang, tekstur di tanah (bukan lantai polos mengkilap)
- [ ] Baterai drone + power Jetson

### Software (tiap shell baru)
```bash
cd ~/catkin_ws && source devel/setup.bash
```

### Parameter FC (sekali, lalu reboot Pixhawk)
```text
Mission Planner → Full Parameter List → Load:
  ~/catkin_ws/params/t265_nongps_ekf3.param
→ Write Params → REBOOT
```
Kunci: `EK3_SRC1_YAW=6` (bukan 1), `EK3_SRC1_POSZ=1` (Baro), GPS off, compass off.

Opsional kecepatan/landing: `params/flight_tuning.param` (setelah walk test lulus).

---

## 1. Bring-up tiap boot (wajib non-GPS)

Jalankan **4 terminal terpisah** di titik takeoff:

**Terminal 1 — T265**
```bash
roslaunch realsense2_camera rs_t265.launch initial_reset:=true
# diamkan ~3 detik sampai tracking lock
```

**Terminal 2 — mavros**
```bash
roslaunch mavros apm.launch fcu_url:=/dev/ttyTHS1:921600
# cek: rostopic echo -n1 /mavros/state  →  connected: True
```

**Terminal 3 — jembatan pose**
```bash
roslaunch vision_to_mavros t265_tf_to_mavros.launch enable_precland:=false
# cek: rostopic hz /mavros/vision_pose/pose  →  ~15–30 Hz
```

**Terminal 4 — origin + home (DI TITIK TAKEOFF)**
```bash
roslaunch mission_control set_origin_home.launch
# origin = (0,0,0) untuk semua WP; home current_gps=True → rel_alt≈0 di tanah
```

### Verifikasi bring-up
```bash
rostopic echo -n1 /mavros/state          # connected: True
rostopic hz /mavros/vision_pose/pose     # ~15–30
```
Di Mission Planner:
- [ ] `VISION_POSITION_ESTIMATE` masuk (Ctrl+F → Mavlink Inspector)
- [ ] `rel_alt` ≈ 0 di tanah
- [ ] Tidak spam “stopped aiding” / “BAD VISION”

---

## 2. Walk test (gerbang mutlak sebelum terbang vision)

**Tujuan:** buktikan jejak T265 searah & seskala dengan gerakan nyata.  
**Motor MATI.** Gendong drone.

### Langkah
1. Bring-up + `set_origin_home` di titik A (titik takeoff).
2. Mission Planner: lihat posisi drone di peta / HUD.
3. Gendong drone, **jalan pelan membentuk kotak ~2 m × 2 m**, balik ke titik A.
4. Bandingkan jejak di MP dengan jalur kaki.

### Lulus
- [ ] Jejak mirip kotak (bukan acak / muter sendiri)
- [ ] Skala kira-kira benar (2 m di lapangan ≈ 2 m di peta)
- [ ] Kembali dekat titik awal (drift kecil)
- [ ] Tidak “BAD VISION” berulang / EKF putus-putus

### Gagal — apa yang dicek

| Gejala | Kemungkinan | Perbaikan |
|---|---|---|
| Jejak muter / acak | Yaw salah; `EK3_SRC1_YAW` bukan 6 | Load ulang `t265_nongps_ekf3.param`, reboot |
| Skala meleset konsisten (mis. 5 m jadi 3–4 m) | Scale error T265 outdoor 20–30% | Ukur 5 m lurus; catat rasio; nanti `scale_factor` di vision_to_mavros (jangan tebak) |
| Jejak geser satu arah terus | Mount miring / `pitch_cam` | Level mount atau set `pitch_cam` di launch T265 |
| Pose hilang / NaN | T265 belum lock | `initial_reset:=true`, diamkan 3 s, cahaya & tekstur cukup |
| “BAD VISION” / stopped aiding | Vision ditolak EKF | Kurangi goyangan; cek rate pose; jangan test di rumput sangat polos dulu |
| `rel_alt` aneh di tanah | Home altitude salah | Ulangi `set_origin_home` (current_gps, bukan alt manual) |
| MP lag parah | Radio SiK 57600 | Bukan error Jetson; lihat `params/sr1_telem_57600.param` |

### Ukur skala (setelah kotak kira-kira OK)
1. Dari origin, jalan **lurus 5 m** (pita ukur).
2. Baca jarak di MP / `local_position`.
3. Rasio = jarak_nyata / jarak_EKF.  
   - ~1.0 → bagus  
   - 1.2–1.4 konsisten → scale error; catat, koreksi software belakangan  
   - Acak tiap kali → tracking jelek, jangan terbang

**Jangan lanjut Loiter/AUTO/GUIDED misi jika walk test belum lulus.**

---

## 3. Tes Stabilize

**Tujuan:** kontrol manual + getaran/arming aman. Tidak butuh posisi T265 bagus.

1. Bring-up (minimal mavros; T265 boleh ikut).
2. RC: mode **Stabilize**.
3. Arm (sesuai safety switch / prosedurmu).
4. Throttle pelan, hover rendah ~0,5–1 m beberapa detik.
5. Mendarat, disarm.

### Lulus
- [ ] Arm/disarm normal  
- [ ] Hover manual terkendali  
- [ ] Tidak osilasi parah  

### Miss / masalah
| Gejala | Tindakan |
|---|---|
| Tidak arm | Pre-arm: RC, safety, EKF, GPS (non-GPS: pastikan param vision) |
| Getar hebat | Tune motor/prop; cek mount FC |
| Drift kuat di Stabilize | Normal sedikit; kalau parah cek CG/kalibrasi accel |

---

## 4. Tes AltHold

**Tujuan:** baro tahan ketinggian. Posisi horizontal masih manual.

1. Mode **AltHold**.
2. Takeoff, lepas stick throttle di tengah.
3. Hover; ketinggian harus relatif tetap.
4. Geser horizontal pelan; mendarat.

### Lulus
- [ ] Ketinggian tertahan  
- [ ] Tidak panjat/turun liar  

### Miss
| Gejala | Tindakan |
|---|---|
| Naik-turun di dekat tanah | Prop-wash ke baro; jangan landing test di tempat sempit dulu |
| `rel_alt` offset | Ulangi set home / origin |

---

## 5. Tes Loiter (ujian utama T265)

**Hanya setelah walk test lulus.**  
Loiter = tahan posisi horizontal pakai EKF+vision.

1. Mode **Loiter** (boleh arm di Loiter jika EKF hijau).
2. Naik pelan ~1 m, **lepas semua stick**.
3. Amati 10–20 detik.

### Lulus
- [ ] Drone diam menggantung (geser kecil wajar)  
- [ ] Tidak kabur / merangkak satu arah  
- [ ] Tidak “BAD VISION” lalu drift  

### Gagal — segera Stabilize, mendarat
| Gejala | Penyebab umum | Perbaikan |
|---|---|---|
| Kabur depan/samping | Vision belum sehat / scale / yaw | Ulangi walk test; cek YAW=6 |
| Merangkak pelan | Drift EKF / vision intermiten | Rate pose, USB, cahaya, tekstur |
| BAD VISION + jalan sendiri | EKF reject vision → dead-reckon | Stabilize; jangan pakai Loiter sesi itu |
| Osilasi kiri-kanan | Tune Loiter / speed terlalu agresif | Turunkan `LOIT_SPEED` / cek `flight_tuning.param` |

**Ulangi Loiter sampai berulang-ulang bagus sebelum AUTO/GUIDED misi.**

---

## 6. Tes AUTO (FC terbang sendiri, tanpa kamera)

AUTO = misi di-upload ke FC; **tidak** pakai ArUco/gate. Hanya waypoint geometris.

### 6a. Survei WP (lapangan)
```bash
# bring-up + set_origin_home di takeoff dulu
rosrun mission_control record_waypoint.py _name:=wp1
# ulangi: gate_double, wp2, ... 
# Betulkan z di config/waypoints.yaml ke ketinggian terbang (survei darat → z≈0)
```

### 6b. Dry-run (AMAN, disarmed)
```bash
# preview offline
rosrun mission_control mission_auto.py _mode:=preview

# push + readback (mavros hidup, disarmed)
roslaunch mission_control phaseD_auto.launch mode:=push
```
Cek log + Mission Planner Plan → Read.

### 6c. Terbang AUTO
1. Takeoff **Stabilize/AltHold**, hover stabil.  
2. **Jangan** switch AUTO dari darat.  
3. Switch **AUTO**.  
4. Jari Stabilize siap.  
5. Selesai / abort → Stabilize → land.

### Miss AUTO
| Gejala | Tindakan |
|---|---|
| Jalur meleset | WP belum survei / origin beda sesi; ulangi origin+survei |
| Kabur di AUTO | Vision/scale; hentikan Stabilize; jangan pakai map-click palsu |
| Push gagal | mavros disconnect; cek `/mavros/state` |

---

## 7. Misi GUIDED gate-only (tanpa ArUco)

`mission_d` **GUIDED** + detektor gate. **Tanpa scan/drop ArUco.**

### Konsep
- WP gate = hover **di depan plane** gate (~1,5–2,5 m), bukan di tengah bukaan.  
- Center (kamera) → commit → fly-through `traverse_through_m` **tanpa** track vision lagi.  
- `traverse_through_m` harus **> jarak WP → plane** agar tembus gate.

### 7a. Persiapan software
```bash
# Kamera depan (USB /dev/video0)
roslaunch mission_control camera_front.launch

# Cek deteksi (arahkan ke gate)
roslaunch mission_control gate_detect.launch
rostopic echo /gate_node/gate
# detected: True, off_x/off_y wajar
```

Misi: `config/mission_gate_only.yaml`
```yaml
legs:
  - {wp: gate_double, action: traverse}
  - {wp: wp3,         action: land}
```

### 7b. Urutan terbang
1. Bring-up T265 + mavros + vision_to_mavros + **set_origin_home**.  
2. Kamera depan + (opsional) gate_detect dulu untuk cek.  
3. **Takeoff Stabilize** → hover stabil.  
4. Switch **GUIDED** (manual di RC).  
5. Jalankan misi **tanpa takeoff**:

```bash
roslaunch mission_control phaseD_mission.launch \
  mission:=gate_only \
  do_takeoff:=false \
  sim_markers:=false \
  use_gate:=true \
  traverse_through_m:=3.0
```

Sesuaikan `traverse_through_m` di lapangan.  
Ganti WP di yaml jika nama survei beda.

6. Jari Stabilize siap. Abort → Stabilize → land.

### 7c. Miss gate-only
| Gejala | Tindakan |
|---|---|
| Tidak gerak | Belum GUIDED; cek mode di `/mavros/state` |
| `detected: False` | Topik kamera / arahkan gate / HSV |
| Center melenceng arah | Kalibrasi `gate_sign_x/y/z`, `gate_swap_xy` |
| off sering > tol | `gate_tol` (default 0.18) atau posisikan lebih center |
| Through belum lewat gate | Naikkan `traverse_through_m` atau WP lebih dekat plane |
| Drift / BAD VISION | Abort Stabilize; ulangi walk test |

### Kalibrasi arah center (sekali di mount)
Detektor bilang “bukaan di kiri gambar” → drone harus geser ke arah yang benar.  
Kalau malah menjauh: balik `gate_sign_x` (atau y/z) di param node / launch.

---

## 8. Urutan mode (ringkas)

```
Bring-up + origin/home
    ↓
Walk test (motor mati)  ──gagal──► perbaiki vision, jangan terbang
    ↓ lulus
Stabilize (manual)
    ↓
AltHold (tahan tinggi)
    ↓
Loiter (tahan posisi)  ──gagal──► Stabilize, ulangi walk test
    ↓ lulus
AUTO (geometris)  dan/atau  GUIDED gate-only
    ↓
(Nanti) GUIDED penuh + ArUco scan/drop
```

---

## 9. Command cheatsheet

```bash
# Bring-up
roslaunch realsense2_camera rs_t265.launch initial_reset:=true
roslaunch mavros apm.launch fcu_url:=/dev/ttyTHS1:921600
roslaunch vision_to_mavros t265_tf_to_mavros.launch enable_precland:=false
roslaunch mission_control set_origin_home.launch

# Cek
rostopic echo -n1 /mavros/state
rostopic hz /mavros/vision_pose/pose

# Survei WP
rosrun mission_control record_waypoint.py _name:=gate_double

# Gate camera + detect
roslaunch mission_control camera_front.launch
roslaunch mission_control gate_detect.launch
rostopic echo /gate_node/gate

# AUTO dry-run
rosrun mission_control mission_auto.py _mode:=preview
roslaunch mission_control phaseD_auto.launch mode:=push

# GUIDED gate-only (setelah Stabilize→GUIDED hover)
roslaunch mission_control phaseD_mission.launch \
  mission:=gate_only do_takeoff:=false sim_markers:=false \
  use_gate:=true traverse_through_m:=3.0
```

---

## 10. File terkait

| File | Isi |
|---|---|
| `params/t265_nongps_ekf3.param` | EKF3 non-GPS |
| `params/flight_tuning.param` | Kecepatan / landing soft |
| `config/waypoints.yaml` | Koordinat WP (hasil survei) |
| `config/mission_gate_only.yaml` | Misi traverse + land, tanpa ArUco |
| `config/mission_seleksi.yaml` | Misi penuh (scan/drop/traverse) |
| `launch/phaseD_mission.launch` | GUIDED mission_d |
| `launch/phaseD_auto.launch` | Upload misi AUTO |
| `PANDUAN_TERBANG_T265.md` | Detail bring-up T265 |
| `docs/HANDOFF.md` | Keputusan teknis & gotcha |

---

## 11. Keselamatan (wajib)

1. Jari siap **Stabilize** setiap mode otomatis.  
2. Jangan arm/Loiter/AUTO dari darat jika vision belum sehat.  
3. Jangan stream setpoint saat takeoff GUIDED dari kode (sudah ditangani di `mission_d` jika `do_takeoff:=true`; untuk gate-only pakai takeoff manual).  
4. Abort: Stabilize → land manual.  
5. Jangan jalankan build/opencode berat saat drone armed.  
6. Walk test lulus = syarat mutlak sebelum Loiter/AUTO/GUIDED misi.

---

*Dokumen ini merangkum alur operasional workspace `~/catkin_ws` (ROS Noetic, ArduCopter EKF3, T265 non-GPS). Update WP dan `traverse_through_m` sesuai lapangan.*

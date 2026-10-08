# Fase B — Guided Autonomous di SITL (mission_control)

Membuktikan Jetson bisa menerbangkan drone otonom ke 1 koordinat lewat mode GUIDED,
diuji di simulasi (ArduPilot SITL + Gazebo iris). TANPA Pixhawk fisik.

## Isi package
- `scripts/mavros_helper.py` — helper koneksi mavros (mode/arm/takeoff/setpoint/pose).
- `scripts/mission_node.py` — Fase B: arm -> GUIDED -> takeoff -> goto 1 titik -> hover.
- `scripts/record_waypoint.py` — survei: catat posisi T265 sekarang jadi waypoint YAML.
- `scripts/aruco_detector.py` — Fase C: inti detektor ArUco 7x7 (OpenCV murni, ada self-test).
- `scripts/aruco_node.py` — Fase C: node ROS detektor ArUco (kamera bawah).
- `scripts/gate_detector.py` — Fase C: inti detektor gate oranye (OpenCV murni, ada self-test).
- `scripts/gate_node.py` — Fase C: node ROS detektor gate (kamera depan).
- `scripts/image_convert.py` — konversi Image<->numpy bersama (pengganti cv_bridge yg rusak).
- `scripts/mission_d.py` — Fase D: state machine misi (takeoff->goto->scan/align-drop->land).
- `scripts/sim_markers.py` — Fase D: mock kamera bawah virtual utk uji align di SITL.
- `config/waypoints.yaml` — koordinat WP (placeholder SITL; ganti hasil survei).
- `config/mission_seleksi.yaml` / `mission_final.yaml` — urutan leg misi.
- `launch/phaseB_goto.launch` — jalankan node misi Fase B.
- `launch/aruco_detect.launch` / `launch/gate_detect.launch` — detektor Fase C.
- `launch/phaseD_mission.launch` — jalankan misi Fase D.
- `msg/ArucoMarker(s).msg`, `msg/Gate.msg` — hasil deteksi.

## Cara uji Fase B (resep TERBUKTI — 4 proses, tiap satu di terminal sendiri)

Tanpa Gazebo (ringan utk Jetson Nano). Kunci: mavros konek ke **MAVProxy**, bukan
langsung ke SITL (koneksi langsung connect tapi stream mati / "No satellites").

**1 — SITL binary (tanpa mavproxy bawaan):**
```bash
cd ~/ardupilot/ArduCopter && sim_vehicle.py -v ArduCopter -f quad --no-mavproxy --no-rebuild -I0
```

**2 — MAVProxy sbg bridge (WAJIB; --streamrate yang memicu stream & GPS lock):**
```bash
mavproxy.py --master tcp:127.0.0.1:5760 --out tcpin:0.0.0.0:14550 --streamrate 10 --daemon
```

**3 — roscore (TERPISAH; jangan biarkan roslaunch yg nyalakan master):**
```bash
roscore
```

**4 — mavros konek ke MAVProxy (BUKAN ke SITL 5760):**
```bash
cd ~/catkin_ws && source devel/setup.bash
roslaunch mavros apm.launch fcu_url:="tcp://127.0.0.1:14550"
# cek: rostopic echo -n1 /mavros/state  -> connected: True
```

**5 — node misi Fase B:**
```bash
cd ~/catkin_ws && source devel/setup.bash
roslaunch mission_control phaseB_goto.launch target_x:=5 target_y:=0 target_z:=2
```
Hasil terverifikasi di SITL: `GUIDED -> arming -> takeoff selesai (z~2) -> goto -> SAMPAI. Hover.`

## Fase C — Detektor ArUco 7x7 (kamera bawah)

Dipakai untuk scan marker (WP1) & align di atas ember sebelum drop (WP2). OpenCV 4.8
(API aruco baru). Node TIDAK pakai cv_bridge (rusak di Jetson ini krn OpenCV 4.8 vs
cv_bridge 4.2) — konversi Image<->numpy manual.

**Self-test inti (tanpa kamera & tanpa ROS) — bukti detektor jalan:**
```bash
python3 ~/catkin_ws/src/mission_control/scripts/aruco_detector.py --selftest
# -> SEMUA LULUS ✅  (deteksi id 0/7/42/123 dari marker sintetis + uji negatif)
```

**Deteksi 1 gambar / kamera (bench):**
```bash
python3 .../aruco_detector.py --image foto.png --save out.png   # anotasi ke out.png
python3 .../aruco_detector.py --camera 0                        # live saat kamera ada
```

**Node ROS:**
```bash
roslaunch mission_control aruco_detect.launch image_topic:=/camera_down/image_raw
# hasil: rostopic echo /aruco_node/markers      (id, off_x, off_y, side_px, [position])
# debug: /aruco_node/debug_image (bgr8, anotasi)  -> rqt_image_view
```
Output per marker: `id`, pusat piksel `cx,cy`, **offset ternormalisasi** `off_x,off_y`
(-1..1; + = kanan/bawah gambar) untuk align, dan `side_px` (proxy jarak).

**Marker lomba (gambar asli di `~/catkin_ws/aruco_image/`, sudah diverifikasi):**
dictionary **DICT_7X7_50**, allowlist `valid_ids=1,2,3,4`. ⚠️ Nomor WP ≠ ID:

| WP | ID | | WP | ID |
|---|---|---|---|---|
| WP1 | 1 | | WP3 | **4** |
| WP2 | 2 | | WP4 | **3** |

Seleksi: WP1–WP3. Final: sampai WP4. (Fase D harus map WP→ID via tabel ini.)

**Pose metrik (opsional, perlu kalibrasi kamera):** set `marker_length` (meter) +
`camera_info_topic` → `position` (x,y,z meter di frame kamera) terisi. Tanpa itu,
align cukup pakai `off_x/off_y` saja.

> Konvensi: offset dilaporkan di ruang GAMBAR. Pemetaan ke gerak drone (maju/geser)
> dilakukan di Fase D sesuai orientasi mounting kamera bawah.

## Fase C — Detektor Gate oranye (kamera depan)

Gate = struktur oranye solid (spec R233 G146 B17 ~ HSV 18,236,233) berbentuk
terowongan. Drone mengincar **BUKAAN** (celah), BUKAN pusat oranye — pada double
gate pusat massa oranye = panel pembagi (kalau diincar -> nabrak). Bukaan dicari
lewat profil kolom; jumlah celah membedakan double/triple.

**Self-test (tanpa kamera & ROS):**
```bash
python3 .../scripts/gate_detector.py --selftest    # single/double/kosong -> LULUS
python3 .../scripts/gate_detector.py --image gate.png --save out.png
python3 .../scripts/gate_detector.py --camera 0
```

**Node ROS:**
```bash
roslaunch mission_control gate_detect.launch image_topic:=/camera_front/image_raw
# hasil: rostopic echo /gate_node/gate   (detected, off_x, off_y, num_openings, area_frac)
# debug: /gate_node/debug_image (bbox oranye, garis bukaan, titik TARGET)
```
Output: `detected`, **`off_x,off_y`** (offset pusat bukaan terpilih, untuk centering),
`area_frac` (proxy kedekatan), `num_openings`, `opening_w_frac`.

> ⚠️ THRESHOLD HSV & logika bukaan WAJIB dikalibrasi di lapangan pakai footage
> kamera NYATA dari DEPAN. Gambar referensi di `~/catkin_ws/gate/*.png` adalah
> ilustrasi 3D MIRING — bukan tampilan head-on drone — jadi bukan uji yang adil
> (self-test head-on sintetis = acuan logika yang benar). Set `hsv_lo`/`hsv_hi`
> via arg launch saat tuning.

## Fase D — State machine misi (uji di SITL)

Menyatukan Fase B (takeoff/goto) + ArUco (scan / align-drop). Konfig terpisah:
koordinat di `waypoints.yaml`, urutan/aksi di `mission_<babak>.yaml` (rujuk nama WP).
Aksi: `scan` (konfirmasi ArUco), `drop` (align lalu servo), `traverse` (gate—DIPARKIR,
skrg sekadar lewat koordinat), `land`. Ingat mapping WP3=id4, WP4=id3.

**Uji di SITL (tanpa kamera):** pakai resep SITL Fase B (SITL+MAVProxy+roscore+mavros),
lalu:
```bash
roslaunch mission_control phaseD_mission.launch mission:=seleksi   # atau final
```
`sim_markers.py` (mock kamera bawah virtual) menghitung offset marker dari posisi
drone live -> loop align TERTUTUP & teruji. Terverifikasi end-to-end:
```
takeoff -> wp1 scan id=1 OK -> traverse -> wp2 ALIGN id=2 -> DROP (DO_SET_SERVO ACCEPTED)
-> traverse -> wp3 LAND -> MISI SELESAI       (final: lanjut scan wp3 id=4 -> wp4 land)
```

**Servo drop:** ch9, PWM 1013=buka(drop)/2015=tutup (param `servo_*`). Saat kamera/servo
NYATA terpasang: matikan mock (`sim_markers:=false`), jalankan `aruco_detect.launch`,
dan KALIBRASI pemetaan offset->gerak (param `align_sign_x/y`, `align_swap_xy`) seperti
walk test. Koordinat WP ganti dgn hasil `record_waypoint.py` di lapangan.

## Self-check matematika (tanpa ROS)
```bash
python3 ~/catkin_ws/src/mission_control/scripts/mavros_helper.py
# -> mavros_helper demo OK
```

## Waypoint recorder (untuk lapangan nyata, nanti)
Saat T265+mavros aktif & EKF origin sudah di-set, bawa drone ke titik lalu:
```bash
rosrun mission_control record_waypoint.py _name:=wp1
```
Hasil di `config/waypoints.yaml` (koordinat relatif titik takeoff).

## Catatan
- Frame setpoint = ENU lokal (X=Timur, Y=Utara, Z=Atas), sama dengan frame T265.
- Di SITL posisi dari EKF SITL; saat real nanti dari T265.

## GOTCHA TAKEOFF (sudah diperbaiki — JANGAN diulang)
ArduCopter GUIDED ≠ PX4 OFFBOARD:
- **JANGAN stream `setpoint_position/local` selama fase takeoff.** GUIDED punya
  sub-mode (TakeOff / Position). `NAV_TAKEOFF` masuk sub-mode TakeOff; bila langsung
  diikuti setpoint posisi, GUIDED pindah ke sub-mode Position saat masih di tanah
  -> tidak naik -> auto-disarm. (Terbukti: NAV_TAKEOFF ACCEPTED tapi z tetap 0 lalu
  "Disarming motors". Tanpa stream -> naik ke 2 m mulus.)
- ArduCopter GUIDED **tidak** butuh stream setpoint kontinu utk tetap di mode
  (itu syarat PX4). Stream setpoint hanya saat fase goto/hover.
- `mission_node.py` sudah menerapkan ini: takeoff dulu (tanpa setpoint) -> baru goto.

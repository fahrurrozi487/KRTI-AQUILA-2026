# KRTI Autonomous VTOL Companion Computer (NVIDIA Jetson Nano)

Repositori sistem kendali otonom dan komputer visi pendamping (*companion computer*) untuk wahana VTOL pada kompetisi **Kontes Robot Terbang Indonesia (KRTI)**.

Sistem berjalan pada **NVIDIA Jetson Nano B01 (Ubuntu 20.04 / ROS Noetic)** yang terhubung secara serial dengan flight controller **Pixhawk 6C (ArduCopter 4.6.3, EKF3)**.

---

## 1. Arsitektur Perangkat Keras & Sensor

| Perangkat | Antarmuka / Jalur | Peran |
|---|---|---|
| **Jetson Nano B01** (4GB) | Companion Computer | Menjalankan ROS Noetic, state machine misi, dan pengolahan citra |
| **Pixhawk 6C** | `TELEM2` ↔ `/dev/ttyTHS1` (921600 baud) | Flight controller, mode GUIDED & fusi EKF3 |
| **GPS + Kompas** | I2C / UART FC | Sumber estimasi posisi utama (`EKF3 SRC1`) |
| **Matek / MicoAir MTF-01** | `SERIAL5` Pixhawk (MAVLink native) | Optical flow + Lidar jarak pendek (`EKF3 SRC2`) untuk stabilitas rendah |
| **Kamera Depan** (Logitech C310) | USB (`/dev/video0`) | Deteksi visual gerbang oranye (Single, Double, Triple Gate) |
| **Kamera Bawah** (Logitech C920) | USB (`/dev/video1`) | Deteksi marker ArUco 7x7, target drop kotak merah, dan garis landasan |
| **Mekanisme Dropper** | Output Servo Channel 8 / 9 | Pelepasan payload via MAVLink `DO_SET_SERVO` (Open: 1013, Close: 2015) |

> **Catatan Intel RealSense T265 (VIO):**
> Pada fase awal, navigasi non-GPS menggunakan kamera VIO Intel RealSense T265. Untuk kompetisi outdoor lapangan rumput, navigasi dialihkan penuh ke **GPS + Optical Flow (MTF-01)** demi mencegah drift visual odometry. Dokumentasi dan implementasi T265 tetap dipertahankan di `docs/ARSITEKTUR_LAMA_T265.md` dan package `vision_to_mavros` sebagai cadangan.

---

## 2. Struktur Perangkat Lunak (ROS Noetic)

Sistem mengadopsi prinsip pemisahan total (*decoupled architecture*):

- **State Machine (`mission_d.py`):** Satu-satunya pengendali urutan penerbangan. Node ini tidak memproses gambar, hanya menerima offset target via topik ROS standar dan mengirim setpoint lokal ENU ke Pixhawk.
- **Detector Nodes:** Node independen yang memproses frame kamera dan menerbitkan pesan koordinat deviasi pixel/meter.

```text
[Kamera Depan] ──> gate_multi_node.py ──(/gate_multi_node/gate)─────┐
                                                                    ├──> mission_d.py ──> /mavros/setpoint_position/local ──> Pixhawk 6C
[Kamera Bawah] ──> wp_marker_node.py  ──(/wp_marker_node/markers)───┤
               ──> line_node.py       ──(/line_node/line)───────────┘
```

**Modul Pengolahan Citra Utama:**

1. `wp_marker_node.py`: Menggabungkan decoder ArUco DICT_7X7_50 (inverted terpal putih-di-hitam untuk WP1 & WP3) dan color detector kotak merah payload (WP2) dalam satu node ringan.
2. `gate_multi_node.py`: Deteksi gerbang ganda/tiga oranye bertingkat dengan segmentasi HSV adaptif dan skew-corrected peak density.
3. `line_node.py`: Algoritma pelacak garis putus-putus lapangan menuju WP4 dengan mekanisme fallback otomatis ke navigasi titik (dead-reckoning) jika garis terputus.

---

## 3. Prasyarat & Lingkungan Kerja

1. **OS:** Ubuntu 20.04 LTS (JetPack 4.6.x / Linux 4.9 Tegra).
2. **ROS:** ROS Noetic (Desktop Full / Base).
3. **Dependensi Python & OpenCV:**

   ```bash
   pip3 install opencv-contrib-python==4.8.0.76 pymavlink pyyaml
   sudo apt-get install ros-noetic-mavros ros-noetic-mavros-extras
   ```

4. **GeographicLib Datasets (Wajib untuk MAVROS):**

   ```bash
   sudo /opt/ros/noetic/lib/mavros/install_geographiclib_datasets.sh
   ```

---

## 4. Prosedur Operasional Penerbangan Lapangan

Urutan eksekusi terminal saat sesi terbang:

### Langkah 1: ROS Core & Driver Kamera

```bash
# Terminal 1
roscore

# Terminal 2 (Nyalakan kamera depan dan bawah)
roslaunch mission_control cameras.launch front_device:=/dev/video0 down_device:=/dev/video1
```

### Langkah 2: Bridge MAVROS ke Flight Controller

```bash
# Terminal 3
roslaunch mavros apm.launch fcu_url:="/dev/ttyTHS1:921600"
```

Pastikan koneksi sukses dengan `rostopic echo -n1 /mavros/state` (`connected: True`).

### Langkah 3: Verifikasi GPS & Konversi Titik Koordinat

Tunggu GPS mencapai 3D Fix (`fix_type >= 3`, `satellites_visible >= 8`):

```bash
rostopic echo -n1 /mavros/gpsstatus/gps1/raw
```

Konversi koordinat GPS global ke frame lokal ENU sesuai origin EKF sesi aktif:

```bash
rosrun mission_control latlon_to_waypoints.py
```

### Langkah 4: Eksekusi Misi Otonom

#### Pilihan A: Misi Penuh Final (Gate + Marker + Line Follower)

```bash
roslaunch mission_control phaseD_mission.launch mission:=final \
  use_gate_multi:=true \
  use_wp_marker:=true \
  use_line_follow:=true \
  servo_channel:=8
```

#### Pilihan B: Misi ArUco + Drop Kotak Merah (WP1 s/d WP5)

Misi fleksibel tanpa gate untuk skenario arena ArUco & payload drop:

```bash
roslaunch mission_control phaseD_aruco_redbox.launch \
  do_takeoff:=true \
  use_line_follow:=false \
  wp2_yaw_direction:=left \
  wp2_yaw_deg:=90
```

**Fitur Recovery:** Jika terjadi kendala di udara, misi dapat dilanjutkan dari waypoint tertentu tanpa mengulang dari awal menggunakan argumen `start_wp:=<nama_wp>` (contoh: `start_wp:=wp3`).

---

## 5. Simulasi SITL (Software-In-The-Loop)

Pengujian state machine misi tanpa wahana fisik:

1. Jalankan ArduCopter SITL:

   ```bash
   cd ~/ardupilot/ArduCopter && sim_vehicle.py -v ArduCopter -f quad --no-mavproxy --no-rebuild -I0
   ```

2. Jalankan MAVProxy Bridge:

   ```bash
   mavproxy.py --master tcp:127.0.0.1:5760 --out tcpin:0.0.0.0:14550 --streamrate 10 --daemon
   ```

3. Hubungkan MAVROS:

   ```bash
   roslaunch mavros apm.launch fcu_url:="tcp://127.0.0.1:14550"
   ```

4. Jalankan script runner SITL:

   ```bash
   ~/startsitl.sh
   ```

---

## 6. Indeks Dokumentasi Teknis (`docs/`)

Setiap subsistem memiliki panduan mendalam di folder `docs/`:

- [`PANDUAN_TERBANG_FINAL.md`](docs/PANDUAN_TERBANG_FINAL.md): Prosedur standar pre-flight hingga landing misi final.
- [`PANDUAN_ARUCO_REDBOX.md`](docs/PANDUAN_ARUCO_REDBOX.md): Rincian parameter, logika delay/hold, dan recovery misi `phaseD_aruco_redbox`.
- [`ARSITEKTUR_SISTEM.md`](docs/ARSITEKTUR_SISTEM.md): Pembahasan rinci pipeline visual servoing, PID control alignment, dan koordinat frame.
- [`ARSITEKTUR_LAMA_T265.md`](docs/ARSITEKTUR_LAMA_T265.md): Catatan evaluasi teknis, masalah covariance, dan alasan transisi dari T265 VIO ke GPS/Flow.
- [`GATE_DETECTOR_DOUBLE_TRIPLE.md`](docs/GATE_DETECTOR_DOUBLE_TRIPLE.md): Analisis deteksi gerbang oranye U-shape bertingkat.
- [`WP_MARKER_DECODER.md`](docs/WP_MARKER_DECODER.md): Pembahasan tuning parameter ArUco custom pada material terpal lapangan.

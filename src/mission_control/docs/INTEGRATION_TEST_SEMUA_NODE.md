# Integration Test: Semua Node Bersamaan (SITL)

Uji integrasi komunikasi antar-node: `gate_multi_node.py` (Task 1/2),
`wp_marker_node.py` (Task 3, termasuk `tail_side` Task 4), `mission_d.py`,
`mavros_helper.py`, dan `mavros` — dijalankan **bersamaan** sebagai satu
sistem, bukan diuji terpisah seperti sesi-sesi sebelumnya.

Lingkungan: SITL murni (`sim_vehicle.py`, tanpa Gazebo/`iq_sim`) di `~/catkin_ws`
pada tanggal 2026-07-31. Semua drone di test ini VIRTUAL.

## 1. Peta topic / komunikasi antar-node

### Publisher

| Node | Topic | Type | Field kunci |
|---|---|---|---|
| `gate_multi_node.py` | `/gate_multi_node/gate` | `mission_control/Gate` | `detected, off_x, off_y, area_frac, num_openings, bbox` |
| `gate_node.py` (lama) | `/gate_node/gate` | `mission_control/Gate` | sama, single-gate |
| `wp_marker_node.py` | `/wp_marker_node/result` | `mission_control/WpMarkerResult` | `wp_id, tail_side, low_conf_tail, reason` (sumber Task 4) |
| `wp_marker_node.py` | `/wp_marker_node/markers` | `mission_control/ArucoMarkers` | kompatibel `aruco_node`, `markers[].{id,off_x,off_y}` |
| `wp_marker_node.py` | `/wp_marker_node/wp2_present` | `std_msgs/Bool` | presence WP2 |
| `sim_markers.py` (mock) | `/aruco_node/markers` (default) | `ArucoMarkers` | mock, posisi dihitung dari pose live |
| `sim_markers.py` (mock) | `/wp_marker_node/result` (default!) | `WpMarkerResult` | mock tail_side — **topic SAMA** dgn wp_marker_node asli |

### Subscriber (`mission_d.py`)

| Param | Default | Diarahkan ke |
|---|---|---|
| `~gate_topic` | `/gate_node/gate` | override `/gate_multi_node/gate` via `use_gate_multi:=true` (baru) |
| `~markers_topic` | `/aruco_node/markers` | override `/wp_marker_node/markers` via `use_wp_marker:=true` (sudah ada) |
| `~wp_marker_result_topic` | `/wp_marker_node/result` | cocok default dgn `wp_marker_node.py` (tak perlu override) |

`mavros_helper.py` hanya subscribe `/mavros/state` & `/mavros/local_position/pose`
— tidak menyentuh topic gate/WP sama sekali.

## 2. Mismatch ditemukan & perbaikan

### 🔴 MISMATCH: `gate_multi_node.py` tidak pernah tersambung ke `mission_d.py`

**Sebelum fix**: `gate_multi_node.py` publish ke `/gate_multi_node/gate`, tapi
`mission_d.py` default subscribe ke `/gate_node/gate` (node lama), dan
`phaseD_mission.launch` **tidak punya jalur apa pun** untuk menyalakan
`gate_multi_node.py` atau mengarahkan `mission_d` ke topic-nya. Kalau
`gate_multi_node.py` dinyalakan manual, dia publish ke topic yang **tidak
ada yang dengarkan** — persis pola "node hidup sendiri-sendiri, sistem
tidak jalan bareng".

**Perbaikan** (`phaseD_mission.launch`, backup di
`~/catkin_ws/backup_manual/phaseD_mission.launch.bak_20260731_222832`):
menambah arg `use_gate_multi` (default `false`), meniru pola `use_wp_marker`
yang sudah ada:
- `use_gate_multi:=true` → include `gate_multi_detect.launch` (bukan
  `gate_detect.launch` lama) + override param `~gate_topic` di node
  `mission_d` jadi `/gate_multi_node/gate`.
- Default `gate_topic` di `mission_d.py` **TIDAK diubah** — misi lama yang
  masih pakai `gate_node.py` (single gate) tidak terpengaruh.
- Konvensi pemakaian (sama seperti `use_wp_marker`/`use_aruco`):
  `use_gate_multi:=true use_gate:=false` — kedua arg independen, harus
  di-set eksplisit berpasangan supaya tak dua node gate jalan bersamaan.

**Verifikasi** (bukan asumsi — dicek langsung):
```
$ rosnode info mission_d
Subscriptions:
 * /gate_multi_node/gate [mission_control/Gate]
 ...
$ rostopic info /gate_multi_node/gate
Publishers: /gate_multi_node
Subscribers: /mission_d
$ rosparam get /mission_d/gate_topic
/gate_multi_node/gate
```

### ⚠️ Temuan tambahan (dicatat, tidak diubah — di luar mandat sesi ini)

1. **Tabrakan publisher di `/wp_marker_node/result`**: `sim_markers.py`
   (default `sim_markers:=true`) publish `WpMarkerResult` mock ke topic
   default yang SAMA PERSIS dengan `wp_marker_node.py` asli. Kalau
   keduanya jalan bersamaan, `mission_d` menerima campuran data mock &
   asli secara acak. **Solusi dipakai di test ini**: jalankan dengan
   `sim_markers:=false use_wp_marker:=true` supaya `wp_marker_node.py`
   jadi satu-satunya sumber — bukan perubahan kode, cukup pilihan
   argumen launch yang benar.
2. **`waypoints.yaml` — `gate_double.z: 0.0`** (WP lain 1.0–2.0m) membuat
   `mission_d` ABORT→LAND (goto timeout 60s) saat leg `traverse gate_double`,
   karena drone diminta terbang ke ketinggian tanah sambil tetap harus
   terbang. Nama backup `waypoints.yaml.backup_20260731_pre_sitl_gate_z_fix`
   menunjukkan ini sudah pernah "ditandai" tapi belum pernah benar-benar
   diperbaiki (isi backup identik dgn file aktif). **`waypoints.yaml` ASLI
   TIDAK diubah** (dipakai misi lapangan sungguhan) — dibuat
   `waypoints_sitl_test.yaml` (override, `gate_double.z: 1.0`) khusus untuk
   test SITL ini. **Rekomendasi**: nilai z definitif untuk `gate_double`
   di `waypoints.yaml` asli perlu dikonfirmasi/diperbaiki terpisah dari
   sesi ini oleh pemilik data survei.
3. Tidak ada mock/sim untuk kamera gate (`sim_gate.py` atau serupa tidak
   ada di repo) — lihat §4.

## 3. Hasil test LANGKAH 1–6

### LANGKAH 1 — Setup SITL
Semua tahap sukses berurutan: SITL ArduCopter (`ArduPilot Ready`) → MAVProxy
bridge (`Detected vehicle 1:1`) → roscore → mavros (`connected: True`) →
`set_origin_home.launch` (`origin dikirim -7.050111, 110.391339, 700.0`,
`home OK`).

### LANGKAH 2 — Nyalakan gate_multi_node + wp_marker_node bersamaan
Tidak ada sumber gambar asli di SITL murni ini (§4) — dibuat publisher
gambar noise sekali-pakai @10Hz ke `/camera_front/image_raw` &
`/camera_down/image_raw` supaya kedua node punya sesuatu untuk diproses
(uji jalur komunikasi, bukan akurasi deteksi).
- Kedua node start tanpa error, tanpa port/resource conflict.
- CPU: `gate_multi_node` ~80%, `wp_marker_node` ~90% (satu core, olah
  noise image @10Hz/5Hz — wajar untuk OpenCV di Jetson Nano).
- `rostopic hz` BERSAMAAN: `/gate_multi_node/gate` ~10.0 Hz (ikut rate
  gambar), `/wp_marker_node/result` ~4.0 Hz (sesuai `throttle_hz=5.0` +
  jitter beban CPU) — keduanya stabil, tidak ada yang macet gara-gara
  yang lain jalan.

### LANGKAH 3 — mission_d baca dari kedua node sekaligus
Dikonfirmasi via `rosnode info`/`rostopic info`/`rosparam get` (§2) bahwa
`mission_d` benar-benar subscribe `/gate_multi_node/gate`,
`/wp_marker_node/markers`, dan `/wp_marker_node/result` — bukan topic node
lama. Perilaku teramati saat mission jalan (leg scan/drop/traverse):
- Scan/drop: marker tak pernah ketemu (wajar, gambar noise) → timeout
  graceful → lanjut, TIDAK macet.
- Heading align (Task 4): `WpMarkerResult wp_id=-1 != expected` → skip
  dengan log jelas, sesuai desain guard di `_align_heading_to_tail()`.
- Traverse: gate tak pernah terdeteksi → `gate_timeout` (20s) → fallback
  "through geometris WP" → TIDAK macet.
- Tidak ada satu pun node yang diam/timeout menunggu node lain yang
  seharusnya sudah publish.

### LANGKAH 4 — Stress test (crash & recovery node)
Satu run misi penuh, node dimatikan (`kill -9`) di tengah leg aktif:

| Aksi | Waktu | Hasil |
|---|---|---|
| Kill `wp_marker_node` saat leg1 SCAN | 22:54:56 | Mission TIDAK macet (scan timeout jalan spt biasa) |
| Auto-respawn `wp_marker_node` | 22:55:30 (+~5s, sesuai `respawn_delay="3"` di `wp_marker_detect.launch`) | HIDUP lagi OTOMATIS, tanpa restart `mission_d` |
| Kill `gate_multi_node` saat leg2 TRAVERSE (fase center) | 22:55:24 | Mission TIDAK macet (gate_timeout jalan spt biasa) |
| Restart manual `gate_multi_node` (sebelum leg5) | 22:55:30–33 | HIDUP lagi setelah restart manual (TIDAK auto — `gate_multi_detect.launch` tak punya `respawn="true"`) |
| `mission_d` reconnect ke node baru | — | OTOMATIS (standar ROS pub/sub by topic name), dikonfirmasi via `rostopic info` menunjukkan `/gate_multi_node` sbg publisher aktif lagi setelah restart, TANPA restart `mission_d` |

**Kesimpulan LANGKAH 4**: `mission_d` resilient terhadap crash node
detector — desain timeout (`scan_timeout`, `align_timeout`, `gate_timeout`,
dipicu independen dari `_healthy()` yang hanya cek armed+pose) sudah cukup
untuk mencegah macet selamanya, baik saat node cuma "tidak mendeteksi"
maupun saat node benar-benar mati. **Asimetri respawn** patut diperhatikan:
`wp_marker_detect.launch` punya `respawn="true"`, `gate_multi_detect.launch`
tidak — rekomendasi: samakan kalau auto-recovery gate juga diinginkan di
lapangan.

### LANGKAH 5 — Misi penuh end-to-end
Dua kali full run (`waypoints_sitl_test.yaml`, `use_gate_multi:=true
use_wp_marker:=true sim_markers:=false enable_heading_align:=true`) —
keduanya selesai `=== MISI SELESAI ===` tanpa macet komunikasi:
takeoff → WP1 scan → gate_double traverse → WP2 drop → yaw 90° → gate_triple
traverse → WP3 land, ~91 detik total (speedup 1x). Run kedua sekaligus jadi
LANGKAH 4 (stress test disisipkan di run ini) dan tetap selesai penuh.

### LANGKAH 6 — Cleanup
Semua proses (SITL, MAVProxy, roscore, mavros, semua node, dummy image
publisher) dimatikan graceful (`SIGINT`, urutan terbalik). Verifikasi akhir
`pgrep` menyeluruh: **bersih total**, tidak ada sisa proses.

## 4. Keterbatasan

- **Node detector di-mock secara input**: tidak ada kamera fisik atau
  Gazebo di SITL ini — gambar yang diproses `gate_multi_node.py` dan
  `wp_marker_node.py` adalah **noise acak**, bukan marker/gate sungguhan.
  Test ini membuktikan **jalur komunikasi** (topic, field, timing, crash
  recovery) bekerja — **BUKAN** akurasi deteksi, yang sudah dievaluasi
  terpisah di sesi Task 1/3.
- Publisher gambar dummy (`publish_dummy_images.py`) & script orkestrasi
  stress test bersifat sekali-pakai untuk sesi ini (disimpan di scratchpad,
  bukan bagian permanen repo).
- `waypoints_sitl_test.yaml` (override `gate_double.z`) HANYA untuk test
  SITL ini — `waypoints.yaml` asli (misi lapangan) tidak disentuh dan
  masih menyimpan nilai `z: 0.0` yang perlu ditinjau ulang terpisah.
- Belum ada uji dengan kamera fisik (D435i) mengirim data real-time —
  langkah lanjutan wajar sebelum uji lapangan sungguhan.

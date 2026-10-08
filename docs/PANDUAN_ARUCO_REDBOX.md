# Panduan: GUIDED ArUco + Red-box Drop (`phaseD_aruco_redbox.launch`)

Misi GUIDED WP1→WP5: scan ArUco WP1 → drop WP2 (align red-box) → yaw WP2→WP3
(arah/besar bisa diganti) → scan ArUco WP3 → WP4 (line_follow opsional) →
`gate_single_final` (lewat saja, TANPA deteksi gate) → land WP5.

---

## 1. Ringkasan rute

| Leg | WP | Aksi | Catatan |
|---|---|---|---|
| 1 | `wp1` | scan (id=1) | = titik takeoff. Marker gak kedeteksi → timeout graceful, tetap lanjut |
| 2 | `wp2` | drop (id=2) | Align pakai **red-box** (`wp_marker_node.py`), bukan ArUco |
| 3 | `wp2` | yaw | Default 90° kiri — **bisa diganti** lewat parameter (§3) |
| 4 | `wp3` | scan (id=3) | Sama seperti leg 1 |
| 5 | `wp4` | line_follow | **Opsional** (§3) — kalau `use_line_follow:=false`, otomatis jadi goto polos ke WP4, tanpa garis fisik. Hold 3s sesudah sampai |
| 6 | `gate_single_final` | goto | **TANPA** deteksi gate sama sekali, cuma lewat koordinatnya. Hold 3s sesudah sampai |
| 7 | `wp5` | land | Hold 3s dulu (hover-konfirmasi) **sebelum** mode LAND aktif |

Servo drop: channel **8**, buka=`1013`, tutup=`2015` (default, tidak perlu
diatur manual).

---

## 2. Prasyarat (tiap sesi terbang)

### 2.1 Bring-up navigasi
```bash
roscore &

roslaunch mavros apm.launch fcu_url:=/dev/ttyACM0:921600
# tunggu sampai fix layak:
rostopic echo -n1 /mavros/gpsstatus/gps1/raw
# lanjut kalau fix_type>=3, satellites_visible>=8, eph<150
```

### 2.2 Survei WP (kalau origin sesi ini beda dari sesi survei terakhir)
```bash
rosrun mission_control record_waypoint.py _name:=wp1
rosrun mission_control record_waypoint.py _name:=wp2   # atas red-box
rosrun mission_control record_waypoint.py _name:=wp3
rosrun mission_control record_waypoint.py _name:=wp4              # kalau line_follow dipakai
rosrun mission_control record_waypoint.py _name:=gate_single_final
rosrun mission_control record_waypoint.py _name:=wp5              # titik land
```
Cek `z` di `config/waypoints.yaml` — harus AGL meter (bukan z≈0 dari
gendong). `qw`/`qz` di file ini **tidak dipakai** oleh `mission_d.py` sama
sekali (byproduct rekam pose, boleh diabaikan) — yaw selama terbang murni
ditentukan mekanisme di §4, bukan angka di `waypoints.yaml`.

### 2.3 Kamera
```bash
roslaunch mission_control camera_down.launch
```
Cuma kamera **bawah** yang dipakai misi ini (ArUco WP1/WP3, red-box WP2,
opsional garis WP3→WP4) — kamera depan/gate **tidak** dinyalakan sama
sekali (`use_gate`/`use_gate_multi` selalu `false` di misi ini).

---

## 3. Jalankan misi

Pilot takeoff manual Stabilize → hover di WP1 → switch **GUIDED**, baru
launch (`do_takeoff:=false`, direkomendasikan):
```bash
roslaunch mission_control phaseD_aruco_redbox.launch do_takeoff:=false
```

### Parameter yang bisa diatur

| Argumen | Default | Kegunaan |
|---|---|---|
| `do_takeoff` | `true` | `false` = pilot sudah takeoff manual & switch GUIDED duluan (disarankan). `true` = node sendiri yang arm+takeoff |
| `takeoff_alt` | `1.0` | Ketinggian target auto-takeoff (m), cuma dipakai kalau `do_takeoff:=true` |
| `use_line_follow` | `false` | `true` = nyalakan `line_node.py`, WP3→WP4 ikuti garis fisik. `false` = WP4 jadi goto polos (tanpa garis) |
| `wp2_yaw_direction` | `left` | Arah putar WP2→WP3: `left` atau `right` — ganti kalau arena pakai bentuk 2 (belok kanan) |
| `wp2_yaw_deg` | `90` | Besar putaran (derajat) WP2→WP3 |
| `start_wp` | *(kosong)* | Retry/recovery — lanjut misi dari WP tertentu, lih. §4.5 |

Contoh — arena bentuk 2 (belok kanan 90°), tanpa garis fisik:
```bash
roslaunch mission_control phaseD_aruco_redbox.launch \
  do_takeoff:=false \
  use_line_follow:=false \
  wp2_yaw_direction:=right
```

Yang **tidak** bisa diubah dari command (dikunci karena memang definisi
misi ini): `mission:=aruco_redbox`, `use_wp_marker:=true use_aruco:=false`
(decoder WP1/WP3/WP2 satu node), `use_gate:=false use_gate_multi:=false`
(gak ada gate di misi ini), `servo_channel:=8`. Kalau butuh ubah salah satu
itu, panggil `phaseD_mission.launch` langsung (lih. §5).

---

## 4. Delay di tiap WP

WP1/WP2/WP3 otomatis dapat jeda dari proses scan/drop-nya sendiri (`scan_timeout`
~5s, `align_timeout` ~20s kalau marker gak kedeteksi). WP4/`gate_single_final`/WP5
tidak punya proses vision yang makan waktu, jadi ditambah **hold 3 detik**
(hardcode di `config/mission_aruco_redbox.yaml`, field `hold_sec` — belum
jadi parameter launch, edit file langsung kalau mau ubah durasinya).

### 4.5 Retry dari WP tertentu (drone crash/keluar kendali di tengah misi)

Kalau misi berhenti di tengah jalan (abort, pilot ambil alih, dsb.), tidak
perlu ulang dari WP1. `start_wp` motong daftar leg ke leg **PERTAMA** yang
WP-nya cocok — leg sebelumnya di-skip total, bukan cuma dilewati cepat.

```bash
# lanjut dari WP3 (skip wp1 scan + wp2 drop + wp2 yaw)
roslaunch mission_control phaseD_aruco_redbox.launch start_wp:=wp3

# drone sudah di darat (disarmed) saat mau retry -> perlu takeoff ulang juga
roslaunch mission_control phaseD_aruco_redbox.launch start_wp:=wp3 do_takeoff:=true
```

WP valid: `wp1`, `wp2`, `wp3`, `wp4`, `gate_single_final`, `wp5`.
⚠️ `wp2` muncul **2x** di misi ini (drop lalu yaw) — `start_wp:=wp2` selalu
lanjut dari leg **drop** (yang pertama cocok), bukan leg yaw. Nama salah
ketik/tidak ada → aman, otomatis fallback ke misi penuh dari WP1 (dengan
log error, bukan diam-diam salah).

---

## 5. Troubleshooting

| Gejala | Penyebab / perbaikan |
|---|---|
| Servo gak buka di WP2 | Cek `off<0.08 -> DROP` muncul di log; kalau `align timeout -> drop apa adanya`, marker/red-box gak kedeteksi tapi drop TETAP jalan (buta) — cek warna/exposure kamera |
| Yaw arah salah | Set `wp2_yaw_direction:=right` (atau `left`) sesuai bentuk arena sebenarnya |
| WP4 gak ikutin garis padahal ada garis fisik | Pastikan `use_line_follow:=true` di command, dan `line_node.py` benar-benar jalan (`rosnode list \| grep line_node`) |
| Drone "nyasar" ke koordinat gate_double/gate_triple | Salah file — misi ini gak pernah menyentuh gate_double/gate_triple sama sekali, cek `mission:=aruco_redbox` bukan `mission:=seleksi`/`final` |
| Goto timeout / abort | Cek `waypoints.yaml` origin sesi ini sama dengan sesi survei terakhir (§2.2) |

---

## 6. File referensi

| File | Isi |
|---|---|
| `config/mission_aruco_redbox.yaml` | 7 leg rute + `hold_sec`, default `yaw_deg`/`direction` |
| `config/waypoints.yaml` | Koordinat ENU lokal wp1-wp5 + `gate_single_final` |
| `scripts/mission_d.py` | State machine misi (semua mission GUIDED, bukan cuma ini) |
| `scripts/wp_marker_node.py` | Decoder ArUco WP1/WP3 + presence red-box WP2, SATU node |
| `launch/phaseD_aruco_redbox.launch` | Entry point misi ini (wrapper `phaseD_mission.launch`) |
| `launch/phaseD_mission.launch` | Generic launcher GUIDED, dipakai semua mission (seleksi/final/aruco_drop/aruco_redbox) |

---

*Misi ini TANPA gate & (opsional) TANPA line-follower — kalau butuh gate
Double/Triple/Single atau line-follower WAJIB (bukan opsional), pakai
`mission:=seleksi`/`mission:=final` lewat `phaseD_mission.launch` langsung,
lih. `PANDUAN_TERBANG_FINAL.md`.*

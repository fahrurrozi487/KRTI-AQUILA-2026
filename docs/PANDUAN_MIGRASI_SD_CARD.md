# Panduan Migrasi Ekosistem ke SD Card Baru (Jetson Nano)

Memindahkan **stack yang sudah jalan sekarang** (bukan rebuild dari tutorial LuckyBird 2019).  
Tujuan: SD baru = ROS Noetic + T265 non-GPS + mission_control (GUIDED/AUTO/gate/ArUco) siap pakai.

**Prinsip:** copy sumber + config + docs; **rebuild** di SD baru; verifikasi walk test & Loiter.

---

## 0. Apa yang dipindah vs tidak

### Wajib dibawa
| Item | Path / isi |
|---|---|
| Workspace misi | `~/catkin_ws/src/` (minimal `mission_control`, `vision_to_mavros`, + deps yang kamu pakai) |
| Config & param | `src/mission_control/config/`, `~/catkin_ws/params/` |
| Docs | `~/catkin_ws/docs/`, `PANDUAN_TERBANG_T265.md`, `AGENTS.md` |
| Dataset (opsional) | `aruco_image/`, `gate/` |
| Catatan origin | lat/lon `set_origin_home` / `phaseD_auto` |
| Daftar paket ROS | lihat §3 |
| Param FC | file `.param` + (ideal) dump full dari MP setelah tune |

### Jangan andalkan copy mentah
| Item | Alasan |
|---|---|
| `catkin_ws/build/`, `devel/`, `logs/` | Rebuild di mesin baru |
| `~/.ros/log` | Sampah sesi |
| Image SD bit-identical (clone disk) | Bisa, tapi sering gagal boot/UUID; lebih aman install bersih + restore data |
| Param EKF2 dari blog LuckyBird | Kamu **EKF3 only** |

### Opsional
| Item | Path |
|---|---|
| ArduPilot SITL | `~/ardupilot` |
| librealsense source | `~/librealsense` |
| Flight logs | `~/flight_logs` |

### Full session agent (OpenCode + Claude Code)
| Item | Path | Perkiraan ukuran di mesin ini |
|---|---|---|
| OpenCode config | `~/.config/opencode/` | ~60 MB (termasuk node_modules plugin) |
| OpenCode data + **sesi chat** | `~/.local/share/opencode/` (`auth.json`, `opencode.db*`) | ~25 MB |
| OpenCode cache | `~/.cache/opencode/` | opsional |
| Claude Code home | `~/.claude/` + `~/.claude.json` | ~20 MB |
| Claude di proyek | `~/catkin_ws/.claude/` | ikut catkin |

**Tanpa ini:** kode/misi tetap ada, history chat & permission agent hilang.  
**Dengan ini:** full session dipindah.

Lihat **§12 Full session OpenCode & Claude Code** di bawah.

---

## 1. Backup di SD lama (sebelum cabut)

Jalankan di Jetson yang sudah jalan:

```bash
# USB/SSD external, contoh mount /media/jetson/BACKUP
export B=/media/jetson/BACKUP/jetson_migrate_$(date +%Y%m%d)
mkdir -p "$B"

# 1) catkin sumber + docs + params (tanpa build)
rsync -aH --info=progress2 \
  --exclude='build/' --exclude='devel/' --exclude='logs/' \
  --exclude='.catkin_tools/' \
  ~/catkin_ws/ "$B/catkin_ws/"

# 2) daftar paket terpasang (untuk restore referensi)
dpkg --get-selections > "$B/dpkg_selections.txt"
apt-mark showmanual > "$B/apt_manual.txt" 2>/dev/null || true

# 3) udev / serial notes (jika ada custom)
sudo cp -a /etc/udev/rules.d/ "$B/udev_rules.d/" 2>/dev/null || true
ls -la /dev/ttyTHS* > "$B/serial_devices.txt" 2>/dev/null || true
groups > "$B/groups.txt"
uname -a > "$B/uname.txt"
cat /etc/nv_tegra_release > "$B/jetpack.txt" 2>/dev/null || true

# 4) opsional: librealsense + ardupilot
# rsync -aH ~/librealsense/ "$B/librealsense/"
# rsync -aH ~/ardupilot/ "$B/ardupilot/" --exclude='.git/objects'  # atau full clone di SD baru

# 5) tar cadangan ringkas
cd /media/jetson/BACKUP
tar czvf jetson_catkin_$(date +%Y%m%d).tar.gz -C "$B" catkin_ws
```

**Minimum absolut jika USB sempit:**
```bash
tar czvf mission_only.tgz \
  -C ~/catkin_ws \
  src/mission_control \
  params docs AGENTS.md PANDUAN_TERBANG_T265.md \
  aruco_image gate
```

Simpan juga di laptop/cloud: `waypoints.yaml`, semua `mission_*.yaml`, semua `params/*.param`.

---

## 2. Siapkan SD card baru

### Image
- Jetson Nano: image **Ubuntu 20.04 + JetPack** yang kamu biasa pakai (cocok dengan ROS Noetic).
- Flash dengan Balena Etcher / `dd` / NVIDIA SDK Manager (sesuai kebiasaanmu).
- Boot, buat user (ideal: tetap `jetson`), set locale/timezone, enable SSH.

### Update dasar
```bash
sudo apt update && sudo apt upgrade -y
sudo apt install -y git curl build-essential cmake python3-pip python3-rosdep
```

### User & serial FC
```bash
sudo usermod -aG dialout $USER
# logout/login

# nonaktifkan nvgetty di UART (ttyTHS1) — sama seperti setup lama
# (ulangi langkah yang dulu kamu pakai; cek: systemctl status nvgetty)
```

Udev rules custom (jika ada di backup) → salin ke `/etc/udev/rules.d/` lalu:
```bash
sudo udevadm control --reload-rules && sudo udevadm trigger
```

---

## 3. Install ROS Noetic + paket

```bash
# ikuti wiki resmi ROS Noetic Ubuntu 20.04 (ros-base atau desktop)
# https://wiki.ros.org/noetic/Installation/Ubuntu

sudo apt install -y \
  ros-noetic-desktop-full \
  ros-noetic-mavros ros-noetic-mavros-extras \
  ros-noetic-usb-cam \
  python3-catkin-tools \
  geographiclib-tools

# GeographicLib datasets (mavros)
# script umum:
# wget https://raw.githubusercontent.com/mavlink/mavros/master/mavros/scripts/install_geographiclib_datasets.sh
# sudo bash ./install_geographiclib_datasets.sh
```

Paket vision (pilih cara yang sama dengan setup lama):
- **realsense2_camera** (realsense-ros) — lewat source di catkin atau apt jika tersedia
- **vision_to_mavros** — dari repo / backup `src/`
- **mission_control** — dari backup

```bash
echo "source /opt/ros/noetic/setup.bash" >> ~/.bashrc
source ~/.bashrc
```

---

## 4. librealsense (T265) di Jetson

**Jangan** ikuti langkah RPi Part 1 LuckyBird mentah.  
Pakai dokumentasi resmi **Jetson**:

https://github.com/IntelRealSense/librealsense/blob/master/doc/installation_jetson.md

Target:
```bash
rs-enumerate-devices
# T265 terlihat
realsense-viewer   # opsional, jika ada display
```

Versi librealsense ↔ realsense-ros harus cocok (seperti di SD lama, mis. 2.50.x / 2.3.x — cek catatan di `PANDUAN_TERBANG_T265.md`).

---

## 5. Restore catkin workspace

```bash
# salin dari backup
mkdir -p ~/catkin_ws/src
rsync -a /media/.../BACKUP/.../catkin_ws/src/ ~/catkin_ws/src/
# atau extract tar

# pastikan ada:
#   mission_control/
#   vision_to_mavros/
#   realsense-ros/ (atau terpasang lewat apt)
#   mavros/ mavlink/ jika kamu build dari source

cd ~/catkin_ws
rosdep update
rosdep install --from-paths src --ignore-src -r -y

catkin build
# atau: catkin_make  (jika dulu pakai itu; workspace ini: catkin build)

echo "source ~/catkin_ws/devel/setup.bash" >> ~/.bashrc
source ~/catkin_ws/devel/setup.bash
```

Salin juga jika belum termasuk di `src`:
```bash
# dari backup root catkin
cp -a .../params ~/catkin_ws/
cp -a .../docs ~/catkin_ws/
cp -a .../AGENTS.md .../PANDUAN_TERBANG_T265.md ~/catkin_ws/ 2>/dev/null || true
```

---

## 6. Restore konfigurasi misi & param FC

### Di Jetson
- `src/mission_control/config/waypoints.yaml` — hasil survei  
- `mission_seleksi.yaml`, `mission_auto_drop.yaml`, `mission_aruco_drop.yaml`, dll.  
- `params/t265_nongps_ekf3.param`, `flight_tuning.param`, `t265_from_TFLuna_delta.param`  

### Di Pixhawk (Mission Planner)
1. Connect FC.  
2. Full Parameter List → **Load** `t265_nongps_ekf3.param` (atau delta dari dump GPS).  
3. Opsional: `flight_tuning.param`.  
4. **Write** → **Reboot FC**.  

Cek kritis:
```
VISO_TYPE = 2
EK3_SRC1_POSXY/VELXY/VELZ = 6
EK3_SRC1_POSZ = 1
EK3_SRC1_YAW = 6
GPS1_TYPE = 0
COMPASS_USE* = 0
SERIAL2_BAUD = 921, SERIAL2_PROTOCOL = 2
```

---

## 7. Kamera USB (C310 / C920)

| Peran | Device tipikal | Launch |
|---|---|---|
| Gate (depan) | `/dev/video0` C310 | `camera_front.launch` (default 1280×720 MJPEG) |
| ArUco (bawah) | `/dev/video1` C920 | `camera_down.launch` |

```bash
v4l2-ctl --list-devices
roslaunch mission_control cameras.launch
# jika tertukar:
# front_device:=/dev/video1 down_device:=/dev/video0
```

ArUco terpal inverted:
```bash
roslaunch mission_control aruco_detect.launch   # invert:=true default
```

---

## 8. Verifikasi berjenjang (jangan loncat)

### 8.1 Navigasi saja
```bash
roslaunch realsense2_camera rs_t265.launch initial_reset:=true
roslaunch mavros apm.launch fcu_url:=/dev/ttyTHS1:921600
roslaunch vision_to_mavros t265_tf_to_mavros.launch enable_precland:=false
roslaunch mission_control set_origin_home.launch
```
- [ ] VISION_POSITION_ESTIMATE di MP  
- [ ] Walk test kotak (target drift kecil seperti SD lama)  
- [ ] Stabilize → AltHold → **Loiter** stabil  

### 8.2 Kamera + detektor
```bash
roslaunch mission_control cameras.launch
roslaunch mission_control gate_detect.launch
roslaunch mission_control aruco_detect.launch
rostopic hz /camera_front/image_raw
rostopic hz /camera_down/image_raw
rostopic echo /gate_node/gate
rostopic echo /aruco_node/markers
```

### 8.3 Misi dry-run (disarmed)
```bash
# AUTO
roslaunch mission_control phaseD_auto.launch \
  route_file:=$(find mission_control)/config/mission_auto_drop.yaml mode:=push

# GUIDED aruco (hanya node, drone disarmed / di meja)
# roslaunch mission_control phaseD_mission.launch mission:=aruco_drop ...
```

### 8.4 Flight
Ikuti `docs/TUTORIAL_TERBANG.md` lalu `PANDUAN_TERBANG_AUTO.md` / `PANDUAN_MISI_SELEKSI.md`.

---

## 9. Mapping LuckyBird (referensi saja)

| LuckyBird | Di SD baru |
|---|---|
| Part 1 install | librealsense **Jetson** doc, bukan RPi |
| Part 2 ROS bridge + Loiter | Stack kamu: realsense → vision_to_mavros → mavros; **param EKF3** |
| Part 3 autonomous | `mission_d` / `mission_auto` (bukan hanya square script) |
| Part 4 non-ROS | **Jangan** ganti stack misi ke non-ROS |
| Part 5 scale/offset | Pakai jika WP meleset / T265 jauh dari CG |

**Jangan** load param EKF2 dari blog 2019.

---

## 10. Checklist migrasi (cetak)

```
BACKUP SD LAMA
[ ] rsync/tar catkin src + params + docs + config
[ ] waypoints.yaml + mission_*.yaml
[ ] dpkg/apt list
[ ] udev / grup dialout
[ ] param FC (.param)
[ ] OpenCode: ~/.config/opencode + ~/.local/share/opencode (full session)
[ ] Claude Code: ~/.claude + ~/.claude.json (full session)

SD BARU
[ ] Image Jetson + network + SSH
[ ] dialout + UART ttyTHS1
[ ] ROS Noetic + mavros + usb_cam + geographiclib
[ ] librealsense + realsense-ros (versi cocok)
[ ] restore src → rosdep → catkin build
[ ] source devel di bashrc
[ ] load param FC EKF3 + reboot
[ ] Node: npm + opencode-ai + claude-code (global)
[ ] restore OpenCode config/share + Claude home
[ ] walk test OK
[ ] Loiter OK
[ ] cameras + gate + aruco OK
[ ] AUTO push dry-run / GUIDED dry-run
[ ] opencode / claude jalan, history sesi muncul
```

---

## 11. File panduan terkait di repo

| File | Isi |
|---|---|
| `docs/PANDUAN_MIGRASI_SD_CARD.md` | File ini |
| `PANDUAN_TERBANG_T265.md` | Bring-up T265 harian |
| `docs/TUTORIAL_TERBANG.md` | Stabilize → Loiter → AUTO/GUIDED |
| `docs/PANDUAN_TERBANG_AUTO.md` | AUTO + drop dari record WP |
| `docs/PANDUAN_MISI_SELEKSI.md` | GUIDED seleksi |
| `docs/PANDUAN_AUTO_DAN_ARUCO_DROP.md` | AUTO drop vs GUIDED ArUco |
| `docs/HANDOFF.md` | Keputusan teknis |
| `params/*` | Param FC |

---

## 12. Full session OpenCode & Claude Code

Tujuan: di SD baru, **history chat + settings + auth + plugin** ikut, bukan cuma kode.

### 12.1 Apa yang disalin (full session)

**OpenCode** (~90 MB di mesin ini):

| Path | Isi |
|---|---|
| `~/.config/opencode/` | `opencode.json`, `AGENTS.md`, `package.json`, plugin `node_modules` |
| `~/.local/share/opencode/` | **`opencode.db`** (semua sesi chat), `auth.json`, log, snapshot |
| `~/.cache/opencode/` | cache model/plugin — **opsional** (bisa di-download ulang) |

**Claude Code** (~20 MB):

| Path | Isi |
|---|---|
| `~/.claude.json` | State global CLI |
| `~/.claude/` | settings, credentials, history, sessions, projects, plugins, session-env |
| `~/catkin_ws/.claude/` | settings proyek — ikut rsync catkin |

**Binary** (install ulang, jangan andalkan copy path absolut saja):

```bash
# biasanya:
~/.npm-global/bin/opencode   → opencode-ai
~/.npm-global/bin/claude     → @anthropic-ai/claude-code
```

### 12.2 Backup di SD lama (tutup agent dulu)

```bash
# Hentikan opencode/claude yang sedang jalan (hindari DB corrupt)
# pkill -f opencode; pkill -f claude   # hati-hati hanya proses agent

export B=/media/jetson/BACKUP/jetson_migrate_$(date +%Y%m%d)
mkdir -p "$B/agent"

# OpenCode full
rsync -aH --info=progress2 ~/.config/opencode/ "$B/agent/opencode-config/"
rsync -aH --info=progress2 ~/.local/share/opencode/ "$B/agent/opencode-share/"
# opsional cache:
# rsync -aH ~/.cache/opencode/ "$B/agent/opencode-cache/"

# Claude Code full
rsync -aH --info=progress2 ~/.claude/ "$B/agent/claude-home/"
cp -a ~/.claude.json "$B/agent/claude.json"

# npm prefix (jika pakai ~/.npm-global)
if [ -d ~/.npm-global ]; then
  rsync -aH ~/.npm-global/ "$B/agent/npm-global/" || true
fi

# arsip
cd /media/jetson/BACKUP
tar czvf jetson_agent_sessions_$(date +%Y%m%d).tar.gz -C "$B" agent
```

**Penting:**
- Jangan `git add` folder agent (ada **API key** di `auth.json` / `.credentials.json`).
- Simpan tar di USB/laptop terenkripsi jika perlu.
- Copy **db + wal + shm** bersama (`opencode.db*`) — jangan hanya `.db`.

### 12.3 Restore di SD baru

```bash
# 1) Node.js + npm (versi mirip SD lama, ideal Node 18+)
#    install sesuai distro / nvm

mkdir -p ~/.npm-global
npm config set prefix ~/.npm-global
echo 'export PATH=$HOME/.npm-global/bin:$PATH' >> ~/.bashrc
source ~/.bashrc

# 2) Install CLI (versi boleh terbaru, atau pin sama SD lama)
npm install -g opencode-ai
npm install -g @anthropic-ai/claude-code

# 3) Restore data (dari USB)
export B=/media/.../jetson_migrate_YYYYMMDD/agent

# tutup agent dulu
mkdir -p ~/.config ~/.local/share
rsync -aH "$B/opencode-config/" ~/.config/opencode/
rsync -aH "$B/opencode-share/" ~/.local/share/opencode/

mkdir -p ~/.claude
rsync -aH "$B/claude-home/" ~/.claude/
cp -a "$B/claude.json" ~/.claude.json
chmod 600 ~/.claude.json ~/.claude/.credentials.json 2>/dev/null || true
chmod 600 ~/.local/share/opencode/auth.json 2>/dev/null || true

# 4) Plugin OpenCode (jika node_modules tidak di-copy penuh)
cd ~/.config/opencode && npm install 2>/dev/null || true
# ponytail / plugin lain: opencode plugin ... jika perlu

# 5) Uji
opencode --version
claude --version
# buka opencode di ~/catkin_ws — history session harus muncul
# buka claude di ~/catkin_ws — project/settings harus kenal
```

### 12.4 Jika history tidak muncul

| Gejala | Cek |
|---|---|
| OpenCode kosong | Path `~/.local/share/opencode/opencode.db` ada? Permission user sama? |
| OpenCode error DB | Pastikan copy `opencode.db`, `opencode.db-wal`, `opencode.db-shm` |
| Auth gagal | `auth.json` / credentials; atau `/connect` ulang di opencode |
| Claude tidak kenal proyek | `~/.claude/projects/` + path home user sama (`/home/jetson`) |
| Home user beda (bukan jetson) | Path di session/project bisa putus — ideal user **`jetson`** sama seperti lama |

### 12.5 Home user harus sama (sangat disarankan)

Session & project path sering absolut (`/home/jetson/...`).  
Di SD baru: buat user **`jetson`** (bukan nama lain) agar full session & path proyek cocok tanpa edit DB.

### 12.6 Yang tidak ikut antar tool

- Chat OpenCode **tidak** bisa dibuka di Claude Code (dan sebaliknya).
- “Memori proyek” yang portable = `AGENTS.md` + `docs/` di catkin (tetap backup terpisah dari agent).

---

## 13. Ringkas

1. **Backup** sumber + config + params + docs (bukan build/).  
2. **Backup full session** OpenCode + Claude (§12).  
3. **Install bersih** OS + ROS + librealsense Jetson.  
4. **Restore & catkin build**.  
5. **Restore agent** + npm global CLI.  
6. **Param EKF3** ke FC.  
7. **Verifikasi** walk test → Loiter → kamera → misi + agent history.  
8. LuckyBird = bacaan konsep, **bukan** resep install SD baru.

*Setelah migrasi: anggap “belum siap lomba” sampai walk test + Loiter di SD baru lulus lagi.*


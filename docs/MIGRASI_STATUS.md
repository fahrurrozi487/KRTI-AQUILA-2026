# Status Migrasi Claude Code → OpenCode

Dokumen ini merekam **apa yang sudah dikerjakan dari sisi Claude Code** untuk persiapan
migrasi, supaya OpenCode langsung paham keadaan terkini tanpa menebak. Dibuat sebagai
pelengkap `MIGRASI_CLAUDE_CODE_KE_OPENCODE.md` (panduan) dan `docs/HANDOFF.md` (serah-terima teknis).

Tanggal: 2026-07 (sesi terakhir Claude Code).

---

## 1. Yang SUDAH dikerjakan (sisi Claude Code)

| Item | File | Status |
|---|---|---|
| Serah-terima teknis lengkap | `docs/HANDOFF.md` | ✅ dibuat, diverifikasi ke repo |
| Rules proyek utama | `AGENTS.md` | ✅ dikoreksi + diperkaya |
| Git ignore untuk catkin | `.gitignore` | ✅ dibuat (belum `git init`) |
| Uploader misi AUTO + dry-run | `src/mission_control/scripts/mission_auto.py` | ✅ dibuat & diuji (preview) |
| Rute AUTO geometris | `src/mission_control/config/mission_auto.yaml` | ✅ dibuat |
| Launch AUTO | `src/mission_control/launch/phaseD_auto.launch` | ✅ dibuat (arg `mode` tersambung) |
| Dokumen status ini | `docs/MIGRASI_STATUS.md` | ✅ file ini |

## 2. Perubahan penting pada `AGENTS.md`

`AGENTS.md` = rules utama OpenCode (dipakai langsung; **tidak ada `CLAUDE.md`**, jadi
Langkah 6 panduan migrasi soal fallback `CLAUDE.md` TIDAK berlaku, dan `/init` tak perlu).

Yang dikoreksi/ditambah:

- **KOREKSI FAKTUAL:** build pakai **`catkin build`** (catkin_tools), BUKAN `catkin_make`.
  Ini bug lama di `AGENTS.md` yang bisa menyesatkan agent.
- **Package layout** ditambah node baru: `mission_auto.py`, `set_origin_home.py`.
- **Gotcha baru** (semua hasil sesi Claude Code terakhir):
  - EKF3-only non-GPS; `EK3_SRC1_YAW` WAJIB 6 (bukan 1/compass yang dimatikan).
  - Origin+home tiap boot via `set_origin_home.launch` di titik takeoff; home `current_gps=True`.
  - T265 = FRONT-facing; `pitch_cam` mengompensasi kemiringan fisik.
  - Guard NaN di `vision_to_mavros.cpp`; T265 launch `initial_reset:=true`.
  - GCS lag = radio SiK 57600 (bukan Jetson/UART); tuning `SR1_*`.
  - Dua jalur autonomous: GUIDED (`mission_d.py`, kamera) vs AUTO (`mission_auto.py`, FC sendiri).

## 3. Jalur AUTO baru (yang dibangun sesi ini)

**Beda dari `mission_d.py` (GUIDED):** AUTO dieksekusi FC sendiri dari misi yang di-upload.
FC tidak tahu ArUco → hanya aksi geometris (waypoint / drop buta / land). Untuk scan/align
berbasis kamera tetap pakai GUIDED `mission_d.py`.

Alur: `record_waypoint.py` (survei) → betulkan `z` di `waypoints.yaml` → `set_origin_home.launch`
di titik takeoff → `phaseD_auto.launch` (generate + push) → takeoff Stabilize → hover cek →
switch AUTO → jari di Stabilize.

Konversi ENU lokal → lat/lon relatif origin: `dLat = dN/111320`, `dLon = dE/(111320·cos lat)`.

### Cara dry-run (AMAN, tak menggerakkan motor)

```bash
# (a) PREVIEW offline — tanpa mavros, cek koordinat & konversi:
rosrun mission_control mission_auto.py _mode:=preview

# (b) PUSH + readback — mavros hidup, disarmed:
roslaunch mission_control phaseD_auto.launch mode:=push
#   upload ke FC lalu baca balik (WaypointPull) untuk konfirmasi item tersimpan.
#   Verifikasi juga di Mission Planner: tab Plan → Read.
```

Preview sudah diuji: origin `-7.0501108, 110.3913388`, menghasilkan 7 item (home + takeoff
+ 5 leg), konversi koordinat benar. `mode` default = `push`.

## 4. Yang BELUM / bukan tugas Claude Code (kerjakan di OpenCode)

- **`git init`** — workspace ini BUKAN git repo. `.gitignore` sudah siap; keputusan root
  repo & init diserahkan ke user. Cek dulu package upstream (`mavros`, `realsense-ros`, dll)
  tidak membawa `.git` sendiri sebelum init di `~/catkin_ws`.
- **Install OpenCode**, `/connect` model, buat global `~/.config/opencode/AGENTS.md`,
  `opencode.json` — semua dieksekusi DI DALAM OpenCode (lihat panduan Langkah 3–13).
- **Memori Claude** (`~/.claude/projects/.../memory/`, 9 file) tidak terbawa otomatis —
  tapi fakta pentingnya sudah dirangkum ke `docs/HANDOFF.md` + `AGENTS.md` + `PANDUAN_TERBANG_T265.md`.

## 5. Artefak Claude Code yang TIDAK relevan untuk OpenCode

- `.claude/settings.local.json` — permission lokal Claude Code saja.
- Tidak ada skills / commands / subagents / MCP / hooks Claude di proyek → tak ada migrasi manual.

---

## Prioritas kerja teknis berikutnya (dari HANDOFF)

1. **Loiter/hover stabil** — gerbang mutlak (crash terakhir di Loiter, walk test belum lulus).
2. Walk test darat → ukur skala (meleset konsisten? → tambal `scale_factor` di `vision_to_mavros.cpp`).
3. Cek jarak T265↔CG (>15cm → offset kompensasi).
4. Survei WP → uji AUTO (dry-run push dulu).
5. Aktifkan gate detector saat footage head-on datang.

#!/usr/bin/env python3
"""
Konversi waypoint lat/lon ASLI -> ENU lokal (waypoints.yaml), relatif ke EKF
origin yang LAGI AKTIF (bukan origin sesi survei lama).

Kenapa: waypoints.yaml biasa direkam fisik (record_waypoint.py, jalan ke titik,
butuh EKF hidup di lokasi). Kalau ganti mode navigasi (T265 <-> GPS) atau EKF
origin beda dari hari survei, waypoint lama BISA MELESET (origin T265 = titik
manual set_origin_home; origin GPS = otomatis dari fix, gak dijamin sama persis).

Solusi: definisikan waypoint dalam lat/lon ASLI (permanen, gak berubah walau
origin beda tiap sesi) sekali di waypoints_latlon.yaml, lalu jalankan script ini
SETIAP SESI SEBELUM misi -- baca origin EKF yang LAGI AKTIF via
/mavros/global_position/gp_origin (topic sama yang dipakai set_origin_home.py
DAN yang otomatis keisi GPS pas fix), convert ke ENU, tulis ke waypoints.yaml.
mission_d.py TIDAK BERUBAH sama sekali -- tetap baca waypoints.yaml (ENU meter)
persis seperti sebelumnya.

Formula sama persis dgn mission_auto.py::_enu_to_latlon (dibalik), biar
konsisten -- bukan rumus baru:
  dlat/dlon -> ENU:  y_n = (lat-lat0)*M_PER_DEG_LAT
               x_e = (lon-lon0)*M_PER_DEG_LAT*cos(radians(lat0))
Altitude TIDAK dikonversi dari geodetic -- diisi manual per WP (meter AGL,
sama konvensi dgn waypoints.yaml/mission_d.py yang ada, relatif takeoff).

Input (waypoints_latlon.yaml), format per WP:
  wp1: {lat: -7.0501050, lon: 110.3913400, alt: 1.0}
  wp2: {lat: ..., lon: ..., alt: ...}

Pakai (mavros harus sudah connect + origin sudah established -- GPS: nunggu
fix; T265: sesudah set_origin_home.launch):
  rosrun mission_control latlon_to_waypoints.py
  rosrun mission_control latlon_to_waypoints.py _in_file:=... _out_file:=...

Preview OFFLINE (tanpa mavros/FC, buat cek angka konversi dulu sebelum
dipercaya) -- kasih origin manual:
  rosrun mission_control latlon_to_waypoints.py _preview:=true _lat0:=-7.0501108 _lon0:=110.3913388

Entri di waypoints.yaml yang TIDAK ada di waypoints_latlon.yaml (mis.
gate_double/gate_triple yang direkam fisik) TIDAK disentuh -- perilaku
merge sama seperti record_waypoint.py, bukan overwrite total.
"""
import math
import os
import sys

import yaml

M_PER_DEG_LAT = 111320.0  # sama persis dgn mission_auto.py, jangan didup-rederive

DEFAULT_IN = os.path.expanduser(
    "~/catkin_ws/src/mission_control/config/waypoints_latlon.yaml")
DEFAULT_OUT = os.path.expanduser(
    "~/catkin_ws/src/mission_control/config/waypoints.yaml")


def latlon_to_enu(lat, lon, lat0, lon0):
    """(lat,lon) -> (x_east, y_north) meter relatif origin (lat0,lon0)."""
    y_n = (lat - lat0) * M_PER_DEG_LAT
    x_e = (lon - lon0) * M_PER_DEG_LAT * math.cos(math.radians(lat0))
    return x_e, y_n


def convert_all(latlon_wps, lat0, lon0):
    out = {}
    for name, w in latlon_wps.items():
        x, y = latlon_to_enu(float(w["lat"]), float(w["lon"]), lat0, lon0)
        out[name] = {
            "x": round(x, 3),
            "y": round(y, 3),
            "z": round(float(w.get("alt", 0.0)), 3),
            "qz": 0.0,
            "qw": 1.0,
        }
    return out


def _load(path):
    with open(os.path.expanduser(path)) as f:
        return yaml.safe_load(f) or {}


def main():
    import rospy

    rospy.init_node("latlon_to_waypoints", anonymous=True)
    in_file = rospy.get_param("~in_file", DEFAULT_IN)
    out_file = rospy.get_param("~out_file", DEFAULT_OUT)
    preview = bool(rospy.get_param("~preview", False))

    if not os.path.exists(os.path.expanduser(in_file)):
        rospy.logerr("[latlon2wp] file input tidak ada: %s", in_file)
        sys.exit(1)
    latlon_wps = _load(in_file)
    if not latlon_wps:
        rospy.logerr("[latlon2wp] %s kosong/gagal dibaca.", in_file)
        sys.exit(1)

    if preview:
        lat0 = float(rospy.get_param("~lat0"))
        lon0 = float(rospy.get_param("~lon0"))
        rospy.loginfo("[latlon2wp] PREVIEW (offline) -- origin manual (%.7f, %.7f)",
                      lat0, lon0)
    else:
        from geographic_msgs.msg import GeoPointStamped
        rospy.loginfo("[latlon2wp] menunggu /mavros/global_position/gp_origin "
                      "(EKF origin AKTIF -- pastikan mavros connect & GPS fix / "
                      "set_origin_home sudah jalan)...")
        try:
            origin = rospy.wait_for_message(
                "/mavros/global_position/gp_origin", GeoPointStamped, timeout=30.0)
        except rospy.ROSException:
            rospy.logerr("[latlon2wp] timeout 30s: gp_origin belum ada. "
                        "mavros connected? GPS fix / set_origin_home sudah jalan?")
            sys.exit(1)
        lat0 = origin.position.latitude
        lon0 = origin.position.longitude
        rospy.loginfo("[latlon2wp] origin EKF AKTIF: (%.7f, %.7f)", lat0, lon0)

    converted = convert_all(latlon_wps, lat0, lon0)

    rospy.loginfo("[latlon2wp] hasil konversi (%d WP):", len(converted))
    for name, w in converted.items():
        rospy.loginfo("  %-15s x=%7.2f  y=%7.2f  z=%5.2f", name, w["x"], w["y"], w["z"])

    if preview:
        rospy.loginfo("[latlon2wp] mode preview -- TIDAK menulis %s.", out_file)
        return

    data = {}
    out_path = os.path.expanduser(out_file)
    if os.path.exists(out_path):
        data = _load(out_path)
    n_new = len(set(converted) - set(data))
    n_overwrite = len(set(converted) & set(data))
    data.update(converted)

    os.makedirs(os.path.dirname(out_path), exist_ok=True)
    with open(out_path, "w") as f:
        yaml.safe_dump(data, f, default_flow_style=False, sort_keys=True)

    rospy.loginfo("[latlon2wp] tersimpan ke %s (%d baru, %d ditimpa, entri lain "
                  "tidak disentuh).", out_path, n_new, n_overwrite)


def _selftest():
    """Self-check murni matematika, tanpa ROS -- assert round-trip lat/lon<->ENU."""
    lat0, lon0 = -7.0501108, 110.3913388
    # titik ~10m timur, ~10m utara dari origin (aproksimasi)
    dlat = 10.0 / M_PER_DEG_LAT
    dlon = 10.0 / (M_PER_DEG_LAT * math.cos(math.radians(lat0)))
    lat, lon = lat0 + dlat, lon0 + dlon
    x, y = latlon_to_enu(lat, lon, lat0, lon0)
    assert abs(x - 10.0) < 1e-6, f"x harus ~10.0, dapat {x}"
    assert abs(y - 10.0) < 1e-6, f"y harus ~10.0, dapat {y}"
    # origin sendiri harus jadi (0,0)
    x0, y0 = latlon_to_enu(lat0, lon0, lat0, lon0)
    assert abs(x0) < 1e-9 and abs(y0) < 1e-9, f"origin harus (0,0), dapat ({x0},{y0})"
    # convert_all: cek format output & merge-safe (tidak mutasi input)
    wps = {"wpA": {"lat": lat, "lon": lon, "alt": 1.5}}
    out = convert_all(wps, lat0, lon0)
    assert abs(out["wpA"]["x"] - 10.0) < 1e-2 and abs(out["wpA"]["y"] - 10.0) < 1e-2
    assert out["wpA"]["z"] == 1.5
    print("latlon_to_waypoints selftest: PASS")


if __name__ == "__main__":
    if "--selftest" in sys.argv:
        _selftest()
    else:
        try:
            main()
        except ImportError:
            print("Butuh ROS environment (rospy) -- pakai --selftest utk cek "
                  "matematika saja tanpa ROS.")
            sys.exit(1)

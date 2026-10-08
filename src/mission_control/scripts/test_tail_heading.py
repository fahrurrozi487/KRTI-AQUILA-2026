#!/usr/bin/env python3
"""Self-test murni Python utk MissionD._tail_to_yaw_cmd() (Task 4, heading align).

Tidak butuh roscore/hardware -- bypass MissionD.__init__ (yang butuh
~waypoints_file dst dari rosparam) via subclass Fake, cuma set 2 atribut
kalibrasi yang dipakai _tail_to_yaw_cmd().

Jalankan: python3 test_tail_heading.py
"""
import os
import sys
import time

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
import rospy
import mission_d

# rospy.Time.now() butuh init_node() (roscore/master) -- kita jalan offline (tanpa
# ROS master, sesuai batasan sesi ini), jadi pakai wall-clock via from_sec() saja.
rospy.Time.now = staticmethod(lambda: rospy.Time.from_sec(time.time()))


class _Fake(mission_d.MissionD):
    def __init__(self, rotate=0, mirror=False, enable=True, stale_sec=2.0):
        self.tail_dir_rotate_steps = rotate
        self.tail_mirror = mirror
        self.enable_heading_align = enable
        self.tail_result_stale_sec = stale_sec
        self._wp_marker_result = None
        self.yaw_calls = []          # rekam pemanggilan _yaw_turn (bukan mavros sungguhan)

    def _yaw_turn(self, x, y, z, yaw_deg=90.0, direction="left"):
        self.yaw_calls.append((yaw_deg, direction))
        return True


class _Header:
    def __init__(self, stamp):
        self.stamp = stamp


class _Result:
    def __init__(self, wp_id, tail_side, low_conf_tail=False, age_sec=0.0):
        self.header = _Header(rospy.Time.now() - rospy.Duration(age_sec))
        self.wp_id = wp_id
        self.tail_side = tail_side
        self.low_conf_tail = low_conf_tail


def _selftest():
    ok = True

    def check(rotate, mirror, expect, label):
        nonlocal ok
        f = _Fake(rotate=rotate, mirror=mirror)
        got = {t: f._tail_to_yaw_cmd(t) for t in ['N', 'E', 'S', 'W']}
        passed = got == expect
        print("  %-28s -> %s %s" % (label, got, "OK" if passed else "GAGAL exp=%s" % expect))
        ok = ok and passed

    print("== _tail_to_yaw_cmd() self-test (tanpa ROS) ==")
    # default: N=tak putar, E=kanan90, S=180, W=kiri90
    check(0, False, {'N': (0.0, 'left'), 'E': (90.0, 'right'),
                     'S': (180.0, 'right'), 'W': (90.0, 'left')},
          "default (rotate=0, mirror=False)")
    # mirror: tukar E<->W
    check(0, True, {'N': (0.0, 'left'), 'E': (90.0, 'left'),
                    'S': (180.0, 'right'), 'W': (90.0, 'right')},
          "mirror=True")
    # rotate 1 step: geser mapping 90 deg (mounting fisik terputar)
    check(1, False, {'N': (90.0, 'right'), 'E': (180.0, 'right'),
                     'S': (90.0, 'left'), 'W': (0.0, 'left')},
          "rotate_steps=1")

    f = _Fake()
    for bad in ('', None, 'X', 'north'):
        passed = f._tail_to_yaw_cmd(bad) is None
        print("  tail_side=%r -> None %s" % (bad, "OK" if passed else "GAGAL"))
        ok = ok and passed

    print()
    print("== _align_heading_to_tail() guard self-test (tanpa ROS/mavros) ==")

    def check_skip(label, f, expected_id=3):
        nonlocal ok
        got = f._align_heading_to_tail(0.0, 0.0, 1.0, expected_id)
        passed = got is True and f.yaw_calls == []
        print("  %-42s -> return=%s yaw_calls=%s %s"
              % (label, got, f.yaw_calls, "OK" if passed else "GAGAL (harusnya skip, tak putar)"))
        ok = ok and passed

    f = _Fake(enable=False)
    f._wp_marker_result = _Result(3, 'E')
    check_skip("enable_heading_align=False", f)

    f = _Fake()
    f._wp_marker_result = None
    check_skip("belum ada WpMarkerResult", f)

    f = _Fake(stale_sec=2.0)
    f._wp_marker_result = _Result(3, 'E', age_sec=5.0)
    check_skip("WpMarkerResult basi (5s > 2s)", f)

    f = _Fake()
    f._wp_marker_result = _Result(1, 'E')   # wp_id=1, tapi leg expected_id=3
    check_skip("wp_id tak cocok (result id=1, expected=3)", f)

    f = _Fake()
    f._wp_marker_result = _Result(3, '', low_conf_tail=True)
    check_skip("low_conf_tail=True / tail_side kosong", f)

    f = _Fake()
    f._wp_marker_result = _Result(3, 'X')   # tail_side tak dikenal
    check_skip("tail_side tak dikenal", f)

    f = _Fake()
    f._wp_marker_result = _Result(3, 'N')   # sudah menghadap -> tak perlu putar
    check_skip("tail_side='N' (sudah menghadap)", f)

    # jalur SUKSES: semua guard lolos -> _yaw_turn() harus dipanggil dgn arg benar
    f = _Fake()
    f._wp_marker_result = _Result(3, 'E')
    got = f._align_heading_to_tail(0.0, 0.0, 1.0, 3)
    passed = got is True and f.yaw_calls == [(90.0, 'right')]
    print("  %-42s -> return=%s yaw_calls=%s %s"
          % ("guard lolos, tail_side='E'", got, f.yaw_calls,
             "OK" if passed else "GAGAL exp=[(90.0,'right')]"))
    ok = ok and passed

    print("HASIL:", "SEMUA LULUS" if ok else "ADA YANG GAGAL")
    return ok


if __name__ == "__main__":
    sys.exit(0 if _selftest() else 1)

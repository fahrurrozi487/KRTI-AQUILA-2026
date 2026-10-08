# Satu marker ArUco terdeteksi (kamera bawah)
int32 id
float64 cx          # pusat marker, x piksel
float64 cy          # pusat marker, y piksel
float64 off_x       # offset ternormalisasi dari pusat gambar, -1..1 (+ = kanan)
float64 off_y       # offset ternormalisasi dari pusat gambar, -1..1 (+ = bawah)
float64 side_px     # rata-rata panjang sisi marker (piksel), proxy jarak
bool has_pose       # true bila position valid (perlu kalibrasi + marker_length)
geometry_msgs/Point position   # posisi marker di frame kamera (meter), bila has_pose

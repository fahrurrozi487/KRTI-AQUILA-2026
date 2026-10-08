# Hasil deteksi garis hitam putus-putus (kamera bawah)
std_msgs/Header header
bool detected             # ada segmen garis cukup besar?
float64 off_x             # offset ternormalisasi centroid segmen, -1..1 (+ = kanan)
float64 off_y             # offset ternormalisasi centroid segmen, -1..1 (+ = bawah)
float64 area_frac         # fraksi piksel gelap di frame (proxy ukuran/jarak segmen)
float64 angle_deg         # arah dash, (-90,90], 0=horizontal. Ambigu 180 deg
int32[] bbox               # [x,y,w,h] kotak segmen terpilih (piksel)

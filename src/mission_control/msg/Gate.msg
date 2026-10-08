# Hasil deteksi gate oranye (kamera depan)
std_msgs/Header header
bool detected            # ada gate cukup besar & bukaan ditemukan?
float64 off_x            # offset ternormalisasi pusat BUKAAN, -1..1 (+ = kanan)
float64 off_y            # offset ternormalisasi pusat BUKAAN, -1..1 (+ = bawah)
float64 area_frac        # fraksi piksel oranye di frame (proxy kedekatan)
int32 num_openings       # jumlah celah (1=single, 2=double, dst)
float64 opening_w_frac   # lebar bukaan terpilih / lebar gambar
int32[] bbox             # [x,y,w,h] kotak oranye keseluruhan (piksel)

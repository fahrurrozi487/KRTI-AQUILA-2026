std_msgs/Header header
bool    found_marker      # body marker terdeteksi dan AR check lolos
int32   wp_id             # 1/3/4 jika match, -1 jika no_match atau not found
int32   hamming           # hamming distance ke pola referensi yg dipilih (-1 jika no marker)
bool    reliable          # true jika wp_id termasuk WP reliable=true (WP3/WP4)
bool    low_conf_tail     # true jika detect_tail_side() return None (orientasi tidak pasti)
string  reason            # "ok" | "no_match" | "decode_failed" | "no_marker" | "no_corners" | "warp_bad"
float32 aspect            # warp aspect ratio (debugging lapangan; 0 jika no marker)
float32 off_x             # offset pusat marker dari tengah frame, -1..+1 (0=tengah); 0 jika no_marker
float32 off_y             # offset pusat marker dari tengah frame, -1..+1 (0=tengah); 0 jika no_marker

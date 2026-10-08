import cv2, os
import gate_multi_detector as G
DPATH="/home/jetson/catkin_ws/gate/dataset_gate_baru"
OUT="/home/jetson/catkin_ws/gate/audit_rel"; os.makedirs(OUT,exist_ok=True)
tun_f,hol_f=G._split_dataset(DPATH,seed=42)
files=tun_f+hol_f
# cari frame yg jadi MULTI di rel0 tapi berkurang di rel.15 -> itu yg 'dibuang'
picks=[]
for f in files:
    im=cv2.imread(f)
    if im is None:continue
    a=G.detect_gates(im,solidity_lo=.15,solidity_hi=.80,rel_area_frac=0)
    b=G.detect_gates(im,solidity_lo=.15,solidity_hi=.80,rel_area_frac=.15)
    if len(a)>len(b):  # ada kandidat yg dibuang filter rel
        picks.append((f,len(a),len(b),a,b))
print("frame yg kandidatnya berkurang krn rel.15:",len(picks))
# ambil 5 contoh dgn selisih terbesar utk audit
picks.sort(key=lambda t:t[1]-t[2],reverse=True)
for f,na,nb,a,b in picks[:5]:
    im=cv2.imread(f)
    bn=os.path.splitext(os.path.basename(f))[0]
    cv2.imwrite(os.path.join(OUT,bn+"_rel0.png"),G._draw_overlay(im,a))
    cv2.imwrite(os.path.join(OUT,bn+"_rel15.png"),G._draw_overlay(im,b))
    # cetak area kandidat yg dibuang (relatif thd terbesar)
    big=a[0]["area"]
    dropped=[g for g in a if g not in b]
    print("  %s  na=%d nb=%d  area_terbesar=%d  dibuang:%s"%(
        bn,na,nb,int(big),[ "%.0f%%"%(100*g["area"]/big) for g in dropped]))
print("saved to",OUT)

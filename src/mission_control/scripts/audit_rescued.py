import cv2, os
import gate_multi_detector as G
DPATH="/home/jetson/catkin_ws/gate/dataset_gate_baru"
OUT="/home/jetson/catkin_ws/gate/audit_rescued"; os.makedirs(OUT,exist_ok=True)
tun_f,hol_f=G._split_dataset(DPATH,seed=42)
files=tun_f+hol_f
rescued=[]
for f in files:
    im=cv2.imread(f)
    if im is None:continue
    base=G.detect_gates(im)  # baseline lo.30 hi.60 rel0
    new=G.detect_gates(im,solidity_lo=.15,solidity_hi=.80,rel_area_frac=.15)
    if len(base)==0 and len(new)==1:
        rescued.append((f,new[0]))
print("frame baseline=0 -> rel.15=1 (diselamatkan):",len(rescued))
# ambil 6 tersebar (indeks merata) utk audit, bukan cuma yg pertama
step=max(1,len(rescued)//6)
picks=rescued[::step][:6]
for f,g in picks:
    im=cv2.imread(f)
    bn=os.path.splitext(os.path.basename(f))[0]
    cv2.imwrite(os.path.join(OUT,bn+"_resc.png"),
                G.detect_gates and G._draw_overlay(im,[g]))
    print("  %s  off=(%.2f,%.2f) area=%d bbox=%s"%(
        bn,g["offset_x"],g["offset_y"],int(g["area"]),g["bbox_outer"]))
print("saved to",OUT)

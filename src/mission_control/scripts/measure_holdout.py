import time, cv2
import gate_multi_detector as G
DPATH="/home/jetson/catkin_ws/gate/dataset_gate_baru"
tun_f,hol_f=G._split_dataset(DPATH,seed=42)
def load(fs):
    o=[]
    for f in fs:
        im=cv2.imread(f)
        if im is not None:o.append(im)
    return o
def stats(imgs,**kw):
    nd=tc=nm=0
    for im in imgs:
        g=G.detect_gates(im,**kw)
        if g:
            nd+=1;tc+=len(g)
            if len(g)>=2:nm+=1
    n=len(imgs)
    return 100.0*nd/n, tc/nd if nd else 0, 100.0*nm/n
tun=load(tun_f);hol=load(hol_f)
configs=[
 ("baseline lo.30 hi.60 rel0",dict()),
 ("lo.15 hi.80 rel0",dict(solidity_lo=.15,solidity_hi=.80)),
 ("lo.15 hi.80 rel.15",dict(solidity_lo=.15,solidity_hi=.80,rel_area_frac=.15)),
 ("lo.15 hi.70 rel.15",dict(solidity_lo=.15,solidity_hi=.70,rel_area_frac=.15)),
]
print("%-26s | %-22s | %-22s | gap"%("config","TUNING det/avg/multi","HOLDOUT det/avg/multi"))
for name,kw in configs:
    dt,at,mt=stats(tun,**kw)
    dh,ah,mh=stats(hol,**kw)
    print("%-26s | %5.1f %.2f %5.1f       | %5.1f %.2f %5.1f       | %.1f"%(name,dt,at,mt,dh,ah,mh,abs(dt-dh)))

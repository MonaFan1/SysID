"""Independent noisy continuous-time plant, sampled at 2 kHz and averaged to 100 Hz."""
import sys
from pathlib import Path
import numpy as np
sys.path.insert(0,str(Path(__file__).resolve().parents[1]))
from identify import fit_candidates
def run_case(J,b):
    rng=np.random.default_rng(710)
    h=.0005;fc=.015;k=.1;offset=.005
    q=0.;w=0.;u=0.;e=0.;nextbit=2.;rows=[(0,-1.5,0,0,0,0,0,0,0,-1.5,0,0)]
    acc=np.zeros(6);last_measured=0
    for n in range(1,30001):
     t=n*h
     stage=0 if t<2 else 1 if t<6 else 2 if t<10 else 3 if t<14 else 4
     if n%4==1:
      if stage in (1,2,3) and t>=nextbit:
       e=rng.choice([-1,1])*.07*(.65 if stage==2 else 1);nextbit=t+{1:.04,2:.1,3:.25}[stage]
      if stage in (0,4):e=0
      u=np.clip(e-2*q-.12*(w+rng.normal(0,.04)),-.2,.2)
     drive=u-b*w-k*q-offset
     friction=np.clip(drive,-fc,fc) if abs(w)<.0003 else fc*np.sign(w)
     w+=(drive-friction)/J*h;q+=w*h
     measured=w+rng.normal(0,.08)
     acc+=np.array([.5*(last_measured+measured),q-1.5,u,u+rng.normal(0,.001),e,e-u])*h
     last_measured=measured
     if n%20==0:
      means=acc/.01
      rows.append((t,q-1.5,means[0],w,means[2],means[3],means[4],means[5],0,means[1],stage,0));acc[:]=0
    meta=dict(center=-1.5,settle_s=2,excite_s=12,status=2,status_text='done',format_version='interval-v3')
    r=fit_candidates(rows,meta)
    for source in ('command','feedback'):
     valid=[x for x in r['fits'][source] if x['available']]
     assert len(valid)==3
     for x in valid:
      assert abs(x['J_kg_m2']/J-1)<.05,x
      assert abs(x['b_nm_s_rad']/b-1)<.15,x
    print('PASS: independent noisy plant, J within 5% and b within 15% across all windows (simulation only).')

for inertia,damping in ((.006,.04),(.012,.09),(.024,.15)):
    run_case(inertia,damping)

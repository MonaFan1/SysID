"""Run after motor_sysid_test has written its synthetic fixtures."""
import sys
from pathlib import Path
sys.path.insert(0,str(Path(__file__).resolve().parents[1]))
from identify import decode,fit_candidates

folder=Path(sys.argv[1])
raw=(folder/'prbs.bin').read_bytes()
rows,meta=decode(raw)
assert meta['status'] is None  # Completion is not encoded in the dump.
for fixed_j in (None,.012):
    result=fit_candidates(rows,meta,fixed_j)
    for source in ('command','feedback'):
        valid=[r for r in result['fits'][source] if r['available']]
        assert len(valid)==3
        for r in valid:
            assert abs(r['J_kg_m2']/.012-1)<.03,r
            assert abs(r['b_nm_s_rad']/.09-1)<.03,r
import struct
old_motor=bytearray(raw)
struct.pack_into('<I',old_motor,24,300)
for bad in (raw[:-1],bytes(old_motor),(folder/'calibration_ok.bin').read_bytes()):
    try: decode(bad)
    except ValueError: pass
    else: raise AssertionError('Wrong layout/mode was accepted')
print('PASS: decoder, free/fixed J fits, synthetic b recovery, rejected wrong format/mode.')

restored_rows,restored_meta=decode((folder/'restored.bin').read_bytes())
assert restored_meta['status']==2 and restored_meta['format_version']=='restored-v2'
assert restored_meta['settle_s']==2 and restored_meta['excite_s']==12 and restored_meta['return_s']==1
assert restored_rows[-1][0]>=15 and {int(r[10]) for r in restored_rows}=={0,1,2,3,4}
assert restored_meta['amplitude']>0.069 and len(restored_rows)>1200
print('PASS: restored header/status/parameters and 15-second stage capture decoding.')

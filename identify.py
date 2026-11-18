"""Decode an IMU-based MotorIdentifier PRBS dump (hex Size C008).

Optional --fit estimates local J,b with Coulomb, stiffness and offset nuisance
terms. Results are candidates: a second capture with a new seed is validation.
"""
import argparse
import csv
import json
from pathlib import Path
import struct

SAMPLE = struct.Struct('<6fII')
CAPACITY = 1536
DUMP_BYTES = 0xC008
COLUMNS = ['time_s','position_relative_rad','omega_imu_rad_s','omega_motor_rad_s',
           'torque_cmd_mean_nm','torque_feedback_mean_nm','stage','flags']

def decode(raw):
    import math
    if len(raw) != DUMP_BYTES:
        raise ValueError('Export &g_sysid.buf_[0], hexadecimal Size C008 (49160 bytes).')
    index,count=struct.unpack_from('<II',raw,0xC000)
    if index != count or not 2 <= count <= CAPACITY:
        raise ValueError('Invalid frozen linear capture counters.')
    rows=[]
    for i in range(count):
        t,u,q,w,ref,fb,stage,flags=SAMPLE.unpack_from(raw,i*32)
        if stage not in (400,401,403) or flags & ~1 or not all(math.isfinite(v) for v in [t,u,q,w,fb]):
            raise ValueError('Not a new PRBS capture; old calibration/long-speed dumps require the old tools.')
        if rows and t <= rows[-1][0]: raise ValueError('Non-increasing time.')
        rows.append((t,q,w,ref,u,fb,0.,0.,0.,0.,{400:0,401:1,403:4}[stage],flags))
    exciting=[r[0] for r in rows if r[10]==1]
    return rows,dict(center=0.,settle_s=1.,
                     excite_s=max(exciting)-1. if exciting else 0.,status=None,
                     status_text='completion not stored; check Ozone status_ = 2',count=count,
                     velocity_source='signed IMU gyro axis; motor speed recorded for comparison',format_version='imu32-prbs')


def fit_candidates(rows, metadata, fixed_j=None):
    import numpy as np
    from scipy.integrate import cumulative_trapezoid
    from scipy.optimize import lsq_linear
    from scipy.signal import savgol_filter
    a = np.asarray(rows)
    t, q = a[:, 0], a[:, 1] - metadata['center']
    # Apply smoothing only to the velocity endpoints, no noisy differentiation.
    dt = float(np.median(np.diff(t)))
    length = max(5, int(round(.06 / dt)) | 1)
    if len(a) < length + 20:
        raise ValueError('Not enough data for a dynamic fit.')
    w = savgol_filter(a[:, 2], length, 2)
    terms = np.column_stack([w, np.sign(w), q, np.ones(len(t))])
    integrals = cumulative_trapezoid(terms, t, axis=0, initial=0)
    report = {'model': 'tau = J*domega/dt + b*omega + fc*sign(omega) + k*q + offset',
              'state': 'IMU yaw axis; base/big yaw fixed, pitch near calibrated zero and fixed',
              'velocity_source': 'signed IMU gyro axis; 60 ms endpoint smoothing',
              'fits': {}, 'warnings': [
                  'Candidates only; validate with another seed/capture.',
                  'Feedback torque is current-derived using configured Kt, not a torque sensor.',
                  'Sensor and actuator transport delays are not independently calibrated.',
                  'If b changes substantially with window/input, it is not yet identified.']}
    if metadata['status'] != 2:
        report['warnings'].append(metadata['status_text'])
    moving = (a[:, 10] >= 1) & (a[:, 10] <= 3) & (np.abs(w) >= .06)
    for source, column in [('feedback', 5), ('command', 4)]:
        # Each logged torque is the mean over the interval ENDING at its row.
        area = np.r_[0., np.cumsum(a[1:, column] * np.diff(t))]
        estimates = []
        for window in [.04, .08, .16]:
            x, y, ends = [], [], []
            for i in range(1, len(t) - 2, 2):
                j = int(np.searchsorted(t, t[i] + window))
                if j >= len(t):
                    continue
                if not np.all((a[i:j+1, 10] >= 1) & (a[i:j+1, 10] <= 3)):
                    continue
                if np.mean(moving[i:j+1]) < .8 or np.any(a[i:j+1, 11].astype(int) & 1):
                    continue
                if np.min(w[i:j+1]) < -.06 and np.max(w[i:j+1]) > .06:
                    continue
                x.append(np.r_[w[j]-w[i], integrals[j]-integrals[i]])
                y.append(area[j]-area[i]); ends.append(t[j])
            if len(x) < 30:
                estimates.append({'window_s': window, 'available': False,
                                  'reason': 'Too few moving windows (sticking/early stop).',
                                  'windows': len(x)})
                continue
            x, y, ends = np.asarray(x), np.asarray(y), np.asarray(ends)
            if fixed_j is not None:
                y = y-fixed_j*x[:,0]
                x = x[:,1:]
            scale = np.linalg.norm(x, axis=0)
            if np.any(scale < 1e-10):
                continue
            lower = np.array([1e-6, 0., 0., -20., -1.])
            upper = np.array([1., 5., .5, 20., 1.])
            if fixed_j is not None:
                lower,upper=lower[1:],upper[1:]
            def solve(mask):
                f = lsq_linear(x[mask]/scale, y[mask],
                               bounds=(lower*scale, upper*scale), tol=1e-12)
                return f.x/scale
            p = solve(np.ones(len(x), dtype=bool))
            residual = x @ p - y
            last_stage = ends >= metadata['settle_s'] + 2*metadata['excite_s']/3
            validation = None
            if np.sum(last_stage) >= 20 and np.sum(~last_stage) >= 30:
                train_p = solve(~last_stage)
                validation = float(np.sqrt(np.mean((x[last_stage]@train_p-y[last_stage])**2)))
            params = np.r_[fixed_j,p] if fixed_j is not None else p
            estimates.append(dict(window_s=window, available=True, windows=len(x),
                J_kg_m2=float(params[0]), b_nm_s_rad=float(params[1]),
                coulomb_nuisance_nm=float(params[2]), stiffness_nuisance_nm_rad=float(params[3]),
                offset_nm=float(params[4]), scaled_condition=float(np.linalg.cond(x/scale)),
                integrated_torque_rmse_nm_s=float(np.sqrt(np.mean(residual**2))),
                last_stage_validation_rmse_nm_s=validation,
                parameter_at_bound=bool(np.any(np.isclose(p, lower, atol=1e-5)) or
                                        np.any(np.isclose(p, upper, atol=1e-5)))))
        report['fits'][source] = estimates
    report['quality'] = dict(duration_s=float(t[-1]), sample_interval_median_s=dt,
        position_range_rad=[float(a[:, 1].min()), float(a[:, 1].max())],
        moving_fraction=float(np.mean(moving)),
        speed_range_rad_s=[float(a[:, 2].min()), float(a[:, 2].max())])
    return report


def main():
    p = argparse.ArgumentParser(description=__doc__)
    p.add_argument('binary', type=Path)
    p.add_argument('--out', type=Path)
    p.add_argument('--fit', action='store_true')
    p.add_argument('--fixed-j', type=float, help='Optional trusted output-shaft inertia in kg*m^2')
    args = p.parse_args()
    try:
        if args.fixed_j is not None and (not __import__("math").isfinite(args.fixed_j) or args.fixed_j <= 0):
            raise ValueError("fixed-j must be positive and finite")
        rows, metadata = decode(args.binary.read_bytes())
        out = args.out or args.binary.with_suffix('.csv')
        with out.open('w', newline='', encoding='utf-8') as f:
            writer = csv.writer(f); writer.writerow(COLUMNS)
            writer.writerows(tuple(r[i] for i in (0,1,2,3,4,5,10,11)) for r in rows)
        out.with_suffix('.metadata.json').write_text(json.dumps(metadata, indent=2), encoding='utf-8')
        if args.fit:
            result = fit_candidates(rows, metadata, args.fixed_j)
            out.with_suffix('.fit.json').write_text(json.dumps(result, indent=2), encoding='utf-8')
            print(json.dumps(result, indent=2))
        print(f"{metadata['status_text']}: {len(rows)} samples, {rows[-1][0]:.3f} s -> {out}")
    except (OSError, ValueError, ImportError) as error:
        p.exit(1, f'Error: {error}\n')


if __name__ == '__main__':
    main()

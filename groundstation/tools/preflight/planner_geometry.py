"""No ROS dependencies: sample PolyTraj's descending-order coefficients."""
import numpy as np


def sample_poly(data, dt=.05):
    durations = np.asarray(data['duration'], dtype=float)
    if data['order'] != 5 or not len(durations) or not np.isfinite(durations).all() or np.any(durations <= 0):
        raise ValueError('Invalid PolyTraj durations/order')
    n = len(durations)
    coeff = np.stack([np.asarray(data[k], dtype=float).reshape(n, 6) for k in ('x', 'y', 'z')], axis=1)
    if not np.isfinite(coeff).all() or durations.sum() > 300:
        raise ValueError('Invalid PolyTraj coefficients/duration')
    pieces = []
    for i, duration in enumerate(durations):
        count = min(1000, max(2, int(np.ceil(duration/dt))+1))
        times = np.linspace(0, duration, count)
        xyz = np.stack([np.polyval(coeff[i, a], times) for a in range(3)], axis=1)
        pieces.append(xyz if i == 0 else xyz[1:])
    return np.concatenate(pieces)


def is_active(status, trajectory, remote_now):
    valid = (status.get('phase') == 'ACTIVE'
            and status.get('trajectory_id') == trajectory.get('id')
            and -.05 <= remote_now - status.get('stamp', 0) <= .75)
    age = remote_now - trajectory.get('start_time', 0)
    # Native FSM trajectories remain current through their full native duration and
    # terminal hold. Measured previews refresh from current odometry instead.
    return valid and age >= -.05 and (status.get('planning_mode') == 'upstream_fsm' or age <= 1.5)

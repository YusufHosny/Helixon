"""
adapted from metric.py from RoNIN (https://github.com/Sachini/ronin/blob/master/source/metric.py)
"""

import numpy as np

def compute_absolute_trajectory_error(est: np.ndarray, gt: np.ndarray):
    """
    The Absolute Trajectory Error (ATE) defined in:
    A Benchmark for the evaluation of RGB-D SLAM Systems
    http://ais.informatik.uni-freiburg.de/publications/papers/sturm12iros.pdf
    Adapted to lerp estimated trajectory between timestamps.

    Args:
        est: estimated trajectory. timestamped in shape (N, 2)
        gt: ground truth trajectory. timestamped in shape (M, 2)

    Return:
        Absolution trajectory error, which is the Root Mean Squared Error between
        two trajectories.
    """
    gt_lerped = np.zeros_like(est[:, 1:])
    gt_lerped[:, 0] = est[:, 0]

    for i, (ti, _, _, _) in enumerate(est):
        if ti in gt[:, 0]:
            gt_lerped[i, :] = gt[np.argmax(gt[:, 0] == ti), 1:]
        else:
            # lerp
            ix_2 = np.argmax(gt[:, 0] > ti)
            ix_1 = gt.shape[0] - np.argmax(np.flip(gt, axis=0) < ti)

            # lerp formula: y12 = y1 + (t12 - t1) * (y2-y1)/(t2-t1 + stability epsilon)
            gt_lerped[i, :] = gt[ix_1, 1:] + (ti-gt[ix_1, 0])*(gt[ix_2, 1:] - gt[ix_1, 1:])/(gt[ix_2, 0] - gt[ix_1, 0] + 1e-9)


    return np.sqrt(np.mean((est[:, 1:] - gt_lerped) ** 2))


def compute_relative_trajectory_error(est, gt, delta, max_delta=-1):
    """
    The Relative Trajectory Error (RTE) defined in:
    A Benchmark for the evaluation of RGB-D SLAM Systems
    http://ais.informatik.uni-freiburg.de/publications/papers/sturm12iros.pdf
    Adapted to lerp estimated trajectory between timestamps.

    Args:
        est: the estimated trajectory. timestamped in shape (N, 2)
        gt: the ground truth trajectory. timestamped in shape (M, 2)
        delta: fixed window size. If set to -1, the average of all RTE up to max_delta will be computed.
        max_delta: maximum delta. If -1 is provided, it will be set to the length of trajectories.

    Returns:
        Relative trajectory error. This is the mean value under different delta.
    """
    gt_lerped = np.zeros_like(est[:, 1:])
    gt_lerped[:, 0] = est[:, 0]

    for i, (ti, _, _, _) in enumerate(est):
        if ti in gt[:, 0]:
            gt_lerped[i, :] = gt[np.argmax(gt[:, 0] == ti), 1:]
        else:
            # lerp
            ix_2 = np.argmax(gt[:, 0] > ti)
            ix_1 = gt.shape[0] - np.argmax(np.flip(gt, axis=0) < ti)

            # lerp formula: y12 = y1 + (t12 - t1) * (y2-y1)/(t2-t1 + stability epsilon)
            gt_lerped[i, :] = gt[ix_1, 1:] + (ti-gt[ix_1, 0])*(gt[ix_2, 1:] - gt[ix_1, 1:])/(gt[ix_2, 0] - gt[ix_1, 0] + 1e-9)

    # remove timestamps from estimated
    est = est[:, 1:]

    if max_delta == -1:
        max_delta = est.shape[0]
    deltas = np.array([delta]) if delta > 0 else np.arange(1, min(est.shape[0], max_delta))
    rtes = np.zeros(deltas.shape[0])
    for i in range(deltas.shape[0]):
        # For each delta, the RTE is computed as the RMSE of endpoint drifts from fixed windows
        # slided through the trajectory.
        err = est[deltas[i]:] + gt_lerped[:-deltas[i]] - est[:-deltas[i]] - gt_lerped[deltas[i]:]
        rtes[i] = np.sqrt(np.mean(err ** 2))

    # The average of RTE of all window sized is returned.
    return np.mean(rtes)


def compute_ate_rte(est, gt, pred_per_min=12000):
    """
    A convenient function to compute ATE and RTE. For sequences shorter than pred_per_min, it computes end sequence
    drift and scales the number accordingly.
    """
    ate = compute_absolute_trajectory_error(est, gt)
    if est.shape[0] < pred_per_min:
        ratio = pred_per_min / est.shape[0]
        rte = compute_relative_trajectory_error(est, gt, delta=est.shape[0] - 1) * ratio
    else:
        rte = compute_relative_trajectory_error(est, gt, delta=pred_per_min)

    return ate, rte


def compute_heading_error(est, gt):
    """
    Args:
        est: the estimated heading as sin, cos values
        gt: the ground truth heading as sin, cos values
    Returns:
        MSE error and angle difference from dot product
    """

    mse_error = np.mean((est-gt)**2)
    dot_prod = np.sum(est * gt, axis=1)
    angle = np.arccos(np.clip(dot_prod, a_min=-1, a_max=1))

    return mse_error, angle
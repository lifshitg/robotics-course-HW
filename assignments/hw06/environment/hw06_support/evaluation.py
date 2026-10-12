"""Evaluation accepts answers explicitly; estimators must never call it mid-run."""
import numpy as np
from .geometry import wrap_angle


def trajectory_metrics(estimate, truth):
    estimate,truth=np.asarray(estimate),np.asarray(truth)
    if estimate.shape != truth.shape or estimate.ndim != 2 or estimate.shape[1] != 3:
        raise ValueError('Expected matching (scans, 3) trajectories')
    if not np.isfinite(estimate).all() or not np.isfinite(truth).all():
        raise ValueError('Trajectories must be finite')
    position=np.linalg.norm(estimate[:,:2]-truth[:,:2],axis=1)
    heading=wrap_angle(estimate[:,2]-truth[:,2])
    return {'position_rmse_m':float(np.sqrt(np.mean(position**2))),
            'final_position_error_m':float(position[-1]),
            'heading_rmse_rad':float(np.sqrt(np.mean(heading**2)))}

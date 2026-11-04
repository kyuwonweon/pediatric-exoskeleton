# kinematic_control

Phase-based hip exo controller: an EKF estimates gait phase, phase rate, stride length and incline
from thigh angle using a gait model fit to the Reznick et al. 2021 dataset, then commands a fraction
of the normal human hip torque at that point.

    sensor -> EKF (uses gait model) -> torque map -> safety -> motor

The dataset is not in git. Download `Normalized.mat` from
https://doi.org/10.6084/m9.figshare.c.5175254 into `~/Desktop/projects/final/data/reznick/`.

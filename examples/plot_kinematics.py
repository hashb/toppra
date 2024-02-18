"""Retime a path subject to kinematic constraints
=================================================

In this example, we will see how can we retime a generic spline-based
path subject to kinematic constraints. This is very simple to do with
`toppra`, as we shall see below. First import the library.

"""

from typing import cast
import toppra as ta
import toppra.constraint as constraint
import toppra.algorithm as algo
import numpy as np
import matplotlib.pyplot as plt
import time

ta.setup_logging("INFO")

################################################################################
# We generate a path with some random waypoints.


def generate_new_problem(seed=9):
    # Parameters
    N_samples = 5
    dof = 7
    np.random.seed(seed)
    way_pts = np.random.randn(N_samples, dof)
    return (
        np.linspace(0, 1, 5),
        way_pts,
        10 + np.random.rand(dof) * 20,
        10 + np.random.rand(dof) * 2,
    )


ss, way_pts, vlims, alims = generate_new_problem()

################################################################################
# Define the geometric path and two constraints.
path = ta.SplineInterpolator(ss, way_pts)
pc_vel = constraint.JointVelocityConstraint(vlims)
pc_acc = constraint.JointAccelerationConstraint(alims)

################################################################################
# We solve the parametrization problem using the
# `ParametrizeConstAccel` parametrizer. This parametrizer is the
# classical solution, guarantee constraint and boundary conditions
# satisfaction.
gridpoints = ta.interpolator.propose_gridpoints(
    path,
    max_seg_length=0.01,
    max_iteration=500,
    min_nb_points=500,
)
gridpoints = np.linspace(0, 1, 10000)
instance = algo.TOPPRA(
    [pc_vel, pc_acc],
    path,
    gridpoints=gridpoints,
    # parametrizer="ParametrizeConstAccel",
    parametrizer="ParametrizeSpline",
)
jnt_traj = instance.compute_trajectory()
assert jnt_traj is not None
jnt_traj = cast(ta.parametrizer.ParametrizeConstAccel, jnt_traj)

################################################################################
# The output trajectory is an instance of
# :class:`toppra.interpolator.AbstractGeometricPath`.

# jnt_traj.plot_parametrization(show=True)

ts_sample = np.linspace(0, jnt_traj.duration, 100)
qs_sample = jnt_traj(ts_sample)
qds_sample = jnt_traj(ts_sample, 1)
qdds_sample = jnt_traj(ts_sample, 2)
qddds_sample = jnt_traj(ts_sample, 3)
fig, axs = plt.subplots(4, 2, sharex=True)
for i in range(path.dof):
    # plot the i-th joint trajectory
    axs[0, 0].plot(ts_sample, qs_sample[:, i], c="C{:d}".format(i))
    axs[1, 0].plot(ts_sample, qds_sample[:, i], c="C{:d}".format(i))
    axs[2, 0].plot(ts_sample, qdds_sample[:, i], c="C{:d}".format(i))
    axs[3, 0].plot(ts_sample, qddds_sample[:, i], c="C{:d}".format(i))

from scipy.interpolate import CubicSpline

# differentiate the spline

spline = qs_sample
d_spline = np.diff(spline, axis=0) / np.diff(ts_sample)[:, None]
dd_spline = np.diff(d_spline, axis=0) / np.diff(ts_sample[1:])[:, None]
ddd_spline = np.diff(dd_spline, axis=0) / np.diff(ts_sample[2:])[:, None]
# sus_dd_spline = CubicSpline(ts_sample, qdds_sample, axis=0)
# ddd_spline = sus_dd_spline(ts_sample, 1)

for i in range(path.dof):
    axs[0, 1].plot(ts_sample, spline[:, i], c="C{:d}".format(i))
    axs[1, 1].plot(ts_sample[1:], d_spline[:, i], c="C{:d}".format(i))
    axs[2, 1].plot(ts_sample[2:], dd_spline[:, i], c="C{:d}".format(i))
    axs[3, 1].plot(ts_sample[3:], ddd_spline[:, i], c="C{:d}".format(i))

axs[3, 0].set_xlabel("Time (s)")
axs[3, 1].set_xlabel("Time (s)")

axs[0, 0].set_ylabel("Position (rad)")
axs[1, 0].set_ylabel("Velocity (rad/s)")
axs[2, 0].set_ylabel("Acceleration (rad/s2)")
axs[3, 0].set_ylabel("Jerk (rad/s3)")

axs[0, 1].set_ylabel("Position (rad)")
axs[1, 1].set_ylabel("Velocity (rad/s)")
axs[2, 1].set_ylabel("Acceleration (rad/s2)")
axs[3, 1].set_ylabel("Jerk (rad/s3)")

plt.show()


################################################################################
# Optionally, we can inspect the output.
instance.compute_feasible_sets()
instance.inspect()

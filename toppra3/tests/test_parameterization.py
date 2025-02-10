import pytest
import numpy as np
from toppra3 import Toppra3Parameterization, RobotLimits


def test_basic_parameterization(two_joint_limits, simple_waypoints_2d):
    # Test basic trajectory parameterization with 2 joints
    parameterizer = Toppra3Parameterization(2)

    # Solve without jerk limits first
    success = parameterizer.solve(
        simple_waypoints_2d, two_joint_limits, use_jerk_limits=False
    )
    assert success, "Failed to solve trajectory parameterization without jerk limits"

    # Get trajectory duration
    duration = parameterizer.get_duration()
    assert duration > 0, "Trajectory duration should be positive"

    # Sample trajectory at different times
    times = np.linspace(0, duration, 10)
    for t in times:
        pos, vel, acc = parameterizer.get_state(t)

        # Check dimensions
        assert pos.shape == (2,)
        assert vel.shape == (2,)
        assert acc.shape == (2,)

        # Verify velocity limits
        assert np.all(np.abs(vel) <= two_joint_limits.max_joint_velocity)

        # Verify acceleration limits
        assert np.all(np.abs(acc) <= two_joint_limits.max_joint_acceleration)


def test_jerk_constrained_parameterization(three_joint_limits, complex_waypoints_3d):
    # Test parameterization with jerk constraints
    parameterizer = Toppra3Parameterization(3)

    # Solve with jerk limits
    success = parameterizer.solve(
        complex_waypoints_3d, three_joint_limits, use_jerk_limits=True
    )
    assert success, "Failed to solve trajectory parameterization with jerk limits"

    duration = parameterizer.get_duration()
    times = np.linspace(0, duration, 20)

    # Store velocities and accelerations to compute numerical jerk
    all_velocities = []
    all_accelerations = []
    dt = times[1] - times[0]

    for t in times:
        pos, vel, acc = parameterizer.get_state(t)
        all_velocities.append(vel)
        all_accelerations.append(acc)

        # Verify basic limits
        assert np.all(np.abs(vel) <= three_joint_limits.max_joint_velocity)
        assert np.all(np.abs(acc) <= three_joint_limits.max_joint_acceleration)

    # Compute numerical jerk and verify limits
    # Note: This is an approximation since we're using finite differences
    all_velocities = np.array(all_velocities)
    all_accelerations = np.array(all_accelerations)

    for i in range(1, len(all_accelerations) - 1):
        numerical_jerk = (all_accelerations[i + 1] - all_accelerations[i - 1]) / (
            2 * dt
        )
        assert np.all(
            np.abs(numerical_jerk) <= three_joint_limits.max_joint_jerk * 1.1
        )  # 10% margin for numerical errors


def test_edge_cases(two_joint_limits):
    # Test edge cases and potential failure modes
    parameterizer = Toppra3Parameterization(2)

    # Test 1: Single waypoint (should fail gracefully)
    with pytest.raises(Exception):
        waypoints = [np.array([0.0, 0.0])]
        parameterizer.solve(waypoints, two_joint_limits)

    # Test 2: Two waypoints (minimum case)
    waypoints = [np.array([0.0, 0.0]), np.array([1.0, 1.0])]
    success = parameterizer.solve(waypoints, two_joint_limits)
    assert success, "Failed to solve with minimum number of waypoints"

    # Test 3: Waypoints very close together
    waypoints = [np.array([0.0, 0.0]), np.array([1e-6, 1e-6]), np.array([2e-6, 2e-6])]
    success = parameterizer.solve(waypoints, two_joint_limits)
    assert success, "Failed to solve with very close waypoints"

    # Test 4: Very tight limits
    tight_limits = RobotLimits(2)
    tight_limits.max_joint_velocity = np.array([0.1, 0.1])
    tight_limits.max_joint_acceleration = np.array([0.2, 0.2])
    tight_limits.max_joint_jerk = np.array([0.5, 0.5])

    waypoints = [np.array([0.0, 0.0]), np.array([1.0, 1.0])]
    success = parameterizer.solve(waypoints, tight_limits)
    assert success, "Failed to solve with tight limits"

    # Verify the trajectory takes longer with tighter limits
    duration_tight = parameterizer.get_duration()

    success = parameterizer.solve(
        waypoints, two_joint_limits
    )  # Solve with original limits
    duration_normal = parameterizer.get_duration()

    assert (
        duration_tight > duration_normal
    ), "Trajectory with tighter limits should take longer"


def test_torque_limits(two_joint_limits):
    """Test trajectory parameterization with torque limits"""
    parameterizer = Toppra3Parameterization(2)

    # Set torque limits
    two_joint_limits.max_joint_torque = np.array([5.0, 5.0])

    waypoints = [
        np.array([0.0, 0.0]),
        np.array([1.0, 1.0]),
        np.array([0.0, 0.0]),  # Return to start to test deceleration
    ]

    success = parameterizer.solve(waypoints, two_joint_limits, use_jerk_limits=True)
    assert success, "Failed to solve trajectory with torque limits"

    # The actual torque verification would require inverse dynamics computation
    # which is not directly available in the Python bindings.
    # Here we just verify the trajectory is feasible.
    duration = parameterizer.get_duration()
    times = np.linspace(0, duration, 10)

    for t in times:
        pos, vel, acc = parameterizer.get_state(t)
        assert np.all(np.isfinite(pos))
        assert np.all(np.isfinite(vel))
        assert np.all(np.isfinite(acc))

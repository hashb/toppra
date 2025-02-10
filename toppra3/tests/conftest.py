import pytest
import numpy as np
from toppra3 import RobotLimits


@pytest.fixture
def two_joint_limits():
    """Fixture providing standard limits for a 2-joint robot"""
    limits = RobotLimits(2)
    limits.max_joint_velocity = np.array([2.0, 2.0])
    limits.max_joint_acceleration = np.array([3.0, 3.0])
    limits.max_joint_jerk = np.array([10.0, 10.0])
    return limits


@pytest.fixture
def three_joint_limits():
    """Fixture providing standard limits for a 3-joint robot"""
    limits = RobotLimits(3)
    limits.max_joint_velocity = np.array([2.0, 2.0, 2.0])
    limits.max_joint_acceleration = np.array([3.0, 3.0, 3.0])
    limits.max_joint_jerk = np.array([10.0, 10.0, 10.0])
    return limits


@pytest.fixture
def simple_waypoints_2d():
    """Fixture providing simple 2D waypoints"""
    return [np.array([0.0, 0.0]), np.array([1.0, 1.0]), np.array([2.0, 2.0])]


@pytest.fixture
def complex_waypoints_3d():
    """Fixture providing more complex 3D waypoints with direction changes"""
    return [
        np.array([0.0, 0.0, 0.0]),
        np.array([1.0, 1.0, 0.5]),
        np.array([0.0, 2.0, 1.0]),
        np.array([-1.0, 1.0, 1.5]),
    ]

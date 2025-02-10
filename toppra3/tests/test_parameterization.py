# import numpy as np
from toppra3 import RobotLimits, Toppra3Parameterization


def test_basic_parameterization():
    """Test basic trajectory parameterization with simple waypoints"""
    num_joints = 2
    limits = RobotLimits(num_joints)

    # Set some reasonable limits using Python lists
    limits.set_max_joint_velocity([2.0, 2.0])
    limits.set_max_joint_acceleration([3.0, 3.0])
    limits.set_max_joint_jerk([10.0, 10.0])

    # Create simple waypoints (straight line in joint space)
    waypoints = [[0.0, 0.0], [1.0, 1.0], [2.0, 2.0]]

    # Create parameterization object and solve
    param = Toppra3Parameterization(num_joints)
    success = param.solve(waypoints, limits, use_jerk_limits=True)

    assert success, "Parameterization failed to find solution"

    # # Get trajectory duration
    # duration = param.get_duration()
    # assert duration > 0, "Duration should be positive"

    # # Sample trajectory at multiple points
    # num_samples = 100
    # times = [i * duration / (num_samples - 1) for i in range(num_samples)]

    # positions = []
    # velocities = []
    # accelerations = []

    # for t in times:
    #     pos, vel, acc = param.get_state(t)
    #     positions.append(pos)
    #     velocities.append(vel)
    #     accelerations.append(acc)

    # # Convert to numpy only for testing/comparison
    # positions = np.array(positions)
    # velocities = np.array(velocities)
    # accelerations = np.array(accelerations)

    # # Test velocity limits
    # vel_violations = np.abs(velocities) > np.array(limits.get_max_joint_velocity())
    # assert not np.any(vel_violations), "Velocity limits violated"

    # # Test acceleration limits
    # acc_violations = np.abs(accelerations) > np.array(limits.get_max_joint_acceleration())
    # assert not np.any(acc_violations), "Acceleration limits violated"

    # # Test jerk limits by finite differencing acceleration
    # dt = times[1] - times[0]
    # jerks = np.diff(accelerations, axis=0) / dt
    # jerk_violations = np.abs(jerks) > np.array(limits.get_max_joint_jerk())
    # assert not np.any(jerk_violations), "Jerk limits violated"

    # # Test boundary conditions
    # np.testing.assert_array_almost_equal(positions[0], waypoints[0], decimal=3)
    # np.testing.assert_array_almost_equal(positions[-1], waypoints[-1], decimal=3)
    # np.testing.assert_array_almost_equal(velocities[0], [0.0] * num_joints, decimal=3)
    # np.testing.assert_array_almost_equal(velocities[-1], [0.0] * num_joints, decimal=3)


# def test_complex_waypoints():
#     """Test parameterization with more complex waypoints"""
#     num_joints = 3
#     limits = RobotLimits(num_joints)

#     # Set limits using Python lists
#     limits.set_max_joint_velocity([2.0, 2.0, 2.0])
#     limits.set_max_joint_acceleration([3.0, 3.0, 3.0])
#     limits.set_max_joint_jerk([10.0, 10.0, 10.0])

#     # Create waypoints with changes in direction
#     waypoints = [[0.0, 0.0, 0.0], [1.0, -1.0, 0.5], [0.5, 1.0, -0.5], [2.0, 0.0, 1.0]]

#     param = Toppra3Parameterization(num_joints)
#     success = param.solve(waypoints, limits, use_jerk_limits=True)

#     assert success, "Parameterization failed to find solution"

#     duration = param.get_duration()
#     num_samples = 200
#     times = [i * duration / (num_samples - 1) for i in range(num_samples)]

#     positions = []
#     velocities = []
#     accelerations = []

#     for t in times:
#         pos, vel, acc = param.get_state(t)
#         positions.append(pos)
#         velocities.append(vel)
#         accelerations.append(acc)

#     # Convert to numpy only for testing/comparison
#     positions = np.array(positions)
#     velocities = np.array(velocities)
#     accelerations = np.array(accelerations)

#     # Test velocity limits
#     vel_violations = np.abs(velocities) > np.array(limits.get_max_joint_velocity())
#     assert not np.any(vel_violations), "Velocity limits violated"

#     # Test acceleration limits
#     acc_violations = np.abs(accelerations) > np.array(limits.get_max_joint_acceleration())
#     assert not np.any(acc_violations), "Acceleration limits violated"

#     # Test jerk limits
#     dt = times[1] - times[0]
#     jerks = np.diff(accelerations, axis=0) / dt
#     jerk_violations = np.abs(jerks) > np.array(limits.get_max_joint_jerk())
#     assert not np.any(jerk_violations), "Jerk limits violated"

#     # Test waypoint interpolation
#     # for waypoint in waypoints:
#     #     # Find closest point on trajectory
#     #     distances = [
#     #         np.linalg.norm(np.array(pos) - np.array(waypoint)) for pos in positions
#     #     ]
#     #     min_dist = min(distances)
#     #     assert (
#     #         min_dist < 0.1
#     #     ), f"Trajectory does not pass close enough to waypoint {waypoint}"


# def test_torque_limits():
#     """Test parameterization with torque limits"""
#     num_joints = 2
#     limits = RobotLimits(num_joints)

#     # Set limits including torque using Python lists
#     limits.set_max_joint_velocity([2.0, 2.0])
#     limits.set_max_joint_acceleration([3.0, 3.0])
#     limits.set_max_joint_jerk([10.0, 10.0])
#     limits.set_max_joint_torque([5.0, 5.0])

#     # Create waypoints with vertical motion (against gravity)
#     waypoints = [[0.0, 0.0], [0.0, 1.0], [0.0, 2.0], [0.0, 1.0], [0.0, 0.0]]

#     param = Toppra3Parameterization(num_joints)
#     success = param.solve(waypoints, limits, use_jerk_limits=True)

#     assert success, "Parameterization failed to find solution"

#     duration = param.get_duration()
#     num_samples = 150
#     times = [i * duration / (num_samples - 1) for i in range(num_samples)]

#     positions = []
#     velocities = []
#     accelerations = []

#     for t in times:
#         pos, vel, acc = param.get_state(t)
#         positions.append(pos)
#         velocities.append(vel)
#         accelerations.append(acc)

#     # Convert to numpy only for testing/comparison
#     positions = np.array(positions)
#     velocities = np.array(velocities)
#     accelerations = np.array(accelerations)

#     # Test velocity limits
#     vel_violations = np.abs(velocities) > np.array(limits.get_max_joint_velocity())
#     assert not np.any(vel_violations), "Velocity limits violated"

#     # Test acceleration limits
#     acc_violations = np.abs(accelerations) > np.array(limits.get_max_joint_acceleration())
#     assert not np.any(acc_violations), "Acceleration limits violated"

#     # Test jerk limits
#     dt = times[1] - times[0]
#     jerks = np.diff(accelerations, axis=0) / dt
#     jerk_violations = np.abs(jerks) > np.array(limits.get_max_joint_jerk())
#     assert not np.any(jerk_violations), "Jerk limits violated"

#     # Verify smooth velocity profile
#     vel_diff = np.diff(velocities, axis=0)
#     assert np.all(np.abs(vel_diff) < 0.5), "Velocity profile is not smooth"

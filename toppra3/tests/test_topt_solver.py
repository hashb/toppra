import pytest
import numpy as np
import toppra3

def test_topt_solver_basic():
    # Create a simple trajectory
    waypoints = np.array([
        [0.0, 0.0],  # start position
        [1.0, 1.0],  # middle position
        [2.0, 0.0],  # end position
    ], dtype=np.float64)
    
    # Create velocity limits
    velocity_limits = np.array([[-1.0, 1.0], [-1.0, 1.0]], dtype=np.float64)
    
    # Create acceleration limits
    acceleration_limits = np.array([[-2.0, 2.0], [-2.0, 2.0]], dtype=np.float64)
    
    # Create solver instance
    solver = toppra3.TOPTSolver()
    
    # Solve the trajectory
    result = solver.solve(waypoints, velocity_limits, acceleration_limits)
    
    # Basic checks
    assert result is not None
    assert isinstance(result, dict)
    assert "success" in result
    assert result["success"]
    
    # Check that the solution contains required fields
    assert "trajectory" in result
    assert "time_points" in result
    assert "velocities" in result
    assert "accelerations" in result
    
    # Check dimensions
    trajectory = result["trajectory"]
    time_points = result["time_points"]
    velocities = result["velocities"]
    accelerations = result["accelerations"]
    
    assert len(time_points) > 0
    assert trajectory.shape[1] == waypoints.shape[1]  # Same number of DOF
    assert velocities.shape[1] == waypoints.shape[1]
    assert accelerations.shape[1] == waypoints.shape[1]

def test_topt_solver_velocity_limits():
    waypoints = np.array([
        [0.0, 0.0],
        [1.0, 1.0],
    ], dtype=np.float64)
    
    # Very restrictive velocity limits
    velocity_limits = np.array([[-0.1, 0.1], [-0.1, 0.1]], dtype=np.float64)
    acceleration_limits = np.array([[-2.0, 2.0], [-2.0, 2.0]], dtype=np.float64)
    
    solver = toppra3.TOPTSolver()
    result = solver.solve(waypoints, velocity_limits, acceleration_limits)
    
    # Check that velocities respect limits
    velocities = result["velocities"]
    assert np.all(velocities[:, 0] >= -0.1 - 1e-10)  # Allow small numerical error
    assert np.all(velocities[:, 0] <= 0.1 + 1e-10)
    assert np.all(velocities[:, 1] >= -0.1 - 1e-10)
    assert np.all(velocities[:, 1] <= 0.1 + 1e-10)

def test_topt_solver_acceleration_limits():
    waypoints = np.array([
        [0.0, 0.0],
        [1.0, 1.0],
    ], dtype=np.float64)
    
    velocity_limits = np.array([[-1.0, 1.0], [-1.0, 1.0]], dtype=np.float64)
    # Very restrictive acceleration limits
    acceleration_limits = np.array([[-0.2, 0.2], [-0.2, 0.2]], dtype=np.float64)
    
    solver = toppra3.TOPTSolver()
    result = solver.solve(waypoints, velocity_limits, acceleration_limits)
    
    # Check that accelerations respect limits
    accelerations = result["accelerations"]
    assert np.all(accelerations[:, 0] >= -0.2 - 1e-10)
    assert np.all(accelerations[:, 0] <= 0.2 + 1e-10)
    assert np.all(accelerations[:, 1] >= -0.2 - 1e-10)
    assert np.all(accelerations[:, 1] <= 0.2 + 1e-10)

def test_topt_solver_invalid_input():
    solver = toppra3.TOPTSolver()
    
    # Test with empty waypoints
    with pytest.raises(ValueError):
        waypoints = np.array([], dtype=np.float64).reshape(0, 2)
        velocity_limits = np.array([[-1.0, 1.0], [-1.0, 1.0]], dtype=np.float64)
        acceleration_limits = np.array([[-2.0, 2.0], [-2.0, 2.0]], dtype=np.float64)
        solver.solve(waypoints, velocity_limits, acceleration_limits)
    
    # Test with mismatched dimensions
    with pytest.raises(ValueError):
        waypoints = np.array([[0.0, 0.0], [1.0, 1.0]], dtype=np.float64)
        velocity_limits = np.array([[-1.0, 1.0]], dtype=np.float64)  # Wrong dimension
        acceleration_limits = np.array([[-2.0, 2.0], [-2.0, 2.0]], dtype=np.float64)
        solver.solve(waypoints, velocity_limits, acceleration_limits)
    
    # Test with invalid limits (min > max)
    with pytest.raises(ValueError):
        waypoints = np.array([[0.0, 0.0], [1.0, 1.0]], dtype=np.float64)
        velocity_limits = np.array([[1.0, -1.0], [-1.0, 1.0]], dtype=np.float64)  # Invalid limits
        acceleration_limits = np.array([[-2.0, 2.0], [-2.0, 2.0]], dtype=np.float64)
        solver.solve(waypoints, velocity_limits, acceleration_limits) 
import cvxpy as cp
import numpy as np
import matplotlib.pyplot as plt

class TrajectoryOptimizer:
    def __init__(self, waypoints, num_points, time_step):
        """
        Initialize the trajectory optimizer.

        :param waypoints: List of waypoints [(x1, y1), (x2, y2), ...]
        :param num_points: Total number of points in the trajectory
        :param time_step: Time step (discrete time period, 1/frequency)
        """
        self.waypoints = np.array(waypoints)  # Waypoints as a numpy array
        self.num_points = num_points  # Total number of points in the trajectory
        self.time_step = time_step  # Time step (1/frequency)

    def optimize_trajectory(self):
        """
        Optimize the trajectory based on the given waypoints and constraints.

        :return: Optimized trajectory points and time array
        """
        # Generate time array for the trajectory
        total_time = self.time_step * (self.num_points - 1)
        time_array = np.linspace(0, total_time, self.num_points)

        # Decision variables for x and y positions
        x = cp.Variable(self.num_points)
        y = cp.Variable(self.num_points)

        # Objective: Minimize the smoothness (second derivative of positions)
        objective = cp.Minimize(
            cp.sum_squares(x[2:] - 2 * x[1:-1] + x[:-2]) +
            cp.sum_squares(y[2:] - 2 * y[1:-1] + y[:-2])
        )

        # Constraints
        constraints = []

        # Ensure trajectory passes through given waypoints at specific indices
        waypoint_indices = np.linspace(0, self.num_points - 1, len(self.waypoints), dtype=int)
        for idx, (wx, wy) in zip(waypoint_indices, self.waypoints):
            constraints.append(x[idx] == wx)
            constraints.append(y[idx] == wy)

        # Solve the optimization problem
        prob = cp.Problem(objective, constraints)
        prob.solve()

        # Extract results
        if prob.status == cp.OPTIMAL:
            positions_x = x.value
            positions_y = y.value
            return time_array, positions_x, positions_y
        else:
            raise ValueError("Optimization failed: " + prob.status)
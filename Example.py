import matplotlib.pyplot as plt
from ObstacleAvoidance import waypoint_generator,convert_obstacles_to_shapely, convert_waypoints
from TrajectoryOptimizer import TrajectoryOptimizer

# Example Usage
if __name__ == "__main__":
    #you can write your own waypoints and obstacles to these list
    #waypoints format [(x1,y1),(x2,y2),....]
    waypoints = [(-7,3),(-2,5),(-1,2),(5,7),(5,2),(3,-3),(-2,-2),(-5,1)]
    #obstacles format [(x1,y1,radius1),(x2,y2,radius2),.....]
    obstacles = [(-5,3,1),(1,3,1),(-1,0,0.5),(3,0,2),(-3,-5,1),(3,6,1)]

    waypoint_generator = waypoint_generator(convert_waypoints(waypoints),convert_obstacles_to_shapely(obstacles),1)
    path = waypoint_generator.generate_waypoints()
    print("Generated Path:", path)

    num_points = 100  # number of points in the trajectory
    time_period = 0.1  # seconds (time step)

    optimizer = TrajectoryOptimizer(path, num_points, time_period)
    time_array, pos_x, pos_y = optimizer.optimize_trajectory()

    print("Optimized X:", pos_x)
    print("Optimized Y:", pos_y)

    # Plot obstacles
    for obs in obstacles:
        circle = plt.Circle((obs[0], obs[1]), obs[2], fill=False, color='r')
        plt.gca().add_artist(circle)

    # Plot original waypoints
    waypoint_x, waypoint_y = zip(*waypoints)
    plt.plot(waypoint_x, waypoint_y, 'o--', label='Original Waypoints')

    # Plot generated safe path
    generated_x, generated_y = zip(*path)
    plt.plot(generated_x, generated_y, linestyle='-', color='orange', label='Generated Path')

    # Plot optimized trajectory
    plt.plot(pos_x, pos_y, 'b-', label='Optimized Trajectory')

    # Plot settings
    plt.axis("equal")
    plt.legend()
    plt.title("Optimized Trajectory with Obstacle Avoidance")
    plt.xlabel("X Coordinate")
    plt.ylabel("Y Coordinate")
    plt.show()

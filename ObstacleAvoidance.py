from shapely.geometry import Point
from shapely.geometry import LineString
import math

#obstacles should be given in the following format: obstalces=[(x1,y1,radius1),(x2,y2,radius2),.....]
def convert_obstacles_to_shapely(obstacles):
    i = 0
    shapely_obstacles = []
    while i < len(obstacles):
        x = obstacles[i][0]
        y = obstacles[i][1]
        radius = obstacles[i][2]
        shapely_obstacles.append(
            Point(x,y).buffer(radius))
        i += 1
    return shapely_obstacles

def convert_waypoints(waypoints):
    wp_list = []
    for wp in waypoints:
        xy_wp = [0,0]
        xy_wp[0] = wp[0]
        xy_wp[1] = wp[1]
        wp_list.append(xy_wp)
    return wp_list

class waypoint_generator:
    # safe_distance can be determined according to requirements of teh vehicle that will be used used. It defines the distance between obstacle and new safe points.
    # safe_distance should be tuned according to the vehicle. safety_coefficient determines how much you want to take risks.
    # after defining safe_distance for a vehicle, made your changes by using safety_coefficient.

    def __init__(self, waypoints, obstacles, safety_coefficient, safe_distance=0.5, turn_angle_check=False):
        self.waypoints = waypoints
        self.obstacles = obstacles
        self.safety_coefficient = safety_coefficient
        self.safe_distance = safe_distance
        self.turn_angle_check = turn_angle_check

# angle between two points
    def point_angle_(self, point1, point2):
        num = point2[1] - point1[1]
        den = point2[0] - point1[0]
        angle = math.atan2(num, den)
        return angle

# distance between two points
    def distance_cartesian(self, point1, point2):
        dx = point1[0] - point2[0]
        dy = point1[1] - point2[1]
        return math.sqrt(dx * dx + dy * dy)

# to find turn angle between waypoints by cosine theorem
    def find_turn_angle(self, point1, point2, point3):
        c = self.distance_cartesian(point1, point3)
        a = self.distance_cartesian(point1, point2)
        b = self.distance_cartesian(point2, point3)
        degree = math.acos(((a * a) + (b * b) - (c * c)) / (2 * a * b))
        return math.degrees(degree)

# to find middle point
    def middle_point(self, point1, point2):
        yeni_nokta = [0, 0]
        yeni_nokta[0] = (point1[0] + point2[0]) / 2
        yeni_nokta[1] = (point1[1] + point2[1]) / 2
        return yeni_nokta

    def find_safe_distance_for_obstacles(self,obstacle):
        obstacle_radius = self.find_radius(obstacle)
        if self.safe_distance >= obstacle_radius:
            distance = obstacle_radius * self.safety_coefficient
        else:
            distance = self.safe_distance * self.safety_coefficient
        return distance

    # there isn't a radius attribure in shapely.
    # to find intersections with obstacles, radius is needed.
    # bound attribute of shapely is used.
    def find_radius(self, obstacle):
        list_bounds = list(obstacle.bounds)
        radius = (list_bounds[2] - list_bounds[0]) / 2
        if radius < 0:
            radius = -radius
        else:
            radius = radius

        return radius
    
    # This will be used to prevent sharp turns. It finds the mirror point according to the center of obstacle.
    def prevent_sharp_turns(self, safe_point, center):
        new_point = [0, 0]
        new_point[0] = (2 * center[0]) - safe_point[0]
        new_point[1] = (2 * center[1]) - safe_point[1]
        return new_point

    def find_new_point_for_one_intersection(self, obstacle_center_coords, intersection, safe_distance, wp1, wp2):
        # Calculate the angle between the obstacle center and the intersection point
        degree_radian = self.point_angle_(obstacle_center_coords, intersection)
        
        # Calculate the new point coordinates directly using sine and cosine
        new_point_x = intersection[0] + (safe_distance * math.cos(degree_radian))
        new_point_y = intersection[1] + (safe_distance * math.sin(degree_radian))
        new_point = [new_point_x, new_point_y]
        
        # Create the updated path
        shapely_path = LineString([wp1, new_point, wp2])
        path_and_new_point = (shapely_path, new_point)
        return path_and_new_point

    def find_new_point_for_two_intersection(self, obstacle_center_coords, intersection_midpoint, obstacle_radius, safe_distance, wp1, wp2):
        # Calculate the angle between the obstacle center and the intersection midpoint
        degree_radian = self.point_angle_(obstacle_center_coords, intersection_midpoint)
        
        # Calculate the total distance from the obstacle center to the new point
        distance_from_center = obstacle_radius + safe_distance
        
        # Calculate the new point coordinates directly using sine and cosine
        new_point_x = obstacle_center_coords[0] + (distance_from_center * math.cos(degree_radian))
        new_point_y = obstacle_center_coords[1] + (distance_from_center * math.sin(degree_radian))
        new_point = [new_point_x, new_point_y]
        
        # Create the updated path
        shapely_path = LineString([wp1, new_point, wp2])
        path_and_new_point = (shapely_path, new_point)
        return path_and_new_point

    def find_intersection(self, wp1, wp2):
        safe_points = []  # List to store all calculated safe points
        center_coords_list = []  # List to store corresponding obstacle centers
        line = LineString([wp1, wp2])  # Line between the two waypoints

        for obstacle in self.obstacles:
            intersection_points = line.intersection(obstacle)
            distance = self.find_safe_distance_for_obstacles(obstacle)

            if intersection_points.is_empty:
                # No intersection with this obstacle
                continue

            try:
                intersection_coords = list(intersection_points.coords)
            except AttributeError:
                # Handle cases where intersection is not a Point or LineString (e.g., GeometryCollection)
                print(f"Unhandled intersection type: {intersection_points.geom_type}")
                continue

            center_coords = list(obstacle.centroid.coords)[0]

            # Iterate through all intersection points
            for i in range(len(intersection_coords) - 1):
                if i == 0 and len(intersection_coords) == 1:
                    # Single intersection (tangential case)
                    path, safe_point = self.find_new_point_for_one_intersection(
                        center_coords, intersection_coords[0], distance, wp1, wp2
                    )
                    safe_points.append(safe_point)
                    center_coords_list.append(center_coords)
                elif i + 1 < len(intersection_coords):
                    # Multiple intersections (general case)
                    intersection1 = intersection_coords[i]
                    intersection2 = intersection_coords[i + 1]
                    midpoint = self.middle_point(intersection1, intersection2)
                    radius = self.find_radius(obstacle)
                    path, safe_point = self.find_new_point_for_two_intersection(
                        center_coords, midpoint, radius, distance, wp1, wp2
                    )
                    safe_points.append(safe_point)
                    center_coords_list.append(center_coords)

        # Select the best safe point if there are multiple
        if len(safe_points) == 0:
            # If no safe points are found, return the direct path
            path = line
        else:
            path = LineString([wp1] + safe_points + [wp2])  # Update the path

        return path, safe_points, center_coords_list

    def generate_waypoints(self):
        i = 0
        n = len(self.waypoints) - 1
        updated_waypoints = [self.waypoints[0]]  # Start with the first waypoint

        while i < n:
            wp1 = self.waypoints[i]
            wp2 = self.waypoints[i + 1]

            # Check for obstacles and find the best path
            path, safe_points, center_coords = self.find_intersection(wp1, wp2)
            print("Center Coords:", center_coords)

            if len(safe_points) != 0:
                print(f"Safe points {safe_points} generated.")

                if self.turn_angle_check:
                    # If mirroring is enabled, evaluate the mirrored points
                    improved_safe_points = []
                    for sp, center in zip(safe_points, center_coords):
                        mirrored_point = self.prevent_sharp_turns(sp, center)
                        
                        if len(updated_waypoints) >= 2:
                            prev_wp = updated_waypoints[-1]
                            prev_prev_wp = updated_waypoints[-2]
                            
                            # Calculate turn angles
                            original_angle = self.find_turn_angle(prev_prev_wp, prev_wp, sp)
                            mirrored_angle = self.find_turn_angle(prev_prev_wp, prev_wp, mirrored_point)

                            # Prefer the point with a safer (larger) turn angle
                            if mirrored_angle > original_angle:
                                improved_safe_points.append(mirrored_point)
                            else:
                                improved_safe_points.append(sp)
                        else:
                            # If not enough waypoints for angle calculation, default to original safe point
                            improved_safe_points.append(sp)

                    safe_points = improved_safe_points

                updated_waypoints.extend(safe_points)  # Add the chosen safe points
            else:
                print("No obstacles detected. Direct path added.")

            # Add the second waypoint (target waypoint)
            updated_waypoints.append(wp2)
            print("--------------------------------------------")
            i += 1

        # Update the self.waypoints with the new waypoint list
        self.waypoints = updated_waypoints
        return updated_waypoints

    # Improved boundary check function
    def boundary_check(self):
        self.waypoints = [
            wp for wp in self.waypoints
            if all(not obstacle.contains(Point(wp)) for obstacle in self.obstacles)
        ]
        print("Waypoints inside obstacles removed.")

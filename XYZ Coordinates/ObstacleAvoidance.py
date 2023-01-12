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

    def __init__(self, waypoints, obstacles, safety_coefficient, safe_distance=0.5):
        self.waypoints = waypoints
        self.obstacles = obstacles
        self.safety_coefficient = safety_coefficient
        self.safe_distance = safe_distance

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

    def find_new_point_for_one_intersection(self, obstacle_center_coords, intersection, safe_distance, wp1, wp2):
        degree_radian = self.point_angle_(obstacle_center_coords, intersection)
        degree = math.degrees(degree_radian)
        if 0 <= degree <= 90:
            new_point_x = intersection[0] + (safe_distance * math.cos(math.radians(degree)))
            new_point_y = intersection[1] + (safe_distance * math.sin(math.radians(degree)))
            new_point = (new_point_x, new_point_y)
        elif 90 <= degree <= 180:
            new_point_x = intersection[0] - (safe_distance * math.cos(math.radians(180 - degree)))
            new_point_y = intersection[1] + (safe_distance * math.sin(math.radians(180 - degree)))
            new_point = (new_point_x, new_point_y)
        elif -180 <= degree <= -90:
            new_point_x = intersection[0] - (safe_distance * math.cos(math.radians(180 + degree)))
            new_point_y = intersection[1] - (safe_distance * math.sin(math.radians(180 + degree)))
            new_point = (new_point_x, new_point_y)

        elif -90 <= degree <= 0:
            new_point_x = intersection[0] + (safe_distance * math.cos(-math.radians(degree)))
            new_point_y = intersection[1] - (safe_distance * math.sin(-math.radians(degree)))
            new_point = (new_point_x, new_point_y)

        shapely_path = LineString([wp1, new_point, wp2])
        path_and_new_point = (shapely_path, new_point)
        return path_and_new_point

    def find_new_point_for_two_intersection(self, obstacle_center_coords, intersection_midpoint, obstalce_radius, safe_distance, wp1, wp2):
        degree_radian = self.point_angle_(obstacle_center_coords, intersection_midpoint)
        degree = math.degrees(degree_radian)
        distance_from_center = obstalce_radius + safe_distance
        if 0 <= degree <= 90:
            new_point_x = obstacle_center_coords[0] + (distance_from_center * math.cos(math.radians(degree)))
            new_point_y = obstacle_center_coords[1] + (distance_from_center * math.sin(math.radians(degree)))
            new_point = (new_point_x, new_point_y)
        elif 90 <= degree <= 180:
            new_point_x = obstacle_center_coords[0] - (distance_from_center * math.cos(math.radians(180 - degree)))
            new_point_y = obstacle_center_coords[1] + (distance_from_center * math.sin(math.radians(180 - degree)))
            new_point = (new_point_x, new_point_y)
        elif -180 <= degree <= -90:
            new_point_x = obstacle_center_coords[0] - (distance_from_center * math.cos(math.radians(180 + degree)))
            new_point_y = obstacle_center_coords[1] - (distance_from_center * math.sin(math.radians(180 + degree)))
            new_point = (new_point_x, new_point_y)
        elif -90 <= degree <= 0:
            new_point_x = obstacle_center_coords[0] + (distance_from_center * math.cos(-math.radians(degree)))
            new_point_y = obstacle_center_coords[1] - (distance_from_center * math.sin(-math.radians(degree)))
            new_point = (new_point_x, new_point_y)

        shapely_path = LineString([wp1, new_point, wp2])
        path_and_new_point = (shapely_path, new_point)
        return path_and_new_point

# Checks whether there is an obstacle between two waypoints. If it there, finds a safe point
    def find_intersection(self, wp1, wp2):
        new_point = 0
        center_coords = 0
        line = LineString([wp1, wp2])
        for obstacle in self.obstacles:
            intersection1 = line.intersection(obstacle)
            distance = self.find_safe_distance_for_obstacles(obstacle)
            if intersection1.is_empty:
                path = line

            elif len(list(intersection1.coords)) == 1:
                list_intersection1 = list(intersection1.coords)
                intersection1_point = list(list_intersection1[0])
                list_bounds = list(obstacle.bounds)
                center = Point([(list_bounds[0] + list_bounds[2]) / 2, (list_bounds[1] + list_bounds[3]) / 2])
                x = list(center.coords)
                center_coords = list(x[0])
                path, new_point = self.find_new_point_for_one_intersection(center_coords, intersection1_point, distance, wp1, wp2,)

            elif len(list(intersection1.coords)) == 2:
                list_bounds = list(obstacle.bounds)
                center = Point([(list_bounds[0] + list_bounds[2]) / 2, (list_bounds[1] + list_bounds[3]) / 2])
                x = list(center.coords)
                center_coords = list(x[0])
                radius = self.find_radius(obstacle)
                list_intersection1 = list(intersection1.coords)
                intersection1_point1 = list(list_intersection1[0])
                intersection1_point2 = list(list_intersection1[1])
                intersection1_mid_point = self.middle_point(intersection1_point1, intersection1_point2)
                path, new_point = self.find_new_point_for_two_intersection(center_coords, intersection1_mid_point, radius, distance, wp1, wp2)

        path_and_new_points = (path, new_point, center_coords)
        return path_and_new_points

# This will be used to prevent sharp turns. It finds the mirror point according to the center of obstacle.
    def prevent_sharp_turns(self, safe_point, center):
        new_point = [0, 0]
        new_point[0] = (2 * center[0]) - safe_point[0]
        new_point[1] = (2 * center[1]) - safe_point[1]
        return new_point

# It removes the waypoints inside the obstacles.
    def boundary_check(self):
        for waypoint in self.waypoints:
            for obstacle in self.obstacles:
                obstacle_bounds = list(obstacle.bounds)
                if obstacle_bounds[0] <= waypoint[0] <= obstacle_bounds[2] and obstacle_bounds[1] <= waypoint[1] <= obstacle_bounds[3]:
                    self.waypoints.remove(waypoint)
                    print("The {wp} point inside the obstacle is removed".format(wp=waypoint))
                    print("--------------------------------------------")

    def generate_waypoints(self):
        i = 0
        x = len(self.waypoints) - 1
        waypoints = [self.waypoints[0]]
        while i < x:
            point1 = self.waypoints[i]
            point2 = self.waypoints[i + 1]
            print("Waypoint{a} ve Waypoint{b}:".format(a=i+1,b=i+2))
            rota, new_point, center = self.find_intersection(point1, point2)
            if new_point != 0:
                print("There is an intersection. New point {a} is found".format(a=new_point))
                print("New point is being checked to prevent sharp turn")
                symetric_point = self.prevent_sharp_turns(new_point, center)
                degree1 = self.find_turn_angle(self.waypoints[i - 1], point1, new_point)
                degree2 = self.find_turn_angle(self.waypoints[i - 1], point1, symetric_point)
                if degree1 >= degree2:
                    waypoints.append(new_point)
                    waypoints.append(point2)
                    print("There isn't a sharp turn, no need to change")
                else:
                    waypoints.append(symetric_point)
                    waypoints.append(point2)
                    print("Sharp turn is prevented. New safe point is {a}".format(a=symetric_point))
            else:
                waypoints.append(point2)
            print("--------------------------------------------")
            i += 1
        return waypoints
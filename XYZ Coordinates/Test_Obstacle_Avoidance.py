import matplotlib.pyplot as plt
from ObstacleAvoidance import waypoint_generator,convert_obstacles_to_shapely, convert_waypoints

#you can write your own waypoints and obstacles to these list
#waypoints format [(x1,y1,z1),(x2,y2,z2),....]
waypoints = [(-7,3,2),(-2,5,2),(-1,2,1),(3,5,1),(5,2,3),(3,-3,3),(-2,-2,2),(-5,1,2)]
#obstacles format [(x1,y1,radius1),(x2,y2,radius2),.....]
obstacles = [(-5,3,1),(1,3,1),(-1,0,0.5),(3,0,2),(-3,-5,1)]

waypoint_generator = waypoint_generator(convert_waypoints(waypoints),convert_obstacles_to_shapely(obstacles),1)
path = waypoint_generator.generate_waypoints()

for obstacle in obstacles:
    print((obstacle[0],obstacle[1]))
    circle = plt.Circle((obstacle[0],obstacle[1]),obstacle[2],fill=False)
    plt.gca().add_artist(circle)

i = 0
while i<len(path)-1:
    plt.plot([path[i][0],path[i+1][0]],[path[i][1],path[i+1][1]])
    i += 1

#you should adjust your axes limits accordingly
plt.axis([-10, 10, -10, 10])
plt.gca().set_aspect(1)
plt.show()

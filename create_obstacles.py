import random
import math
from icecream import ic
from route_opt_utils import RiskZone, CircleObstacle


# generate circle obstacles at random locations within map dimensions, 
# while not covering start or goal
def make_circle_obstacles(start_xy:(int,int),
                          goal_xy:(int,int),
                          mapdimensions:(int,int),
                          obstacleradius:(int,int),
                          obstaclenum:int
                         ) -> list[CircleObstacle]:
    # init
    obstaclelist = []
    # make list of obstacles that do not cover start or goal
    for _ in range(obstaclenum):
        circle = None                       
        startgoal_covered = True                 
        while startgoal_covered:
            # make random circle within map dimensions
            circle = make_random_circle(mapdimensions, obstacleradius)
            obstacle = CircleObstacle((circle[0],circle[1]),circle[2])
            # check if start or goal location is covered by circle
            dist_tostart = math.dist(obstacle.location,start_xy)
            dist_togoal = math.dist(obstacle.location,goal_xy)
            # check if start or goal is covered
            if (dist_tostart <= (obstacle.radius + 10)
                or dist_togoal <= (obstacle.radius + 10)):
                startgoal_covered = True
            else:
                startgoal_covered = False
        # add to list if not covered
        obstaclelist.append(obstacle)
    # return result
    return obstaclelist

def make_circle_riskzones(mapdimensions:(int,int),
                          obstacleradius:(int,int),
                          obstaclenum:int
                         ) -> list[CircleObstacle]:
    # init
    riskzonelist = []
    # make list of obstacles that can cover start or goal
    for _ in range(obstaclenum):
        circle = make_random_circle(mapdimensions, obstacleradius)
        riskzone = CircleObstacle((circle[0],circle[1]),circle[2])
        riskzonelist.append(riskzone)
    # return result
    return riskzonelist

# generate random location for centre of circle within map dimensions 
def make_random_circle(mapdimensions:(int,int),
                     obstacleradius:(int,int)
                    ) -> tuple[int]:
    # init
    mapheight,mapwidth = mapdimensions
    obstacleradius_min,obstacleradius_max = obstacleradius
    # draw random values (# todo klopt nog niet helemaal met dimensions en binnen map)
    x_centre = int(random.uniform(150, mapwidth - 10))         
    y_centre = int(random.uniform(10, mapheight - 10))
    radius = int(random.uniform(obstacleradius_min,obstacleradius_max))
    # return result
    return (x_centre,y_centre,radius)

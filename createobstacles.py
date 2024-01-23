import random
import math
from icecream import ic




# generate circle obstacles at random locations within map dimensions, 
# while not covering start and goal    
def makeCirleObstacles(start_xy, goal_xy, mapdimensions, obstacleradius, obstaclenum) -> list:
    obstaclelist = []
    for _ in range(obstaclenum):
        circle = None                       
        startgoal_covered = True                 
        while startgoal_covered:
            circle = makeRandomCircle(mapdimensions, obstacleradius)
            # check if start or goal location is covered by circle
            dist_tostart = math.dist((circle[0],circle[1]),start_xy)
            dist_togoal = math.dist((circle[0],circle[1]),goal_xy)
            if dist_tostart <= (circle[2] + 10) or dist_togoal <= (circle[2] + 10):
                startgoal_covered = True            # covered
            else:
                startgoal_covered = False           # not covered
        obstaclelist.append(circle)
    #circleobstacles = obstaclelist.copy()    #todo wat hiermee?
    return obstaclelist

def makeRiskZones(mapdimensions, obstacleradius, obstaclenum) -> list:
    riskzonelist = []
    for _ in range(obstaclenum):
        circle = None                       
        circle = makeRandomCircle(mapdimensions, obstacleradius)
        riskzonelist.append(circle)
    return riskzonelist

# generate random location for centre of circle within map dimensions 
def makeRandomCircle(mapdimensions, obstacleradius) -> tuple:
# todo klopt nog niet helemaal met dimensions en binnen map
    mapheight,mapwidth = mapdimensions
    obstacleradius_min,obstacleradius_max = obstacleradius
    x_centre = int(random.uniform(150, mapwidth - 10))         
    y_centre = int(random.uniform(10, mapheight - 10))
    radius = int(random.uniform(obstacleradius_min,obstacleradius_max))
    return (x_centre,y_centre,radius)



# todo - not used for now, file of external circles used laster
class ObstacleCircle:    
    def __init__(self,location,radius) -> None:
        self.location = location
        self.radius = radius
        
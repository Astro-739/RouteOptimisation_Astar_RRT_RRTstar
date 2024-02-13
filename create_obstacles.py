import random
import math
from icecream import ic
from route_opt_utils import RiskZone, CircleObstacle


# generate circle obstacles at random locations within map dimensions, 
# while not covering start or goal
def make_circle_obstacles(start_xy:tuple[int],
                          goal_xy:tuple[int],
                          mapdimensions:tuple[int],
                          obstacleradius:tuple[int],
                          num_obstacles:int
                          ) -> list[CircleObstacle]:
    # init
    obstaclelist = []
    EXTRA_CLEARANCE = 10
    # make list of obstacles that do not cover start or goal
    for _ in range(num_obstacles):
        start_goal_covered = True                 
        while start_goal_covered:
            # make circle obstacle within map dimensions
            circle = make_random_circle(mapdimensions, obstacleradius)
            obstacle = CircleObstacle((circle[0],circle[1]),circle[2])
            # check if start or goal location is covered by circle
            dist_tostart = math.dist(obstacle.location,start_xy)
            dist_togoal = math.dist(obstacle.location,goal_xy)
            if (dist_tostart <= (obstacle.radius + EXTRA_CLEARANCE)
                or dist_togoal <= (obstacle.radius + EXTRA_CLEARANCE)):
                start_goal_covered = True
            else:
                start_goal_covered = False
        # not covered, add to list
        obstaclelist.append(obstacle)
    # return result
    return obstaclelist


def make_circle_riskzones(mapdimensions:tuple[int],
                          obstacleradius:tuple[int],
                          num_obstacles:int
                          ) -> list[CircleObstacle]:
    # init
    riskzonelist = []
    # make list of obstacles that can cover start or goal
    for _ in range(num_obstacles):
        circle = make_random_circle(mapdimensions, obstacleradius)
        riskzone = CircleObstacle((circle[0],circle[1]),circle[2])
        riskzonelist.append(riskzone)
    # return result
    return riskzonelist


def read_circle_riskzones() -> list[CircleObstacle]:
    # issues with startnode
    dict1:dict = {(698, 171): 54, (456, 717): 195, (900, 813): 114, (214, 880): 196, (774, 131): 187, (406, 18): 152, (903, 39): 160, (514, 214): 151, (165, 838): 120, (162, 189): 133, (759, 843): 139, (952, 386): 98}
    dict2:dict = {(192, 51): 199, (609, 738): 104, (555, 791): 139, (661, 663): 162, (672, 78): 127, (582, 768): 179, (951, 588): 120, (736, 740): 94, (161, 346): 162, (839, 757): 184, (266, 158): 63, (685, 565): 75}
    # node = LOSbasenode issue
    dict11:dict = {(306, 633): 191, (206, 532): 67, (282, 350): 179, (880, 677): 160, (987, 952): 186, (159, 720): 182, (806, 90): 115, (333, 34): 174, (518, 60): 100, (814, 709): 192, (678, 438): 145, (169, 304): 139}
    dict12:dict = {(263, 377): 162, (656, 955): 127, (806, 153): 88, (983, 501): 194, (950, 713): 190, (646, 860): 106, (557, 387): 124, (413, 35): 98, (172, 827): 108, (696, 837): 121, (794, 726): 162, (555, 276): 193}
    dict13:dict = {(810, 622): 65, (211, 669): 65, (941, 415): 196, (464, 712): 145, (673, 740): 187, (219, 27): 160, (726, 384): 165, (173, 343): 183, (799, 583): 111, (429, 207): 133, (258, 771): 166, (730, 505): 102}
    # crossing riskzones
    dict21:dict = {(171, 489): 122, (881, 106): 63, (509, 507): 176, (719, 372): 150, (793, 472): 147, (453, 919): 103, (244, 91): 51, (375, 843): 156, (411, 712): 76, (948, 197): 183, (876, 490): 120, (818, 184): 155}
    dict22:dict = {(324, 144): 198, (674, 981): 164, (476, 934): 198, (700, 377): 111, (749, 721): 74, (154, 284): 199, (617, 78): 64, (800, 61): 164, (660, 228): 134, (516, 628): 146, (726, 712): 128, (662, 402): 92}
    dict23:dict = {(957, 896): 178, (276, 51): 179, (500, 819): 84, (451, 702): 134, (557, 603): 171, (780, 433): 167, (942, 348): 125, (541, 98): 125, (270, 352): 191, (353, 392): 124, (247, 665): 196, (788, 241): 80}
    # weights
    dict31:dict = {(844, 483): 79, (829, 542): 90, (242, 875): 92, (156, 946): 189, (417, 591): 198, (401, 91): 70, (578, 567): 90, (315, 350): 90, (710, 428): 75, (910, 821): 119, (415, 545): 128, (476, 530): 195}
    # init
    riskzonelist = []
    for item in dict31.items():
        riskzone = CircleObstacle(item[0],item[1])
        riskzonelist.append(riskzone)
    # return result
    return riskzonelist


# generate random location for centre of circle within map dimensions 
def make_random_circle(mapdimensions:tuple[int],
                       obstacleradius:tuple[int]
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

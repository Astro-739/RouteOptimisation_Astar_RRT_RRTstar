import random
import math
from icecream import ic



class RiskZone:    
    def __init__(self,location:(int,int),radius:(int)) -> None:
        self.location = location
        self.radius = radius
        self.radiusLOS = radius
        

class CircleObstacle:    
    def __init__(self,location:(int,int),radius:(int)) -> None:
        self.location = location
        self.radius = radius
        self.radiusLOS = radius


class SquareObstacle:
    def __init__(self) -> None:
        pass


class TreeNode:
    def __init__(self,location:(int,int)) -> None:
        self.location = location
        self.parent = None
        self.children = []
        self.path_child = None
        self.pathLOS_parent = None
        self.pathLOS_child = None
        self.memberofpath = False
        self.node_cost = 0
        self.path_cost = 0
        self.nodeLOS_cost = 0
        self.pathLOS_cost = 0
        self.neighbourhood = []

    
class TreePath:
    def __init__(self,location:(int,int)) -> None:
        self.location = location
        self.nodes = []            # includes startnode
        self.path_cost = 0


class TreeResults:
    def __init__(self,
                 start_location:(int,int),
                 goal_location:(int,int)
                ) -> None:
        self.start_location = start_location
        self.goal_location = goal_location
        self.randomtree = []        #todo  hoe goed gebruiken?
        self.goalpath = []
        self.goalpathLOS = []


# draw random point within borders of map
def draw_random_sample(mapwidth:int,mapheight:int) -> int:
    x = int(random.uniform(0,mapwidth))        
    y = int(random.uniform(0,mapheight))
    randompoint = (x,y)
    return randompoint


def clear_treepath(path:TreePath) -> None:
    path.nodes = []
    path.path_cost = 0


# determine distance between 2 nodes
def node_distance(node1:TreeNode,node2:TreeNode) -> float: 
    nodedistance = math.dist(node1.location,node2.location)
    return nodedistance


# check if node is in free space or within circle obstacle
def is_freespace_circle(node:TreeNode,
                        obstacles:CircleObstacle,
                        margin:int
                       ) -> bool:
    # check for all obstacles
    for circle in obstacles:
        # collision when point is within circle radius + margin
        if (math.dist((node.location),(circle.location))
            <= (circle.radius + margin)):
            return False
    # no collision detected
    return True


# check if connection crosses a circle obstacle
# todo test formula
def cross_circle_obstacle(node1:TreeNode,
                          node2:TreeNode,
                          obstacles:CircleObstacle,
                          margin:int
                         ) -> bool:
    # check for all obstacles
    for circle in obstacles:
        # divide connection in 100 points to check each
        margin = 0              #!   test, remove
        for i in range(0,101):
            u = i/100                   
            x = node1.location[0] * u + node2.location[0] * (1-u)
            y = node1.location[1] * u + node2.location[1] * (1-u)
            # collision when point is within circle radius + margin
            if math.dist((x,y),(circle.location)) <= (circle.radius + margin):
                return True             
    # no collision detected
    return False


# measure distance of node to every node in tree, return nearest
def nearest_neighbour(node:TreeNode,
                      startnode:TreeNode,
                      randomtree:TreePath,
                      RRTstar:bool
                     ) -> TreeNode:
    # initialise
    d_min = 100000
    nearestnode = startnode
    # find nearest node in random tree
    for neighbournode in randomtree.nodes:
        # nodes already member of a path are not used for neasest neighbour in RRT
        if not RRTstar and neighbournode.memberofpath:
            continue
        # check if neighbour is nearest neighbour
        distance = node_distance(neighbournode,node)
        if distance < d_min:                
            d_min = distance
            nearestnode = neighbournode
    # return nearest neighbour node
    return nearestnode      





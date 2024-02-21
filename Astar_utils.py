import math
from icecream import ic



class GridNode:
    def __init__(self,location:tuple) -> None:
        self.location = location
        self.direction = None
        self.parent = None
        self.goalpath_parent = None
        self.goalpath_child = None
        self.lospath_parent = None
        self.edgelength = 0.0 
        self.edgecost = 100000.0     # edge from parent
        self.f_cost = 100000.0 
        self.g_cost = 100000.0 
        self.h_cost = 100000.0 
        self.lowriskzone_cnt = 0
        self.mediumriskzone_cnt = 0
        self.highriskzone_cnt = 0
        self.riskzones = {}
        #? self.riskzone_lorad = {}    how lorad?? riskzone count lorad?
        self.risk_multiplier = 1


class GridPath:
    def __init__(self,goalnode:GridNode) -> None:
        self.startnode = None
        self.goalnode = goalnode
        self.nodes = []
        self.path_f_cost = 0
        self.riskzones = {}
        self.goal_riskzones = {}
        self.highlightnode = None       # todo  debug
        self.transition_nodes = []


class RiskZone:    
    def __init__(self,location:tuple[int],radius:(int)) -> None:
        self.id = None
        self.location = location
        self.radius = radius
        self.type = None



def create_grid(self) -> None:
    pass

# update node object properties with properties of othernode
def update_node(node:GridNode,othernode:GridNode) -> GridNode:
    #node.location = othernode.location
    node.direction = othernode.direction
    node.parent = othernode.parent
    node.goalpath_parent = othernode.goalpath_parent
    node.goalpath_child = othernode.goalpath_child
    node.lospath_parent = othernode.lospath_parent
    node.edgelength = othernode.edgelength
    node.edgecost = othernode.edgecost
    node.f_cost = othernode.f_cost
    node.g_cost = othernode.g_cost
    node.h_cost = othernode.h_cost
    node.lowriskzone_cnt = othernode.lowriskzone_cnt
    node.mediumriskzone_cnt = othernode.mediumriskzone_cnt
    node.highriskzone_cnt = othernode.highriskzone_cnt
    node.riskzones = othernode.riskzones
    node.risk_multiplier = othernode.risk_multiplier
    return node


# check if is existing gridnode
def is_existing_gridnode(location:tuple[int],gridnodes:list[GridNode]) -> GridNode | None:
    gridnode:GridNode
    for gridnode in gridnodes:
        if gridnode.location == location:
            return gridnode
    return None


# check if node is in free space or within riskzone
def is_freespace(location:tuple[int],riskzones:list[RiskZone],SAFETYMARGIN:int) -> bool:
    # check for all obstacles
    for riskzone in riskzones:
        # collision when point is within circle radius + margin
        if math.dist(location,riskzone.location) < (riskzone.radius + SAFETYMARGIN):
            return False
    # no collision detected
    return True


def is_within_mapdimensions(location:tuple[int],mapdimensions:tuple[int]) -> bool:
    # initialise
    x_map,y_map = location
    mapwidth,mapheight = mapdimensions
    # check within map dimensions
    if x_map < 0 or x_map > mapwidth:
        return False
    if y_map < 0 or y_map > mapheight:
        return False
    return True


# distance from node to closest riskzone edge
def dist_node_riskzone(node:GridNode,riskzones:list[RiskZone]) -> float:
    # init
    dist = []
    # check for all obstacles
    for riskzone in riskzones:
        # distance from node to outer edge of riskzone
        dist.append(math.dist(node.location,riskzone.location) - riskzone.radius)
    return min(dist)
        

# check if connection between 2 nodes crosses a riskzone
def cross_riskzone(node1:GridNode,
                   node2:GridNode,
                   riskzones: RiskZone,
                   goalpath:GridPath,
                   SAFETYMARGIN:int
                   ) -> bool:
    # check for all riskzones
    for riskzone in riskzones:
        # get radius_multiplier
        radius_multiplier = 1.0
        if riskzone.location in goalpath.riskzones:
            radius_multiplier = goalpath.riskzones.get(riskzone.location)
        # check if cross circle
        if cross_circle(node1,node2,riskzone,radius_multiplier,SAFETYMARGIN):
            return True
    # no collision detected
    return False


# check whether a line between two nodes crosses a circle
def cross_circle(node1:GridNode,
                 node2:GridNode,
                 circle:RiskZone,
                 radius_multiplier:float,
                 SAFETYMARGIN: int
                 ) -> bool:
    # circle radius is zero (high risk zone)
    if radius_multiplier == 0.0:
        return False
    # check if line is far enough away from circle to not cross
    line_length = math.dist(node1.location,node2.location)
    dist1 = math.dist(node1.location,circle.location)
    dist2 = math.dist(node2.location,circle.location)
    if ((dist1 or dist2) 
            > (line_length + (circle.radius * radius_multiplier + SAFETYMARGIN))):
        return False
    # when line is close to circle
    # divide connection in 100 points to check each
    for i in range(0,101):
        u = i/100                   
        x = node1.location[0] * u + node2.location[0] * (1-u)
        y = node1.location[1] * u + node2.location[1] * (1-u)
        # collision when point is within circle radius + safety margin
        if (math.dist((x,y),(circle.location)) 
            < ((circle.radius * radius_multiplier + SAFETYMARGIN))):
            return True             
    # no collision detected
    return False

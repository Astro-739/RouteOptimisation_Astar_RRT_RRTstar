import math
import copy
from icecream import ic
from route_opt_utils import CircleObstacle
from Astar_algorithm_multigoal import GridNode, GridPath

        
# smooth final path using Line of Sight (LOS) algorithm
# level 3:  path takes all riskzones into account, 
#           uses path optimisations, such as 
#           transition nodes with local riskzones
#           higher quality results
# check LOS to goal for each node walking from start to goal
# farthest node from goal with LOS becomes node in LOS path
# from this node check LOS for each node from start, etc.
def create_LOS_goalpaths_level3(goalpaths:GridPath,goalfound:bool) -> GridPath | bool:
    # todo check goal found
    if not goalfound: 
        return False
    # create LOS path for each goalpath
    goalpath:GridPath       # todo declare in for statement?
    for goalpath in goalpaths:
        # mark all transitions in goalpath from inside to outside of a riskzone
        # and from outside to inside of a riskzone
        node:GridNode = goalpath.goalnode
        goalpath.transition_nodes.append(goalpath.goalnode)
        while node.location is not goalpath.startnode.location:
            node_risk_multiplier = node.risk_multiplier
            previousnode:GridNode = node.goalpath_parent
            previousnode_risk_multiplier = previousnode.risk_multiplier
            if (node_risk_multiplier > 1 
                    and previousnode_risk_multiplier == 1.0
                    and previousnode.location is not goalpath.startnode.location):
                goalpath.transition_nodes.append(previousnode)
            if (previousnode_risk_multiplier > 1 
                    and node_risk_multiplier == 1.0
                    and node.location is not goalpath.goalnode.location):
                goalpath.transition_nodes.append(node)
            # node moves 1 up the line
            node = previousnode
        # add startnode to transition_nodes
        goalpath.transition_nodes.append(goalpath.startnode)
        # copy list for use later
        transition_nodes_list = goalpath.transition_nodes.copy()
        # LOS checks in sections between transition nodes
        # this gives a better final LOS path
        # use transition nodes list and pop first until walked through entire list 
        while len(transition_nodes_list) > 1:
            LOS_basenode:GridNode = transition_nodes_list.pop(0)
            node = transition_nodes_list[0]
            # find local riskzones, between LOS basenode and node
            LOS_localpath = GridPath(LOS_basenode)
            stepnode:GridNode = LOS_basenode
            while stepnode.location is not node.location:
                # todo update LOS_localpath riskzones
                self.update_gridpath_riskzones(stepnode,LOS_localpath)
                # node moves 1 up the line
                previousnode = stepnode.goalpath_parent
                stepnode = previousnode
            # todo include last node, mainly when node is startnode
            self.update_gridpath_riskzones(stepnode,LOS_localpath)
            # Line of Sight checks
            while LOS_basenode.location is not node.location:
                # if node has LOS with basenode, add it to the LOS path
                # todo node then becomes the next LOS basenode
                if not self.cross_riskzone(node,LOS_basenode,LOS_localpath):
                    # node and LOS basenode become each others parent and child
                    LOS_basenode.LOSpath_parent = node
                    # node becomes next LOSbasenode
                    LOS_basenode = node
                    # LOS checks again from start
                    node = transition_nodes_list[0]
                    continue
                # no LOS, move one node towards LOS basenode over goalpath
                node = node.goalpath_child
                # catch exception
                if node.location == LOS_basenode.location:
                    print("node == LOS_basenode")   # todo
                    break
    # LOS path accessed through LOS path goalnode
    # set LOS path cost
    #?self.pathCostLOS(latestgoalnode)
    print("LOS_goalpaths calculated")
    return goalpaths


# smooth final path using Line of Sight (LOS) algorithm
# level 2:  path takes all riskzones into account, 
#           also path direction inside riskzones covering goal
#           medium quality results
# check LOS to goal for each node walking from start to goal
# farthest node from goal with LOS becomes node in LOS path
# from this node check LOS for each node from start, etc.
def create_LOS_goalpaths_level2(goalpaths:GridPath,goalfound:bool) -> GridPath | bool:
    # check goal found
    if not goalfound: 
        return False
    # latestgoalnode is latest new node at goal location, becomes basenode
    goalpath:GridPath       # todo declare in for statement?
    for goalpath in goalpaths:
        # find first node outside of sams covering target
        outsidenode:GridNode = goalpath.goalnode
        while outsidenode.risk_multiplier > 1:
            # sams covering goalnode are set to fully evade as riskzone
            {goalpath.riskzones.pop(zone) for zone in outsidenode.riskzones if zone in goalpath.riskzones}
            outsidenode = outsidenode.goalpath_parent
        # in some cases the goalpath re-enters sams covering goalnode later
        # these riskzones have to be put back in the goalpath riskzones
        node:GridNode = outsidenode
        while node.location is not goalpath.startnode.location:
            # update goalpath riskzones
            self.update_gridpath_riskzones(node,goalpath)
            # node moves 1 up the line
            previousnode = node.goalpath_parent
            node = previousnode
        # set outside node as first LOS path node after goalnode
        goalpath.goalnode.LOSpath_parent = outsidenode
        goalpath.highlightnode = outsidenode    # todo debug
        # start LOS checks at start node walking towards LOS basenode
        node = goalpath.startnode
        LOS_basenode:GridNode = outsidenode
        while LOS_basenode.location is not goalpath.startnode.location:
            # if node has LOS with basenode, add it to the LOS path
            # node then becomes the next LOS basenode
            if not self.cross_riskzone(node,LOS_basenode,goalpath):
                # node and LOS basenode become each others parent and child
                LOS_basenode.LOSpath_parent = node
                # node becomes next LOSbasenode
                LOS_basenode = node
                # LOS checks again from start
                node = goalpath.startnode
                continue
            # no LOS, move one node towards LOS basenode over goalpath
            node = node.goalpath_child
            # catch exception
            if node.location == LOS_basenode.location:
                print("node == LOS_basenode")   # todo
                break
    # LOS path accessed through LOS path goalnode
    # set LOS path cost
    #?self.pathCostLOS(latestgoalnode)
    print("LOS_goalpaths calculated")
    return goalpaths
    

# smooth final path using Line of Sight (LOS) algorithm
# level 1:  path takes all riskzones into account, 
#           except path direction inside riskzones covering goal
#           simplest code, but lower quality results
# check LOS to goal for each node walking from start to goal
# farthest node from goal with LOS becomes node in LOS path
# from this node check LOS for each node from start, etc.
def create_LOS_goalpaths_level1(goalpaths:GridPath,goalfound:bool) -> GridPath | bool:
    # check goal found
    if not goalfound: 
        return False
    # latestgoalnode is latest new node at goal location, becomes basenode
    goalpath:GridPath       # todo declare in for statement?
    for goalpath in self.goalpaths:
        # start LOS checks at start node walking towards LOS basenode
        node:GridNode = goalpath.startnode
        LOS_basenode:GridNode = goalpath.goalnode
        while LOS_basenode.location is not goalpath.startnode.location:
            # if node has LOS with basenode, add it to the LOS path
            # node then becomes the next LOS basenode
            if not self.cross_riskzone(node,LOS_basenode,goalpath):
                # node and LOS basenode become each others parent and child
                LOS_basenode.LOSpath_parent = node
                # node becomes next LOSbasenode
                LOS_basenode = node
                # LOS checks again from start
                node = goalpath.startnode
                continue
            # move one node towards LOS basenode over goalpath
            node = node.goalpath_child
            # catch exception
            if node.location == LOS_basenode.location:
                print("node == LOS_basenode")   # todo
                break
    # LOS path accessed through LOS path goalnode
    # set LOS path cost
    #?self.pathCostLOS(latestgoalnode)
    print("LOS_goalpaths calculated")
    return goalpaths

# check if connection crosses a riskzone for 2 nodes
def cross_riskzone(self,node1:GridNode,node2:GridNode,goalpath:GridPath) -> bool:
    # check for all obstacles
    for circle in self.obstacles:
        # get radius_multiplier
        radius_multiplier = 1.0
        if circle.location in goalpath.riskzones:
            radius_multiplier = goalpath.riskzones.get(circle.location)
        # check if cross circle
        if self.cross_circle(node1,node2,circle,radius_multiplier):
            return True
    # no collision detected
    return False

# check whether a line between two nodes crosses a circle
def cross_circle(self,
                    node1:GridNode,
                    node2:GridNode,
                    circle:CircleObstacle,
                    radius_multiplier:float
                    ) -> bool:
    # circle radius is zero
    if radius_multiplier == 0.0:
        return False
    # check if line is far enough away from circle to not cross
    line_length = math.dist(node1.location,node2.location)
    dist1 = math.dist(node1.location,circle.location)
    dist2 = math.dist(node2.location,circle.location)
    if ((dist1 or dist2) 
            > (line_length + (circle.radius * radius_multiplier + self.SAFETYMARGIN))):
        return False
    # when line is close to circle
    # divide connection in 100 points to check each
    for i in range(0,101):
        u = i/100                   
        x = node1.location[0] * u + node2.location[0] * (1-u)
        y = node1.location[1] * u + node2.location[1] * (1-u)
        # collision when point is within circle radius + safety margin
        if (math.dist((x,y),(circle.location)) 
            < ((circle.radius * radius_multiplier + self.SAFETYMARGIN))):
            return True             
    # no collision detected
    return False

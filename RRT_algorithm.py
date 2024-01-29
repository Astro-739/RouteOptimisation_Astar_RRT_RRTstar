import random
import math
from icecream import ic
from route_opt_utils import TreeResults, TreeNode, TreePath
from route_opt_utils import draw_random_sample, clear_treepath, node_distance
from route_opt_utils import is_freespace_circle, cross_circle_obstacle, nearest_neighbour



# ----------------------------------------------------------
class RRTAlgorithm:          
    def __init__(self,mapdimensions:(int,int),obstacles:list,start_location:(int,int),
                 goal_location:(int,int),RRTStar:bool) -> None:
        # map dimensions (note: y,x or height,width in pygame)
        self.mapdimensions = mapdimensions
        self.mapheight,self.mapwidth = self.mapdimensions
        # Node class
        self.startnode = TreeNode(start_location)
        self.goalnode = TreeNode(goal_location)
        self.randomtree = TreePath(start_location)
        self.randomtree.nodes.append(self.startnode)
        self.goalpath = TreePath(start_location)
        self.goalpath.nodes.append(self.startnode)
        self.goalpathLOS = TreePath(start_location)
        # algorithm settings
        self.STEPSIZE = 40               # ? option as optional parameter in function
        self.GOALRADIUS = 15
        self.SAFETYMARGIN = 2           # extra clearance distance from obstacle
        self.goalfound = False               
        self.pathfound = False
        # obstacles
        self.obstacles = obstacles
        # results
        self.finalresult = TreeResults(start_location,goal_location)
        # RRT Star
        self.RRTstar = RRTStar
        self.rewire = True
        self.GAMMA = 200    #todo temp value
        # debug
        self.count = 0


    # add node to tree, set parent and cost
    def addTreeNode(self,newnode:TreeNode,parentnode:TreeNode) -> None:
        # add parent to new node
        newnode.parent = parentnode
        # set node cost
        if self.RRTstar:
            self.nodeCost_RRTstar(newnode,parentnode)
        else:
            self.nodeCost(newnode)
        # add child to parent node
        parentnode.children.append(newnode)
        # add to all nodes of entire tree
        self.randomtree.nodes.append(newnode)
    
    # determine node cost and path cost for a node
    def nodeCost(self,node:TreeNode) -> None:
        # set node cost
        node.node_cost = self.STEPSIZE
        # set path cost to this node
        node.path_cost = node.node_cost + node.parent.path_cost

    # determine node cost and path cost for a node
    def nodeCost_RRTstar(self,node:TreeNode,parentnode:TreeNode) -> None:
        # set node cost
        node.node_cost = node_distance(node,parentnode)
        # set path cost to this node
        path_cost = node.node_cost
        tempnode = node
        while tempnode is not self.startnode:
            path_cost = path_cost + tempnode.parent.node_cost
            tempnode = tempnode.parent
        # set path_cost correctly for each node in path
        tempnode = node
        while tempnode is not self.startnode:
            tempnode.path_cost = path_cost
            path_cost = path_cost - tempnode.node_cost
            tempnode = tempnode.parent


    # determine cost of LOS path and LOS nodes
    def pathCostLOS(self,goalnode:TreeNode) -> None:
        # set node cost
        LOSnode = goalnode
        while LOSnode.location is not self.startnode.location:
            LOSnode.nodeLOS_cost = int(node_distance(LOSnode,LOSnode.pathLOS_parent))
            print("LOSnode_cost: ",LOSnode.nodeLOS_cost)
            LOSnode = LOSnode.pathLOS_parent
        # set path cost
        LOSnode = goalnode
        while LOSnode.location is not self.startnode.location:
            goalnode.pathLOS_cost += LOSnode.nodeLOS_cost
            LOSnode = LOSnode.pathLOS_parent

    # RRT* - determine radius of neighbourhood for rewiring
    # radius = GAMMA * (log(n)/n)^(1/2)
    def neighbourhoodRadius(self) -> float:
        num_nodes = float(len(self.randomtree.nodes))
        radius = self.GAMMA * math.sqrt(math.log(num_nodes) / num_nodes)
        if radius < 2 * self.STEPSIZE:
            radius = 2 * self.STEPSIZE     #todo  find better value
        return radius

    # RRT* - find all nodes in own neighbourhood
    def neighbourhoodNodes(self,node:TreeNode) -> list:
        # initialise
        neighbourhoodradius = self.neighbourhoodRadius()
        neighbourhood = []
        # check if neighbours are within neighbourhood for all treenodes
        for treenode in self.randomtree.nodes:
            distance = node_distance(treenode,node)
            if distance < neighbourhoodradius:
                neighbourhood.append(treenode)
        # update node
        node.neighbourhood = neighbourhood


    # RRT* - measure own path cost through neighbouring nodes, return lowest
    #todo check crossobstacle for neighbour??
    def lowestCostNeighbour(self,node:TreeNode,nearestnode:TreeNode) -> TreeNode:
        # initialise
        lowestcostneighbour = nearestnode
        pathcost_min = self.mapheight + self.mapwidth
        # determine neighbourhood size and nodes within neighbourhood
        self.neighbourhoodNodes(node)
        # find lowest cost neighbour
        for neighbournode in node.neighbourhood:
            # new path cost if node is connected with neighbour
            newpathcost = neighbournode.path_cost + node_distance(node,neighbournode)
            if newpathcost < pathcost_min:
                lowestcostneighbour = neighbournode
                pathcost_min = newpathcost
        return lowestcostneighbour
            
    # RRT* - rewire tree around node based on cost
    # check neighbourhood if path cost is lower when node is their parent, rewire
    # todo looping occurs sometimes
    def rewireTree(self,node:TreeNode) -> None:
        # xx
        for neighbournode in node.neighbourhood:
            # -
            newpathcost = node.path_cost + node_distance(node,neighbournode)
            if newpathcost < neighbournode.path_cost:
                neighbournode.parent = node
                neighbournode.path_cost = newpathcost


    # generate new node between target node and nearest node with step size
    # when new node is within goal area, goal node location is used
    def generateStep(self,nearestnode:TreeNode,targetnode:TreeNode) -> bool:                     
        dist = node_distance(nearestnode,targetnode)
        # for the case where random picked the location of the nearest node exactly
        if dist == 0:
            stepnode = None
            return False
        # take step with STEPSIZE in direction of targetnode and create new node
        # x1 = x0 + step/dist * delta_x
        else:
            x_step = nearestnode.location[0] + int((self.STEPSIZE / dist)
                        * (targetnode.location[0] - nearestnode.location[0]))
            y_step = nearestnode.location[1] + int((self.STEPSIZE / dist)
                        * (targetnode.location[1] - nearestnode.location[1]))
            stepnode = TreeNode((x_step,y_step))
        # when step is within goal area, stepnode will have location of goalnode
        if node_distance(stepnode,self.goalnode) <= (self.STEPSIZE + self.GOALRADIUS):
            stepnode.location = self.goalnode.location
            self.goalfound = True          # todo, check juiste plek, moet ook nog toevoegen als node in list
            print("goal found")
        # if step node not in free space, return false
        if not is_freespace_circle(stepnode,self.obstacles,self.SAFETYMARGIN):
            return False
        if self.RRTstar and not self.goalfound:
            parentnode = self.lowestCostNeighbour(stepnode,nearestnode)
        else:
            parentnode = nearestnode
        # if connection of step node crosses obstacle, return false
        if cross_circle_obstacle(parentnode,stepnode,self.obstacles,self.SAFETYMARGIN):
            return False
        # add new node to tree
        self.addTreeNode(stepnode,parentnode)
        if self.RRTstar and self.rewire and not self.goalfound:
            self.rewireTree(stepnode)
        if self.goalfound:         #!
            self.pathToGoal(stepnode)
        return True
        

    # expand tree in direction of random point    
    def randomStep(self) -> None:
        free_randompoint = False
        while_loops = 0
        # generate random target in free space   #todo  deze weghalen?
        while (not free_randompoint) and while_loops < 20:
            randomnode = TreeNode(draw_random_sample(self.mapwidth,self.mapheight))  
            free_randompoint = is_freespace_circle(randomnode,self.obstacles,self.SAFETYMARGIN)        
            while_loops += 1
            if while_loops == 20: print(f"random point generation stopped, none in free space")
        # make step with STEPSIZE from nearest node in direction of random point
        nearestnode = nearest_neighbour(randomnode,self.startnode,self.randomtree,self.RRTstar)
        self.generateStep(nearestnode,randomnode)

    # expand tree in direction of goal    
    def biasStep(self) -> None:
        # goalnode is target
        targetnode = self.goalnode
        nearestnode = nearest_neighbour(targetnode,self.startnode,self.randomtree,self.RRTstar)
        # generate step
        self.generateStep(nearestnode,targetnode)
        
    # build path to goal if goal has been found
    def pathToGoal(self,latestgoalnode:TreeNode) -> bool:
        if not self.goalfound: 
            return False
        # initialise
        clear_treepath(self.goalpath)
        # reconstructing the path backwards from goal using parents
        node = latestgoalnode
        while node is not self.startnode:
            self.goalpath.nodes.insert(0,node)
            # set memeberofpath, excluded in later nearest node searches
            node.memberofpath = True
            # previousnode moves 1 up the line through parent of node
            previousnode = node.parent
            # set path_parent and path_child 
            # will be overwritten if node is member of next goal path
            node.path_parent = previousnode
            previousnode.path_child = node
            # parent node moves 1 up the line
            node = previousnode
        # goalpath own location is start_location, so startnode is not added #todo still true?
        # set exclusion zone for nearest neighbour in later nearest node searches
        if not self.RRTstar:
            self.setExclusionZone()      #todo   test
        # add goalpath to final results
        #?resultlist = [node.location for node in self.goalpath.nodes]
        self.finalresult.goalpath.append(latestgoalnode)
        print("goalpath calculated, path cost: ",latestgoalnode.path_cost)
        # also determine LOS goal path
        if not self.RRTstar:
            self.pathToGoalLOS(latestgoalnode)       #!
        self.pathfound = True
        self.goalfound = False
        return True
    
    # set exclusion zone for nearest node searches
    #todo temp value, funtion of closest obstacle to start?
    # around each path node an exclusion radius that decreases in size towards start?
    def setExclusionZone(self) -> None:
        exclusionradius = 200        #!
        for node in self.randomtree.nodes:
            if node_distance(node,self.goalnode) < exclusionradius:
                node.memberofpath = True


    # reconstructing the goal LOS path with base at goal location
    # checking for LOS from start to goal and moving base to farthest LOS node
    def pathToGoalLOS(self,latestgoalnode:TreeNode) -> bool:
    # smooth final path using Line of Sight algorithm
        # check LOS from goal to each node from start to goal    #todo update description
        # farthest node with LOS becomes node in LOS path
        # from this node check LOS for each node from start, etc.
        # add path to pathLOS result
        if not self.goalfound: 
            return False
        clear_treepath(self.goalpathLOS)    #todo, goalpathLOS used anywhere?
        # latestgoalnode is latest new node at goal location, becomes basenode
        LOSbasenode = latestgoalnode
        # start LOS checks at start node walking towards LOS basenode
        node = self.startnode
        while LOSbasenode.location is not self.startnode.location:
            # if node has LOS with basenode, add it to the LOS path
            # node then becomes the next LOS basenode
            if not cross_circle_obstacle(node,LOSbasenode,self.obstacles,self.SAFETYMARGIN):
                newLOSnode = TreeNode(node.location)
                #todo add node to goalpathLOS?
                # node and LOS basenode become each others parent and child
                LOSbasenode.pathLOS_parent = newLOSnode
                newLOSnode.pathLOS_child = LOSbasenode
                # newLOSnode become next LOSbasenode
                LOSbasenode = newLOSnode
                # LOS checks again from start
                node = self.startnode
                continue
            # move one node towards LOS basenode over goalpath
            node = node.path_child
        # LOS path accessed through path goalnode
        # set LOS path cost
        self.pathCostLOS(latestgoalnode)
        print("goalpathLOS calculated, cost: ",latestgoalnode.pathLOS_cost)
        return True
    

import math
import copy
from icecream import ic
from route_opt_utils import CircleObstacle



class GridNode:
    def __init__(self,location:tuple) -> None:
        self.location = location
        self.direction = None
        self.parent = None
        self.goalpath_parent = None
        self.goalpath_child = None
        self.LOSpath_parent = None
        self.edgelength = 0.0 
        self.edgecost = 100000.0     # edge from parent
        self.f_cost = 100000.0 
        self.g_cost = 100000.0 
        self.h_cost = 100000.0 
        self.lowriskzone_cnt = 0
        self.mediumriskzone_cnt = 0
        self.highriskzone_cnt = 0
        self.riskzones = {}
        self.risk_multiplier = 1


class GridPath:
    def __init__(self,goalnode:GridNode) -> None:
        self.startnode = None
        self.goalnode = goalnode
        self.nodes = []
        self.path_f_cost = 0
        self.riskzones = {}
        self.goal_riskzones = {}        # todo  how to use?
        self.highlightnode = None       # todo  debug
        self.transition_nodes = []

#? double stepsize when far from riskzone
#? store direction (eg. NW) for node to use later for waypoint reduction
#? remove outside node and check for better result
#? calc LOS path cost or assume same as goalpath?
#? explain extra_clearance in create obstacles
#? move from obstacles to riskzones
#? make provisions for LORAD 
#? branch out to 4 "start nodes"

class AStarAlgorithm:
    def __init__(self,
                 start_locations:list[int],
                 goal_locations:list[int],
                 obstacles:list[CircleObstacle],
                 mapdimensions:tuple[int],
                 stepsize:int
                 ) -> None:
        self.start_locations = start_locations
        self.goal_locations = goal_locations
        self.obstacles = obstacles
        self.mapheight,self.mapwidth = mapdimensions
        self.STEPSIZE = stepsize
        # safety margin to prevent a path touching the riskzone
        self.SAFETYMARGIN = 5
        # init node lists
        self.openlist = []
        self.closedlist = []
        self.gridnodes = []
        self.goalpaths = []
        self.goalfound = False
        # init rootnode
        # root node is the basis for multiple startnodes
        self.root_location = (-30,500)
        self.rootnode = GridNode(self.root_location)
        self.rootnode.edgecost = 0.0
        self.rootnode.g_cost = 0.0
        self.set_node_riskzones(self.rootnode)
        self.set_node_riskmultiplier(self.rootnode)
        # init goalnodes
        self.goalnodes = []
        for location in self.goal_locations:
            goalnode = GridNode(location)
            self.set_node_riskzones(goalnode)
            self.set_node_riskmultiplier(goalnode)
            self.goalnodes.append(goalnode)
            self.gridnodes.append(goalnode)
            
    def astar_search(self) -> None:
        # create startnode, goalnode
        # initialise openlist and closedlist
        # put startnode on openlist
        # while openlist not empty:
        # set q-node with lowest f
        # pop q-node off openlist
        # generate children, set parent
        # for each child:
        # set f/g/h cost
        # check if is goal
        # put children in openlist, except:
        #   - is q location
        #   - openlist already has same node with lower f cost
        #   - closedlist already has same node with lower f cost
        # end for loop
        # put q node in closed list
        # end while loop
        
        # add startnodes to openlist as first nodes
        #for startnode in self.startnodes:
        self.openlist.append(self.rootnode)

        iteration = 0
        goalnode_list = self.goalnodes.copy()
        
        while len(self.openlist) > 0 and not self.goalfound:
            # update iteration
            iteration += 1
            # sort openlist on lowest f_cost
            self.openlist.sort(key=lambda x: x.f_cost)
            # qnode is node with lowest f_cost, pop from openlist
            qnode:GridNode = self.openlist.pop(0)
            # generate new gridpoints
            # if conditions are met, child nodes are generated on the gridpoints
            new_children = []
            step_multiplier = 1
            #dist = self.dist_node_riskzone(qnode)
            #if dist > 5 * self.STEPSIZE:
            #    step_multiplier = 2
            grid_step = step_multiplier * self.STEPSIZE
            # first iteration generate grispoints for start locations
            # otherwise generate new gridpoints in all 8 directions N,NE,E,SE,S,SW,W,NW
            if iteration == 1:
                new_gridpoints = self.new_gridpoints_start()
            else:
                new_gridpoints = self.new_gridpoints_NESW(qnode,grid_step)
            # for new gridpoints check if node already exists on location
            # check if existing node needs updates
            # or create new node if no node exists
            # check if goal found at new location
            for location_and_edge in new_gridpoints:
                # new location and edge of gridpoint
                new_location = (location_and_edge[0],location_and_edge[1])
                new_edge = location_and_edge[2]
                # if not within map dimenstions, continue to next gridpoint
                if not self.is_within_mapdimensions(new_location):
                    continue
                # create tempnode at gridpoint
                tempnode = GridNode(new_location)
                tempnode.edgelength = new_edge
                # set riskzone count and multiplier for tempnode
                self.set_node_riskzones(tempnode)
                self.set_edge_riskzones(tempnode,qnode)
                self.set_node_riskmultiplier(tempnode)
                # calculate costs based on risk multiplier
                self.calc_edge_cost(tempnode)
                self.calc_g_cost(tempnode,qnode)
                self.calc_h_cost(tempnode)
                self.calc_f_cost(tempnode)
                # check for existing node on this location
                # if node exists, it is on openlist or closedlist
                node:GridNode = self.existing_gridnode(new_location)
                # if existing node and f_cost lower, leave as is, go to next point
                if node is not None and node.f_cost < tempnode.f_cost:
                    continue
                # if not existing, make new node
                if node is None:
                    node = tempnode
                    self.gridnodes.append(node)
                    new_children.append(node)
                # update node if new or if existing and higher f_cost
                node.parent = qnode
                node.edgecost = tempnode.edgecost
                node.f_cost = tempnode.f_cost
                node.g_cost = tempnode.g_cost
                node.h_cost = tempnode.h_cost
                # check if goal found and update goalnode_list
                goalnode_list = self.check_goal_found(node,goalnode_list)
                # set goal found flag true when all goals found
                if len(goalnode_list) == 0:
                    self.goalfound = True
                    print("all goals found")
            # put all new nodes on openlist
            for child in new_children:
                self.openlist.append(child)
            # qnode goes to closedlist
            self.closedlist.append(qnode)
        # print final status
        print(f"final openlist items: {len(self.openlist)}")
        print(f"final closedlist items: {len(self.closedlist)}")

    # generate new gridpoints in all 8 directions N,NE,E,SE,S,SW,W,NW
    def new_gridpoints_NESW(self,qnode:GridNode,grid_step:int) -> list:
        # init
        new_gridpoints = []
        # new locations
        location_and_edge_north     = (qnode.location[0], 
                                       qnode.location[1] + grid_step, 
                                       grid_step)
        location_and_edge_northeast = (qnode.location[0] + grid_step, 
                                       qnode.location[1] + grid_step, 
                                       math.sqrt(2) * grid_step)
        location_and_edge_east      = (qnode.location[0] + grid_step, 
                                       qnode.location[1], 
                                       grid_step)
        location_and_edge_southeast = (qnode.location[0] + grid_step, 
                                       qnode.location[1] - grid_step, 
                                       math.sqrt(2) * grid_step)
        location_and_edge_south     = (qnode.location[0], 
                                       qnode.location[1] - grid_step, 
                                       grid_step)
        location_and_edge_southwest = (qnode.location[0] - grid_step, 
                                       qnode.location[1] - grid_step, 
                                       math.sqrt(2) * grid_step)
        location_and_edge_west      = (qnode.location[0] - grid_step, 
                                       qnode.location[1], 
                                       grid_step)
        location_and_edge_northwest = (qnode.location[0] - grid_step, 
                                       qnode.location[1] + grid_step, 
                                       math.sqrt(2) * grid_step)
        # add values to list
        new_gridpoints.append(location_and_edge_north)
        new_gridpoints.append(location_and_edge_northeast)
        new_gridpoints.append(location_and_edge_east)
        new_gridpoints.append(location_and_edge_southeast)
        new_gridpoints.append(location_and_edge_south)
        new_gridpoints.append(location_and_edge_southwest)
        new_gridpoints.append(location_and_edge_west)
        new_gridpoints.append(location_and_edge_northwest)
        # return new gridpoints
        return new_gridpoints

    # generate new gridpoints for multiple start locations in first iteration
    def new_gridpoints_start(self) -> list:
        # init
        new_gridpoints = []
        # start locations
        for start_location in self.start_locations:
            location_and_edge = (start_location[0],start_location[1],0)
            # add values to list
            new_gridpoints.append(location_and_edge)
        # return new gridpoints
        return new_gridpoints



    # edge cost to get to point
    # edge cost based on distance and risk multiplier
    # risk multiplier is based on gridpoint location, all edges to point same multiplier
    def calc_edge_cost(self,node:GridNode) -> None:
        node.edgecost = node.edgelength * node.risk_multiplier

    # total path predicted cost (f_cost = g_cost + h_cost)
    def calc_f_cost(self,node:GridNode) -> None:
        node.f_cost = node.g_cost + node.h_cost
    
    # path cost from start to current point
    def calc_g_cost(self,node:GridNode,parentnode:GridNode) -> None:
        node.g_cost = parentnode.g_cost + node.edgecost
    
    # multiple goal heuristic for predicted path cost
    def calc_h_cost(self,node:GridNode):
        # diagonal distance h = D * ((dx + dy) + (sqrt(2) - 2) * min(dx, dy))
        # using Euclidian distance for now (diagonal distance not very distinct in this grid)
        node.h_cost = min([math.dist(node.location,location) for location in self.goal_locations])
    
    def check_goal_found(self,node:GridNode,goalnode_list:list[GridNode]) -> list[GridNode]:
        # radius goal found
        GOALRADIUS = 2 * self.STEPSIZE
        goalnode:GridNode
        # check goal found for each goal location left on goalnode_list
        for goalnode in goalnode_list:
            dist = math.dist(node.location,goalnode.location)
            # goal found when node is closer than goalradius from goalnode
            if dist > 0 and dist < GOALRADIUS:
                # node becomes parent of goalnode
                goalnode.parent = node
                goalnode.edgelength = dist
                self.set_edge_riskzones(node,goalnode)
                # calculate costs for goalnode based on risk multiplier
                self.calc_edge_cost(goalnode)
                self.calc_g_cost(goalnode,node)
                goalnode.h_cost = 0
                self.calc_f_cost(goalnode)
                # remove found goal from list
                goalnode_list.remove(goalnode)
            # case for when node is exactly on goalnode location
            if dist == 0: 
                # goalnode gets all attributes from node
                goalnode = node
                # remove found goal from list
                goalnode_list.remove(goalnode)
        # return updated goalnode_list
        return goalnode_list

    def create_grid(self) -> None:
        pass
    
    def update_node(self) -> None:
        pass
    
    # determine if node is in riskzone (low, medium, high)
    # note: safetymargin is connected to safetymargin in cross_circle()
    def set_node_riskzones(self,node:GridNode) -> None:
        # risk radius multiplier settings
        # low risk:    1.0 - 0.8 
        # medium risk: 0.8 - 0.5 
        # high risk:   0.5 - 0.0
        LOWRISK_RADIUS_MULTIPLIER = 0.8         # todo set as global?
        MEDIUMRISK_RADIUS_MULTIPLIER = 0.5
        HIGHRISK_RADIUS_MULTIPLIER = 0.0
        # check for all obstacles
        for riskzone in self.obstacles:
            # distance between node location and centre of circle
            dist = math.dist(node.location,riskzone.location)
            # if outside cirle, continue
            if dist > riskzone.radius + self.SAFETYMARGIN:
                continue
            # if in highrisk zone
            if dist < MEDIUMRISK_RADIUS_MULTIPLIER * riskzone.radius + self.SAFETYMARGIN:
                node.riskzones.update({riskzone.location:HIGHRISK_RADIUS_MULTIPLIER})
                continue
            # if in mediumrisk zone
            if dist < LOWRISK_RADIUS_MULTIPLIER * riskzone.radius + self.SAFETYMARGIN:
                node.riskzones.update({riskzone.location:MEDIUMRISK_RADIUS_MULTIPLIER})
                continue
            # else in lowrisk zone
            node.riskzones.update({riskzone.location:LOWRISK_RADIUS_MULTIPLIER})

    # risk_multiplier is used to increase edgecost (edgecost = edgelength * risk_multiplier)
    def set_node_riskmultiplier(self,node:GridNode) -> None:
        # initialise
        risk_multiplier = 0
        # risk values translate to costs 
        # which are used by the A* algorithm to determine the lowest cost route
        LOWRISK_VALUE = 5
        MEDIUMRISK_VALUE = 25
        HIGHRISK_VALUE = 50
        # set risk count
        node.lowriskzone_cnt = len([(node.riskzones[key]) for key in [*node.riskzones.keys()] if node.riskzones[key] == 0.8])
        node.mediumriskzone_cnt = len([(node.riskzones[key]) for key in [*node.riskzones.keys()] if node.riskzones[key] == 0.5])
        node.highriskzone_cnt = len([(node.riskzones[key]) for key in [*node.riskzones.keys()] if node.riskzones[key] == 0.0])
        # determine risk_multiplier
        if node.lowriskzone_cnt > 0:
            risk_multiplier = risk_multiplier + LOWRISK_VALUE + (node.lowriskzone_cnt - 1)
        if node.mediumriskzone_cnt > 0:
            risk_multiplier = risk_multiplier + MEDIUMRISK_VALUE + (node.mediumriskzone_cnt - 1)
        if node.highriskzone_cnt > 0:
            risk_multiplier = risk_multiplier + HIGHRISK_VALUE + (node.highriskzone_cnt - 1)
        # default value is 1
        if risk_multiplier == 0:
            risk_multiplier = 1
        # set risk_multiplier
        node.risk_multiplier = risk_multiplier
    
    # determine edge riskzones and update node riskzones 
    # when nodes are in the same riskzone and only the edge crosses another zone
    def set_edge_riskzones(self,node1:GridNode,node2:GridNode) -> None:
        # check for all obstacles
        for circle in self.obstacles:
            # edge check only if node1 and node2 are in the same riskzone
            if node1.riskzones.get(circle.location) is not node2.riskzones.get(circle.location):
                continue
            # get current radius multiplier
            radius_multiplier = 1.0
            if circle.location in node1.riskzones:
                radius_multiplier = node1.riskzones.get(circle.location)
            # check if edge crosses riskzone while nodes are not in riskzone
            # and update radius multiplier
            if self.cross_circle(node1,node2,circle,radius_multiplier):
                if radius_multiplier == 0.5:
                    radius_multiplier = 0.0
                if radius_multiplier == 0.8:
                    radius_multiplier = 0.5
                if radius_multiplier == 1.0:
                    radius_multiplier = 0.8
                # update node1 riskzone to account for edge crossing riskzone
                node1.riskzones.update({circle.location:radius_multiplier})
                # todo check
                print("cross riskzone edge")

    # check if is existing gridnode
    def existing_gridnode(self,location:tuple[int]) -> GridNode | None:
        gridnode:GridNode
        for gridnode in self.gridnodes:
            if gridnode.location == location:
                return gridnode
        return None

    # todo used anywhere?
    # check if node is in free space or within circle obstacle
    def is_freespace(self,location:tuple[int]) -> bool:
        # check for all obstacles
        for riskzone in self.obstacles:
            # collision when point is within circle radius + margin
            if math.dist(location,riskzone.location) < (riskzone.radius + self.SAFETYMARGIN):
                return False
        # no collision detected
        return True
    
    def is_within_mapdimensions(self,location:tuple[int]) -> bool:
        # initialise
        x_map,y_map = location
        # check within map dimensions
        if x_map < 0 or x_map > self.mapwidth:
            return False
        if y_map < 0 or y_map > self.mapheight:
            return False
        return True

    # distance from node to closest riskzone edge
    def dist_node_riskzone(self,node:GridNode) -> float:
        # init
        dist = []
        # check for all obstacles
        for riskzone in self.obstacles:
            # distance from node to outer edge of riskzone
            dist.append(math.dist(node.location,riskzone.location) - riskzone.radius)
        return min(dist)
            
    # build all paths to goal if goals have been found
    def create_goalpaths(self) -> bool:
        # check goalfound
        if not self.goalfound: 
            return False
        # reconstructing the path backwards from goal using parents
        for goalnode in self.goalnodes:
            goalpath = GridPath(goalnode)
            goalpath.path_f_cost = goalnode.f_cost
            ic(goalpath.path_f_cost)        # todo debug
            node:GridNode = goalnode
            while node.location not in self.start_locations:
                # list of all nodes in goalpath
                goalpath.nodes.insert(0,node)
                # update goalpath riskzones, only if higher risk
                self.update_gridpath_riskzones(node,goalpath)
                # previousnode moves 1 up the line through parent of node
                # nodes are copied for each path to be able to use Line of Sight (LOS) optimisation later
                previousnode:GridNode = copy.deepcopy(node.parent)
                node.goalpath_parent = previousnode
                previousnode.goalpath_child = node
                # parent node moves 1 up the line
                node = previousnode
            # add start location to path and update path startnode
            goalpath.nodes.insert(0,node)
            goalpath.startnode = node
            # update goalpath riskzones for startnode
            self.update_gridpath_riskzones(node,goalpath)
            ic(goalpath.riskzones)      # todo
            # add goalpath to total list of goalpaths
            self.goalpaths.append(goalpath)
            ic(len(self.goalpaths))     # todo
        # print results                
        print(f"{len(self.goalpaths)} goalpaths generated")
        return True
    
    # update gridpath riskzones when node has higher risk (lower risk radius)
    def update_gridpath_riskzones(self,node:GridNode,gridpath:GridPath) -> None:
        for riskzone in node.riskzones:
            node_value = node.riskzones.get(riskzone)
            path_value = gridpath.riskzones.get(riskzone)
            if path_value is None or path_value > node_value:
                gridpath.riskzones.update({riskzone:node_value})
        
    # smooth final path using Line of Sight (LOS) algorithm
    # level 3:  path takes all riskzones into account, 
    #           uses path optimisations, such as 
    #           transition nodes with local riskzones
    #           higher quality results
    # check LOS to goal for each node walking from start to goal
    # farthest node from goal with LOS becomes node in LOS path
    # from this node check LOS for each node from start, etc.
    def create_LOS_goalpaths_level3(self) -> bool:
        # check goal found
        if not self.goalfound: 
            return False
        # create LOS path for each goalpath
        goalpath:GridPath       # todo declare in for statement?
        for goalpath in self.goalpaths:
            ic(goalpath.riskzones)     # todo debug
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
            templist = [(node.location,node.risk_multiplier) for node in goalpath.transition_nodes]
            ic(templist)            # todo debug
            templist2 = [(node.location,node.risk_multiplier) for node in goalpath.nodes]
            ic(templist2)            # todo debug
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
                    # update LOS_localpath riskzones
                    self.update_gridpath_riskzones(stepnode,LOS_localpath)
                    # node moves 1 up the line
                    previousnode = stepnode.goalpath_parent
                    stepnode = previousnode
                # include last node, mainly when node is startnode
                self.update_gridpath_riskzones(stepnode,LOS_localpath)
                ic(LOS_localpath.riskzones)     # todo debug
                # Line of Sight checks
                while LOS_basenode.location is not node.location:
                    # if node has LOS with basenode, add it to the LOS path
                    # node then becomes the next LOS basenode
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
            print("next LOS goalpath")
        # LOS path accessed through LOS path goalnode
        # set LOS path cost
        #?self.pathCostLOS(latestgoalnode)
        print("LOS_goalpaths calculated")
        return True


    # smooth final path using Line of Sight (LOS) algorithm
    # level 2:  path takes all riskzones into account, 
    #           also path direction inside riskzones covering goal
    #           medium quality results
    # check LOS to goal for each node walking from start to goal
    # farthest node from goal with LOS becomes node in LOS path
    # from this node check LOS for each node from start, etc.
    def create_LOS_goalpaths_level2(self) -> bool:
        # check goal found
        if not self.goalfound: 
            return False
        # latestgoalnode is latest new node at goal location, becomes basenode
        goalpath:GridPath       # todo declare in for statement?
        for goalpath in self.goalpaths:
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

            ic(goalpath.riskzones)     # todo debug

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
            print("next LOS goalpath")
        # LOS path accessed through LOS path goalnode
        # set LOS path cost
        #?self.pathCostLOS(latestgoalnode)
        print("LOS_goalpaths calculated")
        return True
        

    # smooth final path using Line of Sight (LOS) algorithm
    # level 1:  path takes all riskzones into account, 
    #           except path direction inside riskzones covering goal
    #           simplest code, but lower quality results
    # check LOS to goal for each node walking from start to goal
    # farthest node from goal with LOS becomes node in LOS path
    # from this node check LOS for each node from start, etc.
    def create_LOS_goalpaths_level1(self) -> bool:
        # check goal found
        if not self.goalfound: 
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
            print("next LOS goalpath")
        # LOS path accessed through LOS path goalnode
        # set LOS path cost
        #?self.pathCostLOS(latestgoalnode)
        print("LOS_goalpaths calculated")
        return True

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

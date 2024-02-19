import math
import copy
from icecream import ic
from route_opt_utils import CircleObstacle
from Astar_utils import GridNode, GridPath
from Astar_utils import update_node, is_existing_gridnode, is_within_mapdimensions
from Astar_utils import cross_riskzone, cross_circle

        
        
#? double stepsize when far from riskzone
#? store direction (eg. NW) for node to use later for waypoint reduction
#? remove outside node and check for better result
#? calc LOS path cost or assume same as goalpath?
#? make provisions for LORAD 
#? check goal found not first of 8, but best of 8

class AStarAlgorithm:
    def __init__(self,
                 start_locations:list[int],
                 goal_locations:list[int],
                 riskzones:list[CircleObstacle],
                 mapdimensions:tuple[int],
                 stepsize:int
                 ) -> None:
        self.start_locations = start_locations
        self.goal_locations = goal_locations
        self.riskzones = riskzones
        self.mapdimensions = mapdimensions
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
            #dist = dist_node_riskzone(qnode)
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
                if not is_within_mapdimensions(new_location,self.mapdimensions):
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
                node:GridNode = is_existing_gridnode(new_location,self.gridnodes)
                # if existing node and f_cost lower than tempnode, 
                # leave as is, go to next point
                if node is not None and node.f_cost <= tempnode.f_cost:
                    continue
                # if existing node and f_cost higher than tempnode,
                # update node
                if node is not None and node.f_cost > tempnode.f_cost:
                    node = update_node(node,tempnode)
                    node.parent = qnode
                # if node does not exist, make new node
                if node is None:
                    node = tempnode
                    node.parent = qnode
                    self.gridnodes.append(node)
                    new_children.append(node)
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
        # check for all riskzones
        for riskzone in self.riskzones:
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
        # check for all riskzones
        for circle in self.riskzones:
            # edge check only if node1 and node2 are in the same riskzone
            if node1.riskzones.get(circle.location) is not node2.riskzones.get(circle.location):
                continue
            # get current radius multiplier
            radius_multiplier = 1.0
            if circle.location in node1.riskzones:
                radius_multiplier = node1.riskzones.get(circle.location)
            # check if edge crosses riskzone while nodes are not in riskzone
            # and update radius multiplier
            if cross_circle(node1,node2,circle,radius_multiplier,self.SAFETYMARGIN):
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

    # build all paths to goal if goals have been found
    def create_goalpaths(self) -> bool:
        # check goalfound
        if not self.goalfound: 
            return False
        # reconstructing the path backwards from goal using parents
        for goalnode in self.goalnodes:
            goalpath = GridPath(goalnode)
            goalpath.path_f_cost = goalnode.f_cost
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
            # add goalpath to total list of goalpaths
            self.goalpaths.append(goalpath)
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
                    # update LOS_localpath riskzones
                    self.update_gridpath_riskzones(stepnode,LOS_localpath)
                    # node moves 1 up the line
                    previousnode = stepnode.goalpath_parent
                    stepnode = previousnode
                # include last node, mainly when node is startnode
                self.update_gridpath_riskzones(stepnode,LOS_localpath)
                # Line of Sight checks
                while LOS_basenode.location is not node.location:
                    # if node has LOS with basenode, add it to the LOS path
                    # node then becomes the next LOS basenode
                    if not cross_riskzone(node,LOS_basenode,self.riskzones,
                                          LOS_localpath,self.SAFETYMARGIN):
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
        # LOS path accessed through LOS path goalnode
        # set LOS path cost
        #?self.pathCostLOS(latestgoalnode)
        print("LOS_goalpaths calculated")
        return True

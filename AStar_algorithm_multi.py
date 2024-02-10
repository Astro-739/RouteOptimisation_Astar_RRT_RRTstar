import math
import copy
from icecream import ic
from route_opt_utils import CircleObstacle



class GridNode:
    def __init__(self,location:(int,int)) -> None:
        self.location = location
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
        self.lowriskzones = []
        self.mediumriskzones = []
        self.highriskzones = []
        self.riskzones = {}         # todo test if dict works here
        self.riskmultiplier = 1


class GridPath:
    def __init__(self,startnode:GridNode,goalnode:GridNode) -> None:
        self.startnode = startnode
        self.goalnode = goalnode
        self.nodes = []
        self.path_f_cost = 0
        self.lowriskzones = []
        self.mediumriskzones = []
        self.highriskzones = []
        self.riskzones = {}         # todo test if dictionary works here
        self.highlightnode = None       # todo  debug


class AStarAlgorithm:
    def __init__(self,
                 start_location:(int,int),
                 goal_locations:list[int],
                 obstacles:list[CircleObstacle],
                 mapdimensions:(int,int),
                 stepsize:int
                 ) -> None:
        self.start_location = start_location
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
        # init startnode
        self.startnode = GridNode(self.start_location)
        self.startnode.edgecost = 0.0
        self.startnode.g_cost = 0.0
        self.set_riskzones(self.startnode)
        self.set_riskmultiplier(self.startnode)
        self.gridnodes.append(self.startnode)
        # init goalnodes
        self.goalnodes = []
        for location in self.goal_locations:
            goalnode = GridNode(location)
            self.set_riskzones(goalnode)
            self.set_riskmultiplier(goalnode)
            self.goalnodes.append(goalnode)
            self.gridnodes.append(goalnode)
            ic(goalnode.lowriskzones)         # todo debug
            ic(goalnode.mediumriskzones)         # todo debug
            ic(goalnode.highriskzones)         # todo debug                        
            
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
        
        # add startnode to openlist as first node
        self.openlist.append(self.startnode)

        iteration = 0
        goalnode_list = self.goalnodes.copy()
        
        while len(self.openlist) > 0 and not self.goalfound:
            # update iteration
            iteration += 1
            # sort openlist on lowest f_cost
            self.openlist.sort(key=lambda x: x.f_cost)
            # qnode is node with lowest f_cost, pop from openlist
            qnode:GridNode = self.openlist.pop(0)
            # generate new gridpoints in all 8 directions N,NE,E,SE,S,SW,W,NW
            new_gridpoints = []
            new_children = []
            location_and_edge_north     = (qnode.location[0], 
                                           qnode.location[1] + self.STEPSIZE, 
                                           self.STEPSIZE)
            new_gridpoints.append(location_and_edge_north)
            location_and_edge_northeast = (qnode.location[0] + self.STEPSIZE, 
                                           qnode.location[1] + self.STEPSIZE, 
                                           math.sqrt(2) * self.STEPSIZE)
            new_gridpoints.append(location_and_edge_northeast)
            location_and_edge_east      = (qnode.location[0] + self.STEPSIZE, 
                                           qnode.location[1], 
                                           self.STEPSIZE)
            new_gridpoints.append(location_and_edge_east)
            location_and_edge_southeast = (qnode.location[0] + self.STEPSIZE, 
                                           qnode.location[1] - self.STEPSIZE, 
                                           math.sqrt(2) * self.STEPSIZE)
            new_gridpoints.append(location_and_edge_southeast)
            location_and_edge_south     = (qnode.location[0], 
                                           qnode.location[1] - self.STEPSIZE, 
                                           self.STEPSIZE)
            new_gridpoints.append(location_and_edge_south)
            location_and_edge_southwest = (qnode.location[0] - self.STEPSIZE, 
                                           qnode.location[1] - self.STEPSIZE, 
                                           math.sqrt(2) * self.STEPSIZE)
            new_gridpoints.append(location_and_edge_southwest)
            location_and_edge_west      = (qnode.location[0] - self.STEPSIZE, 
                                           qnode.location[1], 
                                           self.STEPSIZE)
            new_gridpoints.append(location_and_edge_west)
            location_and_edge_northwest = (qnode.location[0] - self.STEPSIZE, 
                                           qnode.location[1] + self.STEPSIZE, 
                                           math.sqrt(2) * self.STEPSIZE)
            new_gridpoints.append(location_and_edge_northwest)
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
                self.set_riskzones(tempnode)
                self.cross_riskzone_edge(tempnode,qnode)      # todo test
                self.set_riskmultiplier(tempnode)
                # calculate costs based on risk multiplier
                self.calc_edge_cost(tempnode)
                self.calc_g_cost(tempnode,qnode)
                self.calc_h_cost(tempnode)
                self.calc_f_cost(tempnode)
                # check for existing node on this location
                # if node exists, it is on openlist or closedlist (in this implementation)
                node:GridNode = None
                node = self.existing_gridnode(new_location)
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

    # edge cost to get to point
    # edge cost based on distance and multiplier
    # multiplier is based on gridpoint location, all edges to point same multiplier
    def calc_edge_cost(self,node:GridNode) -> None:
        node.edgecost = node.edgelength * node.riskmultiplier

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
    def set_riskzones(self,node:GridNode) -> None:
        # risk range multiplier settings
        LOWRISK_RANGE = 0.8
        MEDIUMRISK_RANGE = 0.5
        HIGHRISK_RANGE = 0.0
        # check for all obstacles
        for riskzone in self.obstacles:
            # distance between node location and centre of circle
            dist = math.dist(node.location,riskzone.location)
            # if outside cirle, continue
            if dist > riskzone.radius + self.SAFETYMARGIN:  # todo safety margin here?
                continue
            # if in highrisk zone
            if dist < MEDIUMRISK_RANGE * riskzone.radius:   # todo safety margin here?
                node.highriskzones.append(riskzone)  # todo  .location for debug??
                node.riskzones.update({riskzone.location:HIGHRISK_RANGE})
                continue
            # if in mediumrisk zone
            if dist < LOWRISK_RANGE * riskzone.radius:   # todo safety margin here?
                node.mediumriskzones.append(riskzone)
                node.riskzones.update({riskzone.location:MEDIUMRISK_RANGE})
                continue
            # else in lowrisk zone
            else:
                node.lowriskzones.append(riskzone)
                node.riskzones.update({riskzone.location:LOWRISK_RANGE})
        # set risk count
        #node.lowriskzone_cnt = len(node.lowriskzones)
        #node.mediumriskzone_cnt = len(node.mediumriskzones)
        #node.highriskzone_cnt = len(node.highriskzones)
        #ic(node.riskzones)      # todo   debug

    # riskmultiplier is used to increase edgecost (edgecost = edgelength * riskmultiplier)
    def set_riskmultiplier(self,node:GridNode) -> None:
        # initialise
        riskmultiplier = 0
        # risk values
        LOWRISK_VALUE = 3
        MEDIUMRISK_VALUE = 5
        HIGHRISK_VALUE = 10
        # set risk count
        values = len([(node.riskzones[key]) for key in [*node.riskzones.keys()] if node.riskzones[key] == 0.8])
        node.lowriskzone_cnt = len([(node.riskzones[key]) for key in [*node.riskzones.keys()] if node.riskzones[key] == 0.8])
        node.mediumriskzone_cnt = len([(node.riskzones[key]) for key in [*node.riskzones.keys()] if node.riskzones[key] == 0.5])
        node.highriskzone_cnt = len([(node.riskzones[key]) for key in [*node.riskzones.keys()] if node.riskzones[key] == 0.0])
        # determine riskmultiplier
        if node.lowriskzone_cnt > 0:
            riskmultiplier = riskmultiplier + LOWRISK_VALUE + (node.lowriskzone_cnt - 1)
        if node.mediumriskzone_cnt > 0:
            riskmultiplier = riskmultiplier + MEDIUMRISK_VALUE + (node.mediumriskzone_cnt - 1)
        if node.highriskzone_cnt > 0:
            riskmultiplier = riskmultiplier + HIGHRISK_VALUE + (node.highriskzone_cnt - 1)
        # default value is 1
        if riskmultiplier == 0:
            riskmultiplier = 1
        # set riskmultiplier
        node.riskmultiplier = riskmultiplier
    
    # check if is existing gridnode
    def existing_gridnode(self,location:(int,int)) -> GridNode | None:
        gridnode:GridNode
        for gridnode in self.gridnodes:
            if gridnode.location == location:
                return gridnode
        return None

    # todo used anywhere?
    # check if node is in free space or within circle obstacle
    def is_freespace(self,location:(int,int)) -> bool:
        # check for all obstacles
        for riskzone in self.obstacles:
            # collision when point is within circle radius + margin
            if math.dist(location,riskzone.location) < (riskzone.range + self.SAFETYMARGIN):
                return False
        # no collision detected
        return True
    
    def is_within_mapdimensions(self,location:(int,int)) -> bool:
        # initialise
        x_map,y_map = location
        # check within map dimensions
        if x_map < 0 or x_map > self.mapwidth:
            return False
        if y_map < 0 or y_map > self.mapheight:
            return False
        return True
        
    # build all paths to goal if goals have been found
    def create_goalpaths(self) -> bool:
        # check goalfound
        if not self.goalfound: 
            return False
        # reconstructing the path backwards from goal using parents
        for goalnode in self.goalnodes:
            goalpath = GridPath(self.startnode,goalnode)
            goalpath.path_f_cost = goalnode.f_cost
            node = goalnode
            while node.location is not self.startnode.location:
                # list of all nodes in goalpath
                goalpath.nodes.insert(0,node)
                # lists of riskzones crossed in path
                [goalpath.lowriskzones.append(zone) for zone in node.lowriskzones if zone not in goalpath.lowriskzones]
                [goalpath.mediumriskzones.append(zone) for zone in node.mediumriskzones if zone not in goalpath.mediumriskzones]
                [goalpath.highriskzones.append(zone) for zone in node.highriskzones if zone not in goalpath.highriskzones]
                # todo test dict
                goalpath.riskzones.update(node.riskzones)
                # previousnode moves 1 up the line through parent of node
                # nodes are copied for each path to be able to use Line of Sight (LOS) optimisation later
                previousnode = copy.deepcopy(node.parent)
                node.goalpath_parent = previousnode
                previousnode.goalpath_child = node
                # parent node moves 1 up the line
                node = previousnode
            # add start location to path and update path startnode
            goalpath.nodes.insert(0,node)
            goalpath.startnode = node
            [goalpath.lowriskzones.append(zone) for zone in node.lowriskzones if zone not in goalpath.lowriskzones]
            [goalpath.mediumriskzones.append(zone) for zone in node.mediumriskzones if zone not in goalpath.mediumriskzones]
            [goalpath.highriskzones.append(zone) for zone in node.highriskzones if zone not in goalpath.highriskzones]
            goalpath.riskzones.update(node.riskzones)
            #ic(goalpath.lowriskzones)   # todo
            #ic(goalpath.mediumriskzones)   # todo
            #ic(goalpath.highriskzones)   # todo
            ic(goalpath.riskzones)      # todo
            # add goalpath to total list of goalpaths
            self.goalpaths.append(goalpath)
            ic(len(self.goalpaths))     # todo
        # print results                
        print(f"{len(self.goalpaths)} goalpaths generated")
        return True
        

    # smooth final path using Line of Sight (LOS) algorithm
    # check LOS to goal for each node walking from start to goal
    # farthest node from goal with LOS becomes node in LOS path
    # from this node check LOS for each node from start, etc.
    def create_LOS_goalpaths(self) -> bool:
        # check goal found
        if not self.goalfound: 
            return False
        # latestgoalnode is latest new node at goal location, becomes basenode
        for goalpath in self.goalpaths:
            # find first node outside of sams covering target
            outsidenode = goalpath.goalnode
            while outsidenode.riskmultiplier > 1:
                #ic(goalpath.lowriskzones)       # todo debug
                #ic(goalpath.mediumriskzones)
                #ic(goalpath.highriskzones)

                ic(outsidenode.riskzones)
                ic(goalpath.riskzones)

                [goalpath.lowriskzones.remove(zone) for zone in outsidenode.lowriskzones]
                [goalpath.mediumriskzones.remove(zone) for zone in outsidenode.mediumriskzones]
                [goalpath.highriskzones.remove(zone) for zone in outsidenode.highriskzones]
                {goalpath.riskzones.pop(zone) for zone in outsidenode.riskzones if zone in goalpath.riskzones}
                outsidenode = outsidenode.goalpath_parent
            # set outside node as first LOS path node after goalnode
            goalpath.goalnode.LOSpath_parent = outsidenode
            goalpath.highlightnode = outsidenode    # todo debug
            # sams covering goalnode are set to fully evade as riskzone
            #ic(goalpath.lowriskzones)       # todo debug
            #ic(goalpath.mediumriskzones)
            #ic(goalpath.highriskzones)
            # start LOS checks at start node walking towards LOS basenode
            node = goalpath.startnode
            LOS_basenode = outsidenode
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
                if node.location == LOS_basenode.location:
                    print("node == LOS_basenode")   # todo
                    break
            print("next LOS goalpath")
            #ic([temp for temp in goalpath.nodes if node.LOSpath_parent is not None])          # todo
        # LOS path accessed through LOS path goalnode
        # set LOS path cost
        #?self.pathCostLOS(latestgoalnode)
        print("LOS_goalpath calculated")
        return True
        
    # LOS
    # from goal move outside circle with closest centre (after LOS check?)
    # regular LOS checks
    # when move from inside to outside circle place basenode there
    #    all? or not crossing and only when inside outward? think yes
    # check for nodes inside circles ahead of basenode to determine radius of circles
    
    # option
    # move outside circle goal is in
    # LOS check from goal to first node outside, without taking circle goal is in into account
    # still inside other circle? repeat
    # now outside and only crossing circle might still occur
    
    # option 2 
    # same principle, but start from startnode
    # check if only crossing, or if entering and not coming out

        
    # check if connection crosses a riskzone #todo update description
    def cross_riskzone(self,node1:GridNode,node2:GridNode,goalpath:GridPath) -> bool:
        # check for all obstacles
        for circle in self.obstacles:
            multiplier = 1.0
            #if circle in goalpath.lowriskzones:
            #    multiplier = 0.8
            #if circle in goalpath.mediumriskzones:
            #    multiplier = 0.5
            #if circle in goalpath.highriskzones:
            #    multiplier = 0.0
            if circle.location in goalpath.riskzones:
                multiplier = goalpath.riskzones.get(circle.location)
                #if zone == "low":
                #    multiplier = 0.8
                #if zone == "medium":
                #    multiplier = 0.5
                #if zone == "high":
                #    multiplier = 0.0
            if self.cross_circle(node1,node2,circle,multiplier):
                return True
        # no collision detected
        return False


    # check if connection crosses a riskzone #todo update description
    def cross_riskzone_edge(self,node1:GridNode,node2:GridNode) -> None:
        # check for all obstacles
        for circle in self.obstacles:
            # edge check only if node1 and node2 are in same riskzone
            if node1.riskzones.get(circle.location) is not node2.riskzones.get(circle.location):
                continue

            multiplier = 1.0
            if circle.location in node1.riskzones:
                multiplier = node1.riskzones.get(circle.location)
            # check if edge crosses riskzone while nodes are not in riskzone
            if self.cross_circle(node1,node2,circle,multiplier):
                if multiplier == 0.5:
                    multiplier = 0.0
                if multiplier == 0.8:
                    multiplier = 0.5
                if multiplier == 1.0:
                    multiplier = 0.8
                node1.riskzones.update({circle.location:multiplier})
                # update risk multiplier (used in edge cost)
                #self.set_riskmultiplier(node1)

                print("cross riskzone edge")


    def cross_circle(self,node1:GridNode,node2:GridNode,circle:CircleObstacle,multiplier:float) -> bool:
        # init
        line_length = math.dist(node1.location,node2.location)
        # check if line is far enough away from circle to not cross
        dist1 = math.dist(node1.location,circle.location)
        dist2 = math.dist(node2.location,circle.location)
        if ((dist1 or dist2) 
             > (line_length + (circle.radius * multiplier + self.SAFETYMARGIN))):
            return False
        # divide connection in 100 points to check each
        for i in range(0,101):
            u = i/100                   
            x = node1.location[0] * u + node2.location[0] * (1-u)
            y = node1.location[1] * u + node2.location[1] * (1-u)
            # collision when point is within circle radius + safety margin
            if (math.dist((x,y),(circle.location)) 
                < ((circle.radius * multiplier + self.SAFETYMARGIN))):
                return True             
        # no collision detected
        return False

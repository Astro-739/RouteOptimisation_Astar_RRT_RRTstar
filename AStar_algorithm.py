import math
from RRT_utils import TreeResults
from icecream import ic

class GridNode:
    def __init__(self,location:(int,int)) -> None:
        self.location = location
        self.parent = None
        self.edgelength = 0.0 
        self.edgecost = 100000.0     # edge from parent
        self.f_cost = 100000.0 
        self.g_cost = 100000.0 
        self.h_cost = 100000.0 
        self.lowriskzone_cnt = 0 
        self.mediumriskzone_cnt = 0 
        self.highriskzone_cnt = 0 
        self.riskmultiplyer = 1


class GridPath:
    def __init__(self,location:(int,int)) -> None:
        self.location = location
        self.nodes = []            # includes startnode
        self.path_f_cost = 0


class AStarAlgorithm:
    def __init__(self,start_location,goal_location,obstacles,mapdimensions,stepsize) -> None:
        self.start_location = start_location
        self.goal_location = goal_location
        self.obstacles = obstacles
        self.mapheight,self.mapwidth = mapdimensions
        self.STEPSIZE = stepsize
    
        self.startnode = GridNode(self.start_location)
        self.startnode.edgecost = 0.0
        self.startnode.g_cost = 0.0
        self.goalnode = GridNode(self.goal_location)
        
        self.gridnodes = GridPath(start_location)
        self.gridnodes.nodes.append(self.startnode)
        
        self.openlist = []
        self.closedlist = []
        
        self.goalfound = False
        self.goalpath = []
                
        self.finalresults = TreeResults(start_location,goal_location)
    
    def astarsearch(self) -> None:
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
        
        while len(self.openlist) > 0 and not self.goalfound:
            # update iteration
            iteration += 1
            # sort openlist on lowest f_cost
            self.openlist.sort(key=lambda x: x.f_cost)
            # qnode is node with lowest f_cost, pop from openlist
            qnode = self.openlist.pop(0)
            # generate children, note: north in pygame is south, y-axis is positive down
            new_gridpoints = []
            new_children = []
            location_and_edge_north = (qnode.location[0], qnode.location[1] + self.STEPSIZE, self.STEPSIZE)
            new_gridpoints.append(location_and_edge_north)
            location_and_edge_northeast = (qnode.location[0] + self.STEPSIZE, qnode.location[1] + self.STEPSIZE, math.sqrt(2) * self.STEPSIZE)
            new_gridpoints.append(location_and_edge_northeast)
            location_and_edge_east = (qnode.location[0] + self.STEPSIZE, qnode.location[1], self.STEPSIZE)
            new_gridpoints.append(location_and_edge_east)
            location_and_edge_southeast = (qnode.location[0] + self.STEPSIZE, qnode.location[1] - self.STEPSIZE, math.sqrt(2) * self.STEPSIZE)
            new_gridpoints.append(location_and_edge_southeast)
            location_and_edge_south = (qnode.location[0], qnode.location[1] - self.STEPSIZE, self.STEPSIZE)
            new_gridpoints.append(location_and_edge_south)
            location_and_edge_southwest = (qnode.location[0] - self.STEPSIZE, qnode.location[1] - self.STEPSIZE, math.sqrt(2) * self.STEPSIZE)
            new_gridpoints.append(location_and_edge_southwest)
            location_and_edge_west = (qnode.location[0] - self.STEPSIZE, qnode.location[1], self.STEPSIZE)
            new_gridpoints.append(location_and_edge_west)
            location_and_edge_northwest = (qnode.location[0] - self.STEPSIZE, qnode.location[1] + self.STEPSIZE, math.sqrt(2) * self.STEPSIZE)
            new_gridpoints.append(location_and_edge_northwest)
            
            for location_and_edge in new_gridpoints:
                # new location and edge of gridpoint
                new_location = (location_and_edge[0],location_and_edge[1])
                new_edge = location_and_edge[2]
                # if not within map dimenstions, continue to next gridpoint
                if not self.is_withinmapdimensions(new_location):
                    continue
                # create tempnode at gridpoint
                tempnode = GridNode(new_location)
                tempnode.edgelength = new_edge
                # set riskzone count and multiplyer for tempnode
                self.setriskzonecount(tempnode)
                self.setriskmultiplyer(tempnode)
                # calculate costs based on risk multiplyer
                self.calc_edge_cost(tempnode)
                self.calc_g_cost(tempnode,qnode)
                self.calc_h_cost(tempnode)
                self.calc_f_cost(tempnode)
                # check if goal found
                if tempnode.location == self.goalnode.location:
                    self.goalfound = True
                    print("goal found")
                    # goalnode gets all attributes from tempnode
                    self.goalnode = tempnode
                # check for existing node on this location
                # if node exists, it is on openlist or closedlist (in this implementation)
                node = None
                node = self.existinggridnode(new_location)
                # if existing node and f_cost lower, leave as is, go to next point
                if node is not None and node.f_cost < tempnode.f_cost:
                    continue
                # if not existing, make new node
                if node is None:
                    node = tempnode
                    self.gridnodes.nodes.append(node)
                    print(f"gridnode #{len(self.gridnodes.nodes)} added")
                    new_children.append(node)
                # update node if new or if existing and higher f_cost
                node.parent = qnode
                node.edgecost = tempnode.edgecost
                node.f_cost = tempnode.f_cost
                node.g_cost = tempnode.g_cost
                node.h_cost = tempnode.h_cost
            # put all new nodes on openlist
            for child in new_children:
                # add child to openlist
                self.openlist.append(child)
            # qnode goes to closedlist
            self.closedlist.append(qnode)
            # print status
            print(f"iteration: {iteration}")       
            print(f"new_children: {len(new_children)}")
            print(f"openlist items: {len(self.openlist)}")
            print(f"closedlist items: {len(self.closedlist)}")

    def creategrid(self) -> None:
        pass
    
    def updatenode(self) -> None:
        pass
    
    def setriskzonecount(self,node:GridNode) -> None:
        # initialise
        lowrisk_cnt = 0
        mediumrisk_cnt = 0
        highrisk_cnt = 0
        x_map = node.location[0]
        y_map = node.location[1]
        # risk range settings
        lowrisk_range = 1.0
        mediumrisk_range = 0.8
        highrisk_range = 0.5
        # check for all obstacles
        for circle in self.obstacles:
            # distance between node location and centre of circle
            dist = math.dist((x_map,y_map),(circle[0],circle[1]))
            # if outside cirle, continue
            if dist > circle[2]:
                continue
            # if in highrisk zone
            if dist < highrisk_range * circle[2]:
                highrisk_cnt += 1
                continue
            # if in mediumrisk zone
            if dist < mediumrisk_range * circle[2]:
                mediumrisk_cnt += 1
                continue
            # else in lowrisk zone
            else:
                lowrisk_cnt += 1
        # set risk count
        node.lowriskzone_cnt = lowrisk_cnt
        node.mediumriskzone_cnt = mediumrisk_cnt
        node.highriskzone_cnt = highrisk_cnt

    def setriskmultiplyer(self,node:GridNode) -> None:
        # initialise
        riskmultiplyer = 0
        # risk values
        lowrisk_value = 3
        mediumrisk_value = 5
        highrisk_value = 10
        # determine riskmultiplyer
        if node.lowriskzone_cnt > 0:
            riskmultiplyer = riskmultiplyer + lowrisk_value + (node.lowriskzone_cnt - 1)
        if node.mediumriskzone_cnt > 0:
            riskmultiplyer = riskmultiplyer + mediumrisk_value + (node.mediumriskzone_cnt - 1)
        if node.highriskzone_cnt > 0:
            riskmultiplyer = riskmultiplyer + highrisk_value + (node.highriskzone_cnt - 1)
        # default value is 1
        if riskmultiplyer == 0:
            riskmultiplyer = 1
        # set riskmultiplyer
        node.riskmultiplyer = riskmultiplyer
    
    # edge cost to get to point
    # edge cost based on distance and multiplyer
    # multiplyer is based on gridpoint location, all edges to point same multiplyer
    def calc_edge_cost(self,node:GridNode) -> None:
        node.edgecost = node.edgelength * node.riskmultiplyer

    # total path predicted cost (f_cost = g_cost + h_cost)
    def calc_f_cost(self,node:GridNode) -> None:
        node.f_cost = node.g_cost + node.h_cost
    
    # path cost from start to current point
    def calc_g_cost(self,node:GridNode,parentnode:GridNode) -> None:
        node.g_cost = parentnode.g_cost + node.edgecost
    
    # h = D * (dx + dy) + (sqrt(2) * D - 2 * D) * min(dx, dy)
    # h = D * ((dx + dy) + (sqrt(2) - 2) * min(dx, dy))
    def calc_h_cost(self,node:GridNode):
        (x1,y1) = node.location
        (x2,y2) = self.goal_location
        #delta_x = abs(x2 - x1)
        #delta_y = abs(y2 - y1)
        #h_cost = ((delta_x + delta_y) + (math.sqrt(2) - 2) * min(delta_x,delta_y))
        # using Euclidian distance for now (diagonal distance not very distinct in this grid)
        node.h_cost = math.dist((x1,y1),(x2,y2)) 
    
    # assumption is if existing it is on either openlist or closedlist                
    def existinggridnode(self,location:(int,int)) -> GridNode:
        for gridnode in self.gridnodes.nodes:
            if gridnode.location == location:
                return gridnode
        return None

    # check if node is in free space or within circle obstacle
    def is_freespace(self,location:(int,int)) -> bool:
        margin = 0
        x_map = location[0]
        y_map = location[1]
        # check for all obstacles
        for circle in self.obstacles:
            # collision when point is within circle radius + margin
            if math.dist((x_map,y_map),(circle[0],circle[1])) <= (circle[2] + margin):
                return False
        # no collision detected
        return True
    
    def is_withinmapdimensions(self,location:(int,int)) -> bool:
        # initialise
        x_map = location[0]
        y_map = location[1]
        # check within map dimensions
        if x_map < 0 or x_map > self.mapwidth:
            return False
        if y_map < 0 or y_map > self.mapheight:
            return False
        return True
        
    # build path to goal if goal has been found
    def create_goalpath(self) -> bool:
        # check goalfound
        if not self.goalfound: 
            return False
        # reconstructing the path backwards from goal using parents
        node = self.goalnode
        while node is not self.startnode:
            self.goalpath.insert(0,node)
            # previousnode moves 1 up the line through parent of node
            previousnode = node.parent
            # parent node moves 1 up the line
            node = previousnode
        # startnode is not added #todo still true?
        print("goalpath generated")
        return True
        
        
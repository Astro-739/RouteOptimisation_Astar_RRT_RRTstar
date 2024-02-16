import matplotlib.pyplot as plt
import matplotlib.colors as mcolors
from AStar_algorithm_multi import GridNode,GridPath
from route_opt_utils import CircleObstacle


class RoutingMap_matplotlib:
    def __init__(self,
                 start_xy:tuple[int],
                 goal_locations:list[int],
                 mapdimensions:tuple[int],
                 obstacles:list[CircleObstacle]
                 ) -> None:
        self.start_xy = start_xy
        self.start_x,self.start_y = start_xy
        self.goal_locations = goal_locations
        self.MAPWIDTH,self.MAPHEIGHT = mapdimensions
        self.obstacles = obstacles
    
        #colours
        self.GREY = (0.6,0.6,0.6)
        self.BLUE = (0,0,1)
        self.DARKBLUE = (0,0,0.78)
        self.LIGHTBLUE = (0.3,0.65,1)
        self.GREEN = (0,1,0)
        self.RED = (1,0,0)
        self.WHITE = (1,1,1)
        self.PURPLE = (0.44,0.16,0.39)
        self.YELLOW = "#ffcc00"

        # init plot
        # 22,10 for large monitor
        # 15,7 laptop
        self.fig,(self.ax1,self.ax2) = plt.subplots(1,2,figsize=(15, 7))
        self.ax1.set_xlim([0,self.MAPWIDTH])
        self.ax1.set_ylim([0,self.MAPHEIGHT])
        self.ax2.set_xlim([0,self.MAPWIDTH])
        self.ax2.set_ylim([0,self.MAPHEIGHT])
        self.ax1.set_title("Multiple goal A* algorithm on risk zone map")
        self.ax2.set_title("Goal path Line of Sight optimisation")

    # draw base map with start and goal locations and obstacles
    def draw_basemap(self) -> None:
        # plot start location
        self.ax1.scatter(self.start_x,self.start_y,color=self.GREEN,s=100,marker="o")
        self.ax2.scatter(self.start_x,self.start_y,color=self.GREEN,s=100,marker="o")
        # plot goal locations
        for goal_location in self.goal_locations:
            self.ax1.scatter(goal_location[0],goal_location[1],color=self.GREEN,s=300,marker="o")
            self.ax2.scatter(goal_location[0],goal_location[1],color=self.GREEN,s=300,marker="o")
        # plot circle obstacles
        for circle in self.obstacles:
            circle1 = plt.Circle(circle.location,circle.radius,
                                color=self.RED,lw=2,fill=False)
            circle2 = plt.Circle(circle.location,circle.radius,
                                color=self.RED,lw=2,fill=False)
            self.ax1.add_patch(circle1)
            self.ax2.add_patch(circle2)

    # draw all nodes and their connections
    def draw_tree(self,gridnodes:list[GridNode]) -> None:
        for node in gridnodes:
            self.ax1.scatter(node.location[0],node.location[1],
                        color=self.GREY,s=20,marker="o")
            if not node.location == self.start_xy and node.parent is not None:
                x_data = [node.location[0],node.parent.location[0]]
                y_data = [node.location[1],node.parent.location[1]]
                self.ax1.plot(x_data,y_data,color=self.GREY,lw=0.5)
                
    # draw single node
    def draw_node(self,node:GridNode) -> None:
        self.ax1.scatter(node.location[0],node.location[1],
                    color=self.BLUE,s=50,marker="o")

    # draw all goal paths
    def draw_paths(self,goalpaths:list[GridPath]) -> None:
        for goalpath in goalpaths:        
            # for a path draw each node and connection to node parent 
            node = goalpath.goalnode
            while node.location is not goalpath.startnode.location:
                self.ax1.scatter(node.location[0],node.location[1],
                                 color=self.DARKBLUE,s=20,marker="o")
                x_data = [node.location[0],node.parent.location[0]]
                y_data = [node.location[1],node.parent.location[1]]
                self.ax1.plot(x_data,y_data,color=self.DARKBLUE,lw=0.5)
                # next node
                node = node.parent
            # draw start location
            self.ax1.scatter(node.location[0],node.location[1],
                             color=self.DARKBLUE,s=20,marker="o")
            # draw highlight node
            if goalpath.highlightnode is not None:
                self.ax1.scatter(goalpath.highlightnode.location[0],goalpath.highlightnode.location[1],
                                 color=self.YELLOW,s=60,marker="o")
                

    # draw all Line of Sight (LOS) goal paths
    def draw_LOS_paths(self,goalpaths:list[GridPath]) -> None:
        # set colour list to cycle through TABLEAU_COLORS per goalpath
        colours = []
        [colours.append(colour) for colour in mcolors.TABLEAU_COLORS]
        # for each goalpath draw each node and connection to node parent
        for goalpath in goalpaths:
            colour = colours[goalpaths.index(goalpath) % len(colours)]
            node = goalpath.goalnode
            while node.location is not goalpath.startnode.location:
                # draw node
                self.ax2.scatter(node.location[0],node.location[1],
                            color=colour,s=20,marker="o")
                # catch exception
                if node.LOSpath_parent is None:
                    print("LOS_parent is None:  no LOS path calculated")
                    break
                # draw edge between nodes
                x_data = [node.location[0],node.LOSpath_parent.location[0]]
                y_data = [node.location[1],node.LOSpath_parent.location[1]]
                self.ax2.plot(x_data,y_data,color=colour,lw=1.0)
                # next node
                node = node.LOSpath_parent
            # draw node at start location
            self.ax2.scatter(node.location[0],node.location[1],
                             color=colour,s=20,marker="o")
            # draw goalpath marked_nodes
            for marked_node in goalpath.marked_nodes:
                self.ax1.scatter(marked_node.location[0],marked_node.location[1],
                                 color=self.LIGHTBLUE,s=30,marker="o")

    

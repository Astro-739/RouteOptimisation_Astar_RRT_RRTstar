import matplotlib.pyplot as plt
from AStar_algorithm_multi import GridNode,GridPath
from route_opt_utils import CircleObstacle


class RoutingMap_matplotlib:
    def __init__(self,
                 start_xy:(int,int),
                 goal_locations:list[int],
                 mapdimensions:(int,int),
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

        # init plot
        self.fig,(self.ax1,self.ax2) = plt.subplots(1,2,figsize=(22, 10))
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
        # plot circle obsticles
        for circle in self.obstacles:
            circle1 = plt.Circle(circle.location,circle.radius,
                                color=self.RED,lw=2,fill=False)
            circle2 = plt.Circle(circle.location,circle.radius,
                                color=self.RED,lw=2,fill=False)
            self.ax1.add_patch(circle1)
            self.ax2.add_patch(circle2)

    #
    def draw_tree(self,gridnodes:list[GridNode]) -> None:
        for node in gridnodes:
            self.ax1.scatter(node.location[0],node.location[1],
                        color=self.GREY,s=20,marker="o")
            if not node.location == self.start_xy and node.parent is not None:
                x_data = [node.location[0],node.parent.location[0]]
                y_data = [node.location[1],node.parent.location[1]]
                self.ax1.plot(x_data,y_data,color=self.GREY,lw=0.5)
                
    #    
    def draw_path(self,goalpaths:list[GridPath]) -> None:
        for goalpath in goalpaths:        
            # for a path draw each node and connection to node parent 
            node = goalpath.goalnode
            while node.location is not goalpath.startnode.location:
                self.ax1.scatter(node.location[0],node.location[1],
                            color=self.LIGHTBLUE,s=20,marker="o")
                x_data = [node.location[0],node.parent.location[0]]
                y_data = [node.location[1],node.parent.location[1]]
                self.ax1.plot(x_data,y_data,color=self.LIGHTBLUE,lw=0.5)
                # next node
                node = node.parent

    
    def draw_LOS_path(self,goalpaths:list[GridPath]) -> None:
        
        
        for goalpath in goalpaths:        
            # for a path draw each node and connection to node parent 
            node = goalpath.goalnode
            while node.location is not goalpath.startnode.location:
                self.ax2.scatter(node.location[0],node.location[1],
                            color=self.DARKBLUE,s=20,marker="o")

                if node.LOSpath_parent is None:
                    print("LOS_parent is None:  no LOS path calculated")
                    break

                x_data = [node.location[0],node.LOSpath_parent.location[0]]
                y_data = [node.location[1],node.LOSpath_parent.location[1]]
                self.ax2.plot(x_data,y_data,color=self.DARKBLUE,lw=0.8)
                # next node
                node = node.LOSpath_parent

    

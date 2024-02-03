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

    # draw base map with start and goal locations and obstacles
    def draw_basemap(self) -> None:
        # init plot
        fig,axes = plt.subplots(1,1,figsize=(10, 10))
        plt.xlim([0,self.MAPWIDTH])
        plt.ylim([0,self.MAPHEIGHT])
        plt.title("Multiple goal A* algorithm on risk zone map")
        # plot start location
        plt.scatter(self.start_x,self.start_y,color=self.GREEN,s=100,marker="o")
        # plot goal locations
        for goal_location in self.goal_locations:
            plt.scatter(goal_location[0],goal_location[1],color=self.GREEN,s=300,marker="o")
        # plot circle obsticles
        for circle in self.obstacles:
            circle = plt.Circle(circle.location,circle.radius,
                                color=self.RED,lw=2,fill=False)
            axes.add_patch(circle)

    #
    def draw_tree(self,gridnodes:list[GridNode]) -> None:
        for node in gridnodes:
            plt.scatter(node.location[0],node.location[1],
                        color=self.GREY,s=20,marker="o")
            if not node.location == self.start_xy and node.parent is not None:
                x_data = [node.location[0],node.parent.location[0]]
                y_data = [node.location[1],node.parent.location[1]]
                plt.plot(x_data,y_data,color=self.GREY,lw=0.5)
                
    #    
    def draw_path(self,goalpaths:list[GridPath]) -> None:
        for goalpath in goalpaths:        
            # for a path draw each node and connection to node parent 
            node = goalpath.goalnode
            while node.location is not goalpath.startnode.location:
                plt.scatter(node.location[0],node.location[1],
                            color=self.LIGHTBLUE,s=20,marker="o")
                x_data = [node.location[0],node.parent.location[0]]
                y_data = [node.location[1],node.parent.location[1]]
                plt.plot(x_data,y_data,color=self.LIGHTBLUE,lw=0.5)
                # next node
                node = node.parent

    
    def draw_patLOS(self) -> None:
        pass
    

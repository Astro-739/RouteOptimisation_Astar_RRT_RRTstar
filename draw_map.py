import pygame
import time
import os
import matplotlib.pyplot as plt
from RRT_algorithm import TreeResults
from RRT_algorithm import TreePath
from route_opt_utils import CircleObstacle



class RoutingMap_pygame:
    def __init__(self,
                 start_xy:(int,int),
                 goal_xy:(int,int),
                 mapdimensions:(int,int),
                 obstacles:list[CircleObstacle]
                 ) -> None:
        self.start_xy = start_xy                                # tuple of xy coordinates
        self.goal_xy = goal_xy                                  # tuple of xy coordinates
        self.mapdimensions = mapdimensions                      # tuple of yx dimensions
        self.mapheight,self.mapwidth = self.mapdimensions       # int of y and x dimension
        self.obstacles = obstacles
        
        # window settings
        #os.environ['SDL_VIDEO_WINDOW_POS'] = "%d,%d" % (2200,300)
        pygame.init()
        self.mapwindowname = 'RRT (Star) path planning'
        pygame.display.set_caption(self.mapwindowname)
        self.map = pygame.display.set_mode((self.mapwidth,self.mapheight))
        self.map.fill((255,255,255))
        self.NODERADIUS = 2
        self.NODETHICKNESS = 0
        self.EDGETHICKNESS = 1
        
        # obstacles list
        self.obstacles = obstacles
        
        #colours
        self.GREY = (153, 153, 153)
        self.BLUE = (0,0,255)
        self.DARKBLUE = (0,0,200)
        self.LIGHTBLUE = (77, 166, 255)
        self.GREEN = (0,255,0)
        self.RED = (255,0,0)
        self.WHITE = (255,255,255)
        self.PURPLE = (112,41,99)


    # draw base map with start and goal locations and obstacles
    def draw_basemap(self):
        # goal location
        pygame.draw.circle(self.map,self.GREEN,self.goal_xy,self.NODERADIUS+20,5)
        # start location
        pygame.draw.circle(self.map,self.GREEN,self.start_xy,self.NODERADIUS+8,5)
        # draw start node
        pygame.draw.circle(self.map,self.GREY,self.start_xy,self.NODERADIUS+3,0)
        # draw all obstacles on map
        for circle in self.obstacles:
            pygame.draw.circle(self.map,self.RED,circle.location,circle.radius,self.NODERADIUS+2)
        
    # draw all children and their connections to parent in tree
    def draw_tree(self,randomtree:TreePath):
        for node in randomtree.nodes:
            pygame.draw.circle(self.map,self.GREY,node.location,self.NODERADIUS+3,0)
            if not node.location == self.start_xy and node.parent is not None:
                pygame.draw.line(self.map,self.GREY,node.location,node.parent.location,self.EDGETHICKNESS)
    
    # draw each path starting at goal
    def draw_path(self,
                 results:TreeResults,
                 colour:(int,int,int),
                 linethickness:int
                 ) -> None:
        for goalnode in results.goalpath:
            # for a path draw each node and connection to node parent 
            node = goalnode
            while node.location is not results.start_location:
                pygame.draw.circle(self.map,colour,node.location,self.NODERADIUS+3,0)
                pygame.draw.line(self.map,colour,node.location,node.parent.location,linethickness)
                node = node.parent
        # draw start location
        pygame.draw.circle(self.map,colour,results.start_location,self.NODERADIUS+3,0)

    def draw_pathLOS(self,
                    results:TreeResults,
                    colour:(int,int,int),
                    linethickness:int
                    ) -> None:
        # draw each path starting at goal
        for goalnode in results.goalpath:
            # for a path draw each node and connection to node parent 
            node = goalnode
            # catch None value for pathLOS_parent
            if node.pathLOS_parent is None:
                print("no LOS path calculated")
                break
            while node.location is not results.start_location:
                pygame.draw.circle(self.map,colour,node.location,self.NODERADIUS+3,0)
                pygame.draw.line(self.map,colour,node.location,node.pathLOS_parent.location,linethickness + 1)
                node = node.pathLOS_parent
        # draw start location
        pygame.draw.circle(self.map,colour,results.start_location,self.NODERADIUS+3,0)

    # print text on map window
    def print_text_on_map(self,num_nodes:int) -> None:
        pygame.font.init()
        map_font = pygame.font.SysFont('Arial',14)
        line1 = "nodes: " + str(num_nodes)
        text_surface = map_font.render(line1,False,(0,0,0))
        self.map.blit(text_surface,(self.mapwidth - 80,self.mapheight - 20))
        #line2 = "time: " + str(round(calc_time,2)) + "s"
        #text_surface = map_font.render(line2,False,(0,0,0))
        #self.map.blit(text_surface,(self.mapwidth - 150,self.mapheight - 20))
        pygame.display.update()

    def update_map(self,randomtree:TreePath) -> None:
        self.map.fill((255,255,255))
        self.draw_basemap()
        self.draw_tree(randomtree)
        pygame.display.update()
        pygame.event.clear()
        time.sleep(0.5)



class RoutingMap_matplotlib:
    def __init__(self,
                 start_xy:(int,int),
                 goal_xy:(int,int),
                 mapdimensions:(int,int),
                 obstacles:list[CircleObstacle]
                 ) -> None:
        self.start_xy = start_xy
        self.start_x,self.start_y = start_xy                                # tuple of xy coordinates
        self.goal_x,self.goal_y = goal_xy                                  # tuple of xy coordinates
        self.mapdimensions = mapdimensions                      # tuple of yx dimensions
        self.mapheight,self.mapwidth = self.mapdimensions       # int of y and x dimension
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
        plt.xlim([0,self.mapwidth])
        plt.ylim([0,self.mapheight])
        # plot start and goal location
        plt.scatter(self.start_x,self.start_y,color=self.GREEN,s=100,marker="o")
        plt.scatter(self.goal_x,self.goal_y,color=self.GREEN,s=300,marker="o")
        # plot circle obsticles
        for circle in self.obstacles:
            circle = plt.Circle(circle.location,circle.radius,
                                color=self.RED,lw=2,fill=False)
            axes.add_patch(circle)

    #
    def draw_tree(self,randomtree:TreePath) -> None:
        for node in randomtree.nodes:
            plt.scatter(node.location[0],node.location[1],
                        color=self.GREY,s=20,marker="o")
            if not node.location == self.start_xy and node.parent is not None:
                x_data = [node.location[0],node.parent.location[0]]
                y_data = [node.location[1],node.parent.location[1]]
                plt.plot(x_data,y_data,color=self.GREY,lw=0.5)
                
    #    
    def draw_path(self,results:TreeResults,) -> None:
        for goalnode in results.goalpath:
            # for a path draw each node and connection to node parent 
            node = goalnode
            while node.location is not results.start_location:
                plt.scatter(node.location[0],node.location[1],
                            color=self.LIGHTBLUE,s=20,marker="o")
                x_data = [node.location[0],node.parent.location[0]]
                y_data = [node.location[1],node.parent.location[1]]
                plt.plot(x_data,y_data,color=self.LIGHTBLUE,lw=0.5)
                # next node
                node = node.parent
        # draw start location
        #pygame.draw.circle(self.map,colour,results.start_location,self.NODERADIUS+3,0)

    
    def draw_patLOS(self) -> None:
        pass
    

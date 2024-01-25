import pygame
import time
from icecream import ic
from RRT_algorithm import TreeResults
from RRT_algorithm import TreePath
from route_opt_utils import CircleObstacle

class RRTMap:
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
    def drawBaseMap(self,obstacles:list[CircleObstacle]):
        pygame.draw.circle(self.map,self.GREEN,self.goal_xy,self.NODERADIUS+20,5)       # goal location
        pygame.draw.circle(self.map,self.GREEN,self.start_xy,self.NODERADIUS+8,5)       # start location
        pygame.draw.circle(self.map,self.GREY,self.start_xy,self.NODERADIUS+3,0)        # draw start node
        # draw all obstacles on map
        for circle in obstacles:
            pygame.draw.circle(self.map,self.RED,circle.location,circle.radius,self.NODERADIUS+2)
        
    # draw all children and their connections to parent in tree
    def drawTree(self,randomtree:TreePath):
        for node in randomtree.nodes:
            pygame.draw.circle(self.map,self.GREY,node.location,self.NODERADIUS+3,0)
            if not node.location == self.start_xy and node.parent is not None:
                pygame.draw.line(self.map,self.GREY,node.location,node.parent.location,self.EDGETHICKNESS)
    
    # draw each path starting at goal
    def drawPath(self,
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

    #todo integrate with above?
    def drawPathLOS(self,
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
    def printTextOnMap(self,num_nodes:int) -> None:
        pygame.font.init()
        map_font = pygame.font.SysFont('Arial',14)
        line1 = "nodes: " + str(num_nodes)
        text_surface = map_font.render(line1,False,(0,0,0))
        self.map.blit(text_surface,(self.mapwidth - 80,self.mapheight - 20))
        #line2 = "time: " + str(round(calc_time,2)) + "s"
        #text_surface = map_font.render(line2,False,(0,0,0))
        #self.map.blit(text_surface,(self.mapwidth - 150,self.mapheight - 20))
        pygame.display.update()

    def updateMap(self,
                  obstacles:list[CircleObstacle],
                  randomtree:TreePath
                  ) -> None:
        self.map.fill((255,255,255))
        self.drawBaseMap(obstacles)
        self.drawTree(randomtree)
        pygame.display.update()
        pygame.event.clear()
        time.sleep(0.5)

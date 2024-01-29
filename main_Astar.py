import pygame
import time
import matplotlib.pyplot as plt
from icecream import ic
from AStar_algorithm import AStarAlgorithm
from draw_map import RoutingMap_pygame,RoutingMap_matplotlib
from create_obstacles import make_circle_obstacles
from create_obstacles import make_circle_riskzones



def main():
    # (height,width) in pygame
    mapdimensions = (1000,1000)          
    # locations in (x,y)
    start_location = (100,100)
    goal_location = (800,700)
    # number of obstacles
    num_obstacles = 12
    # (min,max) radius
    obstacleradius = (50,200)
    # Astar algorithm stepsize (must align with start and goal)
    stepsize = 50
    
    
    riskzones = make_circle_riskzones(mapdimensions,
                                      obstacleradius,
                                      num_obstacles)
    astar = AStarAlgorithm(start_location,
                            goal_location,
                            riskzones,
                            mapdimensions,
                            stepsize)
    
    map2 = RoutingMap_matplotlib(start_location,
                                 goal_location,
                                 mapdimensions,
                                 riskzones)
    map2.draw_basemap()
    #plt.show()
    # draw map with start, goal and obstacles
    map = RoutingMap_pygame(start_location,
                        goal_location,
                        mapdimensions,
                        riskzones)
    map.draw_basemap()
    pygame.display.update()
    print("map drawn")
 
 
    time1 = time.time()
    
    astar.astar_search()
    astar.create_goalpath()
    
    time2 = time.time()
    calc_time = time2 - time1    
    
    astar.finalresults.goalpath = astar.goalpath
    numberofnodes = len(astar.gridnodes.nodes)
    
    map2.draw_tree(astar.gridnodes)
    map2.draw_path(astar.finalresults)
    plt.show()
    
    map.draw_tree(astar.gridnodes)
    pygame.display.update()
    print(f"tree drawn with {numberofnodes} nodes")
    print("calctime =", (calc_time))
    
    if astar.goalfound: 
        map.draw_path(astar.finalresults,map.LIGHTBLUE,map.EDGETHICKNESS)
    else:
        print("goal not found")
    
 
    # print text on map window
    map.print_text_on_map(numberofnodes)
    
    pygame.display.update()
    pygame.event.clear()
    pygame.image.save(map.map,"screenshot.jpeg")
    
    running = True                
    while running:
        event = pygame.event.wait()
        if event.type == pygame.QUIT:
            running = False
    pygame.quit ()
    

# start main    
if __name__ == '__main__':
    main()
    

        

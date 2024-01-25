import pygame
import time
import os
from icecream import ic
from AStar_algorithm import AStarAlgorithm
from draw_map import RRTMap
from create_obstacles import make_circle_obstacles
from create_obstacles import make_circle_riskzones



def main():
    # (height,width) in pygame
    mapdimensions = (1000,1000)          
    # locations in (x,y)
    start_location = (100,100)
    goal_location = (800,700)
    # number of obstacles
    obstaclenum = 12
    # (min,max) radius
    obstacleradius = (50,200)
    # Astar algorithm stepsize (must align with start and goal)
    stepsize = 20
    
    
    riskzones = make_circle_riskzones(mapdimensions,
                                      obstacleradius,
                                      obstaclenum)
    a_star = AStarAlgorithm(start_location,
                            goal_location,
                            riskzones,
                            mapdimensions,
                            stepsize)
    
    # draw map with start, goal and obstacles
    map = RRTMap(start_location,
                 goal_location,
                 mapdimensions,
                 riskzones)
    map.drawBaseMap(riskzones)
    pygame.display.update()
    print("map drawn")
 
 
    time1 = time.time()
    
    a_star.astar_search()
    a_star.create_goalpath()
    
    time2 = time.time()
    calc_time = time2 - time1    
    
    a_star.finalresults.goalpath = a_star.goalpath
    numberofnodes = len(a_star.gridnodes.nodes)
    
    map.drawTree(a_star.gridnodes)
    pygame.display.update()
    print(f"tree drawn with {numberofnodes} nodes")
    print("calctime =", (calc_time))
    
    if a_star.goalfound: 
        map.drawPath(a_star.finalresults,map.LIGHTBLUE,map.EDGETHICKNESS)
    else:
        print("goal not found")
    
 
    # print text on map window
    map.printTextOnMap(numberofnodes)
    
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
    

        

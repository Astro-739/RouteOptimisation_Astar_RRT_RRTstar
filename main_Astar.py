import pygame
import time
import os
from icecream import ic
from AStar_algorithm import AStarAlgorithm
from drawmap import RRTMap
from createobstacles import makeCirleObstacles
from createobstacles import makeRiskZones




def main():
    mapdimensions = (1000,1000)          # (height,width) in pygame
    start_location = (100,100)            # (x,y)
    goal_location = (800,700)           # (x,y)
    obstaclenum = 12
    obstacleradius = (50,200)           # min,max radius
    stepsize = 20
    
    time1 = 0
    update_map = False
    
    obstacles = makeCirleObstacles(start_location,goal_location,mapdimensions,
                                   obstacleradius,obstaclenum)
    riskzones = makeRiskZones(mapdimensions,obstacleradius,obstaclenum)
    
    obstacles = riskzones
    
    a_star = AStarAlgorithm(start_location,goal_location,obstacles,mapdimensions,stepsize)
    
    # draw map with start, goal and obstacles
    map = RRTMap(start_location,goal_location,mapdimensions,obstacles)
    map.drawBaseMap(obstacles)
    pygame.display.update()
    print("map drawn")
 
 
    time1 = time.time()
    
    a_star.astarsearch()

    time2 = time.time()
    calc_time = time2 - time1    
    
    a_star.create_goalpath()
    
    a_star.finalresults.goalpath = a_star.goalpath
    
    map.drawTree(a_star.gridnodes)
    pygame.display.update()
    print(f"tree drawn with {len(a_star.gridnodes.nodes)} nodes")
    print("calctime =", (time2 - time1))
    
    if a_star.goalfound: 
        map.drawPath(a_star.finalresults,map.LIGHTBLUE,map.EDGETHICKNESS)
    else:
        print("goal not found")
    
 
    # print text on map window
    map.printTextOnMap(len(a_star.gridnodes.nodes))
    
    pygame.display.update()
    pygame.event.clear()
    pygame.image.save(map.map,"screenshot.jpeg")
    
    running = True                
    while running:
        event = pygame.event.wait()
        if event.type == pygame.QUIT:
            running = False
    pygame.quit ()
    
    
if __name__ == '__main__':
    main()
    

        

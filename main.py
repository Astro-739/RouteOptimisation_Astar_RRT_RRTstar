import pygame
import time
import os
from icecream import ic
from RRT_algorithm import RRTAlgorithm
from AStar_algorithm import AStarAlgorithm
from drawmap import RRTMap
from createobstacles import makeCirleObstacles




def main():
    mapdimensions = (1000,1000)          # (height,width) in pygame
    start_location = (50,50)            # (x,y)
    goal_location = (710,910)           # (x,y)
    obstaclenum = 8
    obstacleradius = (50,200)           # min,max radius
    update_map = True
    RRTstar = False
    
    obstacles = makeCirleObstacles(start_location,goal_location,mapdimensions,
                                   obstacleradius,obstaclenum)
    
    rrt = RRTAlgorithm(mapdimensions,obstacles,start_location,goal_location,RRTstar)
    a_star = AStarAlgorithm(start_location,goal_location,obstacles,mapdimensions)
    
    
    map = RRTMap(start_location,goal_location,mapdimensions,obstacles)
    map.drawBaseMap(obstacles)
    pygame.display.update()
    print("map drawn")
 
 
    
    calc_time = 0
    iteration = 0
    time1 = time.time()  
    while iteration < 1000: #and not rrt.pathfound:
        
        if iteration % 50 == 0: 
            rrt.biasStep()
        else: 
            rrt.randomStep()
        
        if iteration % 100 == 0: 
            print(f"iteration: {iteration}")
            if update_map:    
                map.updateMap(obstacles,rrt.randomtree)
        iteration +=1

    time2 = time.time()
    calc_time = time2 - time1
    num_nodes = len(rrt.randomtree.nodes)
    
    map.drawTree(rrt.randomtree)
    pygame.display.update()
    print(f"final iteration: {iteration}")
    print(f"tree drawn with {num_nodes} nodes")
    print("calctime (incl waits) =", (calc_time))
    
    if rrt.pathfound: 
        map.drawPath(rrt.finalresult,map.LIGHTBLUE,map.EDGETHICKNESS)
        map.drawPathLOS(rrt.finalresult,map.DARKBLUE,map.EDGETHICKNESS+1)
    else:
        print("goal not found")
    
    
    # print text on map window
    map.printTextOnMap(num_nodes)
    pygame.display.update()
    # save screenshot
    pygame.event.clear()
    pygame.image.save(map.map,"screenshot.jpeg")
    # display open until click quit    
    running = True                
    while running:
        event = pygame.event.wait()
        if event.type == pygame.QUIT:
            running = False
    pygame.quit ()
    
    
if __name__ == '__main__':
    main()
    

        

import pygame
import time
from icecream import ic
from RRT_algorithm import RRTAlgorithm
from draw_map import RoutingMap_pygame,RoutingMap_matplotlib
from create_obstacles import make_circle_obstacles




def main():
    # (height,width) in pygame
    mapdimensions = (1000,1000)
    # locations in (x,y)
    start_location = (50,50)
    goal_location = (710,910)
    # number of obstacles
    num_obstacles = 8
    # (min,max) radius
    obstacleradius = (50,200)
    # node stepsize
    stepsize = 20
    
    update_map = True
    RRTstar = False
    
    obstacles = make_circle_obstacles(start_location,
                                      goal_location,
                                      mapdimensions,
                                      obstacleradius,
                                      num_obstacles)
    rrt = RRTAlgorithm(mapdimensions,
                       obstacles,
                       start_location,
                       goal_location,
                       stepsize,
                       RRTstar)
    map = RoutingMap_pygame(start_location,
                            goal_location,
                            mapdimensions,
                            obstacles)
    
    map.draw_basemap()
    pygame.display.update()
    print("map drawn")
 
    iteration = 0
    while iteration < 1000: #and not rrt.pathfound:
        
        if iteration % 50 == 0: 
            rrt.biasStep()
        else: 
            rrt.randomStep()
        
        if iteration % 100 == 0: 
            print(f"iteration: {iteration}")
            if update_map:    
                map.update_map(rrt.randomtree)
        iteration +=1

    num_nodes = len(rrt.randomtree.nodes)
    
    map.draw_tree(rrt.randomtree)
    pygame.display.update()
    print(f"final iteration: {iteration}")
    print(f"tree drawn with {num_nodes} nodes")
    
    if rrt.pathfound: 
        map.draw_path(rrt.finalresult,map.LIGHTBLUE,map.EDGETHICKNESS)
        map.draw_pathLOS(rrt.finalresult,map.DARKBLUE,map.EDGETHICKNESS+1)
    else:
        print("goal not found")
    
    
    # print text on map window
    map.print_text_on_map(num_nodes)
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
    

        

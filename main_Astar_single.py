import time
import matplotlib.pyplot as plt
from icecream import ic
from Astar_algorithm_singlegoal import AStarAlgorithm
from draw_map import RoutingMap_matplotlib
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
    
    map = RoutingMap_matplotlib(start_location,
                                 goal_location,
                                 mapdimensions,
                                 riskzones)
    # draw map with start, goal and obstacles
    map.draw_basemap()
    print("map drawn")
 
    time1 = time.time()
    
    astar.astar_search()
    astar.create_goalpath()
    
    time2 = time.time()
    calc_time = time2 - time1    
    
    astar.finalresults.goalpath = astar.goalpath
    numberofnodes = len(astar.gridnodes.nodes)
    
    map.draw_tree(astar.gridnodes)
    map.draw_path(astar.finalresults)
    
    print(f"tree drawn with {numberofnodes} nodes")
    print("calctime =", (calc_time))
    
    plt.show()
    

# start main    
if __name__ == '__main__':
    main()
    

        

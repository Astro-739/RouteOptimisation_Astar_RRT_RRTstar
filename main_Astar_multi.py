import time
import matplotlib.pyplot as plt
from icecream import ic
from AStar_algorithm_multi import AStarAlgorithm
from draw_map_multi import RoutingMap_matplotlib
from create_obstacles import make_circle_riskzones



def main():
    #  map width (x), map height (y)
    mapdimensions = (1000,1000)          
    # locations in (x,y)
    start_location = (100,100)
    goal_locations = [(800,700),(605,210),(305,408),(205,811),(503,905)]
    # number of obstacles
    num_obstacles = 12
    # (min,max) radius
    obstacleradius = (50,200)
    # Astar algorithm stepsize (must align with start and goal)
    stepsize = 25
    
    riskzones = make_circle_riskzones(mapdimensions,
                                      obstacleradius,
                                      num_obstacles)
    astar = AStarAlgorithm(start_location,
                           goal_locations,
                           riskzones,
                           mapdimensions,
                           stepsize)
    map = RoutingMap_matplotlib(start_location,
                                goal_locations,
                                mapdimensions,
                                riskzones)
    # draw map with start, goal and obstacles
    map.draw_basemap()
    print("map drawn")
 
    time1 = time.time()
    astar.astar_search()
    astar.create_goalpath()
    time2 = time.time()
    
    map.draw_tree(astar.gridnodes)
    map.draw_path(astar.goalpaths)
    
    numberofnodes = len(astar.gridnodes)
    calc_time = time2 - time1
    print(f"tree drawn with {numberofnodes} nodes")
    print("calctime =", (calc_time))
    
    plt.show()
    

# start main    
if __name__ == '__main__':
    main()
    

        

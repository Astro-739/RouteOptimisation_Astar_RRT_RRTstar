import time
import matplotlib.pyplot as plt
from icecream import ic
from AStar_algorithm_multi import AStarAlgorithm
from draw_map_multi import RoutingMap_matplotlib
from create_obstacles import make_circle_riskzones, read_circle_riskzones



def main():
    #  map width (x), map height (y)
    mapdimensions = (1000,1000)          
    # locations in (x,y)
    start_locations = [(10,100),(10,500),(10,900)]
    goal_locations = [(800,700),(605,210),(305,408),(205,811),(503,905)]
    # number of obstacle
    num_obstacles = 12
    # (min,max) radius
    obstacleradius = (50,200)
    # Astar algorithm stepsize
    stepsize = 40
    
    riskzones = make_circle_riskzones(mapdimensions,
                                      obstacleradius,
                                      num_obstacles)
    
    #riskzones = read_circle_riskzones()
    
    astar = AStarAlgorithm(start_locations,
                           goal_locations,
                           riskzones,
                           mapdimensions,
                           stepsize)
    map = RoutingMap_matplotlib(start_locations,
                                goal_locations,
                                mapdimensions,
                                riskzones)
    # draw map with start, goal and obstacles
    map.draw_basemap()
    print("map drawn")
 
    start = time.perf_counter()
    astar.astar_search()
    astar.create_goalpaths()
    astar.create_LOS_goalpaths_level3()
    end = time.perf_counter()
    
    map.draw_tree(astar.gridnodes)
    map.draw_paths(astar.goalpaths)
    map.draw_LOS_paths(astar.goalpaths)
    
    # riskzone locations
    print("zones: ",{circle.location:circle.radius for circle in riskzones})
    print(f"tree drawn with {len(astar.gridnodes)} nodes")
    print("calctime =", (end - start))
        
    plt.savefig("figure.png")
    plt.show()

    

# start main    
if __name__ == '__main__':
    main()
    

        

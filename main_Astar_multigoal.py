import time
import matplotlib.pyplot as plt
from icecream import ic
from Astar_algorithm_multigoal import AStarAlgorithm
from draw_map_multigoal import RoutingMap_matplotlib
from create_riskzones import make_circle_riskzones, read_circle_riskzones



def main():
    # map width (x), map height (y)
    mapdimensions = (1000,1000)          
    # Astar algorithm stepsize
    stepsize = 40
    # locations in (x,y)
    # root location enables multiple start locations
    # root location is not shown in graph
    root_location = (-30, 500)
    start_locations = [(root_location[0] + stepsize, root_location[1] - 400),
                       (root_location[0] + stepsize, root_location[1]),
                       (root_location[0] + stepsize, root_location[1] + 400)]
    #start_locations = [(10,100),(10,500),(10,900)]
    #start_locations = [(100,100)]
    goal_locations = [(800,700),(605,210),(305,408),(205,811),(503,905),(860,235)]
    #goal_locations = [(800,700),(605,210)]
    # number of obstacle
    num_riskzones = 12
    # (min,max) radius
    riskzone_radius = (50,200)
    
    riskzones = make_circle_riskzones(mapdimensions,
                                      riskzone_radius,
                                      num_riskzones)
    
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
    astar.create_los_goalpaths_level3()
    end = time.perf_counter()
    
    map.draw_tree(astar.gridnodes)
    map.draw_paths(astar.goalpaths)
    map.draw_los_paths(astar.goalpaths)
    
    # riskzone locations
    print("zones: ",{circle.location:circle.radius for circle in riskzones})
    print(f"tree drawn with {len(astar.gridnodes)} nodes")
    print("calctime =", (end - start))
        
    plt.savefig("figure.png")
    plt.show()

    

# start main    
if __name__ == '__main__':
    main()
    

        

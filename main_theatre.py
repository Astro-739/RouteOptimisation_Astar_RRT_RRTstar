import random
import create_theatre
from create_theatre import Theatre



def main() -> None:
    
    theatre = Theatre()
    
    theatre.create_random_theatre()
    helper_boxes = False
    theatre.draw_theatre(helper_boxes)
    
    #theatre.create_threatre_mesh()
    


# start main    
if __name__ == '__main__':
    main()
    

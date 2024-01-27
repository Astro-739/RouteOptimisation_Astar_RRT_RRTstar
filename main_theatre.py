import random
import create_theatre
from create_theatre import Theatre



def main() -> None:
    
    theatre = Theatre()
    
    theatre.create_random_theatre()
    theatre.draw_theatre()


# start main    
if __name__ == '__main__':
    main()
    

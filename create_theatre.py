import random
import math
from shapely.geometry import Point,Polygon
import matplotlib.pyplot as plt



class Theatre:
    def __init__(self) -> None:
        pass
    
    def create_random_theatre(self) -> None:
        BLUE_BOX_ABS = [(200,300),(450,700)]
        RED_BOX_ABS = [(550,300),(800,700)]
        AIRFIELD_BOX_REL = [(50,100),(200,200)]
        CV_BOX_REL = [(0,0),(200,50)]
        
    
    def create_threatre_mesh(self) -> None:
        pass
    
    def draw_theatre(self) -> None:
        pass
    
    
class SurfaceObject:
    def __init__(self,location,value,side) -> None:
        self.location = location
        self.value = value
        self.side = side


class Airfield(SurfaceObject):
    def __init__(self, location, value, radius, side) -> None:
        super().__init__(location, value, side)
        self.radius = radius
        self.fighters = []
        self.groundattackers = []
        self.awacs = []
    
    
class Vehicle(SurfaceObject):
    def __init__(self,location,value,velocity,side) -> None:
        super().__init__(location,value,side)
        self.velocity = velocity


class SAM_AAA(SurfaceObject):
    def __init__(self,location,value,velocity,radius,side) -> None:
        super().__init__(location,value,side)
        self.velocity = velocity
        self.radius = radius
    
    
class CV(SurfaceObject):
    def __init__(self,location,value,velocity,radius,side) -> None:
        super().__init__(location,value,side)
        self.velocity = velocity
        self.radius = radius
        self.fighters = []
        self.groundattackers = []
        self.awacs = []
    
    
class Factory(SurfaceObject):
    def __init__(self,location,value,side) -> None:
        super().__init__(location,value,side)
        pass
    
    
class AirObject:
    def __init__(self) -> None:
        pass
    
    
class Fighter(AirObject):
    def __init__(self) -> None:
        super().__init__()
        pass
    
    
class GroundAttacker(AirObject):
    def __init__(self) -> None:
        super().__init__()
        pass
    
    
class AWACS(AirObject):
    def __init__(self) -> None:
        super().__init__()
        pass
    
    
    
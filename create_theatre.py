import random
import math
from shapely.geometry import Point,Polygon,LineString,MultiPoint
from shapely.geometry import box
import matplotlib.pyplot as plt
import numpy as np


class Theatre:
    def __init__(self) -> None:
        self.blue_airfields = []
        self.blue_sams = []
        self.blue_vehicles = []
        self.blue_factories = []
        self.blue_all = []
        self.red_airfields = []
        self.red_sams = []
        self.red_vehicles = []
        self.red_factories = []
        self.red_all = []
        self.all_units = []
        # map dimensions
        self.MAPWIDTH = 1000
        self.MAPHEIGHT = 1000
        # boxes for random unit placement
        self.BLUE_BOX_ABS = (275,400)
        self.BLUE_BOX_HEIGHT = 275
        self.BLUE_BOX_WIDTH = 200
        self.RED_BOX_ABS = (525,325)
        self.RED_BOX_HEIGHT = 325
        self.RED_BOX_WIDTH = 200
        self.BLUE_AIRFIELD_BOX_REL = (25,75)
        self.BLUE_AIRFIELD_BOX_HEIGHT = 150
        self.BLUE_AIRFIELD_BOX_WIDTH = 150
        self.RED_AIRFIELD_BOX_REL = (25,50)
        self.RED_AIRFIELD_BOX_HEIGHT = 200
        self.RED_AIRFIELD_BOX_WIDTH = 150
        self.BLUE_LORAD_N_BOX_REL = (75,175)
        self.BLUE_LORAD_S_BOX_REL = (75,75)
        self.RED_LORAD_N_BOX_REL = (75,175)
        self.RED_LORAD_S_BOX_REL = (75,75)
        self.LORAD_BOX_HEIGHT = 50
        self.LORAD_BOX_WIDTH = 50
        # colours
        self.GREY = (0.6,0.6,0.6)
        self.BLUE = (0,0,1)
        self.DARKBLUE = (0,0,0.78)
        self.LIGHTBLUE = "#66a3ff"
        self.GREEN = (0,1,0)
        self.RED = (1,0,0)
        self.LIGHTRED = "#ff8080"
        self.WHITE = (1,1,1)
        self.PURPLE = (0.44,0.16,0.39)

        self.fig,self.axes = plt.subplots(1,1,figsize=(10, 10))
        plt.xlim([0,self.MAPWIDTH])
        plt.ylim([0,self.MAPHEIGHT])

    
    def create_random_theatre(self) -> None:
        # blue vehicles
        num_vehicles = round(random.uniform(10,15))
        print("blue vehicles: ",num_vehicles)
        for i in range(0,num_vehicles):
            x_random = int(random.uniform(self.BLUE_BOX_ABS[0],
                                          self.BLUE_BOX_ABS[0] + self.BLUE_BOX_WIDTH))
            y_random = int(random.uniform(self.BLUE_BOX_ABS[1],
                                          self.BLUE_BOX_ABS[1] + self.BLUE_BOX_HEIGHT))
            blue_vehicle = Vehicle((x_random,y_random),0,10,"blue")
            self.blue_vehicles.append(blue_vehicle)
            self.blue_all.append(blue_vehicle)

        # blue factories
        num_factories = round(random.uniform(3,5))
        print("blue factories: ",num_factories)
        for i in range(0,num_factories):
            x_random = int(random.uniform(self.BLUE_BOX_ABS[0],
                                          self.BLUE_BOX_ABS[0] + self.BLUE_BOX_WIDTH))
            y_random = int(random.uniform(self.BLUE_BOX_ABS[1],
                                          self.BLUE_BOX_ABS[1] + self.BLUE_BOX_HEIGHT))
            blue_factory = Factory((x_random,y_random),100,"blue")
            self.blue_factories.append(blue_factory)
            self.blue_all.append(blue_factory)

        # blue merad
        num_merad = round(random.uniform(2,5))
        print("blue merad: ",num_merad)
        for i in range(0,num_merad):
            x_random = int(random.uniform(self.BLUE_BOX_ABS[0],
                                          self.BLUE_BOX_ABS[0] + self.BLUE_BOX_WIDTH))
            y_random = int(random.uniform(self.BLUE_BOX_ABS[1],
                                          self.BLUE_BOX_ABS[1] + self.BLUE_BOX_HEIGHT))
            blue_merad = SAM_AAA((x_random,y_random),0,30,"merad",50,"blue")
            self.blue_sams.append(blue_merad)
            self.blue_all.append(blue_merad)
        
        # blue airfield random locations (between 1 and 2 airfields)
        num_airfields = round(random.uniform(1,2))
        print("blue airfields: ",num_airfields)
        for i in range(0,num_airfields):
            x_random = int(random.uniform(self.BLUE_BOX_ABS[0] + self.BLUE_AIRFIELD_BOX_REL[0],
                                          self.BLUE_BOX_ABS[0] + self.BLUE_AIRFIELD_BOX_REL[0] + self.BLUE_AIRFIELD_BOX_WIDTH))
            y_random = int(random.uniform(self.BLUE_BOX_ABS[1] + self.BLUE_AIRFIELD_BOX_REL[1],
                                          self.BLUE_BOX_ABS[1] + self.BLUE_AIRFIELD_BOX_REL[1] + self.BLUE_AIRFIELD_BOX_HEIGHT))
            blue_airfield = Airfield((x_random,y_random),100,90,"blue")
            self.blue_airfields.append(blue_airfield)
            self.blue_all.append(blue_airfield)
            blue_merad = SAM_AAA((x_random+5,y_random+5),0,30,"merad",50,"blue")
            self.blue_sams.append(blue_merad)
            self.blue_all.append(blue_merad)

        # blue lorad N random locations (between 1 and 1)
        num_lorad = round(random.uniform(1,1))
        print("blue lorad N: ",num_lorad)
        if num_lorad == 1:
            x_random = int(random.uniform(self.BLUE_BOX_ABS[0] + self.BLUE_LORAD_N_BOX_REL[0],
                                          self.BLUE_BOX_ABS[0] + self.BLUE_LORAD_N_BOX_REL[0] + self.LORAD_BOX_WIDTH))
            y_random = int(random.uniform(self.BLUE_BOX_ABS[1] + self.BLUE_LORAD_N_BOX_REL[1],
                                          self.BLUE_BOX_ABS[1] + self.BLUE_LORAD_N_BOX_REL[1] + self.LORAD_BOX_HEIGHT))
            blue_lorad = SAM_AAA((x_random,y_random),0,150,"lorad",80,"blue")
            self.blue_sams.append(blue_lorad)
            self.blue_all.append(blue_lorad)

        # blue lorad S random locations (between 0 and 0)
        num_lorad = 0
        print("blue lorad S: ",num_lorad)

        # red vehicles
        num_vehicles = round(random.uniform(15,20))
        print("red vehicles: ",num_vehicles)
        for i in range(0,num_vehicles):
            x_random = int(random.uniform(self.RED_BOX_ABS[0],
                                          self.RED_BOX_ABS[0] + self.RED_BOX_WIDTH))
            y_random = int(random.uniform(self.RED_BOX_ABS[1],
                                          self.RED_BOX_ABS[1] + self.RED_BOX_HEIGHT))
            red_vehicle = Vehicle((x_random,y_random),0,10,"red")
            self.red_vehicles.append(red_vehicle)
            self.red_all.append(red_vehicle)

        # red factories
        num_factories = round(random.uniform(4,6))
        print("red factories: ",num_factories)
        for i in range(0,num_factories):
            x_random = int(random.uniform(self.RED_BOX_ABS[0],
                                          self.RED_BOX_ABS[0] + self.RED_BOX_WIDTH))
            y_random = int(random.uniform(self.RED_BOX_ABS[1],
                                          self.RED_BOX_ABS[1] + self.RED_BOX_HEIGHT))
            red_factory = Factory((x_random,y_random),100,"red")
            self.red_factories.append(red_factory)
            self.red_all.append(red_factory)

        # red merad
        num_merad = round(random.uniform(10,15))
        print("red merad: ",num_merad)
        for i in range(0,num_merad):
            x_random = int(random.uniform(self.RED_BOX_ABS[0],
                                          self.RED_BOX_ABS[0] + self.RED_BOX_WIDTH))
            y_random = int(random.uniform(self.RED_BOX_ABS[1],
                                          self.RED_BOX_ABS[1] + self.RED_BOX_HEIGHT))
            red_merad = SAM_AAA((x_random,y_random),0,40,"merad",50,"red")
            self.red_sams.append(red_merad)
            self.red_all.append(red_merad)

        # red airfield random locations (between 2 and 5 airfields)
        num_airfields = round(random.uniform(2,4))
        print("red airfields: ",num_airfields)
        for i in range(0,num_airfields):
            x_random = int(random.uniform(self.RED_BOX_ABS[0] + self.RED_AIRFIELD_BOX_REL[0],
                                          self.RED_BOX_ABS[0] + self.RED_AIRFIELD_BOX_REL[0] + self.RED_AIRFIELD_BOX_WIDTH))
            y_random = int(random.uniform(self.RED_BOX_ABS[1] + self.RED_AIRFIELD_BOX_REL[1],
                                          self.RED_BOX_ABS[1] + self.RED_AIRFIELD_BOX_REL[1] + self.RED_AIRFIELD_BOX_HEIGHT))
            red_airfield = Airfield((x_random,y_random),100,90,"red")
            self.red_airfields.append(red_airfield)
            self.red_all.append(red_airfield)
            red_merad = SAM_AAA((x_random-5,y_random+5),0,40,"merad",50,"red")
            self.red_sams.append(red_merad)
            self.red_all.append(red_merad)

        # red lorad N random locations (between 1 and 1)
        num_lorad = round(random.uniform(1,1))
        print("red lorad N: ",num_lorad)
        if num_lorad == 1:
            x_random = int(random.uniform(self.RED_BOX_ABS[0] + self.RED_LORAD_N_BOX_REL[0],
                                          self.RED_BOX_ABS[0] + self.RED_LORAD_N_BOX_REL[0] + self.LORAD_BOX_WIDTH))
            y_random = int(random.uniform(self.RED_BOX_ABS[1] + self.RED_LORAD_N_BOX_REL[1],
                                          self.RED_BOX_ABS[1] + self.RED_LORAD_N_BOX_REL[1] + self.LORAD_BOX_HEIGHT))
            red_lorad = SAM_AAA((x_random,y_random),0,250,"lorad",80,"red")
            self.red_sams.append(red_lorad)
            self.red_all.append(red_lorad)

        # red lorad S random locations (between 0 and 1)
        num_lorad = round(random.uniform(0,1))
        print("red lorad S: ",num_lorad)
        if num_lorad == 1:
            x_random = int(random.uniform(self.RED_BOX_ABS[0] + self.RED_LORAD_S_BOX_REL[0],
                                          self.RED_BOX_ABS[0] + self.RED_LORAD_S_BOX_REL[0] + self.LORAD_BOX_WIDTH))
            y_random = int(random.uniform(self.RED_BOX_ABS[1] + self.RED_LORAD_S_BOX_REL[1],
                                          self.RED_BOX_ABS[1] + self.RED_LORAD_S_BOX_REL[1] + self.LORAD_BOX_HEIGHT))
            red_lorad = SAM_AAA((x_random,y_random),0,180,"lorad",80,"red")
            self.red_sams.append(red_lorad)
            self.red_all.append(red_lorad)

    
    def create_threatre_mesh(self) -> None:
        
        points = []
        
        points1 = ([unit.location for unit in self.blue_airfields])
        points2 = ([unit.location for unit in self.blue_factories])
        points3 = ([unit.location for unit in self.blue_sams])
        points4 = ([unit.location for unit in self.blue_vehicles])
        points5 = ([unit.location for unit in self.blue_all])
    
        points = points1 + points2 + points3 + points4
        print("points: ",points)
        print("all points: ",points5)
        
        hull = MultiPoint(points5).convex_hull
        x,y = hull.exterior.xy
        plt.plot(x,y)
        buffer = hull.buffer(50)
        a,b = buffer.exterior.xy
        plt.plot(a,b)
        centroid = hull.centroid
        print("centroid: ",centroid)
        
        plt.scatter(centroid.x,centroid.y,color="purple",s=200,marker="*")
            
        polygon1 = Polygon(points)
        x,y = polygon1.exterior.xy
        plt.plot(x,y)

        bounds = polygon1.bounds
        print("bounds: ",bounds)
        buffer = polygon1.buffer(100)
        a,b = buffer.exterior.xy
        plt.plot(a,b)
        
        box1 = box(*bounds)
        a,b = box1.exterior.xy
        plt.plot(a,b)
        buffer1 = box1.buffer(100)
        a,b = buffer1.exterior.xy
        plt.plot(a,b)
        
        
        
        
        
        
        plt.show()
    
    
    def draw_theatre(self,helper_boxes:bool) -> None:
        # draw helper boxes
        if helper_boxes:
            self.draw_helper_boxes()
        # draw units on map
        # blue vehicles
        for vehicle in self.blue_vehicles:
            plt.scatter(vehicle.location[0],vehicle.location[1],color=self.LIGHTBLUE,s=100,marker="o")
        # blue factories
        for factory in self.blue_factories:
            plt.scatter(factory.location[0],factory.location[1],color="blue",s=100,marker="H")
        # red vehicles
        for vehicle in self.red_vehicles:
            plt.scatter(vehicle.location[0],vehicle.location[1],color=self.LIGHTRED,s=100,marker="o")
        # red factories
        for factory in self.red_factories:
            plt.scatter(factory.location[0],factory.location[1],color="red",s=100,marker="H")
        # blue sams
        for sam in self.blue_sams:
            plt.scatter(sam.location[0],sam.location[1],color=self.LIGHTBLUE,s=100,marker="o")
            circle = plt.Circle(sam.location,sam.radius,color="blue",lw=0.5,fill=False)
            self.axes.add_patch(circle)
        # red sams
        for sam in self.red_sams:
            plt.scatter(sam.location[0],sam.location[1],color=self.LIGHTRED,s=100,marker="o")
            circle = plt.Circle(sam.location,sam.radius,color="red",lw=0.5,fill=False)
            self.axes.add_patch(circle)
        # blue airfields
        for airfield in self.blue_airfields:
            plt.scatter(airfield.location[0],airfield.location[1],color="blue",s=100,marker="s")
        # red airfields
        for airfield in self.red_airfields:
            plt.scatter(airfield.location[0],airfield.location[1],color="red",s=100,marker="D")
    
        # locations
        blue_points = ([unit.location for unit in self.blue_all])
        blue_values = ([unit.value for unit in self.blue_all])
        red_points = ([unit.location for unit in self.red_all])
        red_values = ([unit.value for unit in self.red_all])
        
        # convex hull
        blue_hull = MultiPoint(blue_points).convex_hull
        red_hull = MultiPoint(red_points).convex_hull
        x,y = blue_hull.exterior.xy
        plt.plot(x,y,"b--",lw=0.5)
        x,y = red_hull.exterior.xy
        plt.plot(x,y,"r--",lw=0.5)
        
        # centroid points
        blue_centroid = blue_hull.centroid
        red_centroid = red_hull.centroid
        plt.scatter(blue_centroid.x,blue_centroid.y,color="purple",s=200,marker="*")
        plt.scatter(red_centroid.x,red_centroid.y,color="purple",s=200,marker="*")
        
        
        # centre of gravity points
        blue_cg_x = sum([point[0] for point in blue_points]) / len(blue_points)
        blue_cg_y = sum([point[1] for point in blue_points]) / len(blue_points)        
        print("blue_cg: ",(blue_cg_x,blue_cg_y))
        plt.scatter(blue_cg_x,blue_cg_y,color="blue",s=200,marker="*")

        blue_wcg_x = sum([unit.location[0]*unit.value for unit in self.blue_all]) / sum(blue_values)
        blue_wcg_y = sum([unit.location[1]*unit.value for unit in self.blue_all]) / sum(blue_values)
        print("blue weighted cg: ",(blue_wcg_x,blue_wcg_y))
        plt.scatter(blue_wcg_x,blue_wcg_y,color="blue",s=200,marker="*")
        
        red_cg_x = sum([point[0] for point in red_points]) / len(red_points)
        red_cg_y = sum([point[1] for point in red_points]) / len(red_points)        
        print("red_cg: ",(red_cg_x,red_cg_y))
        plt.scatter(red_cg_x,red_cg_y,color="red",s=200,marker="*")
        
        red_wcg_x = sum([unit.location[0]*unit.value for unit in self.red_all]) / sum(red_values)
        red_wcg_y = sum([unit.location[1]*unit.value for unit in self.red_all]) / sum(red_values)
        print("blue weighted cg: ",(blue_wcg_x,blue_wcg_y))
        plt.scatter(red_wcg_x,red_wcg_y,color="red",s=200,marker="*")
        

        # connecting cgs
        
        
        blue_wcg = np.array([blue_wcg_x,blue_wcg_y])
        red_wcg = np.array([red_wcg_x,red_wcg_y])
        point = np.array([blue_points[0]])

        print("point: ",point)

        dist = np.cross(red_wcg - blue_wcg,point-blue_wcg)/np.linalg.norm(red_wcg - blue_wcg)
        print("dist: ",dist)

        """# examples
        d = norm(np.cross(p2-p1, p1-p3))/norm(p2-p1)

        p1=np.array([0,0])
        p2=np.array([10,10])
        p3=np.array([5,7])
        d=np.cross(p2-p1,p3-p1)/np.linalg.norm(p2-p1)
        """


        plt.show()
    
    
    def draw_helper_boxes(self) -> None:
        # blue box
        x_data = [self.BLUE_BOX_ABS[0],
                  self.BLUE_BOX_ABS[0],
                  self.BLUE_BOX_ABS[0] + self.BLUE_BOX_WIDTH,
                  self.BLUE_BOX_ABS[0] + self.BLUE_BOX_WIDTH,
                  self.BLUE_BOX_ABS[0]]
        y_data = [self.BLUE_BOX_ABS[1],
                  self.BLUE_BOX_ABS[1] + self.BLUE_BOX_HEIGHT,
                  self.BLUE_BOX_ABS[1] + self.BLUE_BOX_HEIGHT,
                  self.BLUE_BOX_ABS[1],
                  self.BLUE_BOX_ABS[1]]
        plt.plot(x_data,y_data,"b--",lw=0.5)
        # blue airfield box
        x_data = [self.BLUE_BOX_ABS[0] + self.BLUE_AIRFIELD_BOX_REL[0],
                  self.BLUE_BOX_ABS[0] + self.BLUE_AIRFIELD_BOX_REL[0],
                  self.BLUE_BOX_ABS[0] + self.BLUE_AIRFIELD_BOX_REL[0] + self.BLUE_AIRFIELD_BOX_WIDTH,
                  self.BLUE_BOX_ABS[0] + self.BLUE_AIRFIELD_BOX_REL[0] + self.BLUE_AIRFIELD_BOX_WIDTH,
                  self.BLUE_BOX_ABS[0] + self.BLUE_AIRFIELD_BOX_REL[0]]
        y_data = [self.BLUE_BOX_ABS[1] + self.BLUE_AIRFIELD_BOX_REL[1],
                  self.BLUE_BOX_ABS[1] + self.BLUE_AIRFIELD_BOX_REL[1] + self.BLUE_AIRFIELD_BOX_HEIGHT,
                  self.BLUE_BOX_ABS[1] + self.BLUE_AIRFIELD_BOX_REL[1] + self.BLUE_AIRFIELD_BOX_HEIGHT,
                  self.BLUE_BOX_ABS[1] + self.BLUE_AIRFIELD_BOX_REL[1],
                  self.BLUE_BOX_ABS[1] + self.BLUE_AIRFIELD_BOX_REL[1]]
        plt.plot(x_data,y_data,"b--",lw=0.5)
        # blue lorad N box
        x_data = [self.BLUE_BOX_ABS[0] + self.BLUE_LORAD_N_BOX_REL[0],
                  self.BLUE_BOX_ABS[0] + self.BLUE_LORAD_N_BOX_REL[0],
                  self.BLUE_BOX_ABS[0] + self.BLUE_LORAD_N_BOX_REL[0] + self.LORAD_BOX_WIDTH,
                  self.BLUE_BOX_ABS[0] + self.BLUE_LORAD_N_BOX_REL[0] + self.LORAD_BOX_WIDTH,
                  self.BLUE_BOX_ABS[0] + self.BLUE_LORAD_N_BOX_REL[0]]
        y_data = [self.BLUE_BOX_ABS[1] + self.BLUE_LORAD_N_BOX_REL[1],
                  self.BLUE_BOX_ABS[1] + self.BLUE_LORAD_N_BOX_REL[1] + self.LORAD_BOX_HEIGHT,
                  self.BLUE_BOX_ABS[1] + self.BLUE_LORAD_N_BOX_REL[1] + self.LORAD_BOX_HEIGHT,
                  self.BLUE_BOX_ABS[1] + self.BLUE_LORAD_N_BOX_REL[1],
                  self.BLUE_BOX_ABS[1] + self.BLUE_LORAD_N_BOX_REL[1]]
        plt.plot(x_data,y_data,"b--",lw=0.5)
        # blue lorad S box
        x_data = [self.BLUE_BOX_ABS[0] + self.BLUE_LORAD_S_BOX_REL[0],
                  self.BLUE_BOX_ABS[0] + self.BLUE_LORAD_S_BOX_REL[0],
                  self.BLUE_BOX_ABS[0] + self.BLUE_LORAD_S_BOX_REL[0] + self.LORAD_BOX_WIDTH,
                  self.BLUE_BOX_ABS[0] + self.BLUE_LORAD_S_BOX_REL[0] + self.LORAD_BOX_WIDTH,
                  self.BLUE_BOX_ABS[0] + self.BLUE_LORAD_S_BOX_REL[0]]
        y_data = [self.BLUE_BOX_ABS[1] + self.BLUE_LORAD_S_BOX_REL[1],
                  self.BLUE_BOX_ABS[1] + self.BLUE_LORAD_S_BOX_REL[1] + self.LORAD_BOX_HEIGHT,
                  self.BLUE_BOX_ABS[1] + self.BLUE_LORAD_S_BOX_REL[1] + self.LORAD_BOX_HEIGHT,
                  self.BLUE_BOX_ABS[1] + self.BLUE_LORAD_S_BOX_REL[1],
                  self.BLUE_BOX_ABS[1] + self.BLUE_LORAD_S_BOX_REL[1]]
        plt.plot(x_data,y_data,"b--",lw=0.5)
        # red box
        x_data = [self.RED_BOX_ABS[0],
                  self.RED_BOX_ABS[0],
                  self.RED_BOX_ABS[0] + self.RED_BOX_WIDTH,
                  self.RED_BOX_ABS[0] + self.RED_BOX_WIDTH,
                  self.RED_BOX_ABS[0]]
        y_data = [self.RED_BOX_ABS[1],
                  self.RED_BOX_ABS[1] + self.RED_BOX_HEIGHT,
                  self.RED_BOX_ABS[1] + self.RED_BOX_HEIGHT,
                  self.RED_BOX_ABS[1],
                  self.RED_BOX_ABS[1]]
        plt.plot(x_data,y_data,"r--",lw=0.5)
        # red airfield box
        x_data = [self.RED_BOX_ABS[0] + self.RED_AIRFIELD_BOX_REL[0],
                  self.RED_BOX_ABS[0] + self.RED_AIRFIELD_BOX_REL[0],
                  self.RED_BOX_ABS[0] + self.RED_AIRFIELD_BOX_REL[0] + self.RED_AIRFIELD_BOX_WIDTH,
                  self.RED_BOX_ABS[0] + self.RED_AIRFIELD_BOX_REL[0] + self.RED_AIRFIELD_BOX_WIDTH,
                  self.RED_BOX_ABS[0] + self.RED_AIRFIELD_BOX_REL[0]]
        y_data = [self.RED_BOX_ABS[1] + self.RED_AIRFIELD_BOX_REL[1],
                  self.RED_BOX_ABS[1] + self.RED_AIRFIELD_BOX_REL[1] + self.RED_AIRFIELD_BOX_HEIGHT,
                  self.RED_BOX_ABS[1] + self.RED_AIRFIELD_BOX_REL[1] + self.RED_AIRFIELD_BOX_HEIGHT,
                  self.RED_BOX_ABS[1] + self.RED_AIRFIELD_BOX_REL[1],
                  self.RED_BOX_ABS[1] + self.RED_AIRFIELD_BOX_REL[1]]
        plt.plot(x_data,y_data,"r--",lw=0.5)
        # red lorad N box
        x_data = [self.RED_BOX_ABS[0] + self.RED_LORAD_N_BOX_REL[0],
                  self.RED_BOX_ABS[0] + self.RED_LORAD_N_BOX_REL[0],
                  self.RED_BOX_ABS[0] + self.RED_LORAD_N_BOX_REL[0] + self.LORAD_BOX_WIDTH,
                  self.RED_BOX_ABS[0] + self.RED_LORAD_N_BOX_REL[0] + self.LORAD_BOX_WIDTH,
                  self.RED_BOX_ABS[0] + self.RED_LORAD_N_BOX_REL[0]]
        y_data = [self.RED_BOX_ABS[1] + self.RED_LORAD_N_BOX_REL[1],
                  self.RED_BOX_ABS[1] + self.RED_LORAD_N_BOX_REL[1] + self.LORAD_BOX_HEIGHT,
                  self.RED_BOX_ABS[1] + self.RED_LORAD_N_BOX_REL[1] + self.LORAD_BOX_HEIGHT,
                  self.RED_BOX_ABS[1] + self.RED_LORAD_N_BOX_REL[1],
                  self.RED_BOX_ABS[1] + self.RED_LORAD_N_BOX_REL[1]]
        plt.plot(x_data,y_data,"r--",lw=0.5)
        # red lorad S box
        x_data = [self.RED_BOX_ABS[0] + self.RED_LORAD_S_BOX_REL[0],
                  self.RED_BOX_ABS[0] + self.RED_LORAD_S_BOX_REL[0],
                  self.RED_BOX_ABS[0] + self.RED_LORAD_S_BOX_REL[0] + self.LORAD_BOX_WIDTH,
                  self.RED_BOX_ABS[0] + self.RED_LORAD_S_BOX_REL[0] + self.LORAD_BOX_WIDTH,
                  self.RED_BOX_ABS[0] + self.RED_LORAD_S_BOX_REL[0]]
        y_data = [self.RED_BOX_ABS[1] + self.RED_LORAD_S_BOX_REL[1],
                  self.RED_BOX_ABS[1] + self.RED_LORAD_S_BOX_REL[1] + self.LORAD_BOX_HEIGHT,
                  self.RED_BOX_ABS[1] + self.RED_LORAD_S_BOX_REL[1] + self.LORAD_BOX_HEIGHT,
                  self.RED_BOX_ABS[1] + self.RED_LORAD_S_BOX_REL[1],
                  self.RED_BOX_ABS[1] + self.RED_LORAD_S_BOX_REL[1]]
        plt.plot(x_data,y_data,"r--",lw=0.5)
        
        

    
    
    
class SurfaceObject:
    def __init__(self,location,value,side) -> None:
        self.location = location
        self.value = value
        self.side = side


class Airfield(SurfaceObject):
    def __init__(self, location, radius, value, side) -> None:
        super().__init__(location, value, side)
        self.radius = radius
        self.fighters = []
        self.groundattackers = []
        self.awacs = []
    
    
class Vehicle(SurfaceObject):
    def __init__(self,location,velocity,value,side) -> None:
        super().__init__(location,value,side)
        self.velocity = velocity


class SAM_AAA(SurfaceObject):
    def __init__(self,location,velocity,radius,type,value,side) -> None:
        super().__init__(location,value,side)
        self.velocity = velocity
        self.radius = radius
        self.type = type
    
    
class CV(SurfaceObject):
    def __init__(self,location,velocity,radius,value,side) -> None:
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
    
    
    
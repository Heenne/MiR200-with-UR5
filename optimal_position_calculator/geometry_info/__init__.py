import sys

from abc import ABC, abstractclassmethod
from matplotlib import pyplot as plot
from math import sin, cos, pi
from math import sqrt, pow

import numpy as np

class EdgeInfo:
    start_point: np.array
    edge_vector: np.array

    def __init__(self, start_point: np.array, end_point: np.array):
        self.start_point = start_point
        self.edge_vector = end_point - start_point

    def __str__(self):
        return "Start point: " + str(self.start_point) + " | Edge vector: " + str(self.edge_vector)


class GeometryContour(ABC):
    #Member
    _corner_point_list: list()
    _edge_list: list

    def __init__(self):
        self._corner_point_list = list()
        self._edge_list = list()

        super().__init__()


    @abstractclassmethod
    def import_contour(self):
        pass
        #Assumption, all contours will be spawned with the centre of mass in the x = 0 and y = 0 point of the world 


    def create_contour_edges(self):
        for counter in range(0, len(self._corner_point_list)):
            start_point: np.array = self._corner_point_list[counter]
            end_point: np.array
            if counter + 1 == len(self._corner_point_list):
                end_point = self._corner_point_list[0]
            else:
                end_point = self._corner_point_list[counter+1]

            edge: EdgeInfo = EdgeInfo(start_point= start_point, end_point= end_point)
            self._edge_list.append(edge)


    def print_contour_info(self):
        print("Corner info:")
        for corner in self._corner_point_list:
            print(corner)

        for edge in self._edge_list:
            print(edge)

    def plot_corners(self, **kwargs):
        block: bool = False
        if 'block' in kwargs:
            block = kwargs.get("block")
        
        for point in self._corner_point_list:
            plot.plot(point[0], point[1], "bo")

        plot.show(block=block)

    def plot_edges(self, **kwargs):
        block: bool = False
        if 'block' in kwargs:
            block = kwargs.get("block")

        for edge in self._edge_list:
            x_start: float = edge.start_point[0]
            y_start: float = edge.start_point[1]
            end_point: np.array = edge.start_point + edge.edge_vector
            x_end: float = end_point[0]
            y_end: float = end_point[1]
            plot.plot([x_start, x_end], [y_start, y_end], 'r-')
            pass

        plot.show(block=block)



class Box(GeometryContour):
    _x_length: float
    _y_length: float

    def __init__(self):
        super().__init__()


    def import_contour(self, **kwargs):
        self._x_length: float = float(kwargs.get("x_length"))
        self._y_length: float = float(kwargs.get("y_length"))

        half_x_length: float = self._x_length / 2
        half_y_length: float = self._y_length / 2

        point1: np.array = np.array([half_x_length, half_y_length])
        point2: np.array = np.array([-half_x_length, half_y_length])
        point3: np.array = np.array([-half_x_length, -half_y_length])
        point4: np.array = np.array([half_x_length, -half_y_length])

        self._corner_point_list.append(point1)
        self._corner_point_list.append(point2)
        self._corner_point_list.append(point3)
        self._corner_point_list.append(point4)

        self.create_contour_edges()
   

class Cylinder(GeometryContour):
    __RESOLUTION: int = 100
    _radius: float

    def __init__(self):
        super().__init__()

    def import_contour(self, **kwargs):
        self._radius = float(kwargs.get("radius"))

        for counter in range(0,self.__RESOLUTION):
            x_value = self._radius * cos(((2*pi)/self.__RESOLUTION) * counter)
            y_value = self._radius * sin(((2*pi)/self.__RESOLUTION) * counter)

            point = np.array([x_value, y_value])
            self._corner_point_list.append(point)
        
        self.create_contour_edges()

class IsoscelesTriangle(GeometryContour):
    __DEFAULT_SIDE_LENGTH: float = 1.0
    __GAZEBO_FACTOR: float = 1000 #This factor is necessary as Gazebo puts a 1000 factor in between. So in the urdf file is a necessary 0.001 factor to show to right dimensions of the model in Gazebo
    _scaled_side_length: float
    _scale_factor: float

    def __init__(self):
        super().__init__()


    def import_contour(self, **kwargs):
        self._scale_factor: float = float(kwargs.get("scale")) * self.__GAZEBO_FACTOR
        
        self._scaled_side_length: float = self.__DEFAULT_SIDE_LENGTH * self._scale_factor
        scaled_height: float = self._calculate_height(self._scaled_side_length)

        point1: np.array = np.array([(self._scaled_side_length / 2), (-scaled_height / 3)])
        point2: np.array = np.array([0, ((2*scaled_height) / 3)])
        point3: np.array = np.array([(-self._scaled_side_length / 2), (-scaled_height / 3)])

        self._corner_point_list.append(point1)
        self._corner_point_list.append(point2)
        self._corner_point_list.append(point3)

        self.create_contour_edges()

    def _calculate_default_height(self) -> float:
        return self._calculate_height(self.__DEFAULT_SIDE_LENGTH)

    def _calculate_height(self, side_length: float) -> float:
        return sqrt(pow(side_length, 2) - pow((side_length / 2), 2))


class RightAngledTriangle(GeometryContour):
    __GAZEBO_FACTOR: float = 1000 #This factor is necessary as Gazebo puts a 1000 factor in between. So in the urdf file is a necessary 0.001 factor to show to right dimensions of the model in Gazebo
    _DEFAULT_X_SIDE_LENGTH: float = 1.0
    _DEFAULT_Y_SIDE_LENGTH: float = 1.0
    _x_scale_factor: float
    _y_scale_factor: float
    _scaled_x_side_length: float
    _scaled_y_side_length: float

    def __init__(self):
        super().__init__()

    def import_contour(self, **kwargs):
        self._x_scale_factor: float = float(kwargs.get("x_scale")) * self.__GAZEBO_FACTOR
        self._y_scale_factor: float = float(kwargs.get("y_scale")) * self.__GAZEBO_FACTOR

        self._scaled_x_side_length = self._DEFAULT_X_SIDE_LENGTH * self._x_scale_factor
        self._scaled_y_side_length = self._DEFAULT_Y_SIDE_LENGTH * self._y_scale_factor

        point1: np.array = np.array([((2*self._scaled_x_side_length) / 3), (-self._scaled_y_side_length / 3)])
        point2: np.array = np.array([(-self._scaled_x_side_length / 3), ((2*self._scaled_y_side_length) / 3)])
        point3: np.array = np.array([(-self._scaled_x_side_length / 3), (-self._scaled_y_side_length / 3)])

        self._corner_point_list.append(point1)
        self._corner_point_list.append(point2)
        self._corner_point_list.append(point3)

        self.create_contour_edges()

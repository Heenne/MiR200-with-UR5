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


    def calculate_area(self) -> float:
        #For formula see: https://en.wikipedia.org/wiki/Centroid under "Of a polygon"
        area: float = 0.0
        for counter in range(0, len(self._corner_point_list)):
            if (counter+1) == len(self._corner_point_list):
                part_area: float = ((self._corner_point_list[counter][0]*self._corner_point_list[0][1]) - (self._corner_point_list[0][0]*self._corner_point_list[counter][1]))    
                area = area + part_area
            else:
                part_area: float = ((self._corner_point_list[counter][0]*self._corner_point_list[counter+1][1]) - (self._corner_point_list[counter+1][0]*self._corner_point_list[counter][1]))    
                area = area + part_area
        
        area = 0.5 * area
        return area


    def calculate_centroid(self) -> np.array:
        #For formula see: https://en.wikipedia.org/wiki/Centroid under "Of a polygon"
        geometry_area: float = self.calculate_area()

        x_centroid: float = 0.0
        y_centroid: float = 0.0

        x_first_factor: float = 0.0
        y_first_factor: float = 0.0
        second_factor: float = 0.0

        for counter in range(0, len(self._corner_point_list)):
            if (counter+1) == len(self._corner_point_list):
                second_factor = ((self._corner_point_list[counter][0] * self._corner_point_list[0][1]) - (self._corner_point_list[0][0] * self._corner_point_list[counter][1]))
                x_first_factor = self._corner_point_list[counter][0] + self._corner_point_list[0][0]
                y_first_factor = self._corner_point_list[counter][1] + self._corner_point_list[0][1]

            else:
                second_factor = ((self._corner_point_list[counter][0] * self._corner_point_list[counter+1][1]) - (self._corner_point_list[counter+1][0] * self._corner_point_list[counter][1]))
                x_first_factor = self._corner_point_list[counter][0] + self._corner_point_list[counter+1][0]
                y_first_factor = self._corner_point_list[counter][1] + self._corner_point_list[counter+1][1]

            x_centroid = x_centroid + (x_first_factor * second_factor)
            y_centroid = y_centroid + (y_first_factor * second_factor)
            
        x_centroid = (1/(6 * geometry_area)) * x_centroid
        y_centroid = (1/(6 * geometry_area)) * y_centroid

        centroid_point: np.array = np.array([x_centroid, y_centroid])
        return centroid_point


    def calculate_orthogonal_vector_point_to_line(self, orthogonal_point: np.array, line: np.array, start_point_line: np.array) -> np.array:
        nominator: float = -(line[0] * start_point_line[0]) - (line[1] * start_point_line[1])
        denominator: float = pow(line[0], 2) + pow(line[1], 2)

        factor: float
        try:
            factor = nominator / denominator
        except ZeroDivisionError:
            print("Devided by zero in the 'calculate_orthogonal_vector_point_to_line' method!")
            return None

        point_on_line: np.array = orthogonal_point + start_point_line + factor * line
        vector_point_to_line: np.array = point_on_line - orthogonal_point
        return vector_point_to_line


    def print_contour_info(self):
        print("Corner info:")

        print("Geometry area: " + str(self.calculate_area()))
        print("Centroid point: " + str(self.calculate_centroid()))

        for corner in self._corner_point_list:
            print(corner)

        for edge in self._edge_list:
            print(edge)

    def plot_corners(self, **kwargs):
        block: bool = self.check_if_block_exists(**kwargs)
        
        for point in self._corner_point_list:
            plot.plot(point[0], point[1], "bo")

        plot.show(block=block)

    def plot_edges(self, **kwargs):
        block: bool = self.check_if_block_exists(**kwargs)

        for edge in self._edge_list:
            x_start: float = edge.start_point[0]
            y_start: float = edge.start_point[1]
            end_point: np.array = edge.start_point + edge.edge_vector
            x_end: float = end_point[0]
            y_end: float = end_point[1]
            plot.plot([x_start, x_end], [y_start, y_end], 'r-')

        plot.show(block=block)

    def plot_centroid(self, **kwargs):
        block: bool = self.check_if_block_exists(**kwargs)

        centroid_point: np.array = self.calculate_centroid()
        plot.plot(centroid_point[0], centroid_point[1], "go")

        plot.show(block=block)

    def plot_orthogonal_vector_centroid_to_edge(self, **kwargs):
        block: bool = self.check_if_block_exists(**kwargs)
        centroid = self.calculate_centroid()
        for edge in self._edge_list:
            orthogonal_vector: np.array = self.calculate_orthogonal_vector_point_to_line(centroid, edge.edge_vector, edge.start_point)
            plot.plot([centroid[0], centroid[0]+orthogonal_vector[0]], [centroid[1], orthogonal_vector[1]], "g-")

        plot.show(block=block)

    def check_if_block_exists(self, **kwargs) -> bool:
        block: bool = False
        if 'block' in kwargs:
            block = kwargs.get("block")
        return block



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

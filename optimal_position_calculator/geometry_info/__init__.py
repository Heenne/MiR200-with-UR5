import sys

from abc import ABC, abstractclassmethod
from matplotlib import pyplot as plot
from math import sin, cos, pi
from math import sqrt, pow

import numpy as np

class EdgeInfo:
    _start_point: np.array = None
    _end_point: np.array = None
    _edge_vector: np.array = None

    def __init__(self, **kwargs):
        if "start_point" in kwargs:
            self.start_point = kwargs.get("start_point")
        if "end_point" in kwargs:    
            self.end_point = kwargs.get("end_point")

        self.calculate_edge_vector()

    def __str__(self):
        output: str = ""
        if self._start_point is not None:
            output = output + "Start point: " + str(self._start_point)
        if self._end_point is not None:
            output = output + " | End point: " + str(self._end_point)
        if self._edge_vector is not None:
            output = output + " | Edge vector: " + str(self._edge_vector)
        return  output

    @property
    def start_point(self):
        return self._start_point

    @start_point.setter
    def start_point(self, value: np.array):
        self._start_point = value
        self.calculate_edge_vector()

    @property
    def end_point(self):
        return self._end_point

    @end_point.setter
    def end_point(self, value: np.array):
        self._end_point = value
        self.calculate_edge_vector()

    @property
    def edge_vector(self):
        return self._edge_vector


    def calculate_edge_vector(self):
        if self._start_point is not None and self._end_point is not None:
            self._edge_vector = self._end_point - self._start_point
    


class GeometryContour(ABC):
    #Member
    _corner_point_list: list()
    _edge_list: list

    def __init__(self):
        self._corner_point_list = list()
        self._edge_list = list()

        super().__init__()


    @property
    def corner_point_list(self) -> list:
        return self._corner_point_list

    
    @property
    def edge_list(self) -> list:
        return self._edge_list


    @abstractclassmethod
    def import_contour(self, **kwargs):
        pass
        #Assumption, all contours will be spawned with the centre of mass in the x = 0 and y = 0 point of the world 


    def import_contour_with_offset(self, contour: 'GeometryContour', offset_value: float):
        for counter in range(0,len(contour.edge_list)):
            before_edge: np.array
            current_edge: np.array

            if (counter - 1) < 0:
                before_edge = contour.edge_list[-1] # last element
            else:
                before_edge = contour.edge_list[counter - 1]

            current_edge = contour.edge_list[counter]
            centroid: np.array = contour.calculate_centroid()

            orthogonal_before_edge: np.array = self.calculate_orthogonal_vector_point_to_line(orthogonal_point= centroid,
                                                                                              line=before_edge.edge_vector,
                                                                                              start_point_line=before_edge.start_point)

            extended_orthogonal_before_edge: np.array = self.extend_vector_by_length(orthogonal_before_edge, offset_value)

            orthogonal_current_edge: np.array = self.calculate_orthogonal_vector_point_to_line(orthogonal_point= centroid,
                                                                                               line=current_edge.edge_vector,
                                                                                               start_point_line=current_edge.start_point)

            extended_orthogonal_current_edge: np.array = self.extend_vector_by_length(orthogonal_current_edge, offset_value)

            start_point: np.array = self.calculator_vector_line_intersection_point(global_lead_vector= centroid,
                                                                                   lead_vector_1=extended_orthogonal_current_edge,
                                                                                   direction_vector_1=current_edge.edge_vector,
                                                                                   lead_vector_2=extended_orthogonal_before_edge,
                                                                                   direction_vector_2=before_edge.edge_vector)

            self._corner_point_list.append(start_point)

        self.create_contour_edges()


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


    def extend_vector_by_length(self, vector_to_extend: np.array, length_to_extend: float) -> np.array :
        extended_vector: np.array
        length_of_vector: float = np.linalg.norm(vector_to_extend)
        extended_vector = ((length_of_vector + length_to_extend) / length_of_vector) * vector_to_extend
        return extended_vector


    def calculator_vector_line_intersection_point(self, global_lead_vector: np.array, lead_vector_1: np.array, direction_vector_1: np.array, lead_vector_2: np.array, direction_vector_2: np.array) -> np.array:
        factor_1_numerator: float = ((lead_vector_2[1] * direction_vector_2[0]) + (lead_vector_1[0] * direction_vector_2[1]) - (lead_vector_2[0] * direction_vector_2[1]) - (lead_vector_1[1] * direction_vector_2[0]))
        factor_1_denumerator: float = ((direction_vector_1[1] * direction_vector_2[0]) - (direction_vector_1[0] * direction_vector_2[1]))

        factor_1: float = factor_1_numerator / factor_1_denumerator

        return global_lead_vector + lead_vector_1 + (factor_1 * direction_vector_1)
        

    # def calculate_extended_corner_position(self, edge: EdgeInfo, centroid: np.array, contour_offset: float) -> np.array:
    #     orthogonal_vector: np.array = self.calculate_orthogonal_vector_point_to_line(centroid, edge.edge_vector, edge.start_point)
    #     extended_orthogonal_vector: np.array = self.extend_vector_by_length(orthogonal_vector, contour_offset)
    #     print(extended_orthogonal_vector)
    #     centroid_to_corner_vector: np.array = edge.start_point - centroid
    #     print(centroid_to_corner_vector)

    #     factor_numerator = ((extended_orthogonal_vector[1] * edge.edge_vector[0]) - (extended_orthogonal_vector[0] * edge.edge_vector[1]))
    #     factor_denominator = ((centroid_to_corner_vector[1] * edge.edge_vector[0]) - (centroid_to_corner_vector[0] * edge.edge_vector[1]))
        
    #     factor: float
    #     try:
    #         print(str(factor_numerator) + " | " + str(factor_denominator))
    #         factor = factor_numerator / factor_denominator
    #     except ZeroDivisionError:
    #         print("Devided by zero in the 'calculate_extended_corner_position' method!")
    #         return None

    #     print(factor)

    #     extended_corner: np.array = centroid + (factor * centroid_to_corner_vector)
        
    #     return extended_corner


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
            orthogonal_vector = self.extend_vector_by_length(orthogonal_vector, 0.3)
            plot.plot([centroid[0], centroid[0]+orthogonal_vector[0]], [centroid[1], orthogonal_vector[1]], "g-")

        plot.show(block=block)

    
    # def plot_extended_corners(self):

    #     for edge in self._edge_list:
    #         extended_corner = self.calculate_extended_corner_position(edge, self.calculate_centroid(), 0.3)
    #         plot.plot([edge.start_point[0] , extended_corner[0]], [edge.start_point[1] , extended_corner[1]], "g-")

    #     plot.show(block=False)


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

import sys

from abc import ABC, abstractclassmethod
from matplotlib import pyplot as plot
import numpy as np

from geometry_info.edge_info import EdgeInfo


class GeometryContour:
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

            start_point: np.array = self.calculate_vector_line_intersection_point(lead_vector_1=extended_orthogonal_current_edge,
                                                                                   direction_vector_1=current_edge.edge_vector,
                                                                                   lead_vector_2=extended_orthogonal_before_edge,
                                                                                   direction_vector_2=before_edge.edge_vector)

            self._corner_point_list.append(start_point)

        self.create_contour_edges()


    def add_contour_corner(self, additional_corner: np.array):
        self._corner_point_list.append(additional_corner)
        self.create_contour_edges()


    def replace_contour_corner(self, index: int, new_corner_point: np.array):
        self._corner_point_list[index] = new_corner_point
        self.create_contour_edges()


    def get_index_of_corner(self, searched_corner: np.array) -> int:
        index: int = 0
        for corner_point in self._corner_point_list:
            if corner_point is searched_corner:
                return index

            index = index + 1
        return -1


    def create_contour_edges(self):
        self._edge_list.clear()
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


    def calculate_distance_point_to_line(self, point: np.array, line: np.array, start_point_line: np.array) -> float:
        vector_point_to_line: np.array = self.calculate_orthogonal_vector_point_to_line(point, line, start_point_line)
        return np.linalg.norm(vector_point_to_line)


    def get_closest_edge_to_point(self, point: np.array) -> EdgeInfo:
        shortest_edge: EdgeInfo = self._edge_list[0]
        shortest_distance: float = self.calculate_distance_point_to_line(point, shortest_edge.edge_vector, shortest_edge.start_point)
        for edge_info in self._edge_list:
            new_distance: float = self.calculate_distance_point_to_line(point, edge_info.edge_vector, edge_info.start_point)
            if new_distance < shortest_distance:
                shortest_edge = edge_info
                shortest_distance = new_distance

        return shortest_edge


    def extend_vector_by_length(self, vector_to_extend: np.array, length_to_extend: float) -> np.array :
        extended_vector: np.array
        length_of_vector: float = np.linalg.norm(vector_to_extend)
        extended_vector = ((length_of_vector + length_to_extend) / length_of_vector) * vector_to_extend
        return extended_vector


    def calculate_vector_line_intersection_point(self, lead_vector_1: np.array, direction_vector_1: np.array, lead_vector_2: np.array, direction_vector_2: np.array) -> np.array:
        factor_1_numerator: float = self._calculate_vector_line_intersection_factor_1_numerator(lead_vector_1, lead_vector_2, direction_vector_2)
        factor_1_denumerator: float = self._calculate_vector_line_intersection_factor_1_denumerator(direction_vector_1, direction_vector_2)

        factor_1: float = factor_1_numerator / factor_1_denumerator

        return lead_vector_1 + (factor_1 * direction_vector_1)
    

    def calculate_vector_intersection_point(self, lead_vector_1: np.array, direction_vector_1: np.array, lead_vector_2: np.array, direction_vector_2: np.array):
        factor_1_numerator: float = self._calculate_vector_line_intersection_factor_1_numerator(lead_vector_1, lead_vector_2, direction_vector_2)
        factor_1_denumerator: float = self._calculate_vector_line_intersection_factor_1_denumerator(direction_vector_1, direction_vector_2)
        factor_2_numerator: float = self._calculate_vector_line_intersection_factor_2_numerator(lead_vector_1, direction_vector_1, lead_vector_2)
        factor_2_denumerator: float = self._calculate_vector_line_intersection_factor_2_denumerator(direction_vector_1, direction_vector_2)

        factor_1: float = round(factor_1_numerator / factor_1_denumerator, 10)
        factor_2: float = round(factor_2_numerator / factor_2_denumerator, 10)

        if factor_1 > 0 and factor_1 < 1 and factor_2 > 0 and factor_2 < 1:
            return lead_vector_1 + (factor_1 * direction_vector_1)
        else:
            return None

    def _calculate_vector_line_intersection_factor_1_numerator(self, lead_vector_1: np.array, lead_vector_2: np.array, direction_vector_2: np.array) -> float:
        return ((lead_vector_2[1] * direction_vector_2[0]) + (lead_vector_1[0] * direction_vector_2[1]) - (lead_vector_2[0] * direction_vector_2[1]) - (lead_vector_1[1] * direction_vector_2[0]))


    def _calculate_vector_line_intersection_factor_1_denumerator(self, direction_vector_1: np.array, direction_vector_2: np.array) -> float:
        return ((direction_vector_1[1] * direction_vector_2[0]) - (direction_vector_1[0] * direction_vector_2[1]))


    def _calculate_vector_line_intersection_factor_2_numerator(self, lead_vector_1: np.array, direction_vector_1: np.array, lead_vector_2: np.array) -> float:
        return ((lead_vector_1[1] * direction_vector_1[0]) + (lead_vector_2[0] * direction_vector_1[1]) - (lead_vector_1[0] * direction_vector_1[1]) - (lead_vector_2[1] * direction_vector_1[0]))


    def _calculate_vector_line_intersection_factor_2_denumerator(self, direction_vector_1: np.array, direction_vector_2: np.array) -> float:
        return ((direction_vector_2[1] * direction_vector_1[0]) - (direction_vector_2[0] * direction_vector_1[1]))

    def get_x_max(self):
        max_x: float = self._corner_point_list[0][0] #Get first x value as init value
        for corner_point in self._corner_point_list:
            if(max_x < corner_point[0]):
                max_x = corner_point[0]

        return max_x

    def get_x_min(self):
        min_x: float = self._corner_point_list[0][0] #Get first x value as init value
        for corner_point in self._corner_point_list:
            if(min_x > corner_point[0]):
                min_x = corner_point[0]

        return min_x


    def get_y_max(self):
        max_y: float = self._corner_point_list[0][1] #Get first y value as init value
        for corner_point in self._corner_point_list:
            if(max_y < corner_point[1]):
                max_y = corner_point[1]

        return max_y

    def get_y_min(self):
        min_y: float = self._corner_point_list[0][1] #Get first y value as init value
        for corner_point in self._corner_point_list:
            if(min_y > corner_point[1]):
                min_y = corner_point[1]

        return min_y
    

    def is_point_in_contour(self, point: np.array) -> bool:
        #First simple check if point is in max values of polygon
        if point[0] > self.get_x_max() or point[0] < self.get_x_min() or point[1] > self.get_y_max() or point[1] < self.get_y_min():
            return False

        testing_direction_vector: np.array = np.array([1.34567,0.2345678]) #Selected a very weird vector because I see no solution to find out if this vector iscrossing through a corner or just being a tangent to the corner. This makes it impossible to distinguish between in or out of the contour
        #This is an easy but ugly fix
        
        intersections: int = 0

        for edge_info in self._edge_list:
            factor_1_numerator: float= self._calculate_vector_line_intersection_factor_1_numerator(edge_info.start_point, point, testing_direction_vector)
            factor_1_denumerator: float= self._calculate_vector_line_intersection_factor_1_denumerator(edge_info.edge_vector, testing_direction_vector)
            factor_2_numerator: float= self._calculate_vector_line_intersection_factor_2_numerator(edge_info.start_point, edge_info.edge_vector, point)
            factor_2_denumerator: float= self._calculate_vector_line_intersection_factor_2_denumerator(edge_info.edge_vector, testing_direction_vector)

            if factor_1_denumerator == 0 or factor_2_denumerator == 0:
                continue

            factor_1: float = factor_1_numerator / factor_1_denumerator
            factor_2: float = factor_2_numerator / factor_2_denumerator

            if factor_1 >= 0 and factor_1 < 1 and factor_2 >= 0:
                intersections = intersections + 1

        if (intersections % 2) == 1:
            return True
        else:
            return False


    def do_edges_intersect(self) -> bool:
        for edge_info_outer in self._edge_list:
            for edge_info_inner in self._edge_list:
                if edge_info_inner is edge_info_outer:
                    continue

                intersection_point: np.array = self.calculate_vector_intersection_point(edge_info_outer.start_point, 
                                                                                        edge_info_outer.edge_vector,
                                                                                        edge_info_inner.start_point,
                                                                                        edge_info_inner.edge_vector)
                if intersection_point is not None:
                    return True

        return False



    def print_info(self):
        print("Contour info: ")
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


    def check_if_block_exists(self, **kwargs) -> bool:
        block: bool = False
        if 'block' in kwargs:
            block = kwargs.get("block")
        return block

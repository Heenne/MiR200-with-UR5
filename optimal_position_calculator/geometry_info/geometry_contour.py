import sys

from abc import ABC, abstractclassmethod
from matplotlib import pyplot as plot
from matplotlib import colors as mcolors
import numpy as np
from math import sin, cos, pi

from geometry_info.edge_info import EdgeInfo


class GeometryContour:
    # Member
    # Contains the translation of the geometry coordinate system
    _geometric_centroid_world_cs: np.array
    # The geometric centroid of the contour should allways be at 0/0 in the geometry coordinate system
    _geometric_centroid_geometry_cs: np.array = np.array([0.0, 0.0])
    _rotation_radian: float
    # Contains the rotation and translation of the centroid from the world cs
    _tf_world_to_geometry_cs: np.array
    _tf_geometry_to_world_cs: np.array

    _corner_point_list_geometry_cs: list
    _edge_list_geometry_cs: list

    def __init__(self, geometric_centroid_world_cs: np.array = np.array([0, 0]), rotation_radian: float = 0.0):
        """Init method for the GeometryContour class.

        :param geometric_centroid_world_cs: If the geometric controid of the geometry is not at the center
        of the world coordinate system, then insert the lead vector here otherwise, defaults to np.array([0, 0])
        :type geometric_centroid_world_cs: np.array, optional
        :param rotation_radian: In [rad]. If the coordinate system of the geometry is rotated in comparisson to the
        world coordinate system then insert the angle here in radian, defaults to 0.0
        :type rotation_radian: float, optional
        """
        # rotation_radian: float = ((2 * pi) / 360) * rotation_degrees
        self._set_tf_matrix_to_world_cs(geometric_centroid_world_cs, rotation_radian)

        self._corner_point_list_geometry_cs = list()
        self._edge_list_geometry_cs = list()

    # region Properties
    @property
    def tf_world_to_geometry_cs(self) -> np.array:
        """Property for the transformation from the world coordinate system to the geometry coordinate system.

        :return: 3x3 transformation matrix with 2x2 rotation and 2x1 translation
        :rtype: np.array
        """
        return self._tf_world_to_geometry_cs

    @property
    def tf_geometry_to_world_cs(self) -> np.array:
        """Property for the transformation from the geometry coordinate system to the world coordinate system.

        :return: 3x3 transformation matrix with 2x2 rotation and 2x1 translation
        :rtype: np.array
        """
        return self._tf_geometry_to_world_cs

    @property
    def geometric_centroid_world_cs(self) -> np.array:
        """Geometric centroid (not center of mass) in the world coordinate system.

        :return: 2x1 vector
        :rtype: np.array
        """
        return self._geometric_centroid_world_cs

    @property
    def geometric_centroid_geometry_cs(self) -> np.array:
        """Geometric centroid (not center of mass) in the geometry coordinate system.

        :return: 2x1 vector
        :rtype: np.array
        """
        return self._geometric_centroid_geometry_cs

    @property
    def rotation_radian(self) -> float:
        """Rotation of the geometry coordinate system to the world coordinate system.
        This is also contained in the transformation matrix.

        :return: Rotation in radian
        :rtype: float
        """
        return self._rotation_radian

    @property
    def corner_point_list_geometry_cs(self) -> list:
        """List of all corners of the contour.
        Corner points are specified in the geometry coordinate system.

        :return: list of all corner elements which are specified as 2x1 vectors
        :rtype: list
        """
        return self._corner_point_list_geometry_cs

    @property
    def edge_list_geometry_cs(self) -> list:
        """List of all edge elements which are stored as EdgeInfo objects.
        The edge vectors are not specified directly in a coordinate system but the start_point of
        each edge is specified in the geometry coordinate system.

        :return: list of all edge elements as EdgeInfo objects
        :rtype: list
        """
        return self._edge_list_geometry_cs

    @property
    def x_max(self):
        x_max: float = self._corner_point_list_geometry_cs[0][0]  # Get first x value as init value
        for corner_point in self._corner_point_list_geometry_cs:
            if(x_max < corner_point[0]):
                x_max = corner_point[0]

        return x_max

    @property
    def x_min(self):
        x_min: float = self._corner_point_list_geometry_cs[0][0]  # Get first x value as init value
        for corner_point in self._corner_point_list_geometry_cs:
            if(x_min > corner_point[0]):
                x_min = corner_point[0]

        return x_min

    @property
    def y_max(self):
        y_max: float = self._corner_point_list_geometry_cs[0][1]  # Get first y value as init value
        for corner_point in self._corner_point_list_geometry_cs:
            if(y_max < corner_point[1]):
                y_max = corner_point[1]

        return y_max

    @property
    def y_min(self):
        y_min: float = self._corner_point_list_geometry_cs[0][1]  # Get first y value as init value
        for corner_point in self._corner_point_list_geometry_cs:
            if(y_min > corner_point[1]):
                y_min = corner_point[1]

        return y_min
    # endregion

    # region Geometry handling methods
    def import_contour_with_offset(self, contour: 'GeometryContour', offset_value: float):
        # TODO Docstring
        self._set_tf_matrix_to_world_cs(contour.geometric_centroid_world_cs, contour.rotation_radian)

        for counter in range(0, len(contour.edge_list_geometry_cs)):
            before_edge: np.array
            current_edge: np.array

            if (counter - 1) < 0:
                before_edge = contour.edge_list_geometry_cs[-1]  # last element
            else:
                before_edge = contour.edge_list_geometry_cs[counter - 1]

            current_edge = contour.edge_list_geometry_cs[counter]
            # centroid: np.array = contour.calculate_centroid()
            centroid: np.array = np.array([0, 0])  # Centroid should always be a the center. Where the geometry cs is.

            orthogonal_before_edge: np.array = self.calculate_orthogonal_vector_point_to_line(
                orthogonal_point_geometry_cs=centroid,
                line=before_edge.edge_vector,
                start_point_line_geometry_cs=before_edge.start_point)

            extended_orthogonal_before_edge: np.array = self.extend_vector_by_length(orthogonal_before_edge,
                                                                                     offset_value)

            orthogonal_current_edge: np.array = self.calculate_orthogonal_vector_point_to_line(
                orthogonal_point=centroid,
                line=current_edge.edge_vector,
                start_point_line=current_edge.start_point)

            extended_orthogonal_current_edge: np.array = self.extend_vector_by_length(orthogonal_current_edge,
                                                                                      offset_value)

            start_point: np.array = self.calculate_vector_line_intersection_point(
                lead_vector_1=extended_orthogonal_current_edge,
                direction_vector_1=current_edge.edge_vector,
                lead_vector_2=extended_orthogonal_before_edge,
                direction_vector_2=before_edge.edge_vector)

            self._corner_point_list_geometry_cs.append(start_point)

        self.create_contour_edges()

    def _set_transformations(self, geometric_centroid_world_cs: np.array, rotation_radian: float):
        """Method for setting transformation from geometry cs to world and other way around.

        :param geometric_centroid_world_cs: geometric centroid of the contour
        :type geometric_centroid_world_cs: np.array
        :param rotation_radian: rotation of the geometry cs to the world cs
        :type rotation_radian: float
        """
        self._geometric_centroid_world_cs = geometric_centroid_world_cs
        self._rotation_radian = rotation_radian

        self._tf_world_to_geometry_cs = np.array(
            [[cos(rotation_radian), -sin(rotation_radian), geometric_centroid_world_cs[0]],
             [sin(rotation_radian), cos(rotation_radian), geometric_centroid_world_cs[1]],
             [0, 0, 1]])
        self._tf_geometry_to_world_cs = np.linalg.inv(self._tf_world_to_geometry_cs)

    def add_contour_corner_world_cs(self, additional_corner_world_cs: np.array):
        # TODO Docstring
        additional_corner_geometry_cs: np.array = self._tf_geometry_to_world_cs.dot(
            additional_corner_world_cs)
        self.add_contour_corner_geometry_cs(additional_corner_geometry_cs)

    def add_contour_corner_geometry_cs(self, additional_corner_geometry_cs: np.array):
        # TODO Docstring
        self._corner_point_list_geometry_cs.append(additional_corner_geometry_cs)
        self.create_contour_edges()
        # TODO update the centroid point and transformation matrix here

    def replace_contour_corner_world_cs(self, corner_index: int, new_corner_point_world_cs: np.array):
        # TODO Docstring
        new_corner_point_geometry_cs: np.array = self._tf_geometry_to_world_cs.dot(
            new_corner_point_world_cs)
        self.replace_contour_corner_geometry_cs(new_corner_point_geometry_cs)

    def replace_contour_corner_geometry_cs(self, corner_index: int, new_corner_point_geometry_cs: np.array):
        # TODO Docstring
        self._corner_point_geometry_cs_list[corner_index] = new_corner_point_geometry_cs
        self.create_contour_edges()
        # TODO update the centroid point and transformation matrix here

    def create_contour_edges(self):
        """This method deletes all current edges.
        Then uses the corners of the current contour to create new edges between them.
        The order of the corners are used to create the edges.
        """
        self._edge_list_geometry_cs.clear()
        for counter in range(0, len(self._corner_point_list_geometry_cs)):
            start_point: np.array = self._corner_point_list_geometry_cs[counter]
            end_point: np.array
            if counter + 1 == len(self._corner_point_list_geometry_cs):
                end_point = self._corner_point_list_geometry_cs[0]
            else:
                end_point = self._corner_point_list_geometry_cs[counter+1]

            edge: EdgeInfo = EdgeInfo(start_point=start_point, end_point=end_point)
            self._edge_list_geometry_cs.append(edge)

    def get_closest_edge_to_point(self, point: np.array) -> EdgeInfo:
        # TODO Docstring
        closest_edge: EdgeInfo = self._edge_list_geometry_cs[0]
        shortest_distance: float = self.calculate_distance_point_to_line(point,
                                                                         closest_edge.edge_vector,
                                                                         closest_edge.start_point)
        for edge_info in self._edge_list_geometry_cs:
            new_distance: float = self.calculate_distance_point_to_line(point,
                                                                        edge_info.edge_vector,
                                                                        edge_info.start_point)
            if new_distance < shortest_distance:
                closest_edge = edge_info
                shortest_distance = new_distance

        return closest_edge

    def get_longest_edge(self) -> EdgeInfo:
        # TODO Docstring
        longest_edge: EdgeInfo = self._edge_list_geometry_cs[0]  # initialize with first value in list
        longest_edge_length: float = np.linalg.norm(longest_edge.edge_vector)
        for edge in self._edge_list_geometry_cs:
            new_edge_length: float = np.linalg.norm(edge.edge_vector)
            if new_edge_length > longest_edge_length:
                longest_edge_length = new_edge_length
                longest_edge = edge

        return longest_edge

    def get_shortest_edge(self) -> EdgeInfo:
        # TODO Docstring
        shortest_edge: EdgeInfo = self._edge_list_geometry_cs[0]
        shortest_edge_length: float = np.linalg.norm(shortest_edge.edge_vector)
        for edge in self._edge_list_geometry_cs:
            new_edge_length: float = np.linalg.norm(edge.edge_vector)
            if new_edge_length < shortest_edge_length:
                shortest_edge_length = new_edge_length
                shortest_edge = edge

        return shortest_edge

    def extend_vector_by_length(self, vector_to_extend: np.array, length_to_extend: float) -> np.array:
        extended_vector: np.array
        length_of_vector: float = np.linalg.norm(vector_to_extend)
        extended_vector = ((length_of_vector + length_to_extend) / length_of_vector) * vector_to_extend
        return extended_vector

    def is_point_in_contour(self, point: np.array) -> bool:
        # TODO Docstring
        # First simple check if point is in max values of polygon
        if (point[0] > self.x_max() or point[0] < self.x_min() or
                point[1] > self.y_max() or point[1] < self.y_min()):
            return False

        # Selected a very weird vector because I see no solution to find out if this vector is crossing through a corner
        # or just being a tangent to the corner.
        # This makes it impossible to distinguish between in or out of the contour
        # This is an easy but ugly fixs
        testing_direction_vector: np.array = np.array([1.34567, 0.2345678])

        intersections: int = 0

        for edge_info in self._edge_list_geometry_cs:
            factor_1_numerator: float = self._calculate_vector_line_intersection_factor_1_numerator(
                edge_info.start_point, point, testing_direction_vector)
            factor_1_denumerator: float = self._calculate_vector_line_intersection_factor_1_denumerator(
                edge_info.edge_vector, testing_direction_vector)
            factor_2_numerator: float = self._calculate_vector_line_intersection_factor_2_numerator(
                edge_info.start_point, edge_info.edge_vector, point)
            factor_2_denumerator: float = self._calculate_vector_line_intersection_factor_2_denumerator(
                edge_info.edge_vector, testing_direction_vector)

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
        # TODO Docstring
        for edge_info_outer in self._edge_list_geometry_cs:
            for edge_info_inner in self._edge_list_geometry_cs:
                if edge_info_inner is edge_info_outer:
                    continue

                intersection_point: np.array = self.calculate_vector_intersection_point(edge_info_outer.start_point,
                                                                                        edge_info_outer.edge_vector,
                                                                                        edge_info_inner.start_point,
                                                                                        edge_info_inner.edge_vector)
                if intersection_point is not None:
                    return True

        return False
    # endregion

    # region Calculation methods
    def calculate_area(self) -> float:
        """Method for calculating the area in the contour.
        For formula see: https://en.wikipedia.org/wiki/Centroid under "Of a polygon".

        :return: Area in the contour
        :rtype: float
        """
        area: float = 0.0
        for counter in range(0, len(self._corner_point_list_geometry_cs)):
            if (counter+1) == len(self._corner_point_list_geometry_cs):
                part_area: float = ((self._corner_point_list_geometry_cs[counter][0] *
                                     self._corner_point_list_geometry_cs[0][1]) -
                                    (self._corner_point_list_geometry_cs[0][0] *
                                     self._corner_point_list_geometry_cs[counter][1]))
                area = area + part_area
            else:
                part_area: float = ((self._corner_point_list_geometry_cs[counter][0] *
                                     self._corner_point_list_geometry_cs[counter+1][1]) -
                                    (self._corner_point_list_geometry_cs[counter+1][0] *
                                     self._corner_point_list_geometry_cs[counter][1]))
                area = area + part_area

        area = 0.5 * area
        return area

    def calculate_centroid(self) -> np.array:
        """Method for calculating the geometric centroid of the contour
        For formula see: https://en.wikipedia.org/wiki/Centroid under "Of a polygon"

        :return: 2x1 vector in the geometry cs that points to the geometric centroid
        :rtype: np.array
        """
        geometry_area: float = self.calculate_area()

        x_centroid: float = 0.0
        y_centroid: float = 0.0

        x_first_factor: float = 0.0
        y_first_factor: float = 0.0
        second_factor: float = 0.0

        for counter in range(0, len(self._corner_point_list_geometry_cs)):
            if (counter+1) == len(self._corner_point_list_geometry_cs):
                second_factor = ((self._corner_point_list_geometry_cs[counter][0] *
                                  self._corner_point_list_geometry_cs[0][1]) -
                                 (self._corner_point_list_geometry_cs[0][0] *
                                  self._corner_point_list_geometry_cs[counter][1]))
                x_first_factor = (self._corner_point_list_geometry_cs[counter][0] +
                                  self._corner_point_list_geometry_cs[0][0])
                y_first_factor = (self._corner_point_list_geometry_cs[counter][1] +
                                  self._corner_point_list_geometry_cs[0][1])

            else:
                second_factor = ((self._corner_point_list_geometry_cs[counter][0] *
                                  self._corner_point_list_geometry_cs[counter+1][1]) -
                                 (self._corner_point_list_geometry_cs[counter+1][0] *
                                  self._corner_point_list_geometry_cs[counter][1]))
                x_first_factor = (self._corner_point_list_geometry_cs[counter][0] +
                                  self._corner_point_list_geometry_cs[counter+1][0])
                y_first_factor = (self._corner_point_list_geometry_cs[counter][1] +
                                  self._corner_point_list_geometry_cs[counter+1][1])

            x_centroid = x_centroid + (x_first_factor * second_factor)
            y_centroid = y_centroid + (y_first_factor * second_factor)

        x_centroid = (1/(6 * geometry_area)) * x_centroid
        y_centroid = (1/(6 * geometry_area)) * y_centroid

        centroid_point: np.array = np.array([x_centroid, y_centroid])
        return centroid_point

    def calculate_orthogonal_vector_point_to_line(self,
                                                  orthogonal_point_geometry_cs: np. array,
                                                  line: np.array,
                                                  start_point_line_geometry_cs: np.array) -> np.array:
        # TODO Docstring
        nominator: float = -(line[0] * start_point_line_geometry_cs[0]) - (line[1] * start_point_line_geometry_cs[1])
        denominator: float = pow(line[0], 2) + pow(line[1], 2)

        factor: float
        try:
            factor = nominator / denominator
        except ZeroDivisionError:
            print("Devided by zero in the 'calculate_orthogonal_vector_point_to_line' method!")
            return None

        point_on_line: np.array = orthogonal_point_geometry_cs + start_point_line_geometry_cs + factor * line
        vector_point_to_line: np.array = point_on_line - orthogonal_point_geometry_cs
        return vector_point_to_line

    def calculate_distance_point_to_line(self,
                                         point_geometry_cs: np.array,
                                         line: np.array,
                                         start_point_line_geometry_cs: np.array) -> float:
        """Method which calculates the shortest distance from a point to line.
        Line is not limited to the length of the vector from the start_point.
        Shortest distance uses the orthogonal vector from point to line.

        :param point_geometry_cs: Point from where the distance to line should be calculated
        :type point_geometry_cs: np.array
        :param line: Line from where the distance should be calculated
        :type line: np.array
        :param start_point_line_geometry_cs: Start point of the line (lead vector)
        :type start_point_line_geometry_cs: np.array
        :return: Distance point to line
        :rtype: float
        """
        vector_point_to_line: np.array = self.calculate_orthogonal_vector_point_to_line(
            orthogonal_point_geometry_cs=point_geometry_cs,
            line=line,
            start_point_line_geometry_cs=start_point_line_geometry_cs)
        return np.linalg.norm(vector_point_to_line)

    def calculate_contour_length(self) -> float:
        """Sums up all lengths of the edges.

        :return: Length of all edges of the contour
        :rtype: float
        """
        total_length: float = 0.0
        for edge in self._edge_list_geometry_cs:
            edge_length: float = np.linalg.norm(edge.edge_vector)
            total_length += edge_length

        return total_length

    def calculate_vector_line_intersection_point(self,
                                                 lead_vector_1: np.array,
                                                 direction_vector_1: np.array,
                                                 lead_vector_2: np.array,
                                                 direction_vector_2: np.array) -> np.array:
        """This method calculates the point where two vector lines intersect.
        Notice that the direction vectors will be extended with a factor from +/- infinite.
        For calculating intersaction point of two vectors that is in the length of the vectors see:
        calculate_vector_intersection_point
        Watch out that both lead_vectors are in the same coordinate system

        :param lead_vector_1: lead vector of the first vector line
        :type lead_vector_1: np.array
        :param direction_vector_1: direction vector of the first vector line
        :type direction_vector_1: np.array
        :param lead_vector_2: lead vector of the second vector line
        :type lead_vector_2: np.array
        :param direction_vector_2: direction of the second vector line
        :type direction_vector_2: np.array
        :return: 2x1 point where both lines intersect
        :rtype: np.array
        """
        factor_1_numerator: float = self._calculate_vector_line_intersection_factor_1_numerator(lead_vector_1,
                                                                                                lead_vector_2,
                                                                                                direction_vector_2)
        factor_1_denumerator: float = self._calculate_vector_line_intersection_factor_1_denumerator(direction_vector_1,
                                                                                                    direction_vector_2)

        # TODO check for ZeroDivision
        factor_1: float = factor_1_numerator / factor_1_denumerator

        return lead_vector_1 + (factor_1 * direction_vector_1)

    def calculate_vector_intersection_point(self,
                                            lead_vector_1: np.array,
                                            direction_vector_1: np.array,
                                            lead_vector_2: np.array,
                                            direction_vector_2: np.array) -> np.array:
        """This method calculates the point where two vectors intersect.
        Notice that the direction vectors will NOT be extended with a factor from +/- infinite.
        For calculating intersaction point of two vectors lines that will be extended:
        calculate_vector_line_intersection_point
        Watch out that both lead_vectors are in the same coordinate system

        :param lead_vector_1: lead vector of the first vector line
        :type lead_vector_1: np.array
        :param direction_vector_1: direction vector of the first vector line
        :type direction_vector_1: np.array
        :param lead_vector_2: lead vector of the second vector line
        :type lead_vector_2: np.array
        :param direction_vector_2: direction of the second vector line
        :type direction_vector_2: np.array
        :return: 2x1 point where both lines intersect
        :rtype: np.array
        """
        factor_1_numerator: float = self._calculate_vector_line_intersection_factor_1_numerator(lead_vector_1,
                                                                                                lead_vector_2,
                                                                                                direction_vector_2)
        factor_1_denumerator: float = self._calculate_vector_line_intersection_factor_1_denumerator(direction_vector_1,
                                                                                                    direction_vector_2)
        factor_2_numerator: float = self._calculate_vector_line_intersection_factor_2_numerator(lead_vector_1,
                                                                                                direction_vector_1,
                                                                                                lead_vector_2)
        factor_2_denumerator: float = self._calculate_vector_line_intersection_factor_2_denumerator(direction_vector_1,
                                                                                                    direction_vector_2)

        # TODO check for ZeroDivision
        factor_1: float = round(factor_1_numerator / factor_1_denumerator, 10)
        factor_2: float = round(factor_2_numerator / factor_2_denumerator, 10)

        if factor_1 > 0 and factor_1 < 1 and factor_2 > 0 and factor_2 < 1:
            return lead_vector_1 + (factor_1 * direction_vector_1)
        else:
            return None

    def _calculate_vector_line_intersection_factor_1_numerator(self,
                                                               lead_vector_1: np.array,
                                                               lead_vector_2: np.array,
                                                               direction_vector_2: np.array) -> float:
        # TODO Docstring
        return ((lead_vector_2[1] * direction_vector_2[0]) + (lead_vector_1[0] * direction_vector_2[1]) -
                (lead_vector_2[0] * direction_vector_2[1]) - (lead_vector_1[1] * direction_vector_2[0]))

    def _calculate_vector_line_intersection_factor_1_denumerator(self,
                                                                 direction_vector_1: np.array,
                                                                 direction_vector_2: np.array) -> float:
        # TODO Docstring
        return ((direction_vector_1[1] * direction_vector_2[0]) - (direction_vector_1[0] * direction_vector_2[1]))

    def _calculate_vector_line_intersection_factor_2_numerator(self,
                                                               lead_vector_1: np.array,
                                                               direction_vector_1: np.array,
                                                               lead_vector_2: np.array) -> float:
        # TODO Docstring
        return ((lead_vector_1[1] * direction_vector_1[0]) + (lead_vector_2[0] * direction_vector_1[1]) -
                (lead_vector_1[0] * direction_vector_1[1]) - (lead_vector_2[1] * direction_vector_1[0]))

    def _calculate_vector_line_intersection_factor_2_denumerator(self,
                                                                 direction_vector_1: np.array,
                                                                 direction_vector_2: np.array) -> float:
        # TODO Docstring
        return ((direction_vector_2[1] * direction_vector_1[0]) - (direction_vector_2[0] * direction_vector_1[1]))

    def calculate_distance_closest_edge_to_point(self, point: np.array) -> float:
        # TODO Docstring
        closest_edge: EdgeInfo = self.get_closest_edge_to_point(point)
        return self.calculate_distance_point_to_line(point, closest_edge.edge_vector, closest_edge.start_point)

    def calculate_longest_edge_length(self) -> float:
        # TODO Docstring
        longest_edge: EdgeInfo = self.get_longest_edge()
        return np.linalg.norm(longest_edge.edge_vector)

    def calculate_shortest_edge_length(self) -> float:
        # TODO Docstring
        shortest_edge: EdgeInfo = self.get_shortest_edge()
        return np.linalg.norm(shortest_edge.edge_vector)
    # endregion

    # region Print and plot methods
    def print_info(self):
        # TODO Docstring
        print("Contour info: ")
        print("Geometry area: " + str(self.calculate_area()))
        print("Centroid point: " + str(self.geometric_centroid_world_cs))

        for corner in self._corner_point_list_geometry_cs:
            print(corner)

        for edge in self._edge_list_geometry_cs:
            print(edge)

    def plot_corners(self, **kwargs):
        # TODO Docstring
        block: bool = self.check_if_block_exists(**kwargs)

        for point in self._corner_point_list_geometry_cs:
            plot.plot(point[0], point[1], "bo")

        plot.show(block=block)

    def plot_edges(self, **kwargs):
        # TODO Docstring
        block: bool = self.check_if_block_exists(**kwargs)
        color: str = self.check_if_color_exists(**kwargs)

        for edge in self._edge_list_geometry_cs:
            x_start: float = edge.start_point[0]
            y_start: float = edge.start_point[1]
            end_point: np.array = edge.start_point + edge.edge_vector
            x_end: float = end_point[0]
            y_end: float = end_point[1]
            plot.plot([x_start, x_end], [y_start, y_end], linestyle="-", color=color)

        plot.show(block=block)

    def plot_centroid(self, **kwargs):
        # TODO Docstring
        block: bool = self.check_if_block_exists(**kwargs)

        centroid_point: np.array = self.calculate_centroid()
        plot.plot(centroid_point[0], centroid_point[1], "go")

        plot.show(block=block)

    def plot_orthogonal_vector_centroid_to_edge(self, **kwargs):
        # TODO Docstring
        block: bool = self.check_if_block_exists(**kwargs)
        centroid = self.calculate_centroid()
        for edge in self._edge_list_geometry_cs:
            orthogonal_vector: np.array = self.calculate_orthogonal_vector_point_to_line(centroid,
                                                                                         edge.edge_vector,
                                                                                         edge.start_point)
            orthogonal_vector = self.extend_vector_by_length(orthogonal_vector, 0.3)
            plot.plot([centroid[0], centroid[0]+orthogonal_vector[0]], [centroid[1], orthogonal_vector[1]], "g-")

        plot.show(block=block)
    # endregion

    # region Helper methods
    def check_if_block_exists(self, **kwargs) -> bool:
        # TODO Docstring
        block: bool = False
        if "block" in kwargs:
            block = kwargs.get("block")
        return block

    def check_if_color_exists(self, **kwargs) -> str:
        # TODO Docstring
        color: str = mcolors.CSS4_COLORS["red"]
        if "color" in kwargs:
            color = mcolors.CSS4_COLORS[str(kwargs.get("color"))]
        return color
    # endregion

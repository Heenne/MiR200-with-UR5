from matplotlib import pyplot as plot
from matplotlib import colors as mcolors
import numpy as np
from math import sin, cos, atan2
from typing import List

from geometry_info.edge_info import EdgeInfo


class GeometryContour:
    # Member
    # Contains the translation of the geometry coordinate system in world cs
    _lead_vector_world_cs: np.array
    _geometry_cs_rotation: float
    # Contains the rotation and translation of the centroid from the world cs
    _tf_world_to_geometry_cs: np.array
    _tf_geometry_to_world_cs: np.array

    _corner_point_list_geometry_cs: list
    _edge_list_geometry_cs: list

    def __init__(self, lead_vector_world_cs: np.array = np.array([0, 0]), world_to_geometry_cs_rotation: float = 0.0):
        """Init method for the GeometryContour class.

        :param lead_vector_world_cs: Lead vector to the geometry cs if it is not in the same position as the world cs
        :type lead_vector_world_cs: np.array, optional
        :param world_to_geometry_cs_rotation: In [rad]. If the coordinate system of the geometry is rotated
        in comparison to the world coordinate system then insert the angle here in radian, defaults to 0.0
        :type world_to_geometry_cs_rotation: float, optional
        """
        self._set_geometry_transformations(lead_vector_world_cs=lead_vector_world_cs,
                                           geometry_cs_rotation=world_to_geometry_cs_rotation)
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
    def lead_vector_world_cs(self) -> np.array:
        """Geometric centroid (not center of mass) in the world coordinate system.

        :return: 2x1 vector
        :rtype: np.array
        """
        return self._lead_vector_world_cs

    @property
    def world_to_geometry_rotation(self) -> float:
        """Rotation of the geometry coordinate system to the world coordinate system.
        This is also contained in the transformation matrix.

        :return: Rotation in radian
        :rtype: float
        """
        return self._geometry_cs_rotation

    @property
    def corner_point_list_geometry_cs(self) -> List[np.array]:
        """List of all corners of the contour.
        Corner points are specified in the geometry coordinate system.

        :return: list of all corner elements which are specified as 2x1 vectors
        :rtype: list
        """
        return self._corner_point_list_geometry_cs

    @property
    def corner_point_list_world_cs(self) -> List[np.array]:
        """List of all corners of the contour.
        Corner points are specified in the world coordinate system.

        :return: [description]
        :rtype: list
        """
        return [self.transform_vector_geometry_to_world_cs(corner) for corner in self.corner_point_list_geometry_cs]

    @property
    def edge_list_geometry_cs(self) -> List[EdgeInfo]:
        """List of all edge elements which are stored as EdgeInfo objects.
        The edge vectors are not specified directly in a coordinate system but the start_point of
        each edge is specified in the geometry coordinate system.

        :return: list of all edge elements as EdgeInfo objects in geometry cs
        :rtype: list
        """
        return self._edge_list_geometry_cs

    @property
    def edge_list_world_cs(self) -> List[EdgeInfo]:
        """List of all edge elements which are stored as EdgeInfo objects.
        The edge vectors are not specified directly in a coordinate system but the start_point of
        each edge is specified in the world coordinate system.

        :return: list of all edge elements as EdgeInfo objects in world cs
        :rtype: list
        """
        return [EdgeInfo(start_point=self.transform_vector_geometry_to_world_cs(edge.start_point),
                         end_point=self.transform_vector_geometry_to_world_cs(edge.end_point))
                for edge in self.edge_list_geometry_cs]

    @property
    def x_max_geometry_cs(self) -> float:
        """Maximal x value of a corner of the contour seen from the geometry cs

        :return: x value as float
        :rtype: float
        """
        return self._get_x_max_from_list(self.corner_point_list_geometry_cs)

    @property
    def x_max_world_cs(self) -> float:
        """Maximal x value of a corner of the contour seen from the world cs

        :return: x value as float
        :rtype: float
        """
        return self._get_x_max_from_list(self.corner_point_list_world_cs)

    @property
    def x_min_geometry_cs(self) -> float:
        """Minimal x value of a corner of the contour seen from the geometry cs

        :return: x value as float
        :rtype: float
        """
        return self._get_x_min_from_list(self.corner_point_list_geometry_cs)

    @property
    def x_min_world_cs(self) -> float:
        """Minimal x value of a corner of the contour seen from the world cs

        :return: x value as float
        :rtype: float
        """
        return self._get_x_min_from_list(self.corner_point_list_world_cs)

    @property
    def y_max_geometry_cs(self) -> float:
        """Maximal y value of a corner of the contour seen from the geometry cs

        :return: y value as float
        :rtype: float
        """
        return self._get_y_max_from_list(self.corner_point_list_geometry_cs)

    @property
    def y_max_world_cs(self) -> float:
        """Maximal y value of a corner of the contour seen from the world cs

        :return: y value as float
        :rtype: float
        """
        return self._get_y_max_from_list(self.corner_point_list_world_cs)

    @property
    def y_min_geometry_cs(self) -> float:
        """Minimal y value of a corner of the contour seen from the geometry cs

        :return: y value as float
        :rtype: float
        """
        return self._get_y_min_from_list(self.corner_point_list_geometry_cs)

    @property
    def y_min_world_cs(self) -> float:
        """Minimal y value of a corner of the contour seen from the world cs

        :return: y value as float
        :rtype: float
        """
        return self._get_y_min_from_list(self.corner_point_list_world_cs)

    # endregion

    # region Geometry handling methods
    def import_contour_with_offset(self, contour: 'GeometryContour', offset_value: float) -> None:
        # TODO Docstring
        self._set_geometry_transformations(contour.lead_vector_world_cs, contour.world_to_geometry_rotation)

        for counter in range(0, len(contour.edge_list_geometry_cs)):
            bef_edge: np.array  # Edge before the current edge -> counter-1 in the list
            cur_edge: np.array  # Current edge in the list -> counter in the list

            if (counter - 1) < 0:
                bef_edge = contour.edge_list_geometry_cs[-1]  # last element
            else:
                bef_edge = contour.edge_list_geometry_cs[counter - 1]

            cur_edge = contour.edge_list_geometry_cs[counter]
            centroid_geometry_cs: np.array = contour.calc_centroid_geometry_cs()

            ortho_bef_edge_geometry_cs: np.array = self.calc_ortho_vector_point_to_line(
                orthogonal_point=centroid_geometry_cs,
                line=bef_edge.edge_vector,
                lead_vector=bef_edge.start_point)

            ext_ortho_bef_edge_geometry_cs: np.array = self.extend_vector_by_length(ortho_bef_edge_geometry_cs,
                                                                                    offset_value)

            ortho_cur_edge_geometry_cs: np.array = self.calc_ortho_vector_point_to_line(
                orthogonal_point=centroid_geometry_cs,
                line=cur_edge.edge_vector,
                lead_vector=cur_edge.start_point)

            ext_ortho_cur_edge_geometry_cs: np.array = self.extend_vector_by_length(ortho_cur_edge_geometry_cs,
                                                                                    offset_value)

            new_corner_point_geometry_cs: np.array = self.calc_vector_line_intersection_point(
                lead_vector_1=ext_ortho_cur_edge_geometry_cs,
                direction_vector_1=cur_edge.edge_vector,
                lead_vector_2=ext_ortho_bef_edge_geometry_cs,
                direction_vector_2=bef_edge.edge_vector)

            self._corner_point_list_geometry_cs.append(new_corner_point_geometry_cs)

        self.create_contour_edges()

    def _set_geometry_transformations(self, lead_vector_world_cs: np.array, geometry_cs_rotation: float) -> None:
        """Method for setting transformation from geometry cs to world and other way around.

        :param lead_vector_world_cs: geometric centroid of the contour
        :type lead_vector_world_cs: np.array
        :param geometry_cs_rotation: rotation of the geometry cs to the world cs
        :type geometry_cs_rotation: float
        """
        self._lead_vector_world_cs = lead_vector_world_cs
        self._geometry_cs_rotation = geometry_cs_rotation

        self._tf_geometry_to_world_cs: np.array = self._create_transformation_matrix(lead_vector_world_cs,
                                                                                     geometry_cs_rotation)
        self._tf_world_to_geometry_cs = np.linalg.inv(self._tf_geometry_to_world_cs)

    def _create_transformation_matrix(self, lead_vector: np.array, rotation: float) -> np.array:
        tf_matrix: np.array = np.array([[cos(rotation), -sin(rotation), lead_vector[0]],
                                        [sin(rotation), cos(rotation), lead_vector[1]],
                                        [0, 0, 1]])
        return tf_matrix

    def move_coordinate_system(self, new_lead_vector_world_cs: np.array = np.array([0, 0]),
                               new_geometry_cs_rotation: float = 0.0) -> None:
        """
        This method moves the coordinate system from the current position and rotation to a new pose.
        All corner points stay in the same position seen from the world cs.

        :param new_lead_vector_world_cs: New lead vector that specifies the translation of the new cs
        :type new_lead_vector_world_cs: np.array
        :param new_geometry_cs_rotation: Rotation of the world cs to geometry cs
        :type new_geometry_cs_rotation: float
        """
        new_tf_geometry_to_world_cs: np.array = self._create_transformation_matrix(new_lead_vector_world_cs,
                                                                                   new_geometry_cs_rotation)
        new_tf_world_to_geometry: np.array = np.linalg.inv(new_tf_geometry_to_world_cs)
        tf_old_to_new_geometry_cs: np.array = new_tf_world_to_geometry.dot(self._tf_geometry_to_world_cs)

        corner_point_list_new_geometry_cs: List[np.array] = list()
        for corner in self._corner_point_list_geometry_cs:
            ext_corner: np.array = np.append(corner, [1])
            ext_new_corner_geometry_cs: np.array = tf_old_to_new_geometry_cs.dot(ext_corner)
            corner_point_list_new_geometry_cs.append(ext_new_corner_geometry_cs[:-1])

        self._lead_vector_world_cs = new_lead_vector_world_cs
        self._geometry_cs_rotation = new_geometry_cs_rotation
        self._tf_geometry_to_world_cs = new_tf_geometry_to_world_cs
        self._tf_world_to_geometry_cs = new_tf_world_to_geometry

        self._corner_point_list_geometry_cs = corner_point_list_new_geometry_cs
        self.create_contour_edges()

    def move_contour(self, new_lead_vector_world_cs: np.array, new_geometry_cs_rotation: float) -> None:
        """
        This method sets the translation and rotation of the geometry coordination system.
        The points defined in the geometry cs stay the same.

        :param new_lead_vector_world_cs: New lead vector from world cs to geometry cs
        :type new_lead_vector_world_cs: np.array
        :param new_geometry_cs_rotation: New rotation degree in radian
        :type new_geometry_cs_rotation: float

        """
        new_tf_geometry_to_world_cs: np.array = self._create_transformation_matrix(new_lead_vector_world_cs,
                                                                                   new_geometry_cs_rotation)
        new_tf_world_to_geometry_cs: np.array = np.linalg.inv(new_tf_geometry_to_world_cs)

        self._lead_vector_world_cs = new_lead_vector_world_cs
        self._geometry_cs_rotation = new_geometry_cs_rotation
        self._tf_geometry_to_world_cs = new_tf_geometry_to_world_cs
        self._tf_world_to_geometry_cs = new_tf_world_to_geometry_cs

    def rotate_contour(self, new_geometry_cs_rotation: float) -> None:
        """
        This method sets the rotation of the geometry coordination system.
        The points defined in the geometry cs stay the same to be rotated with the coordinate system

        :param new_geometry_cs_rotation: New rotation degree in radian
        :type new_geometry_cs_rotation: float
        """
        new_tf_geometry_to_world_cs: np.array = self._create_transformation_matrix(self._lead_vector_world_cs,
                                                                                   new_geometry_cs_rotation)
        new_tf_world_to_geometry_cs: np.array = np.linalg.inv(new_tf_geometry_to_world_cs)

        self._geometry_cs_rotation = new_geometry_cs_rotation
        self._tf_geometry_to_world_cs = new_tf_geometry_to_world_cs
        self._tf_world_to_geometry_cs = new_tf_world_to_geometry_cs

    def add_contour_corner_world_cs(self, additional_corner_world_cs: np.array):
        # TODO Docstring
        additional_corner_geometry_cs: np.array = self.transform_vector_world_to_geometry_cs(
            additional_corner_world_cs)
        self.add_contour_corner_geometry_cs(additional_corner_geometry_cs)

    def add_contour_corner_geometry_cs(self, additional_corner_geometry_cs: np.array):
        # TODO Docstring
        self._corner_point_list_geometry_cs.append(additional_corner_geometry_cs)
        self.create_contour_edges()
        # TODO update the centroid point and transformation matrix here

    def replace_contour_corner_world_cs(self, corner_index: int, new_corner_point_world_cs: np.array):
        # TODO Docstring
        new_corner_point_geometry_cs: np.array = self.transform_vector_world_to_geometry_cs(
            new_corner_point_world_cs)
        self.replace_contour_corner_geometry_cs(corner_index, new_corner_point_geometry_cs)

    def replace_contour_corner_geometry_cs(self, corner_index: int, new_corner_point_geometry_cs: np.array):
        # TODO Docstring
        self._corner_point_list_geometry_cs[corner_index] = new_corner_point_geometry_cs
        self.create_contour_edges()
        # TODO update the centroid point and transformation matrix here

    def create_contour_edges(self):
        """This method deletes all current edges.
        Then uses the corners of the current contour to create new edges between them.
        The order of the corners are used to create the edges.
        """
        self._edge_list_geometry_cs.clear()
        for counter in range(0, len(self._corner_point_list_geometry_cs)):
            start_point_geometry_cs: np.array = self._corner_point_list_geometry_cs[counter]
            end_point_geometry_cs: np.array
            if counter + 1 == len(self._corner_point_list_geometry_cs):
                end_point_geometry_cs = self._corner_point_list_geometry_cs[0]
            else:
                end_point_geometry_cs = self._corner_point_list_geometry_cs[counter + 1]

            edge: EdgeInfo = EdgeInfo(start_point=start_point_geometry_cs, end_point=end_point_geometry_cs)
            self._edge_list_geometry_cs.append(edge)

    def get_closest_edge_to_point(self, point_geometry_cs: np.array) -> EdgeInfo:
        # TODO Docstring
        closest_edge_geometry_cs: EdgeInfo = self._edge_list_geometry_cs[0]
        shortest_distance: float = self.calc_distance_point_to_line(point_geometry_cs,
                                                                    closest_edge_geometry_cs.edge_vector,
                                                                    closest_edge_geometry_cs.start_point)
        for edge_geometry_cs in self._edge_list_geometry_cs:
            new_distance: float = self.calc_distance_point_to_line(point_geometry_cs,
                                                                   edge_geometry_cs.edge_vector,
                                                                   edge_geometry_cs.start_point)
            if new_distance < shortest_distance:
                closest_edge_geometry_cs = edge_geometry_cs
                shortest_distance = new_distance

        return closest_edge_geometry_cs

    def get_longest_edge(self) -> EdgeInfo:
        """[summary]

        :return: Longest edge of the contour defined in the geometry cs
        :rtype: EdgeInfo
        """
        # TODO Docstring
        longest_edge_geometry_cs: EdgeInfo = self._edge_list_geometry_cs[0]  # initialize with first value in list
        longest_edge_length: float = np.linalg.norm(longest_edge_geometry_cs.edge_vector)
        for edge_geometry_cs in self._edge_list_geometry_cs:
            new_edge_length: float = np.linalg.norm(edge_geometry_cs.edge_vector)
            if new_edge_length > longest_edge_length:
                longest_edge_length = new_edge_length
                longest_edge_geometry_cs = edge_geometry_cs

        return longest_edge_geometry_cs

    def get_shortest_edge(self) -> EdgeInfo:
        """[summary]

        :return: Shortest edge of the contour defined in the geometry cs
        :rtype: EdgeInfo
        """
        # TODO Docstring
        shortest_edge_geometry_cs: EdgeInfo = self._edge_list_geometry_cs[0]
        shortest_edge_length: float = np.linalg.norm(shortest_edge_geometry_cs.edge_vector)
        for edge_geometry_cs in self._edge_list_geometry_cs:
            new_edge_length: float = np.linalg.norm(edge_geometry_cs.edge_vector)
            if new_edge_length < shortest_edge_length:
                shortest_edge_length = new_edge_length
                shortest_edge_geometry_cs = edge_geometry_cs

        return shortest_edge_geometry_cs

    def extend_vector_by_length(self, vector_to_extend: np.array, length_to_extend: float) -> np.array:
        """This method extends a vector by a given amount not a percentage!

        :param vector_to_extend: Vector that should be extended (2x1 np.array)
        :type vector_to_extend: np.array
        :param length_to_extend: Amount that the vector should be extended (f.e. 0.3 meter)
        :type length_to_extend: float
        :return: Extended vector by specified length
        :rtype: np.array
        """
        extended_vector: np.array
        length_of_vector: float = np.linalg.norm(vector_to_extend)
        extended_vector = ((length_of_vector + length_to_extend) / length_of_vector) * vector_to_extend
        return extended_vector

    def is_geometry_cs_point_in_contour(self, point_geometry_cs: np.array) -> bool:
        """Checks if the given point (which must be defined in geometry cs) is in the contour.
        For using a point in world cs, please use the is_world_cs_point_in_contour method.

        :param point_geometry_cs: Point that will be checked if it is in the contour in geometry cs
        :type point_geometry_cs: np.array
        :return: True if point is in contour
        :rtype: bool
        """
        # First simple check if point is in max values of polygon
        if (point_geometry_cs[0] > self.x_max_geometry_cs or point_geometry_cs[0] < self.x_min_geometry_cs or
                point_geometry_cs[1] > self.y_max_geometry_cs or point_geometry_cs[1] < self.y_min_geometry_cs):
            return False

        # Selected a very weird vector because I see no solution to find out if this vector is crossing through a corner
        # or just being a tangent to the corner.
        # This makes it impossible to distinguish between in or out of the contour
        # This is an easy but ugly fixs
        testing_direction_vector: np.array = np.array([1.34567, 0.2345678])

        intersections: int = 0

        for edge_geometry_cs in self._edge_list_geometry_cs:
            factor_1_numerator: float = self._calc_vector_line_intersection_factor_1_numerator(
                edge_geometry_cs.start_point, point_geometry_cs, testing_direction_vector)
            factor_1_denominator: float = self._calc_vector_line_intersection_factor_1_denominator(
                edge_geometry_cs.edge_vector, testing_direction_vector)
            factor_2_numerator: float = self._calc_vector_line_intersection_factor_2_numerator(
                edge_geometry_cs.start_point, edge_geometry_cs.edge_vector, point_geometry_cs)
            factor_2_denominator: float = self._calc_vector_line_intersection_factor_2_denominator(
                edge_geometry_cs.edge_vector, testing_direction_vector)

            if factor_1_denominator == 0 or factor_2_denominator == 0:
                continue

            factor_1: float = factor_1_numerator / factor_1_denominator
            factor_2: float = factor_2_numerator / factor_2_denominator

            if 0 <= factor_1 < 1 and factor_2 >= 0:
                intersections = intersections + 1

        if (intersections % 2) == 1:
            return True
        else:
            return False

    def is_world_cs_point_in_contour(self, point_world_cs: np.array) -> bool:
        """Same method as is_geometry_cs_point_in_contour but the point to check is automatically transformed into the
        geometry cs.

        :param point_world_cs: Point that will be checked if it is in the contour in world cs
        :type point_world_cs: np.arra
        :return: True if point is in contour
        :rtype: bool
        """
        point_geometry_cs: np.array = self.transform_vector_world_to_geometry_cs(point_world_cs)
        return self.is_geometry_cs_point_in_contour(point_geometry_cs)

    def do_contour_edges_intersect(self) -> bool:
        """Check if the edges of the contour intersect.
        If this is the case, the contour is not valid!

        :return: True if any edge intersects with another one
        :rtype: bool
        """
        for edge_outer_geometry_cs in self._edge_list_geometry_cs:
            for edge_inner_geometry_cs in self._edge_list_geometry_cs:
                if edge_inner_geometry_cs is edge_outer_geometry_cs:
                    continue

                intersection_point_geometry_cs: np.array = self.calc_vector_intersection_point(
                    edge_outer_geometry_cs.start_point,
                    edge_outer_geometry_cs.edge_vector,
                    edge_inner_geometry_cs.start_point,
                    edge_inner_geometry_cs.edge_vector)
                if intersection_point_geometry_cs is not None:
                    return True

        return False

    def transform_vector_world_to_geometry_cs(self, vector_world_cs: np.array) -> np.array:
        # TODO Docstring
        ext_vector_world_cs: np.array = np.append(vector_world_cs, [1])
        ext_vector_geometry_cs: np.array = self._tf_world_to_geometry_cs.dot(ext_vector_world_cs)
        return ext_vector_geometry_cs[:-1]

    def transform_vector_geometry_to_world_cs(self, vector_geometry_cs: np.array) -> np.array:
        # TODO Docstring
        ext_vector_geometry_cs: np.array = np.append(vector_geometry_cs, [1])
        ext_vector_world_cs: np.array = self._tf_geometry_to_world_cs.dot(ext_vector_geometry_cs)
        return ext_vector_world_cs[:-1]

    def is_contour_colliding(self, contour_to_check: "GeometryContour") -> bool:
        # TODO Docstring
        # Faster and easier check is if a corner is in the other contour.
        # If a corner point is in the contour, there is a collision
        for corner in self.corner_point_list_world_cs:
            if contour_to_check.is_world_cs_point_in_contour(corner):
                return True

        # If no corner of the current contour is in the other contour.
        # Check if an edge of one contour is colliding with an edge of the other contour
        for current_edge_world_cs in self.edge_list_world_cs:
            for edge_to_check_world_cs in self.edge_list_world_cs:
                if self.calc_vector_intersection_point(current_edge_world_cs.start_point,
                                                       current_edge_world_cs.edge_vector,
                                                       edge_to_check_world_cs.start_point,
                                                       edge_to_check_world_cs.edge_vector) is not None:
                    return True

        return False

    # endregion

    # region Calculation methods
    def calc_area(self) -> float:
        """Method for calculating the area in the contour.
        For formula see: https://en.wikipedia.org/wiki/Centroid under "Of a polygon".

        :return: Area in the contour
        :rtype: float
        """
        area: float = 0.0
        for counter in range(0, len(self._corner_point_list_geometry_cs)):
            if (counter + 1) == len(self._corner_point_list_geometry_cs):
                part_area: float = ((self._corner_point_list_geometry_cs[counter][0] *
                                     self._corner_point_list_geometry_cs[0][1]) -
                                    (self._corner_point_list_geometry_cs[0][0] *
                                     self._corner_point_list_geometry_cs[counter][1]))
                area = area + part_area
            else:
                part_area: float = ((self._corner_point_list_geometry_cs[counter][0] *
                                     self._corner_point_list_geometry_cs[counter + 1][1]) -
                                    (self._corner_point_list_geometry_cs[counter + 1][0] *
                                     self._corner_point_list_geometry_cs[counter][1]))
                area = area + part_area

        area = 0.5 * area
        return area

    def calc_centroid_geometry_cs(self) -> np.array:
        """Method for calculating the geometric centroid of the contour
        For formula see: https://en.wikipedia.org/wiki/Centroid under "Of a polygon"

        :return: 2x1 vector in the geometry cs that points to the geometric centroid
        :rtype: np.array
        """
        geometry_area: float = self.calc_area()

        x_centroid: float = 0.0
        y_centroid: float = 0.0

        for counter in range(0, len(self._corner_point_list_geometry_cs)):
            x_first_factor: float
            y_first_factor: float
            second_factor: float
            if (counter + 1) == len(self._corner_point_list_geometry_cs):
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
                                  self._corner_point_list_geometry_cs[counter + 1][1]) -
                                 (self._corner_point_list_geometry_cs[counter + 1][0] *
                                  self._corner_point_list_geometry_cs[counter][1]))
                x_first_factor = (self._corner_point_list_geometry_cs[counter][0] +
                                  self._corner_point_list_geometry_cs[counter + 1][0])
                y_first_factor = (self._corner_point_list_geometry_cs[counter][1] +
                                  self._corner_point_list_geometry_cs[counter + 1][1])

            x_centroid = x_centroid + (x_first_factor * second_factor)
            y_centroid = y_centroid + (y_first_factor * second_factor)

        x_centroid = (1 / (6 * geometry_area)) * x_centroid
        y_centroid = (1 / (6 * geometry_area)) * y_centroid

        centroid_point_geometry_cs: np.array = np.array([x_centroid, y_centroid])
        return centroid_point_geometry_cs

    def calc_centroid_world_cs(self) -> np.array:
        """Same method as calc_centroid_geometry_cs but directly returns the centroid in world cs

        :return: 2x1 vector to the centroid of the contour in world cs
        :rtype: np.array
        """
        centroid_geometry_cs: np.array = self.calc_centroid_geometry_cs()
        return self.transform_vector_geometry_to_world_cs(centroid_geometry_cs)

    def calc_ortho_vector_point_to_line(self,
                                        orthogonal_point: np.array,
                                        line: np.array,
                                        lead_vector: np.array) -> np.array:
        """General method for calculating the orthogonal vector from a line to a point.
        Notice that the line will be expanded to +/- infinity.

        :param orthogonal_point: Point from where the orthogonal vector will go to the line
        :type orthogonal_point: np.array
        :param line: Vector which will be expanded to +/- infinity and
        from where the orthogonal vector will go to the point
        :type line: np.array
        :param lead_vector: lead vector of the line
        :type lead_vector: np.array
        :return: 2x1 vector which resembles the vector from "orthogonal_point" to the line
        :rtype: np.array
        """
        nominator: float = -(line[0] * lead_vector[0]) - (line[1] * lead_vector[1])
        denominator: float = pow(line[0], 2) + pow(line[1], 2)

        factor: float
        try:
            factor = nominator / denominator
        except ZeroDivisionError:
            print("Divided by zero in the 'calculate_orthogonal_vector_point_to_line' method!")
            return None

        point_on_line: np.array = orthogonal_point + lead_vector + factor * line
        vector_point_to_line: np.array = point_on_line - orthogonal_point
        return vector_point_to_line

    def calc_distance_point_to_line(self,
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
        vector_point_to_line: np.array = self.calc_ortho_vector_point_to_line(
            orthogonal_point=point_geometry_cs,
            line=line,
            lead_vector=start_point_line_geometry_cs)
        return np.linalg.norm(vector_point_to_line)

    def calc_contour_length(self) -> float:
        """Sums up all lengths of the edges.

        :return: Length of all edges of the contour
        :rtype: float
        """
        total_length: float = 0.0
        for edge in self._edge_list_geometry_cs:
            edge_length: float = np.linalg.norm(edge.edge_vector)
            total_length += edge_length

        return total_length

    def calc_vector_line_intersection_point(self,
                                            lead_vector_1: np.array,
                                            direction_vector_1: np.array,
                                            lead_vector_2: np.array,
                                            direction_vector_2: np.array) -> np.array:
        """This method calculates the point where two vector lines intersect.
        Notice that the direction vectors will be extended with a factor from +/- infinite.
        For calculating intersection point of two vectors that is in the length of the vectors see:
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
        factor_1_numerator: float = self._calc_vector_line_intersection_factor_1_numerator(lead_vector_1,
                                                                                           lead_vector_2,
                                                                                           direction_vector_2)
        factor_1_denumerator: float = self._calc_vector_line_intersection_factor_1_denominator(direction_vector_1,
                                                                                               direction_vector_2)

        # TODO check for ZeroDivision
        factor_1: float = factor_1_numerator / factor_1_denumerator

        return lead_vector_1 + (factor_1 * direction_vector_1)

    def calc_vector_intersection_point(self,
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
        factor_1_numerator: float = self._calc_vector_line_intersection_factor_1_numerator(lead_vector_1,
                                                                                           lead_vector_2,
                                                                                           direction_vector_2)
        factor_1_denominator: float = self._calc_vector_line_intersection_factor_1_denominator(direction_vector_1,
                                                                                               direction_vector_2)
        factor_2_numerator: float = self._calc_vector_line_intersection_factor_2_numerator(lead_vector_1,
                                                                                           direction_vector_1,
                                                                                           lead_vector_2)
        factor_2_denominator: float = self._calc_vector_line_intersection_factor_2_denominator(direction_vector_1,
                                                                                               direction_vector_2)

        # TODO check for ZeroDivision
        factor_1: float = round(factor_1_numerator / factor_1_denominator, 10)
        factor_2: float = round(factor_2_numerator / factor_2_denominator, 10)

        if 0 < factor_1 < 1 and 0 < factor_2 < 1:
            return lead_vector_1 + (factor_1 * direction_vector_1)
        else:
            return None

    def _calc_vector_line_intersection_factor_1_numerator(self,
                                                          lead_vector_1: np.array,
                                                          lead_vector_2: np.array,
                                                          direction_vector_2: np.array) -> float:
        # TODO Docstring
        return ((lead_vector_2[1] * direction_vector_2[0]) + (lead_vector_1[0] * direction_vector_2[1]) -
                (lead_vector_2[0] * direction_vector_2[1]) - (lead_vector_1[1] * direction_vector_2[0]))

    def _calc_vector_line_intersection_factor_1_denominator(self,
                                                            direction_vector_1: np.array,
                                                            direction_vector_2: np.array) -> float:
        # TODO Docstring
        return (direction_vector_1[1] * direction_vector_2[0]) - (direction_vector_1[0] * direction_vector_2[1])

    def _calc_vector_line_intersection_factor_2_numerator(self,
                                                          lead_vector_1: np.array,
                                                          direction_vector_1: np.array,
                                                          lead_vector_2: np.array) -> float:
        # TODO Docstring
        return ((lead_vector_1[1] * direction_vector_1[0]) + (lead_vector_2[0] * direction_vector_1[1]) -
                (lead_vector_1[0] * direction_vector_1[1]) - (lead_vector_2[1] * direction_vector_1[0]))

    def _calc_vector_line_intersection_factor_2_denominator(self,
                                                            direction_vector_1: np.array,
                                                            direction_vector_2: np.array) -> float:
        # TODO Docstring
        return (direction_vector_2[1] * direction_vector_1[0]) - (direction_vector_2[0] * direction_vector_1[1])

    def calc_distance_closest_edge_to_point(self, point: np.array) -> float:
        # TODO Docstring
        closest_edge: EdgeInfo = self.get_closest_edge_to_point(point)
        return self.calc_distance_point_to_line(point, closest_edge.edge_vector, closest_edge.start_point)

    def calc_longest_edge_length(self) -> float:
        # TODO Docstring
        longest_edge: EdgeInfo = self.get_longest_edge()
        return np.linalg.norm(longest_edge.edge_vector)

    def calc_shortest_edge_length(self) -> float:
        # TODO Docstring
        shortest_edge: EdgeInfo = self.get_shortest_edge()
        return np.linalg.norm(shortest_edge.edge_vector)

    def calc_farthest_corner_to_point(self, point_world_cs: np.array) -> np.array:
        # TODO Docstring
        # Initialize the best values with the first point in the corner list
        farthest_point: np.array = self.corner_point_list_world_cs[0]
        farthest_distance: float = np.linalg.norm(point_world_cs - farthest_point)

        for corner in self.corner_point_list_world_cs:
            corner_to_point_vector: np.array = point_world_cs - corner
            distance: float = np.linalg.norm(corner_to_point_vector)

            if distance > farthest_distance:
                farthest_point = corner
                farthest_distance = distance

        return farthest_point

    def calc_farthest_distance_corner_to_point(self, point_world_cs: np.array) -> float:
        # TODO Docstring
        farthest_point: np.array = self.calc_farthest_corner_to_point(point_world_cs)
        return np.linalg.norm(farthest_point)

    # endregion

    # region Print and plot methods
    def print_info(self):
        # TODO Docstring
        print("Contour info: ")
        print("Geometry area: " + str(self.calc_area()))
        print("Centroid point: " + str(self.lead_vector_world_cs))

        for corner in self.corner_point_list_world_cs:
            print(corner)

        for edge in self._edge_list_geometry_cs:
            print(edge)

    def plot_corners(self, **kwargs):
        # TODO Docstring
        block: bool = self._check_if_block_exists(**kwargs)
        for point_world_cs in self.corner_point_list_world_cs:
            plot.plot(point_world_cs[0], point_world_cs[1], "bo")

        plot.show(block=block)

    def plot_edges(self, **kwargs):
        # TODO Docstring
        block: bool = self._check_if_block_exists(**kwargs)
        color: str = self._check_if_color_exists(**kwargs)

        for edge_world_cs in self.edge_list_world_cs:
            plot.plot([edge_world_cs.start_point[0], edge_world_cs.end_point[0]],
                      [edge_world_cs.start_point[1], edge_world_cs.end_point[1]],
                      linestyle="-",
                      color=color)

        plot.show(block=block)

    def plot_centroid(self, **kwargs):
        # TODO Docstring
        block: bool = self._check_if_block_exists(**kwargs)

        centroid_point_world_cs: np.array = self.calc_centroid_world_cs()
        plot.plot(centroid_point_world_cs[0], centroid_point_world_cs[1], "go")

        plot.show(block=block)

    def plot_orthogonal_vector_centroid_to_edge(self, **kwargs):
        # TODO Docstring
        block: bool = self._check_if_block_exists(**kwargs)
        centroid_geometry_cs: np.array = self.calc_centroid_geometry_cs()
        centroid_world_cs: np.array = self.transform_vector_geometry_to_world_cs(centroid_geometry_cs)
        for edge in self._edge_list_geometry_cs:
            orthogonal_vector: np.array = self.calc_ortho_vector_point_to_line(centroid_geometry_cs,
                                                                               edge.edge_vector,
                                                                               edge.start_point)
            orthogonal_vector = self.extend_vector_by_length(orthogonal_vector, 0.3)
            plot.plot([centroid_world_cs[0], centroid_world_cs[0] + orthogonal_vector[0]],
                      [centroid_world_cs[1], centroid_world_cs[1] + orthogonal_vector[1]],
                      "g-")

        plot.show(block=block)

    # endregion

    # region Helper methods
    def _check_if_block_exists(self, **kwargs) -> bool:
        # TODO Docstring
        block: bool = False
        if "block" in kwargs:
            block = kwargs.get("block")
        return block

    def _check_if_color_exists(self, **kwargs) -> str:
        # TODO Docstring
        color: str = mcolors.CSS4_COLORS["red"]
        if "color" in kwargs:
            color = mcolors.CSS4_COLORS[str(kwargs.get("color"))]
        return color

    def _get_x_max_from_list(self, point_list: list) -> float:
        """Maximal x value of a list which is filled with points (2x1 np.array elements)

        :return: x value as float
        :rtype: float
        """
        x_max: float = point_list[0][0]  # Get first x value as init value
        for point in point_list:
            if x_max < point[0]:
                x_max = point[0]

        return x_max

    def _get_x_min_from_list(self, point_list: list) -> float:
        """Minimal x value of a list which is filled with points (2x1 np.array elements)

        :return: x value as float
        :rtype: float
        """
        x_min: float = point_list[0][0]  # Get first x value as init value
        for point in point_list:
            if x_min > point[0]:
                x_min = point[0]

        return x_min

    def _get_y_max_from_list(self, point_list: list) -> float:
        """Maximal y value of a list which is filled with points (2x1 np.array elements)

        :return: Max y value as float
        :rtype: float
        """
        y_max: float = point_list[0][1]  # Get first y value as init value
        for point in point_list:
            if y_max < point[1]:
                y_max = point[1]

        return y_max

    def _get_y_min_from_list(self, point_list: list) -> float:
        """Minimal y value of a list which is filled with points (2x1 np.array elements)

        :return: Min y value as float
        :rtype: float
        """
        y_min: float = point_list[0][1]  # Get first y value as init value
        for point in point_list:
            if y_min > point[1]:
                y_min = point[1]

        return y_min

    def _get_translation_from_tf(self, tf_matrix: np.array) -> np.array:
        """
        Method for getting the translation vector of the transformation matrix.

        :param tf_matrix: transformation matrix in a 3x3 shape
        :type tf_matrix: np.array
        :return: translation vector in a 2x1 shape
        :rtype: np.array
        """
        translation_vector: np.array = tf_matrix[0:2, 2]  # 0:2 because 0 is included but 2 not. So it is 0 to 1.
        return translation_vector

    def _get_rotation_from_tf(self, tf_matrix: np.array) -> float:
        """
        Method for getting the rotation value from the transformation matrix

        :param tf_matrix: transformation matrix in a 3x3 shape
        :type tf_matrix: np.array
        :return: rotation value in radian
        :rtype: float
        """
        rotation: float = atan2(tf_matrix[1, 0], tf_matrix[0, 0])
        return rotation

    # endregion

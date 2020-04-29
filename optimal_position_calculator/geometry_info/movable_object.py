import sys

from math import sin, cos, pi
from math import sqrt, pow
import numpy as np

from geometry_info.geometry_contour import GeometryContour


class MovableObject(GeometryContour):
    # This factor is necessary as Gazebo puts a 1000 factor in between.
    # So in the urdf file is a necessary 0.001 factor to show to right dimensions of the model in Gazebo
    _GAZEBO_FACTOR: float = 1000

    mass: float
    height: float

    def __init__(self, geometric_centroid_world_cs: np.array = np.array([0, 0]), rotation_radian: float = 0.0):
        super().__init__(geometric_centroid_world_cs=geometric_centroid_world_cs, rotation_radian=rotation_radian)

    def import_urdf_info(self, **kwargs):
        """Import urdf info in the current class.

        :Keyword Arguments:
        :param height: Height of the object in z direction
        :type height: float
        :param mass: Mass of the object
        :type mass: float
        """
        # 0.1 because default value is 0.1m and for Gazebofactor see MovableObject class
        self.height = float(kwargs.get("height")) * 0.1 * self._GAZEBO_FACTOR
        self.mass = float(kwargs.get("mass"))

    def print_info(self):
        super().print_info()

        print("Movable object info: ")
        print("Height: " + str(self.height))
        print("Mass: " + str(self.mass))


class Box(MovableObject):
    _x_length: float
    _y_length: float

    def __init__(self, geometric_centroid_world_cs: np.array = np.array([0, 0]), rotation_radian: float = 0.0):
        super().__init__(geometric_centroid_world_cs=geometric_centroid_world_cs, rotation_radian=rotation_radian)

    def import_urdf_info(self, **kwargs):
        super().import_urdf_info()
        self._x_length: float = float(kwargs.get("x_length"))
        self._y_length: float = float(kwargs.get("y_length"))

        half_x_length: float = self._x_length / 2
        half_y_length: float = self._y_length / 2

        point1_geometry_cs: np.array = np.array([half_x_length, half_y_length])
        point2_geometry_cs: np.array = np.array([-half_x_length, half_y_length])
        point3_geometry_cs: np.array = np.array([-half_x_length, -half_y_length])
        point4_geometry_cs: np.array = np.array([half_x_length, -half_y_length])

        self._corner_point_geometry_cs_list.append(point1_geometry_cs)
        self._corner_point_geometry_cs_list.append(point2_geometry_cs)
        self._corner_point_geometry_cs_list.append(point3_geometry_cs)
        self._corner_point_geometry_cs_list.append(point4_geometry_cs)

        self.create_contour_edges()


class Cylinder(MovableObject):
    __RESOLUTION: int = 30
    _radius: float

    def __init__(self, geometric_centroid_world_cs: np.array = np.array([0, 0]), rotation_radian: float = 0.0):
        super().__init__(geometric_centroid_world_cs=geometric_centroid_world_cs, rotation_radian=rotation_radian)

    def import_urdf_info(self, **kwargs):
        super().import_urdf_info(**kwargs)
        self._radius = float(kwargs.get("radius"))

        for counter in range(0, self.__RESOLUTION):
            x_value = self._radius * cos(((2*pi)/self.__RESOLUTION) * counter)
            y_value = self._radius * sin(((2*pi)/self.__RESOLUTION) * counter)

            point_geometry_cs = np.array([x_value, y_value])
            self._corner_point_geometry_cs_list.append(point_geometry_cs)

        self.create_contour_edges()


class IsoscelesTriangle(MovableObject):
    __DEFAULT_SIDE_LENGTH: float = 1.0
    _scaled_side_length: float
    _scale_factor: float

    def __init__(self, geometric_centroid_world_cs: np.array = np.array([0, 0]), rotation_radian: float = 0.0):
        super().__init__(geometric_centroid_world_cs=geometric_centroid_world_cs, rotation_radian=rotation_radian)

    def import_urdf_info(self, **kwargs):
        super().import_urdf_info(**kwargs)
        self._scale_factor: float = float(kwargs.get("scale")) * self._GAZEBO_FACTOR

        self._scaled_side_length: float = self.__DEFAULT_SIDE_LENGTH * self._scale_factor
        scaled_height: float = self._calculate_height(self._scaled_side_length)

        point1_geometry_cs: np.array = np.array([(self._scaled_side_length / 2), (-scaled_height / 3)])
        point2_geometry_cs: np.array = np.array([0, ((2*scaled_height) / 3)])
        point3_geometry_cs: np.array = np.array([(-self._scaled_side_length / 2), (-scaled_height / 3)])

        self._corner_point_geometry_cs_list.append(point1_geometry_cs)
        self._corner_point_geometry_cs_list.append(point2_geometry_cs)
        self._corner_point_geometry_cs_list.append(point3_geometry_cs)

        self.create_contour_edges()

    def _calculate_default_height(self) -> float:
        return self._calculate_height(self.__DEFAULT_SIDE_LENGTH)

    def _calculate_height(self, side_length: float) -> float:
        return sqrt(pow(side_length, 2) - pow((side_length / 2), 2))


class RightAngledTriangle(MovableObject):
    __DEFAULT_X_SIDE_LENGTH: float = 1.0
    __DEFAULT_Y_SIDE_LENGTH: float = 1.0
    _x_scale_factor: float
    _y_scale_factor: float
    _scaled_x_side_length: float
    _scaled_y_side_length: float

    def __init__(self, geometric_centroid_world_cs: np.array = np.array([0, 0]), rotation_radian: float = 0.0):
        super().__init__(geometric_centroid_world_cs=geometric_centroid_world_cs, rotation_radian=rotation_radian)

    def import_urdf_info(self, **kwargs):
        super().import_urdf_info(**kwargs)
        self._x_scale_factor: float = float(kwargs.get("x_scale")) * self._GAZEBO_FACTOR
        self._y_scale_factor: float = float(kwargs.get("y_scale")) * self._GAZEBO_FACTOR

        self._scaled_x_side_length = self.__DEFAULT_X_SIDE_LENGTH * self._x_scale_factor
        self._scaled_y_side_length = self.__DEFAULT_Y_SIDE_LENGTH * self._y_scale_factor

        point1_geometry_cs: np.array = np.array([((2*self._scaled_x_side_length) / 3),
                                                 (-self._scaled_y_side_length / 3)])
        point2_geometry_cs: np.array = np.array([(-self._scaled_x_side_length / 3),
                                                 ((2*self._scaled_y_side_length) / 3)])
        point3_geometry_cs: np.array = np.array([(-self._scaled_x_side_length / 3),
                                                 (-self._scaled_y_side_length / 3)])

        self._corner_point_geometry_cs_list.append(point1_geometry_cs)
        self._corner_point_geometry_cs_list.append(point2_geometry_cs)
        self._corner_point_geometry_cs_list.append(point3_geometry_cs)

        self.create_contour_edges()

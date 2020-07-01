import sys
import os

from matplotlib import pyplot as plot
import numpy as np
import random
from math import sin, cos, pi
import copy
from matplotlib import colors as mcolors

from geometry_info.geometry_contour import GeometryContour
from geometry_info.movable_object import MovableObject, Box, Cylinder, IsoscelesTriangle, RightAngledTriangle
from geometry_info.edge_info import EdgeInfo

from urdf_reader import URDFReader
from urdf_reader import GeometryType


NUMBER_OF_ROBOTS: int = 3


def plot_contour_info(object_to_move, extended_object_contour, grip_area):
    object_to_move.print_info()
    # object_to_move.plot_corners(block=False)
    object_to_move.plot_edges(block=False, color="black")
    object_to_move.plot_centroid(block=False, color="red")

    # extended_object_contour.print_info()
    # extended_object_contour.plot_corners(block=False)
    # extended_object_contour.plot_edges(block=False)
    #
    grip_area.print_info()
    # grip_area.plot_corners(block=False)
    grip_area.plot_edges(block=False, line_style="--", color="black")


def create_grid(contour: GeometryContour) -> list:
    x_max_area: float = contour.get_x_max()
    x_min_area: float = contour.get_x_min()
    y_max_area: float = contour.get_y_max()
    y_min_area: float = contour.get_y_min()

    grid_point_list: list = list()
    x_list: np.array = np.arange(x_min_area, x_max_area, 0.05)  # Change to linspace and calculate how many points I want? Should be better as Internet tells
    y_list: np.array = np.arange(y_min_area, y_max_area, 0.05)
    for x_counter in x_list:
        for y_counter in y_list:
            grid_point: np.array = np.array([x_counter, y_counter])
            if contour.is_point_in_contour(grid_point):
                grid_point_list.append(grid_point)

    return grid_point_list


if __name__ == '__main__':
    script_dir = os.path.dirname(__file__)
    urdf_file_path = os.path.join(script_dir, '../urdf_form_creator/urdf/basic_geometry.urdf')
    urdf_reader: URDFReader = URDFReader(urdf_file_path)

    object_to_move: MovableObject

    size_info_list: list = urdf_reader.size_info.split()
    object_contour_params: dict = dict()
    if(urdf_reader.geometry_type == GeometryType.Box):
        object_contour_params["x_length"] = float(size_info_list[0])
        object_contour_params["y_length"] = float(size_info_list[1])
        object_to_move = Box()

    elif(urdf_reader.geometry_type == GeometryType.Cylinder):
        object_contour_params["radius"] = float(size_info_list[0])
        object_to_move = Cylinder()

    elif(urdf_reader.geometry_type == GeometryType.IsoscelesTriangle):
        object_contour_params["scale"] = float(size_info_list[0])
        object_to_move = IsoscelesTriangle()

    elif(urdf_reader.geometry_type == GeometryType.RightAngledTriangle):
        object_contour_params["x_scale"] = float(size_info_list[0])
        object_contour_params["y_scale"] = float(size_info_list[1])
        object_to_move = RightAngledTriangle()

    object_contour_params["height"] = float(size_info_list[2])
    object_contour_params["mass"] = float(urdf_reader.mass)
    object_to_move.import_urdf_info(**object_contour_params)

    extended_object_contour: GeometryContour = GeometryContour()
    extended_object_contour.import_contour_with_offset(object_to_move, 0.4)

    grip_area: GeometryContour = GeometryContour()
    grip_area.import_contour_with_offset(object_to_move, -0.1)

    grid_point_list = create_grid(grip_area)

    # init gripping positions
    grip_point_area: GeometryContour = GeometryContour()
    centroid_object_to_move: np.array = object_to_move.calculate_centroid()
    distance_from_centroid: float = 0.1
    angle_diff_between_robot: float = (2*pi) / (NUMBER_OF_ROBOTS)
    # offset_rotation: float = random.uniform((pi / 180), pi)
    offset_rotation:float = 0.0
    # distance_from_centroid: float = random.uniform(0.01, 0.5)
    for counter in range(0, NUMBER_OF_ROBOTS):
        grip_point: np.array = np.array([distance_from_centroid * cos((angle_diff_between_robot * counter) + (pi / 180) + offset_rotation),
                                         distance_from_centroid * sin((angle_diff_between_robot * counter) + (pi / 180) + offset_rotation)])
        grip_point_area.add_contour_corner(grip_point)

    # grip_point_area.plot_corners(color="green", marker_size=10)
    # grip_point_area.plot_edges()
    original_grip_point_area = copy.deepcopy(grip_point_area)
    # end init gripping position

    # Method for optimizing the gripping position
    # Multiple problems! Nimmt am Ende immer die gleiche Seite, obwohl diese nichtmehr verbessert werden kann.
    # Lokale Minima werden angenommen und Optimierer kommt da nichtmehr raus

    for counter in range(0, 4):
        # closest_edge: EdgeInfo = grip_point_area.get_closest_edge_to_point(centroid_object_to_move)
        # index_of_corner: int = grip_point_area.get_index_of_corner(closest_edge.start_point)  # Move start point
        index_of_corner: int = counter % NUMBER_OF_ROBOTS
        print(index_of_corner)
        # distance_list: list = list()
        # for edge in grip_point_area.edge_list:
        #     distance_list.append(grip_point_area.calculate_distance_point_to_line(centroid_object_to_move, edge._edge_vector, edge._start_point))
        # print("0: " + str(distance_list[0]) + " | 1: " + str(distance_list[1]) + " | 2: " + str(distance_list[2]))
        # plot.plot(closest_edge.start_point[0], closest_edge.start_point[1], "ko")
        copy_of_grip_point_area: GeometryContour = copy.deepcopy(grip_point_area)

        # Right fitness calculation
        A: np.array = np.zeros((NUMBER_OF_ROBOTS, NUMBER_OF_ROBOTS), dtype=float)
        # F: np.array = np.zeros(NUMBER_OF_ROBOTS)
        l: np.array = np.zeros(NUMBER_OF_ROBOTS, dtype=float)

        for line_counter, grip_edge in enumerate(grip_point_area.edge_list):
            for column_counter, grip_corner in enumerate(grip_point_area.corner_point_list):
                if np.array_equal(grip_corner, grip_edge.start_point) or \
                        np.array_equal(grip_corner, grip_edge.end_point):
                    A[line_counter, column_counter] = 0
                else:
                    A[line_counter, column_counter] = \
                        grip_point_area.calculate_distance_point_to_line(grip_corner, grip_edge.edge_vector, grip_edge.start_point)
            l[line_counter] = grip_point_area.calculate_distance_point_to_line(centroid_object_to_move,
                                                                               grip_edge.edge_vector,
                                                                               grip_edge.start_point)
        print("A:")
        print(A)
        print("l:")
        print(l)
        F: np.array = np.linalg.pinv(A).dot(l)
        print("F:")
        print(F)
        print(np.linalg.pinv(A).dot(l))

        best_result: float = 100000000
        best_point: np.array = np.array([])
        for grid_point in grid_point_list:
            copy_of_grip_point_area.replace_contour_corner(index_of_corner, grid_point)
            if not copy_of_grip_point_area.is_point_in_contour(centroid_object_to_move):
                continue

            if copy_of_grip_point_area.do_edges_intersect():
                continue

            # plot.plot(grid_point[0], grid_point[1], marker="o", color=mcolors.CSS4_COLORS["lightsteelblue"])
            result: float = 0.0
            for edge_info in copy_of_grip_point_area.edge_list:
                distance: float = copy_of_grip_point_area.calculate_distance_point_to_line(centroid_object_to_move, edge_info.edge_vector, edge_info.start_point)
                result = result + (1/pow(distance, 2))

            if best_result >= result:
                best_result = result
                best_point = grid_point

        grip_point_area.replace_contour_corner(index_of_corner, best_point)
        # if counter == 9:
        #     plot.plot(best_point[0], best_point[1], "ko")
    # end method for optimizing grip position

    grip_point_area.plot_corners(color="green", marker_size=10)
    grip_point_area.plot_edges()
    # original_grip_point_area.plot_corners(color="green")
    # original_grip_point_area.plot_edges()

    ur5_base_link_pose_list_list: list = list()

    for grip_point in grip_point_area.corner_point_list:
        arm_base_pose_possible: GeometryContour = GeometryContour()

        for counter in range(0, 100):
            pose: np.array = np.array([grip_point[0] + 0.75*cos(((2*pi)/100) * counter), grip_point[1] + 0.75 * sin(((2*pi)/100) * counter)])
            arm_base_pose_possible.add_contour_corner(pose)

        list_with_points = create_grid(arm_base_pose_possible)

        possible_ur5_base_link_pose_list: list = list()
        for point in list_with_points:
            if not extended_object_contour.is_point_in_contour(point):
                possible_ur5_base_link_pose_list.append(point)

        ur5_base_link_pose_list_list.append(possible_ur5_base_link_pose_list)

    # Random einen Punkt in einer Punktwolke auswählen
    # Anschließend wieder alle bis auf einen Punkt statisch machen und den einen optimieren.
    # Danach den nächsten Punkt verschieben usw.

    plot_contour_info(object_to_move, extended_object_contour, grip_area)

    axis: plot.Axes = plot.gca()  # Get current axis object and set x and y to be equal so a square is a square
    axis.axis("equal")
    plot.show()  # Let this be the last command so the plots wont be closed instantly

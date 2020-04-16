import sys
import os

from matplotlib import pyplot as plot
import numpy as np
import random
from math import sin, cos, pi

from geometry_info.geometry_contour import GeometryContour
from geometry_info.movable_object import MovableObject, Box, Cylinder, IsoscelesTriangle, RightAngledTriangle
from geometry_info.edge_info import EdgeInfo

from urdf_reader import URDFReader
from urdf_reader import GeometryType


NUMBER_OF_ROBOTS: int = 4


def plot_contour_info(object_to_move, extended_object_contour, grip_area):
    object_to_move.print_info()
    object_to_move.plot_corners(block=False)
    object_to_move.plot_edges(block=False)
    object_to_move.plot_centroid(block=False)
    
    extended_object_contour.print_info()
    extended_object_contour.plot_corners(block=False)
    extended_object_contour.plot_edges(block=False)

    grip_area.print_info()
    grip_area.plot_corners(block=False)
    grip_area.plot_edges(block=False)


def create_grid(contour: GeometryContour) -> list:
    x_max_area: float = contour.get_x_max()
    x_min_area: float = contour.get_x_min()
    y_max_area: float = contour.get_y_max()
    y_min_area: float = contour.get_y_min()
    
    grid_point_list: list = list()
    x_list: np.array = np.arange(x_min_area, x_max_area, 0.05) #Change to linspace and calculate how many points I want? Should be better as Internet tells
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

    #init gripping positions
    grip_point_area: GeometryContour = GeometryContour()
    centroid_object_to_move: np.array = object_to_move.calculate_centroid()
    init_distance_from_centroid: float = 0.1
    angle_diff_between_robot: float = (2*pi) / (NUMBER_OF_ROBOTS)
    for counter in range(0, NUMBER_OF_ROBOTS):
        grip_point: np.array = np.array([init_distance_from_centroid * cos((angle_diff_between_robot * counter) + (pi / 180)),
                                         init_distance_from_centroid * sin((angle_diff_between_robot * counter) + (pi / 180))])
        grip_point_area.add_contour_corner(grip_point)
    
    grip_point_area.plot_corners()
    grip_point_area.plot_edges()
    # end init gripping position

    # Method for optimizing the gripping position
    # Multiple problems! Nimmt am Ende immer die gleiche Seite, obwohl diese nichtmehr verbessert werden kann.
    # Lokale Minima werden angenommen und Optimierer kommt da nichtmehr raus

    for counter in range(0,10):
        closest_edge: EdgeInfo = grip_point_area.get_closest_edge_to_point(centroid_object_to_move)
        index_of_corner: int = grip_point_area.get_index_of_corner(closest_edge.start_point) #Move start point
        plot.plot(closest_edge.start_point[0], closest_edge.start_point[1], "ko")
        copy_of_grip_point_area: GeometryContour = grip_point_area

        best_result: float = 100000000
        best_point: np.array
        for grid_point in grid_point_list:
            copy_of_grip_point_area.replace_contour_corner(index_of_corner, grid_point)
            if not copy_of_grip_point_area.is_point_in_contour(centroid_object_to_move):
                continue

            if copy_of_grip_point_area.do_edges_intersect():
                continue
            
            result: float= 0.0
            for edge_info in copy_of_grip_point_area.edge_list:
                distance: float = copy_of_grip_point_area.calculate_distance_point_to_line(centroid_object_to_move, edge_info.edge_vector, edge_info.start_point)
                result = result + (1/pow(distance,2))

            if best_result >= result:
                best_result = result
                best_point = grid_point

        grip_point_area.replace_contour_corner(index_of_corner, best_point)
    # end method for optimizing grip position

    

    
    grip_point_area.plot_corners()
    grip_point_area.plot_edges()

    ur5_base_link_pose_list_list: list = list()

    for grip_point in grip_point_area.corner_point_list:
        arm_base_pose_possible: GeometryContour = GeometryContour()
        
        for counter in range(0,100):
            pose: np.array = np.array([grip_point[0] + 0.75*cos(((2*pi)/100) * counter), grip_point[1] + 0.75 * sin(((2*pi)/100) * counter)])
            arm_base_pose_possible.add_contour_corner(pose)

        list_with_points = create_grid(arm_base_pose_possible)
        
        possible_ur5_base_link_pose_list: list = list()
        for point in list_with_points:
            if not extended_object_contour.is_point_in_contour(point):
                possible_ur5_base_link_pose_list.append(point)
        
        ur5_base_link_pose_list_list.append(possible_ur5_base_link_pose_list)

    #Random ienen Punkt in einer Punktwolke auswählen
    #Anschließend wieder alle bis auf einen Punkt statisch machen und den einen optimieren.
    #Danach den nächsten Punkt verschieben usw.

    

    
    
        

    
    



        



    plot_contour_info(object_to_move, extended_object_contour, grip_area)

    axis: plot.Axes = plot.gca() # Get current axis object and set x and y to be equal so a square is a square
    axis.axis("equal")
    plot.show() #Let this be the last command so the plots wont be closed instantly






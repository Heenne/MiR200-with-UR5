import sys
import os

from matplotlib import pyplot as plot
import numpy as np
import random

from geometry_info.geometry_contour import GeometryContour
from geometry_info.movable_object import MovableObject, Box, Cylinder, IsoscelesTriangle, RightAngledTriangle

from urdf_reader import URDFReader
from urdf_reader import GeometryType


NUMBER_OF_ROBOTS: int = 3


def plot_contour_info(object_to_move, extended_object_contour, extended_object_contour2, grip_area):
    object_to_move.print_info()
    object_to_move.plot_corners(block=False)
    object_to_move.plot_edges(block=False)
    object_to_move.plot_centroid(block=False)
    
    extended_object_contour.print_info()
    extended_object_contour.plot_corners(block=False)
    extended_object_contour.plot_edges(block=False)

    extended_object_contour2.print_info()
    extended_object_contour2.plot_corners(block=False)
    extended_object_contour2.plot_edges(block=False)

    grip_area.print_info()
    grip_area.plot_corners(block=False)
    grip_area.plot_edges(block=False)





def init_grip_point(area: GeometryContour) -> np.array:
    grip_point: np.array = create_random_point_in_area(area)
    while(not area.is_point_in_contour(grip_point)):
        grip_point = create_random_point_in_area(area)

    return grip_point


def create_random_point_in_area(area: GeometryContour) -> np.array:
    x_max_area: float = area.get_x_max()
    x_min_area: float = area.get_x_min()
    y_max_area: float = area.get_y_max()
    y_min_area: float = area.get_y_min()

    x_random_point: float = round(random.uniform(x_min_area, x_max_area), 2)
    y_random_point: float = round(random.uniform(y_min_area, y_max_area), 2) 
    grip_point: np.array = np.array([x_random_point, y_random_point])

    return grip_point


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
    extended_object_contour.import_contour_with_offset(object_to_move, 0.8)
   
    extended_object_contour2: GeometryContour = GeometryContour()
    extended_object_contour2.import_contour_with_offset(object_to_move, 1.2)

    grip_area: GeometryContour = GeometryContour()
    grip_area.import_contour_with_offset(object_to_move, -0.1)



    # Lay grid over area to grip
    x_max_area: float = grip_area.get_x_max()
    x_min_area: float = grip_area.get_x_min()
    y_max_area: float = grip_area.get_y_max()
    y_min_area: float = grip_area.get_y_min()
    
    grid_point_list: list = list()
    x_list: np.array = np.arange(x_min_area, x_max_area, 0.05) #Change to linspace and calculate how many points I want? Should be better as Internet tells
    y_list: np.array = np.arange(y_min_area, y_max_area, 0.05)
    for x_counter in x_list:
        for y_counter in y_list:
            grid_point: np.array = np.array([x_counter, y_counter])
            if grip_area.is_point_in_contour(grid_point):
                grid_point_list.append(grid_point)
                plot.plot(grid_point[0], grid_point[1], "co")


    grip_point_area: GeometryContour = GeometryContour()
    centroid_object_to_move: np.array = object_to_move.calculate_centroid()
    for counter in range(0, NUMBER_OF_ROBOTS):
        grip_point_area.add_contour_corner(init_grip_point(grip_area))
    
    while (not grip_point_area.is_point_in_contour(centroid_object_to_move)):
        grip_point_area: GeometryContour = GeometryContour()
        for counter in range(0, NUMBER_OF_ROBOTS):
            grip_point_area.add_contour_corner(init_grip_point(grip_area))  

    




    grip_point_area.plot_corners()
    grip_point_area.plot_edges()





    plot_contour_info(object_to_move, extended_object_contour, extended_object_contour2, grip_area)

    axis: plot.Axes = plot.gca() # Get current axis object and set x and y to be equal so a square is a square
    axis.axis("equal")
    plot.show() #Let this be the last command so the plots wont be closed instantly






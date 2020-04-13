import sys
import os

from matplotlib import pyplot as plot
import numpy as np

from geometry_info.geometry_contour import GeometryContour
from geometry_info.movable_object import MovableObject, Box, Cylinder, IsoscelesTriangle, RightAngledTriangle

from urdf_reader import URDFReader
from urdf_reader import GeometryType



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

    print(str(grip_area.is_point_in_contour(np.array([0,0.8]))))

    plot_contour_info(object_to_move, extended_object_contour, extended_object_contour2, grip_area)

    axis: plot.Axes = plot.gca() # Get current axis object and set x and y to be equal so a square is a square
    axis.axis("equal")
    plot.show() #Let this be the last command so the plots wont be closed instantly




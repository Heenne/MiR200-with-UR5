import sys
import os

from matplotlib import pyplot as plot

# from geometry_contour import GeometryContour
from geometry_info import GeometryContour, Box, Cylinder, IsoscelesTriangle, RightAngledTriangle

from urdf_reader import URDFReader
from urdf_reader import GeometryType

if __name__ == '__main__':
    script_dir = os.path.dirname(__file__)
    print(script_dir)
    urdf_file_path = os.path.join(script_dir, '../urdf_form_creator/urdf/basic_geometry.urdf')
    print(urdf_file_path)
    urdf_reader: URDFReader = URDFReader(urdf_file_path)

    object_contour: GeometryContour

    size_info_list: list = urdf_reader.size_info.split()
    object_contour_params: dict = dict()
    if(urdf_reader.geometry_type == GeometryType.Box):
        object_contour_params["x_length"] = float(size_info_list[0])
        object_contour_params["y_length"] = float(size_info_list[1])
        object_contour = Box()

    elif(urdf_reader.geometry_type == GeometryType.Cylinder):
        object_contour_params["radius"] = float(size_info_list[0])
        object_contour = Cylinder()

    elif(urdf_reader.geometry_type == GeometryType.IsoscelesTriangle):
        object_contour_params["scale"] = float(size_info_list[0])
        object_contour = IsoscelesTriangle()

    elif(urdf_reader.geometry_type == GeometryType.RightAngledTriangle):
        object_contour_params["x_scale"] = float(size_info_list[0])
        object_contour_params["y_scale"] = float(size_info_list[1])
        object_contour = RightAngledTriangle()
        
        
    object_contour.import_contour(**object_contour_params)
    object_contour.print_contour_info()
    object_contour.plot_corners(block=False)
    object_contour.plot_edges(block=False)
    object_contour.plot_centroid(block=False)
    object_contour.plot_orthogonal_vector_centroid_to_edge(block = False)

    plot.show() #Let this be the last command so the plots wont be closed instantly
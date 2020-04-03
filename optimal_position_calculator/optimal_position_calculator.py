import sys
import os

from contour import contour
from urdf_reader import URDFReader

if __name__ == '__main__':
    script_dir = os.path.dirname(__file__)
    print(script_dir)
    urdf_file_path = os.path.join(script_dir, '../urdf_form_creator/urdf/basic_geometry.urdf')
    print(urdf_file_path)
    object_contour: contour = contour()
    urdf_reader: URDFReader = URDFReader(urdf_file_path)
    pass
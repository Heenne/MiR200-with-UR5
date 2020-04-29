import sys
from enum import IntEnum

from lxml import etree


class GeometryType(IntEnum):
    Box = 0
    Cylinder = 1
    IsoscelesTriangle = 2
    RightAngledTriangle = 3


class URDFReader:
    _xml_root_node: etree.ElementTree
    geometry_type: GeometryType
    size_info: str
    mass: float

    object_name: str
    link_name: str

    def __init__(self, file_path: str):
        """Init method for the URDF Reader class.
        Method will take the urdf file in the specified file path and import the needed data.

        :param file_path: path to urdf file that should be read
        :type file_path: str
        """
        self._xml_root_node = etree.ElementTree()
        self._xml_root_node = etree.parse(file_path)

        self.object_name = self._xml_root_node.getroot().get('name')
        self.link_name = self._xml_root_node.find("link").get('name')
        collision_geometry_node: etree.ElementTree = self._xml_root_node.find("link").find("collision").find("geometry")
        self.geometry_type = self._str_to_geometry_type(collision_geometry_node)

        if self.geometry_type == GeometryType.Box:
            self.size_info = collision_geometry_node.find("box").get("size")
        elif self.geometry_type == GeometryType.Cylinder:
            self.size_info = collision_geometry_node.find("cylinder").get("radius")
            self.size_info = self.size_info + " " + collision_geometry_node.find("cylinder").get("length")
        elif self.geometry_type == GeometryType.IsoscelesTriangle:
            self.size_info = collision_geometry_node.find("mesh").get("scale")
        elif self.geometry_type == GeometryType.RightAngledTriangle:
            self.size_info = collision_geometry_node.find("mesh").get("scale")

        self.mass = float(self._xml_root_node.find("link").find("inertial").find("mass").get("value"))

        self.print_urdf_info()

    def print_urdf_info(self):
        """Print the relevant info that was read from the urdf file
        """
        print("object name: " + self.object_name, end="\n")
        print("link name: " + self.link_name, end="\n")
        print("Geometry type: " + str(self.geometry_type), end="\n")
        print("Size info: " + self.size_info, end="\n")
        print("Mass: " + str(self.mass), end="\n")

    def _str_to_geometry_type(self, geometry_xml_element: etree.Element) -> GeometryType:
        """Get the type of geometry that is specified by the urdf file

        :param geometry_xml_element: Element from the xml parser that points to the link/collision/geometry node
        :type geometry_xml_element: etree.Element
        :return: Type of Geometry that is specified in the urdf file
        :rtype: GeometryType
        """
        if geometry_xml_element.find("box") is not None:
            return GeometryType.Box
        elif geometry_xml_element.find("cylinder") is not None:
            return GeometryType.Cylinder
        elif geometry_xml_element.find("mesh") is not None:
            filename: str = geometry_xml_element.find("mesh").get("filename")
            if "Isosceles_Triangle" in filename:
                return GeometryType.IsoscelesTriangle
            elif "Right_Angled_Triangle" in filename:
                return GeometryType.RightAngledTriangle
        else:
            return None

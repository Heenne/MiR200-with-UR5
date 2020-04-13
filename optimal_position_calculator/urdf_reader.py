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
        self._xml_root_node = etree.ElementTree()
        self._xml_root_node = etree.parse(file_path)
        
        self.object_name = self._xml_root_node.getroot().get('name')
        self.link_name = self._xml_root_node.find("link").get('name')
        self.geometry_type = self.__str_to_geometry_type(self._xml_root_node.find("link").find("collision").find("geometry"))
        
        if self.geometry_type==GeometryType.Box:
            self.size_info = self._xml_root_node.find("link").find("collision").find("geometry").find("box").get("size")
        elif self.geometry_type==GeometryType.Cylinder:
            self.size_info = self._xml_root_node.find("link").find("collision").find("geometry").find("cylinder").get("radius")
        elif self.geometry_type==GeometryType.IsoscelesTriangle:
            self.size_info = self._xml_root_node.find("link").find("collision").find("geometry").find("mesh").get("scale")
        elif self.geometry_type==GeometryType.RightAngledTriangle:
            self.size_info = self._xml_root_node.find("link").find("collision").find("geometry").find("mesh").get("scale")

        self.mass = float(self._xml_root_node.find("link").find("inertial").find("mass").get("value"))

        self.print_urdf_info()

    
    def print_urdf_info(self):
        print("object name: " + self.object_name, end="\n")
        print("link name: " + self.link_name, end="\n")
        print("Geometry type: " + str(self.geometry_type), end="\n")
        print("Size info: " + self.size_info, end="\n")
        print("Mass: " + str(self.mass), end="\n")
    
    
    def __str_to_geometry_type(self, geometry_xml_element: etree.Element) -> GeometryType:
        if geometry_xml_element.find("box") != None: 
            return GeometryType.Box
        elif geometry_xml_element.find("cylinder") != None:
            return GeometryType.Cylinder
        elif geometry_xml_element.find("mesh") != None:
            filename : str = geometry_xml_element.find("mesh").get("filename")
            if "Isosceles_Triangle" in filename:
                return GeometryType.IsoscelesTriangle
            elif "Right_Angled_Triangle" in filename:
                return GeometryType.RightAngledTriangle
        else:
            return None
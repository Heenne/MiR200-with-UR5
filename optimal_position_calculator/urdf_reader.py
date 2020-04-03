import sys
from enum import IntEnum

from lxml import etree

from contour import contour

class GeometryType(IntEnum):
    Box = 0
    Cylinder = 1
    IsoscelesTriangle = 2
    RightAngledTriangle = 3


class URDFReader:
    __xml_root_node: etree.ElementTree
    __geometry_type: GeometryType
    __size_info: str
    
    __object_name: str
    __link_name: str


    def __init__(self, file_path: str):
        self.__xml_root_node = etree.ElementTree()
        self.__xml_root_node = etree.parse(file_path)
        
        self.__object_name = self.__xml_root_node.getroot().get('name')
        self.__link_name = self.__xml_root_node.find("link").get('name')
        self.__geometry_type = self.__str_to_geometry_type(self.__xml_root_node.find("link").find("collision").find("geometry"))
        
        if self.__geometry_type==GeometryType.Box:
            self.__size_info = self.__xml_root_node.find("link").find("collision").find("geometry").find("box").get("size")
        elif self.__geometry_type==GeometryType.Cylinder:
            self.__size_info = self.__xml_root_node.find("link").find("collision").find("geometry").find("cylinder").get("radius")
        elif self.__geometry_type==GeometryType.IsoscelesTriangle:
            self.__size_info = self.__xml_root_node.find("link").find("collision").find("geometry").find("mesh").get("scale")
        elif self.__geometry_type==GeometryType.RightAngledTriangle:
            self.__size_info = self.__xml_root_node.find("link").find("collision").find("geometry").find("mesh").get("scale")

        self.print_urdf_info()

    
    def print_urdf_info(self):
        print("object name: " + self.__object_name, end="\n")
        print("link name: " + self.__link_name, end="\n")
        print("Geometry type: " + str(self.__geometry_type), end="\n")
        print("Size info: " + self.__size_info, end="\n")
    
    
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

    def get_geometry_type(self) -> GeometryType:
        return self.__geometry_type

    def get_size_info(self) -> str:
        return self.__size_info

    def get_object_name(self) -> str:
        return self.__object_name

    def get_link_name(self) -> str:
        return self.__link_name
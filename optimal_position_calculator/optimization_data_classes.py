from dataclasses import dataclass
from typing import List

from geometry_info.geometry_contour import GeometryContour
from geometry_info.MuR205 import MuR205


@dataclass
class MuR205PoseOptimizationInfo:
    # Contains the points where the ur5 base link should be positioned
    ur5_base_link_pose_contour: GeometryContour
    # Contains a list with all MiR205 contours
    # Each contour belongs to one corner of the ur5_base_link_pose_contour GeometryContour
    mur205_contour_list: List[MuR205]

    # Cannot insert this object in key. So storing fitness here this will fix it for now
    fitness: float

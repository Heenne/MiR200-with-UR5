import numpy as np

from geometry_info.geometry_contour import GeometryContour


class MuR205(GeometryContour):
    # region Member
    _lead_vector_mir_to_ur5: np.array

    # Contains rotation and translation of the mir base link cs to the ur5 base link cs
    _tf_mir_to_ur5: np.array
    _tf_ur5_to_mir: np.array
    # endregion

    def __init__(self, mir_base_link_world_cs: np.array = np.array([0, 0]), mir_base_link_cs_rotation:float = 0.0):
        super().__init__(lead_vector_world_cs=mir_base_link_world_cs,
                         world_to_geometry_cs_rotation=mir_base_link_cs_rotation)

        # Add outer dimensions of the MiR200 base to corner list
        self.add_contour_corner_geometry_cs(np.array([0.4475, 0.291]))
        self.add_contour_corner_geometry_cs(np.array([-0.4475, 0.291]))
        self.add_contour_corner_geometry_cs(np.array([-0.4475, -0.291]))
        self.add_contour_corner_geometry_cs(np.array([0.4475, -0.291]))

        # Create contour edges of the MiR200 base platform
        self.create_contour_edges()

        # tf between mir base link and ur5 base link
        self._set_mur205_transformation()

    # region Properties
    @property
    def lead_vector_base_link_to_ur5(self):
        return self._lead_vector_mir_to_ur5

    @property
    def lead_vector_ur5_to_base_link(self) -> np.array:
        # TODO Docstring
        return -1 * self._lead_vector_mir_to_ur5
    # endregion

    # region private methods
    def _set_mur205_transformation(self):
        # Define lead vector from mir-base-link to ur5-base-link
        # These values were measured with a laser tracking system
        self._lead_vector_mir_to_ur5 = np.array([0.244311, -0.140242])

        self._tf_ur5_to_mir = np.array([[1, 0, self._lead_vector_mir_to_ur5[0]],
                                        [0, 1, self._lead_vector_mir_to_ur5[1]],
                                        [0, 0, 1]])
        self._tf_mir_to_ur5 = np.linalg.inv(self._tf_ur5_to_mir)
    # endregion

    # region public methods
    def rotate_around_ur5_base_cs(self):
        # when rotating around the ur5 base link remember to update the mir to world tf, lead vector and rotation!
        pass
    # endregion

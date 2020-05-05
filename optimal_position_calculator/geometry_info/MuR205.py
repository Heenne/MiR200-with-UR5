import numpy as np

from geometry_info.geometry_contour import GeometryContour


class MuR205(GeometryContour):
    # region Member
    _lead_vector_mir_to_ur5: np.array

    # Contains rotation and translation of the mir base link cs to the ur5 base link cs
    _tf_mir_to_ur5_cs: np.array
    _tf_ur5_to_mir_cs: np.array

    # endregion

    def __init__(self, mir_base_link_world_cs: np.array = np.array([0, 0]), mir_base_link_cs_rotation: float = 0.0):
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
        # TODO Docstring
        return self._lead_vector_mir_to_ur5

    @property
    def lead_vector_ur5_to_base_link(self) -> np.array:
        # TODO Docstring
        return -1 * self._lead_vector_mir_to_ur5

    @property
    def tf_ur5_to_world_cs(self) -> np.array:
        # TODO Docstring
        return self._tf_geometry_to_world_cs.dot(self._tf_ur5_to_mir_cs)

    @property
    def tf_world_to_ur5_cs(self) -> np.array:
        # TODO Docstring
        return self._tf_mir_to_ur5_cs.dot(self._tf_world_to_geometry_cs)

    # endregion

    # region private methods
    def _set_mur205_transformation(self):
        # TODO Docstring
        # Define lead vector from mir-base-link to ur5-base-link
        # These values were measured with a laser tracking system
        self._lead_vector_mir_to_ur5 = np.array([0.244311, -0.140242])

        # There is only translation between mir and ur5 base link, no rotation
        self._tf_ur5_to_mir_cs = self._create_transformation_matrix(self._lead_vector_mir_to_ur5, 0)
        self._tf_mir_to_ur5_cs = np.linalg.inv(self._tf_ur5_to_mir_cs)

    # endregion

    # region public methods
    def move_mur205_by_ur5_base_link(self, ur5_lead_vector_world_cs: np.array, mur205_rotation_world_cs: float = 0.0):
        tf_ur5_to_world_cs: np.array = self._create_transformation_matrix(ur5_lead_vector_world_cs,
                                                                          mur205_rotation_world_cs)
        new_tf_mir_to_world_cs: np.array = tf_ur5_to_world_cs.dot(self._tf_mir_to_ur5_cs)
        new_tf_world_to_mir_cs: np.array = np.linalg.inv(new_tf_mir_to_world_cs)

        self._lead_vector_world_cs = self._get_translation_from_tf(new_tf_mir_to_world_cs)
        self._geometry_cs_rotation = self._get_rotation_from_tf(new_tf_mir_to_world_cs)
        self._tf_geometry_to_world_cs = new_tf_mir_to_world_cs
        self._tf_world_to_geometry_cs = new_tf_world_to_mir_cs

    def rotate_relative_around_ur5_base_cs(self, relative_rotation_radian: float):
        # TODO Docstring
        ur5_lead_vector_world_cs: np.array = self._get_translation_from_tf(self.tf_ur5_to_world_cs)

        new_tf_ur5_to_world_cs: np.array = self._create_transformation_matrix(
            ur5_lead_vector_world_cs,
            self._geometry_cs_rotation + relative_rotation_radian)

        new_tf_mir_to_world_cs: np.array = new_tf_ur5_to_world_cs.dot(self._tf_mir_to_ur5_cs)

        self._lead_vector_world_cs = self._get_translation_from_tf(new_tf_mir_to_world_cs)
        self._geometry_cs_rotation = self._get_rotation_from_tf(new_tf_mir_to_world_cs)
        self._tf_geometry_to_world_cs = new_tf_mir_to_world_cs
        self._tf_world_to_geometry_cs = np.linalg.inv(new_tf_mir_to_world_cs)

    def rotate_absolute_around_ur5_base_cs(self, absolute_rotation_radian: float):
        # TODO Docstring
        ur5_lead_vector_world_cs: np.array = self._get_translation_from_tf(self.tf_ur5_to_world_cs)

        new_tf_ur5_to_world_cs: np.array = self._create_transformation_matrix(ur5_lead_vector_world_cs,
                                                                              absolute_rotation_radian)

        new_tf_mir_to_world_cs: np.array = new_tf_ur5_to_world_cs.dot(self._tf_mir_to_ur5_cs)

        self._lead_vector_world_cs = self._get_translation_from_tf(new_tf_mir_to_world_cs)
        self._geometry_cs_rotation = self._get_rotation_from_tf(new_tf_mir_to_world_cs)
        self._tf_geometry_to_world_cs = new_tf_mir_to_world_cs
        self._tf_world_to_geometry_cs = np.linalg.inv(new_tf_mir_to_world_cs)
    # endregion

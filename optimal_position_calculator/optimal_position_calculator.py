import os

from matplotlib import pyplot as plot
# from matplotlib import colors as mcolors
import random
import numpy as np
from math import sin, cos, pi
from typing import List
from copy import deepcopy

from geometry_info.geometry_contour import GeometryContour
from geometry_info.movable_object import MovableObject, Box, Cylinder, IsoscelesTriangle, RightAngledTriangle
from geometry_info.MuR205 import MuR205

from optimization_data_classes import MuR205PoseOptimizationInfo

from urdf_reader import URDFReader
from urdf_reader import GeometryType
from launch_reader import LaunchReader

NUMBER_OF_ROBOTS: int = 3


def plot_contour_info(object_to_move, extended_object_contour, grip_area):
    object_to_move.print_info()
    object_to_move.plot_corners(block=False)
    object_to_move.plot_edges(block=False)
    object_to_move.plot_centroid(block=False)

    extended_object_contour.print_info()
    extended_object_contour.plot_corners(block=False)
    extended_object_contour.plot_edges(block=False)

    grip_area.print_info()
    grip_area.plot_corners(block=False)
    grip_area.plot_edges(block=False)


def init_grip_point_contour(lead_vector_world_cs: np.array,
                            max_distance_from_centroid: float,
                            number_of_robots: int) -> GeometryContour:
    grip_point_contour: GeometryContour = GeometryContour()

    # Random offset from 0 degree (right side) in mathematical positive direction. Range from 1°-180°
    offset_rotation: float = random.uniform((pi / 180), pi)
    angle_diff_between_robot: float = (2 * pi) / NUMBER_OF_ROBOTS

    for counter in range(0, number_of_robots):
        distance_from_centroid: float = random.uniform(0.01, max_distance_from_centroid)
        grip_point_world_cs: np.array = np.array(
            [lead_vector_world_cs[0] + distance_from_centroid *
             cos((angle_diff_between_robot * counter) + offset_rotation),
             lead_vector_world_cs[1] + distance_from_centroid *
             sin((angle_diff_between_robot * counter) + offset_rotation)])

        grip_point_contour.add_contour_corner_world_cs(grip_point_world_cs)

    return grip_point_contour


def init_grip_contour_population(max_population: int,
                                 lead_vector: np.array,
                                 max_distance_from_centroid: float,
                                 number_of_robots: int) -> list:
    grip_contour_population: list = list()
    for counter in range(0, max_population):
        new_grip_contour: GeometryContour = init_grip_point_contour(lead_vector,
                                                                    max_distance_from_centroid,
                                                                    number_of_robots)
        grip_contour_population.append(new_grip_contour)

    return grip_contour_population


def calculate_fitness_of_grip_contour(grip_contour: GeometryContour,
                                      centroid_object_to_move_world_cs: np.array) -> float:
    fitness: float = 0.0
    if not grip_contour.is_world_cs_point_in_contour(centroid_object_to_move_world_cs):
        return fitness
    if grip_contour.do_contour_edges_intersect():
        return fitness

    # for edge in grip_contour.edge_list:
    #     distance_to_centroid: float = grip_contour.calculate_distance_point_to_line(centroid_object_to_move,
    #                                                                                 edge.edge_vector,
    #                                                                                 edge.start_point)
    #     fitness += pow(distance_to_centroid, 2)
    fitness = pow(grip_contour.calc_distance_closest_edge_to_point(centroid_object_to_move_world_cs), 2)

    # Add additional fitness parameters here

    return fitness


def calculate_fitness_sum_of_grip_population(grip_contour_population: list, centroid_object_to_move: np.array) -> float:
    fitness_sum: float = 0.0
    for grip_contour in grip_contour_population:
        fitness: float = calculate_fitness_of_grip_contour(grip_contour, centroid_object_to_move)
        fitness_sum += fitness

    return fitness_sum


def calc_fitness_mur205_positions(mur205_pose_opti_info: MuR205PoseOptimizationInfo,
                                  ur5_base_link_boundary_list: List[GeometryContour],
                                  object_to_move_centroid_world_cs: np.array) -> float:
    """
    TODO
    :param mur205_pose_opti_info:
    :type mur205_pose_opti_info:
    :param ur5_base_link_boundary_list:
    :type ur5_base_link_boundary_list:
    :param object_to_move_centroid_world_cs:
    :type object_to_move_centroid_world_cs:
    :return: Smallest value will be best!
    :rtype:
    """
    # Calculate the reference value for the ur5 base link contour length fitness
    # This is used to unify the contour value to get a value between 0-1 for adding it to other
    # fitness values
    largest_ur5_base_link_contour: GeometryContour = GeometryContour(object_to_move_centroid_world_cs)
    for ur5_base_link_boundary in ur5_base_link_boundary_list:
        farthest_corner: np.array = ur5_base_link_boundary.calc_farthest_corner_to_point(
            object_to_move_centroid_world_cs)
        largest_ur5_base_link_contour.add_contour_corner_world_cs(farthest_corner)

    largest_ur5_base_link_contour_value: float = largest_ur5_base_link_contour.calc_contour_length()
    ur5_base_link_contour_length: float = mur205_pose_opti_info.ur5_base_link_pose_contour.calc_contour_length()
    # Smallest value (0) will be best
    contour_length_fitness: float = ur5_base_link_contour_length / largest_ur5_base_link_contour_value

    # Calculate the reference for the mir205 contour length fitness
    # largest_mur205_contour: GeometryContour = GeometryContour(object_to_move_centroid_world_cs)
    # for mur205_contour in mur205_pose_opti_info.mur205_contour_list:
    #     farthest_corner: np.array = mur205_contour.calc_farthest_corner_to_point(object_to_move_centroid_world_cs)
    #     largest_mur205_contour.add_contour_corner_world_cs(farthest_corner)
    #
    # largest_mur205_contour_value: float = largest_mur205_contour.calc_contour_length()
    # mur205_contour_length: float =

    contour_length_weighting: float = 1.0  # weighting of the contour length
    fitness: float = contour_length_weighting * contour_length_fitness

    return fitness


def calc_fitness_sum_mur205_population(mur205_pose_optimization_population: List[MuR205PoseOptimizationInfo],
                                       ur5_base_link_boundary_list: List[GeometryContour],
                                       object_to_move_centroid_world_cs: np.array) -> float:
    fitness_sum: float = 0.0
    for mur205_pose_opti_info in mur205_pose_optimization_population:
        fitness: float = calc_fitness_mur205_positions(mur205_pose_opti_info,
                                                       ur5_base_link_boundary_list,
                                                       object_to_move_centroid_world_cs)
        fitness_sum += fitness

    return fitness_sum


def mutate_geometry_contour(contour_to_mutate: GeometryContour,
                            min_mutation: float,
                            max_mutation: float,
                            border_contour: GeometryContour):
    """[summary]
    TODO
    :param contour_to_mutate: [description]
    :type contour_to_mutate: GeometryContour
    :param min_mutation: [description]
    :type min_mutation: float
    :param max_mutation: [description]
    :type max_mutation: float
    :param border_contour: [description]
    :type border_contour: GeometryContour
    :return: [description]
    :rtype: GeometryContour
    """
    index_of_mutated_corner: int = random.randrange(0, len(contour_to_mutate.corner_point_list_geometry_cs))

    corner_was_replaced: bool = False

    while not corner_was_replaced:
        corner_to_mutate_geometry_cs: np.array = \
            contour_to_mutate.corner_point_list_geometry_cs[index_of_mutated_corner].copy()

        is_additive_mutation_x: bool = bool(random.getrandbits(1))
        is_additive_mutation_y: bool = bool(random.getrandbits(1))
        mutation_value_x: float = random.uniform(min_mutation, max_mutation)
        mutation_value_y: float = random.uniform(min_mutation, max_mutation)

        if is_additive_mutation_x:
            corner_to_mutate_geometry_cs[0] += mutation_value_x
        else:
            corner_to_mutate_geometry_cs[0] -= mutation_value_x

        if is_additive_mutation_y:
            corner_to_mutate_geometry_cs[1] += mutation_value_y
        else:
            corner_to_mutate_geometry_cs[1] -= mutation_value_y

        if border_contour.is_geometry_cs_point_in_contour(corner_to_mutate_geometry_cs):
            contour_to_mutate.replace_contour_corner_geometry_cs(index_of_mutated_corner, corner_to_mutate_geometry_cs)
            corner_was_replaced = True

    # plot.plot(contour_to_mutate.corner_point_list[index_of_mutated_corner][0],
    #           contour_to_mutate.corner_point_list[index_of_mutated_corner][1],
    #           color=mcolors.CSS4_COLORS["pink"],
    #           marker="o")


def mutate_geometry_contour_with_dead_zones(mur205_opti_info: MuR205PoseOptimizationInfo,
                                            min_mutation_distance: float,
                                            max_mutation_distance: float,
                                            min_mutation_rotation: float,
                                            max_mutation_rotation: float,
                                            outside_border_list: List[GeometryContour],
                                            object_to_move: GeometryContour,
                                            deadzone_list: List[GeometryContour] = None) -> MuR205PoseOptimizationInfo:
    """Method for the mutation of corner-point of a contour.
    But this time each corner has its individual outside_border and there are multiple deadzones where
    no corner is allowed to be.
    The decision if the mutation happens was allready decided before calling this function

    :param mur205_opti_info: [description]
    :type mur205_opti_info: GeometryContour
    :param min_mutation_distance: [description]
    :type min_mutation_distance: float
    :param max_mutation_distance: [description]
    :type max_mutation_distance: float
    :param min_mutation_rotation:
    :type min_mutation_rotation:
    :param max_mutation_rotation:
    :type max_mutation_rotation:
    :param outside_border_list: [description]
    :type outside_border_list: GeometryContour
    :param object_to_move:
    :type object_to_move:
    :param deadzone_list: List of all deadzones that should be respected, defaults to list()
    :type deadzone_list: list, optional
    :return: [description]
    :rtype: MuR205PoseOptimizationInfo
    """
    if deadzone_list is None:
        deadzone_list = list()

    index_of_mutated_corner: int = random.randrange(0, len(mur205_opti_info.mur205_contour_list))

    corner_was_replaced: bool = False

    while not corner_was_replaced:
        corner_to_mutate_world_cs: np.array = \
            mur205_opti_info.ur5_base_link_pose_contour.corner_point_list_world_cs[index_of_mutated_corner].copy()

        is_additive_mutation_x: bool = bool(random.getrandbits(1))
        is_additive_mutation_y: bool = bool(random.getrandbits(1))
        mutation_value_x: float = random.uniform(min_mutation_distance, max_mutation_distance)
        mutation_value_y: float = random.uniform(min_mutation_distance, max_mutation_distance)

        if is_additive_mutation_x:
            corner_to_mutate_world_cs[0] += mutation_value_x
        else:
            corner_to_mutate_world_cs[0] -= mutation_value_x

        if is_additive_mutation_y:
            corner_to_mutate_world_cs[1] += mutation_value_y
        else:
            corner_to_mutate_world_cs[1] -= mutation_value_y

        # Check if the mutated corner point is in one of the listed deadzones
        corner_in_deadzone: bool = False
        for deadzone in deadzone_list:
            if deadzone.is_world_cs_point_in_contour(corner_to_mutate_world_cs):
                corner_in_deadzone = True
                break

        if outside_border_list[index_of_mutated_corner].is_world_cs_point_in_contour(corner_to_mutate_world_cs) and \
                not corner_in_deadzone:
            mur205_opti_info.ur5_base_link_pose_contour.replace_contour_corner_world_cs(index_of_mutated_corner,
                                                                                        corner_to_mutate_world_cs)
            corner_was_replaced = True

    mur205_was_rotated: bool = False
    while not mur205_was_rotated:
        mur205_to_rotate: MuR205 = deepcopy(mur205_opti_info.mur205_contour_list[index_of_mutated_corner])

        is_additive_mutation_rotation: bool = bool(random.getrandbits(1))
        mutation_relative_rotation: float = random.uniform(min_mutation_rotation, max_mutation_rotation)

        if is_additive_mutation_rotation:
            mur205_to_rotate.rotate_relative_around_ur5_base_cs(mutation_relative_rotation)
        else:
            mur205_to_rotate.rotate_relative_around_ur5_base_cs(-mutation_relative_rotation)

        if not mur205_to_rotate.is_contour_colliding(object_to_move):
            mur205_opti_info.mur205_contour_list[index_of_mutated_corner] = mur205_to_rotate
            mur205_was_rotated = True

    return mur205_opti_info


def create_circle_contour(centre_point_world_cs: np.array, radius: float) -> GeometryContour:
    resolution: int = 30
    circle_contour: GeometryContour = GeometryContour(lead_vector_world_cs=centre_point_world_cs)

    for counter in range(0, resolution):
        pose_geometry_cs: np.array = np.array([radius * cos(((2 * pi) / resolution) * counter),
                                               radius * sin(((2 * pi) / resolution) * counter)])
        circle_contour.add_contour_corner_geometry_cs(pose_geometry_cs)

    return circle_contour


if __name__ == '__main__':
    script_dir = os.path.dirname(__file__)
    urdf_file_path = os.path.join(script_dir, "../urdf_form_creator/urdf/basic_geometry.urdf")
    urdf_reader: URDFReader = URDFReader(urdf_file_path)

    launch_file_path = os.path.join(script_dir, "../urdf_form_creator/launch/spawn_basic_geometry.launch")
    launch_reader: LaunchReader = LaunchReader(launch_file_path)

    object_to_move: MovableObject = MovableObject()

    size_info_list: list = urdf_reader.size_info.split()
    object_contour_params: dict = dict()
    lead_vector: np.array = np.array([launch_reader.x_position, launch_reader.y_position])
    if urdf_reader.geometry_type == GeometryType.Box:
        object_contour_params["x_length"] = float(size_info_list[0])
        object_contour_params["y_length"] = float(size_info_list[1])
        object_contour_params["height"] = float(size_info_list[2])
        object_to_move = Box(lead_vector_world_cs=lead_vector,
                             world_to_geometry_cs_rotation=launch_reader.z_rotation)

    elif urdf_reader.geometry_type == GeometryType.Cylinder:
        object_contour_params["radius"] = float(size_info_list[0])
        object_contour_params["height"] = float(size_info_list[1])
        object_to_move = Cylinder(lead_vector_world_cs=lead_vector,
                                  world_to_geometry_cs_rotation=launch_reader.z_rotation)

    elif urdf_reader.geometry_type == GeometryType.IsoscelesTriangle:
        object_contour_params["scale"] = float(size_info_list[0])
        object_contour_params["height"] = float(size_info_list[2])
        object_to_move = IsoscelesTriangle(lead_vector_world_cs=lead_vector,
                                           world_to_geometry_cs_rotation=launch_reader.z_rotation)

    elif urdf_reader.geometry_type == GeometryType.RightAngledTriangle:
        object_contour_params["x_scale"] = float(size_info_list[0])
        object_contour_params["y_scale"] = float(size_info_list[1])
        object_contour_params["height"] = float(size_info_list[2])
        object_to_move = RightAngledTriangle(lead_vector_world_cs=lead_vector,
                                             world_to_geometry_cs_rotation=launch_reader.z_rotation)

    object_contour_params["mass"] = float(urdf_reader.mass)
    object_to_move.import_urdf_info(**object_contour_params)

    # Contour where robot base has to be outside
    extended_object_contour: GeometryContour = GeometryContour()
    extended_object_contour.import_contour_with_offset(object_to_move, 0.3)

    # Contour where the gripper tip must be inside
    grip_area: GeometryContour = GeometryContour()
    grip_area.import_contour_with_offset(object_to_move, -0.1)

    centroid_object_to_move_world_cs: np.array = object_to_move.calc_centroid_world_cs()
    plot.plot(centroid_object_to_move_world_cs[0], centroid_object_to_move_world_cs[1], "go")

    MAX_POPULATION: int = 30  # Dont specify odd amounts, allways even!

    # The values for MAX and MIN_STEP_SIZE are just randomly picked and tweeked so it worked good
    MAX_STEP_SIZE: float = round((grip_area.calc_longest_edge_length() / 5), 3)
    MIN_STEP_SIZE: float = MAX_STEP_SIZE / 50
    print("max step size: " + str(MAX_STEP_SIZE) + " | min step size: " + str(MIN_STEP_SIZE))

    grip_contour_population: list = init_grip_contour_population(MAX_POPULATION,
                                                                 centroid_object_to_move_world_cs,
                                                                 grip_area.calc_distance_closest_edge_to_point(
                                                                     centroid_object_to_move_world_cs),
                                                                 NUMBER_OF_ROBOTS)

    # 100 is just a good middle value where the corners have time to get further apart and its still relativly quick
    for cylce_counter in range(0, 100):
        # Create mating pool
        fitness_sum: float = calculate_fitness_sum_of_grip_population(grip_contour_population,
                                                                      centroid_object_to_move_world_cs)

        mating_pool: list = list()
        for grip_contour in grip_contour_population:
            contour_fitness: float = calculate_fitness_of_grip_contour(grip_contour, centroid_object_to_move_world_cs)
            mating_pool_chance: float = contour_fitness / fitness_sum
            mating_pool_instances: int = int(round(mating_pool_chance * MAX_POPULATION, 0))
            for counter in range(0, mating_pool_instances):
                mating_pool.append(grip_contour)

        # mating pool was not filed with enough parents. pick random parent to fill the pool to the desired amount
        while len(mating_pool) < len(grip_contour_population):
            mating_pool.append(grip_contour_population[random.randrange(0, len(grip_contour_population))])

        # Crossover of parents
        mur205_next_gen_population: List[GeometryContour] = list()
        while len(mating_pool) > 1:
            first_parent_index: int = random.randrange(0, len(mating_pool))
            first_parent: GeometryContour = mating_pool.pop(first_parent_index)
            second_parent_index: int = random.randrange(0, len(mating_pool))
            second_parent: GeometryContour = mating_pool.pop(second_parent_index)

            # Point for splitting the corners and giving them to the children.
            # At least split between 1 and 2 or n-1 and n so at least one point will be separatd
            gene_crossover_point: int = random.randrange(1, (len(first_parent.corner_point_list_geometry_cs) - 1))

            first_child: GeometryContour = \
                GeometryContour(lead_vector_world_cs=first_parent.lead_vector_world_cs,
                                world_to_geometry_cs_rotation=first_parent.world_to_geometry_rotation)
            second_child: GeometryContour = \
                GeometryContour(lead_vector_world_cs=first_parent.lead_vector_world_cs,
                                world_to_geometry_cs_rotation=first_parent.world_to_geometry_rotation)

            for counter in range(0, gene_crossover_point):
                first_child.add_contour_corner_geometry_cs(first_parent.corner_point_list_geometry_cs[counter])
                second_child.add_contour_corner_geometry_cs(second_parent.corner_point_list_geometry_cs[counter])

            for counter in range(gene_crossover_point, len(first_parent.corner_point_list_geometry_cs)):
                first_child.add_contour_corner_geometry_cs(second_parent.corner_point_list_geometry_cs[counter])
                second_child.add_contour_corner_geometry_cs(first_parent.corner_point_list_geometry_cs[counter])

            mur205_next_gen_population.append(first_child)
            mur205_next_gen_population.append(second_child)

        # Mutation
        MUTATION_CHANCE: float = 0.6

        for mur205_child_pose in mur205_next_gen_population:
            child_mutation_chance: float = random.random()

            if child_mutation_chance < MUTATION_CHANCE:  # Mutation
                mutate_geometry_contour(mur205_child_pose, MIN_STEP_SIZE, MAX_STEP_SIZE, grip_area)

        # Survivor selection
        total_population: dict = dict()
        for member in grip_contour_population:
            total_population[member] = calculate_fitness_of_grip_contour(member, centroid_object_to_move_world_cs)

        for member in mur205_next_gen_population:
            total_population[member] = calculate_fitness_of_grip_contour(member, centroid_object_to_move_world_cs)

        sorted_total_population: list = sorted(total_population.items(), key=lambda member: member[1], reverse=True)

        next_population: list = list()
        for counter in range(0, MAX_POPULATION):
            next_population.append(sorted_total_population[counter][0])

        grip_contour_population = next_population

    total_population: dict = dict()
    for contour in grip_contour_population:
        total_population[contour] = calculate_fitness_of_grip_contour(contour, centroid_object_to_move_world_cs)

    sorted_total_population: list = sorted(total_population.items(), key=lambda member: member[1], reverse=True)

    best_grip_contour: GeometryContour = sorted_total_population[0][0]
    best_grip_contour.plot_edges(color="green")

    ur5_base_link_boundary_list: List[GeometryContour] = list()
    for corner_point_world_cs in best_grip_contour.corner_point_list_world_cs:
        ur5_grip_boundary = create_circle_contour(corner_point_world_cs, 0.75)
        ur5_grip_boundary.plot_edges(color="black")
        ur5_base_link_boundary_list.append(ur5_grip_boundary)

    # Initialization
    mur205_optimization_population: List[MuR205PoseOptimizationInfo] = list()

    for population_member_counter in range(0, MAX_POPULATION):
        # Create one object to contain all information for the optimization
        # Member first contains default/empty elements
        mur205_pose_opti_info: MuR205PoseOptimizationInfo = MuR205PoseOptimizationInfo(GeometryContour(),
                                                                                       list(),
                                                                                       0.0)
        for ur5_base_link_boundary in ur5_base_link_boundary_list:
            # Initialize the first member of the optimization info
            # Pick a random point in the ur5 base link boundary that is valid
            ur5_base_link_pose: np.array = np.array([0, 0])
            point_valid: bool = False
            while not point_valid:
                x_pose = random.uniform(ur5_base_link_boundary.x_min_world_cs,
                                        ur5_base_link_boundary.x_max_world_cs)
                y_pose = random.uniform(ur5_base_link_boundary.y_min_world_cs,
                                        ur5_base_link_boundary.y_max_world_cs)
                ur5_base_link_pose = np.array([x_pose, y_pose])
                if ur5_base_link_boundary.is_world_cs_point_in_contour(ur5_base_link_pose) and not \
                        extended_object_contour.is_world_cs_point_in_contour(ur5_base_link_pose):
                    mur205_pose_opti_info.ur5_base_link_pose_contour.add_contour_corner_world_cs(ur5_base_link_pose)
                    point_valid = True

            # Initialize the second member of the optimization info
            # Pick a random orientation of the mir200 base that is valid
            mur205_contour: MuR205 = MuR205()
            mur205_contour.move_mur205_by_ur5_base_link(ur5_base_link_pose)

            orientation_valid: bool = False
            while not orientation_valid:
                rand_orientation: float = random.uniform(0, (2 * pi))
                mur205_contour.rotate_absolute_around_ur5_base_cs(rand_orientation)

                if not mur205_contour.is_contour_colliding(object_to_move):
                    mur205_pose_opti_info.mur205_contour_list.append(mur205_contour)
                    orientation_valid = True

        mur205_optimization_population.append(mur205_pose_opti_info)

    for counter in range(0, 40):
        # Create mating pool
        ur5_base_fitness_sum: float = calc_fitness_sum_mur205_population(
            mur205_optimization_population,
            ur5_base_link_boundary_list,
            object_to_move.calc_centroid_world_cs())

        mating_pool: List[MuR205PoseOptimizationInfo] = list()
        for mur205_pose_opti_info in mur205_optimization_population:
            contour_fitness: float = calc_fitness_mur205_positions(mur205_pose_opti_info,
                                                                   ur5_base_link_boundary_list,
                                                                   object_to_move.calc_centroid_world_cs())
            mating_pool_chance: float = contour_fitness / ur5_base_fitness_sum
            mating_pool_instances: int = int(round(mating_pool_chance * MAX_POPULATION, 0))
            for pool_instance_counter in range(0, mating_pool_instances):
                mating_pool.append(mur205_pose_opti_info)

        while len(mating_pool) < len(mur205_optimization_population):
            mating_pool.append(mur205_optimization_population[
                                   random.randrange(0, len(mur205_optimization_population))])

        mur205_next_gen_population: List[MuR205PoseOptimizationInfo] = list()
        while len(mating_pool) > 1:
            first_parent_index: int = random.randrange(0, len(mating_pool))
            first_parent: MuR205PoseOptimizationInfo = mating_pool.pop(first_parent_index)
            second_parent_index: int = random.randrange(0, len(mating_pool))
            second_parent: MuR205PoseOptimizationInfo = mating_pool.pop(second_parent_index)

            # Point for splitting the corners and giving them to the children.
            # At least split between 1 and 2 or n-1 and n so at least one point will be separatd
            gene_crossover_point: int = random.randrange(1, (len(first_parent.mur205_contour_list) - 1))

            first_child: MuR205PoseOptimizationInfo = MuR205PoseOptimizationInfo(
                GeometryContour(
                    lead_vector_world_cs=first_parent.ur5_base_link_pose_contour.lead_vector_world_cs,
                    world_to_geometry_cs_rotation=first_parent.ur5_base_link_pose_contour.world_to_geometry_rotation),
                list(),
                0.0)
            second_child: MuR205PoseOptimizationInfo = MuR205PoseOptimizationInfo(
                GeometryContour(
                    lead_vector_world_cs=first_parent.ur5_base_link_pose_contour.lead_vector_world_cs,
                    world_to_geometry_cs_rotation=first_parent.ur5_base_link_pose_contour.world_to_geometry_rotation),
                list(),
                0.0)

            corner_point_list_length: int = len(first_parent.ur5_base_link_pose_contour.corner_point_list_geometry_cs)

            first_child.ur5_base_link_pose_contour.add_contour_corner_list_geometry_cs(
                first_parent.ur5_base_link_pose_contour.corner_point_list_geometry_cs[0:gene_crossover_point])
            first_child.mur205_contour_list.extend(first_parent.mur205_contour_list[0:gene_crossover_point])

            second_child.ur5_base_link_pose_contour.add_contour_corner_list_geometry_cs(
                second_parent.ur5_base_link_pose_contour.corner_point_list_geometry_cs[0:gene_crossover_point])
            second_child.mur205_contour_list.extend(second_parent.mur205_contour_list[0:gene_crossover_point])

            first_child.ur5_base_link_pose_contour.add_contour_corner_list_geometry_cs(
                second_parent.ur5_base_link_pose_contour.corner_point_list_geometry_cs[gene_crossover_point:
                                                                                       corner_point_list_length])
            first_child.mur205_contour_list.extend(second_parent.mur205_contour_list[gene_crossover_point:
                                                                                     corner_point_list_length])

            second_child.ur5_base_link_pose_contour.add_contour_corner_list_geometry_cs(
                first_parent.ur5_base_link_pose_contour.corner_point_list_geometry_cs[gene_crossover_point:
                                                                                      corner_point_list_length])
            second_child.mur205_contour_list.extend(first_parent.mur205_contour_list[gene_crossover_point:
                                                                                     corner_point_list_length])

            mur205_next_gen_population.append(first_child)
            mur205_next_gen_population.append(second_child)

        # Mutation
        MUTATION_CHANCE: float = 0.6
        MAX_STEP_SIZE: float = 0.005
        MIN_STEP_SIZE: float = 0.001
        MAX_ROTATION: float = pi/4
        MIN_ROTATION: float = pi/32

        deadzone_list: List[GeometryContour] = list()
        deadzone_list.append(extended_object_contour)

        for mur205_child_pose in mur205_next_gen_population:
            child_mutation_chance: float = random.random()

            if child_mutation_chance < MUTATION_CHANCE:  # Mutation
                mutate_geometry_contour_with_dead_zones(mur205_opti_info=mur205_child_pose,
                                                        min_mutation_distance=MIN_STEP_SIZE,
                                                        max_mutation_distance=MAX_STEP_SIZE,
                                                        min_mutation_rotation=MIN_ROTATION,
                                                        max_mutation_rotation=MAX_ROTATION,
                                                        outside_border_list=ur5_base_link_boundary_list,
                                                        object_to_move=object_to_move,
                                                        deadzone_list=deadzone_list)

        # Survivor selection
        for member in mur205_optimization_population:
            member.fitness = calc_fitness_mur205_positions(member,
                                                           ur5_base_link_boundary_list,
                                                           object_to_move.calc_centroid_world_cs())

        for member in mur205_next_gen_population:
            member.fitness = calc_fitness_mur205_positions(member,
                                                           ur5_base_link_boundary_list,
                                                           object_to_move.calc_centroid_world_cs())

        mur205_optimization_population.extend(mur205_next_gen_population)

        sorted_total_population: list = sorted(mur205_optimization_population,
                                               key=lambda member: member.fitness,
                                               reverse=True)

        mur205_optimization_population = sorted_total_population[0:MAX_POPULATION]
        # for population_counter in range(0, MAX_POPULATION):
        #     next_population.append(sorted_total_population[population_counter][0])

    for mur205_opti_info in mur205_optimization_population:
        # contour.plot_edges(color="orange")
        mur205_opti_info.fitness = calc_fitness_mur205_positions(mur205_opti_info,
                                                                 ur5_base_link_boundary_list,
                                                                 object_to_move.calc_centroid_world_cs())

    sorted_total_population: list = sorted(mur205_optimization_population,
                                           key=lambda member: member.fitness,
                                           reverse=True)

    best_grip_contour: MuR205PoseOptimizationInfo = sorted_total_population[0]
    best_grip_contour.ur5_base_link_pose_contour.plot_edges(color="green")

    for mur205 in best_grip_contour.mur205_contour_list:
        mur205.plot_edges(color="pink")

    # mir_contour: MuR205 = MuR205()
    # mir_contour.move_mur205_by_ur5_base_link(best_grip_contour.corner_point_list_world_cs[0], -(pi / 2))
    # mir_contour.plot_edges(color="black")
    #
    # mir_contour.rotate_relative_around_ur5_base_cs((3 * pi) / 4)
    # mir_contour.plot_edges(color="pink")
    # print(mir_contour.is_contour_colliding(object_to_move))

    plot_contour_info(object_to_move, extended_object_contour, grip_area)

    axis: plot.Axes = plot.gca()  # Get current axis object and set x and y to be equal so a square is a square
    axis.axis("equal")
    plot.show()  # Let this be the last command so the plots wont be closed instantly

import os

from matplotlib import pyplot as plot
# from matplotlib import colors as mcolors
import random
import numpy as np
from math import sin, cos, pi
from typing import List

from geometry_info.geometry_contour import GeometryContour
from geometry_info.movable_object import MovableObject, Box, Cylinder, IsoscelesTriangle, RightAngledTriangle
from geometry_info.MuR205 import MuR205

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


def calculate_fitness_of_ur5_base_contour(ur5_base_contour: GeometryContour, ) -> float:
    fitness: float
    fitness = 1 / pow(ur5_base_contour.calc_contour_length(), 2)

    return fitness


def calculate_fitness_sum_of_ur5_base_population(ur5_base_contour_population: list) -> float:
    fitness_sum: float = 0.0
    for ur5_base_contour in ur5_base_contour_population:
        fitness: float = calculate_fitness_of_ur5_base_contour(ur5_base_contour)
        fitness_sum += fitness

    return fitness_sum


def mutate_geometry_contour(contour_to_mutate: GeometryContour,
                            min_mutation: float,
                            max_mutation: float,
                            border_contour: GeometryContour) -> GeometryContour:
    """[summary]

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

    return contour_to_mutate


def mutate_geometry_contour_with_dead_zones(contour_to_mutate: GeometryContour,
                                            min_mutation_distance: float,
                                            max_mutation_distance: float,
                                            outside_border_list: List[GeometryContour],
                                            deadzone_list: List[GeometryContour] = None) -> GeometryContour:
    """Method for the mutation of corner-point of a contour.
    But this time each corner has its individual outside_border and there are multiple deadzones where
    no corner is allowed to be.
    The decision if the mutation happens was allready decided before calling this function

    :param contour_to_mutate: [description]
    :type contour_to_mutate: GeometryContour
    :param min_mutation_distance: [description]
    :type min_mutation_distance: float
    :param max_mutation_distance: [description]
    :type max_mutation_distance: float
    :param outside_border_list: [description]
    :type outside_border_list: GeometryContour
    :param deadzone_list: List of all deadzones that should be respected, defaults to list()
    :type deadzone_list: list, optional
    :return: [description]
    :rtype: GeometryContour
    """
    if deadzone_list is None:
        deadzone_list = list()

    index_of_mutated_corner: int = random.randrange(0, len(contour_to_mutate.corner_point_list_geometry_cs))

    corner_was_replaced: bool = False

    while not corner_was_replaced:
        corner_to_mutate_geometry_cs: np.array = \
            contour_to_mutate.corner_point_list_geometry_cs[index_of_mutated_corner].copy()

        is_additive_mutation_x: bool = bool(random.getrandbits(1))
        is_additive_mutation_y: bool = bool(random.getrandbits(1))
        mutation_value_x: float = random.uniform(min_mutation_distance, max_mutation_distance)
        mutation_value_y: float = random.uniform(min_mutation_distance, max_mutation_distance)

        if is_additive_mutation_x:
            corner_to_mutate_geometry_cs[0] += mutation_value_x
        else:
            corner_to_mutate_geometry_cs[0] -= mutation_value_x

        if is_additive_mutation_y:
            corner_to_mutate_geometry_cs[1] += mutation_value_y
        else:
            corner_to_mutate_geometry_cs[1] -= mutation_value_y

        # Check if the mutated corner point is in one of the listed deadzones
        corner_not_in_deadzone: bool = True
        for deadzone in deadzone_list:
            if deadzone.is_geometry_cs_point_in_contour(corner_to_mutate_geometry_cs):
                corner_not_in_deadzone = False
                break

        if (outside_border_list[index_of_mutated_corner].is_world_cs_point_in_contour(
                contour_to_mutate.transform_vector_geometry_to_world_cs(corner_to_mutate_geometry_cs)) and
                corner_not_in_deadzone):
            contour_to_mutate.replace_contour_corner_geometry_cs(index_of_mutated_corner, corner_to_mutate_geometry_cs)
            corner_was_replaced = True

    return contour_to_mutate


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
    for cylce_counter in range(0, 50):
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
        next_gen: list = list()
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

            next_gen.append(first_child)
            next_gen.append(second_child)

        # Mutation
        MUTATION_CHANCE: float = 0.6

        for child in next_gen:
            child_mutation_chance: float = random.random()

            if child_mutation_chance < MUTATION_CHANCE:  # Mutation
                child = mutate_geometry_contour(child, MIN_STEP_SIZE, MAX_STEP_SIZE, grip_area)

        # Survivor selection
        total_population: dict = dict()
        for member in grip_contour_population:
            total_population[member] = calculate_fitness_of_grip_contour(member, centroid_object_to_move_world_cs)

        for member in next_gen:
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

    posible_ur5_base_link_pose_list: List[GeometryContour] = list()
    for corner_point_world_cs in best_grip_contour.corner_point_list_world_cs:
        temp = create_circle_contour(corner_point_world_cs, 0.75)
        temp.plot_edges(color="black")
        posible_ur5_base_link_pose_list.append(temp)

    # Initialization
    ur5_base_link_population: list = list()

    for population_member_counter in range(0, MAX_POPULATION):
        ur5_base_link_pose_contour: GeometryContour = GeometryContour()
        for posible_ur5_base_link_pose in posible_ur5_base_link_pose_list:
            point_valid: bool = False
            while not point_valid:
                x_pose = random.uniform(posible_ur5_base_link_pose.x_min_world_cs,
                                        posible_ur5_base_link_pose.x_max_world_cs)
                y_pose = random.uniform(posible_ur5_base_link_pose.y_min_world_cs,
                                        posible_ur5_base_link_pose.y_max_world_cs)
                pose_to_check_world_cs: np.array = np.array([x_pose, y_pose])
                if posible_ur5_base_link_pose.is_world_cs_point_in_contour(pose_to_check_world_cs) and not \
                        extended_object_contour.is_world_cs_point_in_contour(pose_to_check_world_cs):
                    ur5_base_link_pose_contour.add_contour_corner_world_cs(pose_to_check_world_cs)
                    point_valid = True
        ur5_base_link_population.append(ur5_base_link_pose_contour)

    for counter in range(0, 40):
        # Create mating pool
        ur5_base_fitness_sum: float = calculate_fitness_sum_of_ur5_base_population(ur5_base_link_population)

        mating_pool: list = list()
        for ur5_base_link_contour in ur5_base_link_population:
            contour_fitness: float = calculate_fitness_of_ur5_base_contour(ur5_base_link_contour)
            mating_pool_chance: float = contour_fitness / ur5_base_fitness_sum
            mating_pool_instances: int = int(round(mating_pool_chance * MAX_POPULATION, 0))
            for pool_instance_counter in range(0, mating_pool_instances):
                mating_pool.append(ur5_base_link_contour)

        while len(mating_pool) < len(ur5_base_link_population):
            mating_pool.append(ur5_base_link_population[random.randrange(0, len(ur5_base_link_population))])

        next_gen: list = list()
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

            for gene_counter in range(0, gene_crossover_point):
                first_child.add_contour_corner_geometry_cs(first_parent.corner_point_list_geometry_cs[gene_counter])
                second_child.add_contour_corner_geometry_cs(second_parent.corner_point_list_geometry_cs[gene_counter])

            for gene_counter in range(gene_crossover_point, len(first_parent.corner_point_list_geometry_cs)):
                first_child.add_contour_corner_geometry_cs(second_parent.corner_point_list_geometry_cs[gene_counter])
                second_child.add_contour_corner_geometry_cs(first_parent.corner_point_list_geometry_cs[gene_counter])

            next_gen.append(first_child)
            next_gen.append(second_child)

        # Mutation
        MUTATION_CHANCE: float = 0.6
        MAX_STEP_SIZE: float = 0.005
        MIN_STEP_SIZE: float = 0.001

        deadzone_list: list = list()
        deadzone_list.append(extended_object_contour)

        for index, child in enumerate(next_gen):
            child_mutation_chance: float = random.random()

            if child_mutation_chance < MUTATION_CHANCE:  # Mutation
                mutate_geometry_contour_with_dead_zones(contour_to_mutate=child,
                                                        min_mutation_distance=MIN_STEP_SIZE,
                                                        max_mutation_distance=MAX_STEP_SIZE,
                                                        outside_border_list=posible_ur5_base_link_pose_list,
                                                        deadzone_list=deadzone_list)

        # Survivor selection
        total_population: dict = dict()
        for member in ur5_base_link_population:
            total_population[member] = calculate_fitness_of_ur5_base_contour(member)

        for member in next_gen:
            total_population[member] = calculate_fitness_of_ur5_base_contour(member)

        sorted_total_population: list = sorted(total_population.items(), key=lambda member: member[1], reverse=True)

        next_population: list = list()
        for population_counter in range(0, MAX_POPULATION):
            next_population.append(sorted_total_population[population_counter][0])

        ur5_base_link_population = next_population

    total_population: dict = dict()
    for contour in ur5_base_link_population:
        # contour.plot_edges(color="orange")
        total_population[contour] = calculate_fitness_of_ur5_base_contour(contour)

    sorted_total_population: list = sorted(total_population.items(), key=lambda member: member[1], reverse=True)

    best_grip_contour: GeometryContour = sorted_total_population[0][0]
    best_grip_contour.plot_edges(color="green")

    mir_contour: MuR205 = MuR205()
    mir_contour.move_mur205_by_ur5_base_link(best_grip_contour.corner_point_list_world_cs[0], -(pi/2))
    mir_contour.plot_edges(color="black")

    mir_contour.rotate_relative_around_ur5_base_cs(pi/4)
    mir_contour.plot_edges(color="pink")

    plot_contour_info(object_to_move, extended_object_contour, grip_area)

    axis: plot.Axes = plot.gca()  # Get current axis object and set x and y to be equal so a square is a square
    axis.axis("equal")
    plot.show()  # Let this be the last command so the plots wont be closed instantly

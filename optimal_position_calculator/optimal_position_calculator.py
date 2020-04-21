import sys
import os

from matplotlib import pyplot as plot
from matplotlib import colors as mcolors
import random
import numpy as np
from math import sin, cos, pi

from geometry_info.geometry_contour import GeometryContour
from geometry_info.movable_object import MovableObject, Box, Cylinder, IsoscelesTriangle, RightAngledTriangle
from geometry_info.edge_info import EdgeInfo

from urdf_reader import URDFReader
from urdf_reader import GeometryType


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


def create_grid(contour: GeometryContour) -> list:
    x_max_area: float = contour.get_x_max()
    x_min_area: float = contour.get_x_min()
    y_max_area: float = contour.get_y_max()
    y_min_area: float = contour.get_y_min()

    grid_point_list: list = list()
    x_list: np.array = np.arange(x_min_area, x_max_area, 0.1)  # Change to linspace and calculate how many points I want? Should be better as Internet tells
    y_list: np.array = np.arange(y_min_area, y_max_area, 0.1)
    for x_counter in x_list:
        for y_counter in y_list:
            grid_point: np.array = np.array([x_counter, y_counter])
            if contour.is_point_in_contour(grid_point):
                grid_point_list.append(grid_point)

    return grid_point_list


def init_grip_point_contour(centroid: np.array, max_distance_from_centroid: float, number_of_robots: int) -> GeometryContour:
    grip_point_contour: GeometryContour = GeometryContour()

    offset_rotation: float = random.uniform((pi/180), pi)  # Random offset from 0 degree (right side) in mathematical positive direction. Range from 1°-180°
    angle_diff_between_robot: float = (2*pi) / (NUMBER_OF_ROBOTS)

    for counter in range(0, number_of_robots):
        distance_from_centroid: float = random.uniform(0.01, max_distance_from_centroid)
        grip_point: np.array = np.array([distance_from_centroid * cos((angle_diff_between_robot * counter) + offset_rotation),
                                         distance_from_centroid * sin((angle_diff_between_robot * counter) + offset_rotation)])
        grip_point_contour.add_contour_corner(grip_point)

    return grip_point_contour


def init_grip_contour_population(max_population: int, centroid: np.array, max_distance_from_centroid: float, number_of_robots: int) -> list:
    grip_contour_population: list = list()
    for counter in range(0, max_population):
        new_grip_contour: GeometryContour = init_grip_point_contour(centroid, max_distance_from_centroid, number_of_robots)
        grip_contour_population.append(new_grip_contour)

    return grip_contour_population


def calculate_fitness_of_grip_contour(grip_contour: GeometryContour, centroid_object_to_move: np.array) -> float:
    fitness: float = 0.0
    if not grip_contour.is_point_in_contour(centroid_object_to_move):
        return fitness
    if grip_contour.do_edges_intersect():
        return fitness

    # for edge in grip_contour.edge_list:
    #     distance_to_centroid: float = grip_contour.calculate_distance_point_to_line(centroid_object_to_move, edge.edge_vector, edge.start_point)
    #     fitness += pow(distance_to_centroid, 2)

    fitness = pow(grip_contour.get_shortest_edge_length(), 2)

    # Add additional fitness parameters here

    return fitness


def calculate_fitness_sum_of_grip_population(grip_contour_population: list, centroid_object_to_move: np.array) -> float:
    fitness_sum: float = 0.0
    for grip_contour in grip_contour_population:
        fitness: float = calculate_fitness_of_grip_contour(grip_contour, centroid_object_to_move)
        fitness_sum += fitness

    return fitness_sum


def mutate_geometry_contour(contour_to_mutate: GeometryContour, min_mutation: float, max_mutation: float) -> GeometryContour:
    index_of_mutated_corner: int = random.randrange(0, len(contour_to_mutate.corner_point_list))
    is_additive_mutation_x: bool = bool(random.getrandbits(1))
    is_additive_mutation_y: bool = bool(random.getrandbits(1))
    mutation_value_x: float = random.uniform(min_mutation, max_mutation)
    mutation_value_y: float = random.uniform(min_mutation, max_mutation)

    if is_additive_mutation_x:
        contour_to_mutate.corner_point_list[index_of_mutated_corner][0] += mutation_value_x
    else:
        contour_to_mutate.corner_point_list[index_of_mutated_corner][0] -= mutation_value_x

    if is_additive_mutation_y:
        contour_to_mutate.corner_point_list[index_of_mutated_corner][1] += mutation_value_y
    else:
        contour_to_mutate.corner_point_list[index_of_mutated_corner][1] -= mutation_value_y

    plot.plot(contour_to_mutate.corner_point_list[index_of_mutated_corner][0], contour_to_mutate.corner_point_list[index_of_mutated_corner][1], color=mcolors.CSS4_COLORS["pink"], marker="o")

    return contour_to_mutate


if __name__ == '__main__':
    script_dir = os.path.dirname(__file__)
    urdf_file_path = os.path.join(script_dir, '../urdf_form_creator/urdf/basic_geometry.urdf')
    urdf_reader: URDFReader = URDFReader(urdf_file_path)

    object_to_move: MovableObject

    size_info_list: list = urdf_reader.size_info.split()
    object_contour_params: dict = dict()
    if(urdf_reader.geometry_type == GeometryType.Box):
        object_contour_params["x_length"] = float(size_info_list[0])
        object_contour_params["y_length"] = float(size_info_list[1])
        object_to_move = Box()

    elif(urdf_reader.geometry_type == GeometryType.Cylinder):
        object_contour_params["radius"] = float(size_info_list[0])
        object_to_move = Cylinder()

    elif(urdf_reader.geometry_type == GeometryType.IsoscelesTriangle):
        object_contour_params["scale"] = float(size_info_list[0])
        object_to_move = IsoscelesTriangle()

    elif(urdf_reader.geometry_type == GeometryType.RightAngledTriangle):
        object_contour_params["x_scale"] = float(size_info_list[0])
        object_contour_params["y_scale"] = float(size_info_list[1])
        object_to_move = RightAngledTriangle()

    object_contour_params["height"] = float(size_info_list[2])
    object_contour_params["mass"] = float(urdf_reader.mass)
    object_to_move.import_urdf_info(**object_contour_params)

    # Contour where robot base has to be outside
    extended_object_contour: GeometryContour = GeometryContour()
    extended_object_contour.import_contour_with_offset(object_to_move, 0.4)

    # Contour where the gripper tip must be inside
    grip_area: GeometryContour = GeometryContour()
    grip_area.import_contour_with_offset(object_to_move, -0.1)

    # grid_point_list = create_grid(grip_area) Not needed anymore because switching from grid to steps

    centroid_object_to_move: np.array = object_to_move.calculate_centroid()

    MAX_POPULATION: int = 50  # Dont specify odd amounts, allways even!

    MAX_STEP_SIZE: float = round((grip_area.get_longest_edge_length()/100), 3)
    MIN_STEP_SIZE: float = MAX_STEP_SIZE / 1

    grip_contour_population: list = init_grip_contour_population(MAX_POPULATION, centroid_object_to_move, grip_area.get_closest_distance_edge_to_point(centroid_object_to_move), NUMBER_OF_ROBOTS)

    for cylce_counter in range(0, 40):
        fitness_sum: float = calculate_fitness_sum_of_grip_population(grip_contour_population, centroid_object_to_move)

        mating_pool: list = list()
        for grip_contour in grip_contour_population:
            # grip_contour.plot_corners()
            # grip_contour.plot_edges()
            # grip_contour.print_info()

            mating_pool_instances: int = int(round((calculate_fitness_of_grip_contour(grip_contour, centroid_object_to_move)/fitness_sum) * 10, 0))
            for counter in range(0, mating_pool_instances):
                mating_pool.append(grip_contour)

        # mating pool was not filed with enough parents. pick random parent to fill the pool to the desired amount
        while len(mating_pool) < len(grip_contour_population):
            mating_pool.append(grip_contour_population[random.randrange(0, len(grip_contour_population))])

        next_gen: list = list()
        # Crossover of parents
        while len(mating_pool) > 1:
            first_parent_index: int = random.randrange(0, len(mating_pool))
            first_parent: GeometryContour = mating_pool.pop(first_parent_index)
            second_parent_index: int = random.randrange(0, len(mating_pool))
            second_parent: GeometryContour = mating_pool.pop(second_parent_index)

            # Point for splitting the corners and giving them to the children. At least split between 1 and 2 or n-1 and n so at least one point will be separatd
            gene_crossover_point: int = random.randrange(1, (len(first_parent.corner_point_list)-1))

            first_child: GeometryContour = GeometryContour()
            second_child: GeometryContour = GeometryContour()

            for counter in range(0, gene_crossover_point):
                first_child.add_contour_corner(first_parent.corner_point_list[counter])
                second_child.add_contour_corner(second_parent.corner_point_list[counter])

            for counter in range(gene_crossover_point, len(first_parent._corner_point_list)):
                first_child.add_contour_corner(second_parent.corner_point_list[counter])
                second_child.add_contour_corner(first_parent.corner_point_list[counter])

            first_child_mutation_chance: int = random.randrange(0, 100)
            second_child_mutation_chance: int = random.randrange(0, 100)

            if first_child_mutation_chance >= 30:
                first_child = mutate_geometry_contour(first_child, MIN_STEP_SIZE, MAX_STEP_SIZE)
                first_child.create_contour_edges()

            if second_child_mutation_chance >= 30:
                second_child = mutate_geometry_contour(second_child, MIN_STEP_SIZE, MAX_STEP_SIZE)
                first_child.create_contour_edges()

            next_gen.append(first_child)
            next_gen.append(second_child)

        # Survivor selection
        total_population: dict = dict()
        for member in grip_contour_population:
            total_population[member] = calculate_fitness_of_grip_contour(member, centroid_object_to_move)

        for member in next_gen:
            total_population[member] = calculate_fitness_of_grip_contour(member, centroid_object_to_move)

        total_population = sorted(total_population.items(), key=lambda member: member[1], reverse=True)

        next_population: list = list()
        for counter in range(0, MAX_POPULATION):
            next_population.append(total_population[counter][0])

        grip_contour_population = next_population

    for contour in grip_contour_population:
        contour.create_contour_edges()
        contour.plot_edges()
        contour.print_info()

    # for contour in next_population:
    #     contour.plot_edges(color="green")

    # Method for optimizing the gripping position
    # Multiple problems! Nimmt am Ende immer die gleiche Seite, obwohl diese nichtmehr verbessert werden kann.
    # Lokale Minima werden angenommen und Optimierer kommt da nichtmehr raus

    # for counter in range(0, 10):
    #     closest_edge: EdgeInfo = grip_point_area.get_closest_edge_to_point(centroid_object_to_move)
    #     index_of_corner: int = grip_point_area.get_index_of_corner(closest_edge.start_point)  # Move start point
    #     plot.plot(closest_edge.start_point[0], closest_edge.start_point[1], "ko")
    #     copy_of_grip_point_area: GeometryContour = grip_point_area

    #     best_result: float = 100000000
    #     best_point: np.array
    #     for grid_point in grid_point_list:
    #         copy_of_grip_point_area.replace_contour_corner(index_of_corner, grid_point)
    #         if not copy_of_grip_point_area.is_point_in_contour(centroid_object_to_move):
    #             continue

    #         if copy_of_grip_point_area.do_edges_intersect():
    #             continue

    #         result: float = 0.0
    #         for edge_info in copy_of_grip_point_area.edge_list:
    #             distance: float = copy_of_grip_point_area.calculate_distance_point_to_line(centroid_object_to_move, edge_info.edge_vector, edge_info.start_point)
    #             result = result + (1/pow(distance, 2))

    #         if best_result >= result:
    #             best_result = result
    #             best_point = grid_point

    #     grip_point_area.replace_contour_corner(index_of_corner, best_point)
    # end method for optimizing grip position

    # grip_point_area.plot_corners()
    # grip_point_area.plot_edges()

    # ur5_base_link_pose_list_list: list = list()

    # for grip_point in grip_point_area.corner_point_list:
    #     arm_base_pose_possible: GeometryContour = GeometryContour()

    #     for counter in range(0, 100):
    #         pose: np.array = np.array([grip_point[0] + 0.75*cos(((2*pi)/100) * counter), grip_point[1] + 0.75 * sin(((2*pi)/100) * counter)])
    #         arm_base_pose_possible.add_contour_corner(pose)

    #     list_with_points = create_grid(arm_base_pose_possible)

    #     possible_ur5_base_link_pose_list: list = list()
    #     for point in list_with_points:
    #         if not extended_object_contour.is_point_in_contour(point):
    #             possible_ur5_base_link_pose_list.append(point)

    #     ur5_base_link_pose_list_list.append(possible_ur5_base_link_pose_list)

    # Random einen Punkt in einer Punktwolke auswählen
    # Anschließend wieder alle bis auf einen Punkt statisch machen und den einen optimieren.
    # Danach den nächsten Punkt verschieben usw.

    # optimized_grip_contour: GeometryContour = GeometryContour()
    # Get one random point from each list to start the optimizing process
    # for pose_list in ur5_base_link_pose_list_list:
    #     length_of_list: int = len(ur5_base_link_pose_list_list)
    #     random_index: int = random.randint(0, length_of_list)
    #     optimized_grip_contour.add_contour_corner(pose_list[random_index])

    # for counter in range(0, 3):
    #     corner_to_change: int = counter % 4  # 0 = first corner and so on
    #     print(corner_to_change)
    #     current_area: float = optimized_grip_contour.calculate_area()
    #     current_point: np.array = optimized_grip_contour.corner_point_list[corner_to_change]

    #     best_area: float = current_area
    #     best_point: np.array = current_point
    #     copy_contour: GeometryContour = optimized_grip_contour
    #     for point in ur5_base_link_pose_list_list[corner_to_change]:  # First corner comes from the first list and so on
    #         copy_contour.replace_contour_corner(corner_to_change, point)
    #         if best_area > copy_contour.calculate_area():
    #             best_area = copy_contour.calculate_area()
    #             best_point = point

    #     if best_point is not current_point:
    #         optimized_grip_contour.replace_contour_corner(corner_to_change, best_point)

    # optimized_grip_contour.plot_corners()
    # optimized_grip_contour.plot_edges()

    plot_contour_info(object_to_move, extended_object_contour, grip_area)

    axis: plot.Axes = plot.gca()  # Get current axis object and set x and y to be equal so a square is a square
    axis.axis("equal")
    plot.show()  # Let this be the last command so the plots wont be closed instantly

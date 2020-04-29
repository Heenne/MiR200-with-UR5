import sys

from lxml import etree


class LaunchReader:
    _launch_root_node: etree.ElementTree

    x_position: float = 0.0
    y_position: float = 0.0
    z_position: float = 0.0
    z_rotation: float = 0.0

    def __init__(self, file_path: str):
        """Init method for the URDF Reader class.
        Method will take the launch file in the specified file path and import the needed data.

        :param file_path: path to launch file that should be read
        :type file_path: str
        """
        self._launch_root_node = etree.ElementTree()
        self._launch_root_node = etree.parse(file_path)

        node_node: etree.ElementTree = self._launch_root_node.find("node")
        spawn_args: str = node_node.get("args")
        self._get_spawn_pose_from_string(spawn_args)

        self.print_launch_info()

    def print_launch_info(self):
        """Print the relevant info that was read from the launch file
        """
        print("Spawn pose:")
        print("x: " + str(self.x_position) + " | y: " + str(self.y_position) + " | z: " +
              str(self.z_position) + " | z_rotation: " + str(self.z_rotation))

    def _get_spawn_pose_from_string(self, spawn_args: str):
        """Get the spawn position and rotation from the spawn args in the launch file

        :param spawn_args: string from the "args" in the launch file in the robot/node node
        :type spawn_args: str
        """
        spawn_pose_list: list = spawn_args.split()
        print(spawn_pose_list)
        if "-x" in spawn_pose_list:
            index_of_x_position: int = spawn_pose_list.index("-x")
            self.x_position = float(spawn_pose_list[index_of_x_position + 1])

        if "-y" in spawn_pose_list:
            index_of_y_position: int = spawn_pose_list.index("-y")
            self.y_position = float(spawn_pose_list[index_of_y_position + 1])

        if "-z" in spawn_pose_list:
            index_of_z_position: int = spawn_pose_list.index("-z")
            self.z_position = float(spawn_pose_list[index_of_z_position + 1])

        if "-Y" in spawn_pose_list:
            index_of_z_rotation: int = spawn_pose_list.index("-Y")
            print(index_of_z_rotation)
            self.z_rotation = float(spawn_pose_list[index_of_z_rotation + 1])

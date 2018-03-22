from pylaunch import Node

import os.path as pt
import random
import sys
import unittest

import roslaunch
import rosgraph


class TestPylaunchNode(unittest.TestCase):

    def test_pylaunch_node(self):
        dut = self.build_dut()
        roslaunch_node = self.pylaunch_node_to_roslaunch_node(dut)

        self.assertEquals(roslaunch_node.package, "test_package_name")
        self.assertEquals(roslaunch_node.type, "test_node_type")
        self.assertEquals(roslaunch_node.name, "test_node_name")

    def test_respawn(self):
        dut = self.build_dut()
        roslaunch_node = self.pylaunch_node_to_roslaunch_node(dut)
        self.assertEquals(roslaunch_node.respawn, False)

        dut = self.build_dut(respawn=True)
        roslaunch_node = self.pylaunch_node_to_roslaunch_node(dut)
        self.assertEquals(roslaunch_node.respawn, True)

        delay = random.randint(10, 100)
        dut = self.build_dut(respawn=True, respawn_delay=delay)
        roslaunch_node = self.pylaunch_node_to_roslaunch_node(dut)
        self.assertEquals(roslaunch_node.respawn, True)
        self.assertEquals(roslaunch_node.respawn_delay, delay)

    def build_dut(self, **kwargs):
        return Node(
            package_name="test_package_name",
            node_type="test_node_type",
            node_name="test_node_name",
            **kwargs
        )

    def pylaunch_node_to_roslaunch_node(self, node):
        launch_config =  roslaunch.config.ROSLaunchConfig()
        loader = roslaunch.loader.Loader()

        # For some reason, pylaunch nodes expect this extra item to be stuffed
        # onto the loader, even though it's not a loader property. . .
        # Seems like it should be another argument, or the 'loader' parameter
        # should be renamed loader_with_extra_property_i_need or somthing
        loader.root_context = roslaunch.loader.LoaderContext(
            rosgraph.names.get_ros_namespace(),
            pt.splitext(pt.split(sys.argv[0])[-1])[0]
        )


        node.process(loader, launch_config)

        self.assertEquals(
            len(launch_config.nodes),
            1,
            "Expected 1 node. Found {}".format(len(launch_config.nodes))
        )

        return launch_config.nodes[0]

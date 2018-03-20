from pylaunch import Node

import os.path as pt
import sys
import unittest

import roslaunch
import rosgraph


class TestPylaunchNode(unittest.TestCase):

    def test_pylaunch_node(self):
        dut = Node(
            package_name="test_package_name",
            node_type="test_node_type",
            node_name="test_node_name"
        )

        roslaunch_node = self.pylaunch_node_to_roslaunch_node(dut)

        self.assertEquals(roslaunch_node.package, "test_package_name")
        self.assertEquals(roslaunch_node.type, "test_node_type")
        self.assertEquals(roslaunch_node.name, "test_node_name")


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

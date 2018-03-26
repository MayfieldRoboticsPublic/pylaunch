#!/usr/bin/env python

import unittest
import pylaunch as pl
import rospy
import time
import std_msgs.msg as std
import rosgraph as rg
import os.path as pt


def wait_for_subscriber_to_topic(topic_name, time_out):
    master = rg.Master('')
    topic_published = False
    start_time = time.time()
    while not topic_published:
        pubs, subs, _ = master.getSystemState()
        topics = subs
        for topic in topics:
            if topic[0] == topic_name:
                topic_published = True
                break
        time.sleep(1/30)
        if (time.time() - start_time) > time_out:
            return False

    return topic_published


class TestPylaunch(unittest.TestCase):

    def test_launch_config(self):
        """
            Haven't looked into a good way of testing launched nodes
            so we'll settle for launching the entire thing and
            catching all exceptions as a sanity check.
        """
        configs = [
            pl.Node("rospy_tutorials",
                    "talker",
                    "talker2",
                    params={'calibrate_time': False},
                    remaps=[('chatter', 'hello_topic')]),
            pl.Include('pylaunch',
                       'listener.launch',
                       params={'some_arg': '21'})
        ]

        p = pl.PyRosLaunch(configs)
        p.start()
        rospy.init_node('test')
        has_subscriber = False

        try:
            rospy.wait_for_message('/hello_topic', std.String, 10)
            has_subscriber = wait_for_subscriber_to_topic('/hello', 10)
        except rospy.exceptions.ROSException:
            self.fail("Failed to launch nodes.")
        finally:
            p.shutdown()

        if not has_subscriber:
            self.fail("Failed to launch nodes.")

    def test_launch_file_runner(self):
        p = pl.LaunchFileRunner("pylaunch", "talker.launch")
        p.start()
        rospy.init_node('test')
        try:
            rospy.wait_for_message('/chatter', std.String, 10)
        except rospy.exceptions.ROSException:
            self.fail("Failed to launch nodes.")
        finally:
            p.shutdown()

    def test_include_remap(self):
        p = pl.PyRosLaunch([
            pl.Include('pylaunch',
                       'talker.launch',
                       remaps=[('chatter', 'blah_blah')])
        ])
        p.start()
        rospy.init_node('test')
        try:
            rospy.wait_for_message('/blah_blah', std.String, 10)
        except rospy.exceptions.ROSException:
            self.fail("Failed to launch nodes.")
        finally:
            p.shutdown()

    def test_param(self):
        p = pl.PyRosLaunch([
            pl.Node('rospy_tutorials', 'talker', 'talker2'),
            pl.Param('int_param', 5),
            pl.Param('command_param', command='echo hello')
        ])
        p.start()

        rospy.init_node('test')
        try:
            rospy.wait_for_message('/chatter', std.String, 10)
            int_param = rospy.get_param('int_param')
            command_param = rospy.get_param('command_param')

            self.assertTrue(type(int_param) == int)
            self.assertTrue(int_param == 5)
            self.assertTrue(type(command_param) == str)
            self.assertTrue(command_param == "hello\n")
        except rospy.exceptions.ROSException:
            self.fail("Failed to launch nodes.")
        finally:
            p.shutdown()

    def test_node_param(self):
        p = pl.PyRosLaunch([
            pl.Node('rospy_tutorials',
                    'talker',
                    'talker',
                    params={'use_may_nav': False})
        ])
        p.start()
        rospy.init_node('test')
        try:
            rospy.wait_for_message('/chatter', std.String, 10)
            use_may_nav = rospy.get_param('/talker/use_may_nav')
            self.assertEquals(type(use_may_nav), bool)
            self.assertFalse(use_may_nav)
        except rospy.exceptions.ROSException:
            self.fail("Failed to launch nodes.")
        finally:
            p.shutdown()

    def test_ros_param(self):
        p = pl.PyRosLaunch([
            pl.Node('rospy_tutorials',
                    'talker',
                    'talker',
                    rosparams=[
                        pl.RosParam(
                            pt.join(pl.pkg_path('pylaunch'),
                                    'test',
                                    'param.yml')
                        )
                    ])
        ])
        p.start()
        rospy.init_node('test')
        try:
            rospy.wait_for_message('/chatter', std.String, 10)
            param = rospy.get_param('/talker/a_param')
            self.assertTrue(1332 == param)
            self.assertTrue(type(param) == int)
        except rospy.exceptions.ROSException:
            self.fail("Failed to launch nodes.")
        finally:
            p.shutdown()


if __name__ == '__main__':
    unittest.main()

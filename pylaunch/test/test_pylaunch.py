#!/usr/bin/env python

import unittest
import pylaunch as pl

class TestPylaunch(unittest.TestCase):

    def test_launch_config(self):
        """
            Haven't looked into a good way of testing launched nodes
            so we'll settle for launching the entire thing and
            catching all exceptions as a sanity check.
        """
        try:
            configs = [pl.Node("rospy_tutorials", "talker", "talker2",
                            params={'calibrate_time': False},
                            remaps=[('chatter', 'hello_topic')]),
                       pl.Include('pylaunch', 'listener.launch', args={'some_arg': '21'})]

            p = pl.PyRosLaunch(configs)
            p.start()
            time.sleep(10)
            p.shutdown()
        except:
            self.fail("Shouldn't raise exceptions.")

    def test_launch_file_runner(self):
        try:
            p = pl.LaunchFileRunner("pylaunch", "talker.launch")
            p.start()
            time.sleep(10)
            p.shutdown()
        except:
            self.fail("Shouldn't raise exceptions.")

if __name__ == '__main__':
    import rosunit
    rosunit.unitrun('pylaunch', 'test_pylaunch', TestPylaunch)

import pylaunch as pl

configs = [pl.Node("rospy_tutorials", "talker", "talker2",
                params={'calibrate_time': False},
                remaps=[('chatter', 'hello_topic')]),
           pl.Include('pylaunch', 'listener.launch', args={'some_arg': '21'})]

pl.launch(configs)

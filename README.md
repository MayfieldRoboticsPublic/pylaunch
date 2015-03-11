# pylaunch
Library for writing and calling roslaunch files in Python.

### Example
```python
import pylaunch as pl

configs = [pl.Node("rospy_tutorials", "talker", "talker2",
                # passing in node pararmeters
                params={'calibrate_time': False},
                # remapping topics
                remaps=[('chatter', 'hello_topic')]),
           # including external launch files
           pl.Include('pylaunch', 'listener.launch', args={'some_arg': '21'})]

pl.launch(configs)
```
Or, for more control over roslaunch, use:

```python
p = PyRosLaunch(config_list)
p.start()
raw_input("press enter to stop") # or p.spin(), p.spinOnce(), etc.
p.shutdown() #call this to kill all nodes launched.
```

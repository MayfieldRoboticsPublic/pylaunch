import pylaunch as pl
p = pl.LaunchFileRunner("pylaunch", "talker.launch")
p.start()
p.spin()


set -e

apt-get update -qq
apt-get install -yq build-essential ros-indigo-rospy-tutorials

mkdir -p /root/ws/src
ln -s /root/pylaunch /root/ws/src/

. /opt/ros/indigo/setup.sh
cd /root/ws
catkin_make
. devel/setup.sh

nosetests -v /root/pylaunch/pylaunch/test

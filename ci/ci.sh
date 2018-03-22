set -e

apt-get update -qq
apt-get install -yq build-essential ros-indigo-rospy-tutorials python-pip

# Need coverage to generate a nosetest coverage report
pip install coverage

mkdir -p /root/ws/src
ln -s /root/pylaunch /root/ws/src/

. /opt/ros/indigo/setup.sh
cd /root/ws
catkin_make
. devel/setup.sh

cd /root/pylaunch/pylaunch
nosetests -v test

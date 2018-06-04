export ROSDIR=/opt/ros/kinetic
source ${ROSDIR}/setup.bash
BASEDIR=$(dirname `realpath $BASH_SOURCE`)
export ROS_PACKAGE_PATH="${BASEDIR}:${ROS_PACKAGE_PATH}"
export PYTHONPATH="${PYTHONPATH}:${BASEDIR}/pythonpath"

echo "ROS_PACKAGE_PATH: " ${ROS_PACKAGE_PATH}
echo "PYTHONPATH: " ${PYTHONPATH}
echo
echo "test things with:"
echo "  rosmsg md5 rai_msgs/arr"
echo "  rossrv md5 rai_msgs/StringString"

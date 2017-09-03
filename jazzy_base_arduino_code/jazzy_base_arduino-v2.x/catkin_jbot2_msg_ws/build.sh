#!/bin/sh
SOURCE=/tmp/jbot/jazzybot_hardware_and_Arduino_code/jazzy_base_arduino_code/jazzy_base_arduino-v2.x/catkin_jbot2_msg_ws/src
DEST=/tmp/catkin_jbot2_msg_ws

rsync -av $SOURCE* $DEST/.
catkin_make clean

catkin_make


rm -rf install
catkin_make install

echo "Now:"
echo " cd install"
echo " . setup.bash"
echo " rosrun rosserial_arduino make_libraries.py ~/Arduino/libraries"


cmake_minimum_required(VERSION 2.8.3)

project(korg_nanokontrol)

find_package(catkin REQUIRED)

catkin_package(
      CATKIN_DEPENDS rospy sensor_msgs)

install(PROGRAMS nodes/kontrol.py DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})

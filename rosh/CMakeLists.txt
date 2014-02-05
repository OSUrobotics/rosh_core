cmake_minimum_required(VERSION 2.8.3)
project(rosh)
find_package(catkin REQUIRED)
catkin_package()
catkin_python_setup()

catkin_add_nosetests(test/test_rosh_impl_topic.py)

install(PROGRAMS
    rosh
    nodes/roshlet.py
    DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})
cmake_minimum_required(VERSION 2.8.3)
project(rosh)

find_package(catkin REQUIRED COMPONENTS rospy)

catkin_python_setup()
catkin_package()

install(PROGRAMS
    rosh
    nodes/roshlet.py
    DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})

install(PROGRAMS
	shell.py
	DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)

# catkin_add_nosetests(test/test_rosh_impl_topic.py)

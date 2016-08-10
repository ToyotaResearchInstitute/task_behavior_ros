cmake_minimum_required(VERSION 2.8.3)
project(task_behavior_ros)

find_package(catkin REQUIRED)

catkin_python_setup()

catkin_package()

###########
## Build ##
###########

#############
## Install ##
#############
install(DIRECTORY example
        DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)

#############
## Testing ##
#############

catkin_add_nosetests(test)

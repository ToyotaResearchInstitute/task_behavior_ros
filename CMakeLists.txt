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
if (CATKIN_ENABLE_TESTING)
  find_package(rostest REQUIRED)
  add_rostest(test/test_action.test)
  add_rostest(test/test_batterystate.test)
  add_rostest(test/test_introspection.test)
  add_rostest(test/test_service.test)
  add_rostest(test/test_time.test)
  add_rostest(test/test_topic.test)
endif()

cmake_minimum_required(VERSION 2.8.3)
project(system_monitor)

find_package(catkin)

catkin_package()

install(DIRECTORY bin
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
  PATTERN "*.py"
  PERMISSIONS OWNER_EXECUTE OWNER_WRITE OWNER_READ
)

##############################################################################
# CMake
##############################################################################

cmake_minimum_required(VERSION 2.8.0)
project(robotican_gui)

##############################################################################
# Catkin
##############################################################################

# qt_build provides the qt cmake glue, roscpp the comms for a default talker
find_package(catkin REQUIRED COMPONENTS qt_build roscpp)
include_directories(${catkin_INCLUDE_DIRS} ${QT_INCLUDES} ${CMAKE_CURRENT_BINARY_DIR})
include_directories(src)
# Use this to define what the package will export (e.g. libs, headers).
# Since the default here is to produce only a binary, we don't worry about
# exporting anything.

find_package(catkin REQUIRED COMPONENTS
ric_board
        cmake_modules
        roscpp
        rospy
        std_msgs
        move_base_msgs
        actionlib
        moveit_core
        moveit_ros_planning
        moveit_ros_planning_interface
        robotican_common
        cv_bridge
        pcl_conversions
        pcl_ros
        tf
        moveit_core
        moveit_ros_planning
        moveit_ros_planning_interface
        
        pr2_controllers_msgs
        )

## System dependencies are found with CMake's conventions
find_package(Boost REQUIRED COMPONENTS system)
find_package(cmake_modules REQUIRED)
find_package(TinyXML REQUIRED)

catkin_package(
        CATKIN_DEPENDS
        roscpp
        rospy
        std_msgs
        moveit_core
        moveit_ros_planning
        moveit_ros_planning_interface
        LIBRARIES
        INCLUDE_DIRS
        include
        DEPENDS
        TinyXML
)

include_directories(
        ${catkin_INCLUDE_DIRS}
        ${TinyXML_INCLUDE_DIRS}
)

include_directories(include ${catkin_INCLUDE_DIRS} ${TinyXML_INCLUDE_DIRS})
#target_link_libraries(${PROJECT_NAME} ${catkin_LIBRARIES} ${TinyXML_LIBRARIES})

##############################################################################
# Qt Environment
##############################################################################
# this comes from qt_build's qt-ros.cmake which is automatically 
# included via the dependency call in package.xml
rosbuild_prepare_qt4(QtCore QtGui) # Add the appropriate components to the component list here

##############################################################################
# Sections
##############################################################################

file(GLOB QT_FORMS RELATIVE ${CMAKE_CURRENT_SOURCE_DIR} resources/ui/*.ui)
file(GLOB QT_RESOURCES RELATIVE ${CMAKE_CURRENT_SOURCE_DIR} resources/images/image_resource_file.qrc)
file(GLOB_RECURSE QT_MOC RELATIVE ${CMAKE_CURRENT_SOURCE_DIR} FOLLOW_SYMLINKS include/robotican_gui/*.hpp)



set(CMAKE_AUTORCC ON)
QT4_WRAP_UI(QT_FORMS_HPP ${QT_FORMS})
QT4_WRAP_CPP(QT_MOC_HPP ${QT_MOC} include/event_signal.h include/event_slot.h)
QT4_ADD_RESOURCES(RIC_GUI_RESOURCES_RCC resources/images/image_resource_file.qrc)
##############################################################################
# Sources
##############################################################################

file(GLOB_RECURSE QT_SOURCES RELATIVE ${CMAKE_CURRENT_SOURCE_DIR} FOLLOW_SYMLINKS src/*.cpp)

##############################################################################
# Binaries
##############################################################################
set(PROJ_FILES
        src/main.cpp
        src/gui_manager.cpp
        include/gui_manager.h
        src/event_slot.cpp
        include/event_slot.h
        src/event_signal.cpp
        include/event_signal.h
        src/led.cpp
        include/led.h
        include/robotican_gui.h
        src/arm_move.cpp
        include/arm_move.h
        include/listeners.hpp)

add_executable(robotican_gui ${QT_SOURCES} ${QT_RESOURCES_CPP} ${QT_FORMS_HPP} ${QT_MOC_HPP} ${PROJ_FILES} ${RIC_GUI_RESOURCES_RCC})
add_dependencies(robotican_gui ${ric_board_EXPORTED_TARGETS})
target_link_libraries(robotican_gui ${QT_LIBRARIES} ${catkin_LIBRARIES})
install(TARGETS robotican_gui RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})


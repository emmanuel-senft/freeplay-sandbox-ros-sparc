cmake_minimum_required(VERSION 2.8.3)
project(freeplay_sandbox)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS 
        dynamic_reconfigure
        freeplay_sandbox_msgs
        roscpp 
        tf 
        cv_bridge 
        image_transport 
        visualization_msgs
        image_geometry 
        chilitags_catkin
        nodelet)

## System dependencies
find_package(OpenCV REQUIRED)

message(STATUS "OpenCV version: ${OpenCV_VERSION}")
if(2.9.0 VERSION_GREATER ${OpenCV_VERSION})
    message(FATAL "OpenCV3 required!")
endif()

generate_dynamic_reconfigure_options(
      cfg/estimate_focus.cfg
)

###################################
## catkin specific configuration ##
###################################
## The catkin_package macro generates cmake config files for your package
## Declare things to be passed to dependent projects
## LIBRARIES: libraries you create in this project that dependent projects also need
## CATKIN_DEPENDS: catkin_packages dependent projects also need
## DEPENDS: system dependencies of this project that dependent projects also need
catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES ros_markers
  CATKIN_DEPENDS 
    tf
    nodelet
    freeplay_sandbox_msgs
  DEPENDS OpenCV
  LIBRARIES 
)


###########
## Build ##
###########

include_directories(include ${catkin_INCLUDE_DIRS})

add_definitions(-std=c++11)

set(LIBS 
    ${catkin_LIBRARIES}
    ${OpenCV_LIBRARIES}
    )


add_executable(sandbox_map_and_plan src/map_and_plan.cpp)
target_link_libraries(sandbox_map_and_plan ${LIBS})

#add_library(markers_nodelet src/chilitagsdetector.cpp src/nodelet.cpp)
#add_dependencies(markers_nodelet ${catkin_EXPORTED_TARGETS})
#target_link_libraries(markers_nodelet ${LIBS})

add_executable(sandtray_localisation 
    src/sandtray_localisation/chilitagsdetector.cpp 
    src/sandtray_localisation/sandtray_localisation.cpp)
target_link_libraries(sandtray_localisation ${LIBS})

add_executable(estimate_focus src/estimate_focus.cpp)
add_dependencies(estimate_focus ${PROJECT_NAME}_gencfg)
target_link_libraries(estimate_focus ${LIBS})

add_executable(visualfocus_assessment src/visualfocus_assessment.cpp)
target_link_libraries(visualfocus_assessment ${LIBS})

#############
## Install ##
#############

install(TARGETS
    sandbox_map_and_plan
    sandtray_localisation
    estimate_focus
    visualfocus_assessment
   ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
   LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
   RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(DIRECTORY
    launch/
    DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/launch
)

install(DIRECTORY
    urdf/
    DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/urdf
)


install(DIRECTORY
    meshes/
    DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/meshes
)

catkin_install_python(PROGRAMS
    nodes/move_sandbox_items
    nodes/play
    nodes/zones_from_image
    nodes/nao_behaviours
    nodes/sandtray_gaze_viz
    nodes/transformations.py
    nodes/woz_play
    nodes/pilot_recorder
    DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})

install(FILES
    config/sr300.cfg
    DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/config
)

#install(FILES
#    nodelet_markers.xml
#    DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
#)


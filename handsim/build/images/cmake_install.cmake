# Install script for directory: /home/haptix-e15-463/haptix/haptix_controller/handsim/images

# Set the install prefix
IF(NOT DEFINED CMAKE_INSTALL_PREFIX)
  SET(CMAKE_INSTALL_PREFIX "/usr/local")
ENDIF(NOT DEFINED CMAKE_INSTALL_PREFIX)
STRING(REGEX REPLACE "/$" "" CMAKE_INSTALL_PREFIX "${CMAKE_INSTALL_PREFIX}")

# Set the install configuration name.
IF(NOT DEFINED CMAKE_INSTALL_CONFIG_NAME)
  IF(BUILD_TYPE)
    STRING(REGEX REPLACE "^[^A-Za-z0-9_]+" ""
           CMAKE_INSTALL_CONFIG_NAME "${BUILD_TYPE}")
  ELSE(BUILD_TYPE)
    SET(CMAKE_INSTALL_CONFIG_NAME "RelWithDebInfo")
  ENDIF(BUILD_TYPE)
  MESSAGE(STATUS "Install configuration: \"${CMAKE_INSTALL_CONFIG_NAME}\"")
ENDIF(NOT DEFINED CMAKE_INSTALL_CONFIG_NAME)

# Set the component getting installed.
IF(NOT CMAKE_INSTALL_COMPONENT)
  IF(COMPONENT)
    MESSAGE(STATUS "Install component: \"${COMPONENT}\"")
    SET(CMAKE_INSTALL_COMPONENT "${COMPONENT}")
  ELSE(COMPONENT)
    SET(CMAKE_INSTALL_COMPONENT)
  ENDIF(COMPONENT)
ENDIF(NOT CMAKE_INSTALL_COMPONENT)

# Install shared libraries without execute permission?
IF(NOT DEFINED CMAKE_INSTALL_SO_NO_EXE)
  SET(CMAKE_INSTALL_SO_NO_EXE "1")
ENDIF(NOT DEFINED CMAKE_INSTALL_SO_NO_EXE)

IF(NOT CMAKE_INSTALL_COMPONENT OR "${CMAKE_INSTALL_COMPONENT}" STREQUAL "Unspecified")
  list(APPEND CMAKE_ABSOLUTE_DESTINATION_FILES
   "/usr/local/share/gazebo-7/media/gui/arat/arat_icons/spring.jpg;/usr/local/share/gazebo-7/media/gui/arat/arat_icons/grasp_1.jpg;/usr/local/share/gazebo-7/media/gui/arat/arat_icons/grasp_3.jpg;/usr/local/share/gazebo-7/media/gui/arat/arat_icons/grasp_2.jpg;/usr/local/share/gazebo-7/media/gui/arat/arat_icons/grasp_4.jpg;/usr/local/share/gazebo-7/media/gui/arat/arat_icons/grasp_5.jpg;/usr/local/share/gazebo-7/media/gui/arat/arat_icons/grasp_6.jpg;/usr/local/share/gazebo-7/media/gui/arat/arat_icons/grip_1.jpg;/usr/local/share/gazebo-7/media/gui/arat/arat_icons/grip_3.jpg;/usr/local/share/gazebo-7/media/gui/arat/arat_icons/grip_2.jpg;/usr/local/share/gazebo-7/media/gui/arat/arat_icons/grip_4.jpg;/usr/local/share/gazebo-7/media/gui/arat/arat_icons/hand_left.svg;/usr/local/share/gazebo-7/media/gui/arat/arat_icons/hand_right.svg;/usr/local/share/gazebo-7/media/gui/arat/arat_icons/pinch_1.jpg;/usr/local/share/gazebo-7/media/gui/arat/arat_icons/pinch_2.jpg;/usr/local/share/gazebo-7/media/gui/arat/arat_icons/settings.png")
  IF (CMAKE_WARN_ON_ABSOLUTE_INSTALL_DESTINATION)
    message(WARNING "ABSOLUTE path INSTALL DESTINATION : ${CMAKE_ABSOLUTE_DESTINATION_FILES}")
  ENDIF (CMAKE_WARN_ON_ABSOLUTE_INSTALL_DESTINATION)
  IF (CMAKE_ERROR_ON_ABSOLUTE_INSTALL_DESTINATION)
    message(FATAL_ERROR "ABSOLUTE path INSTALL DESTINATION forbidden (by caller): ${CMAKE_ABSOLUTE_DESTINATION_FILES}")
  ENDIF (CMAKE_ERROR_ON_ABSOLUTE_INSTALL_DESTINATION)
FILE(INSTALL DESTINATION "/usr/local/share/gazebo-7/media/gui/arat/arat_icons" TYPE FILE FILES
    "/home/haptix-e15-463/haptix/haptix_controller/handsim/images/spring.jpg"
    "/home/haptix-e15-463/haptix/haptix_controller/handsim/images/grasp_1.jpg"
    "/home/haptix-e15-463/haptix/haptix_controller/handsim/images/grasp_3.jpg"
    "/home/haptix-e15-463/haptix/haptix_controller/handsim/images/grasp_2.jpg"
    "/home/haptix-e15-463/haptix/haptix_controller/handsim/images/grasp_4.jpg"
    "/home/haptix-e15-463/haptix/haptix_controller/handsim/images/grasp_5.jpg"
    "/home/haptix-e15-463/haptix/haptix_controller/handsim/images/grasp_6.jpg"
    "/home/haptix-e15-463/haptix/haptix_controller/handsim/images/grip_1.jpg"
    "/home/haptix-e15-463/haptix/haptix_controller/handsim/images/grip_3.jpg"
    "/home/haptix-e15-463/haptix/haptix_controller/handsim/images/grip_2.jpg"
    "/home/haptix-e15-463/haptix/haptix_controller/handsim/images/grip_4.jpg"
    "/home/haptix-e15-463/haptix/haptix_controller/handsim/images/hand_left.svg"
    "/home/haptix-e15-463/haptix/haptix_controller/handsim/images/hand_right.svg"
    "/home/haptix-e15-463/haptix/haptix_controller/handsim/images/pinch_1.jpg"
    "/home/haptix-e15-463/haptix/haptix_controller/handsim/images/pinch_2.jpg"
    "/home/haptix-e15-463/haptix/haptix_controller/handsim/images/settings.png"
    )
ENDIF(NOT CMAKE_INSTALL_COMPONENT OR "${CMAKE_INSTALL_COMPONENT}" STREQUAL "Unspecified")


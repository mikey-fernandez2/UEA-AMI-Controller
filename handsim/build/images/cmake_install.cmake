# Install script for directory: /home/haptix-e15-463/haptix/haptix_controller/handsim/images

# Set the install prefix
if(NOT DEFINED CMAKE_INSTALL_PREFIX)
  set(CMAKE_INSTALL_PREFIX "/usr/local")
endif()
string(REGEX REPLACE "/$" "" CMAKE_INSTALL_PREFIX "${CMAKE_INSTALL_PREFIX}")

# Set the install configuration name.
if(NOT DEFINED CMAKE_INSTALL_CONFIG_NAME)
  if(BUILD_TYPE)
    string(REGEX REPLACE "^[^A-Za-z0-9_]+" ""
           CMAKE_INSTALL_CONFIG_NAME "${BUILD_TYPE}")
  else()
    set(CMAKE_INSTALL_CONFIG_NAME "Debug")
  endif()
  message(STATUS "Install configuration: \"${CMAKE_INSTALL_CONFIG_NAME}\"")
endif()

# Set the component getting installed.
if(NOT CMAKE_INSTALL_COMPONENT)
  if(COMPONENT)
    message(STATUS "Install component: \"${COMPONENT}\"")
    set(CMAKE_INSTALL_COMPONENT "${COMPONENT}")
  else()
    set(CMAKE_INSTALL_COMPONENT)
  endif()
endif()

# Install shared libraries without execute permission?
if(NOT DEFINED CMAKE_INSTALL_SO_NO_EXE)
  set(CMAKE_INSTALL_SO_NO_EXE "1")
endif()

# Is this installation the result of a crosscompile?
if(NOT DEFINED CMAKE_CROSSCOMPILING)
  set(CMAKE_CROSSCOMPILING "FALSE")
endif()

# Set default install directory permissions.
if(NOT DEFINED CMAKE_OBJDUMP)
  set(CMAKE_OBJDUMP "/usr/bin/x86_64-linux-gnu-objdump")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  list(APPEND CMAKE_ABSOLUTE_DESTINATION_FILES
   "/usr/local/share/gazebo-7/media/gui/arat/arat_icons/spring.jpg;/usr/local/share/gazebo-7/media/gui/arat/arat_icons/grasp_1.jpg;/usr/local/share/gazebo-7/media/gui/arat/arat_icons/grasp_3.jpg;/usr/local/share/gazebo-7/media/gui/arat/arat_icons/grasp_2.jpg;/usr/local/share/gazebo-7/media/gui/arat/arat_icons/grasp_4.jpg;/usr/local/share/gazebo-7/media/gui/arat/arat_icons/grasp_5.jpg;/usr/local/share/gazebo-7/media/gui/arat/arat_icons/grasp_6.jpg;/usr/local/share/gazebo-7/media/gui/arat/arat_icons/grip_1.jpg;/usr/local/share/gazebo-7/media/gui/arat/arat_icons/grip_3.jpg;/usr/local/share/gazebo-7/media/gui/arat/arat_icons/grip_2.jpg;/usr/local/share/gazebo-7/media/gui/arat/arat_icons/grip_4.jpg;/usr/local/share/gazebo-7/media/gui/arat/arat_icons/hand_left.svg;/usr/local/share/gazebo-7/media/gui/arat/arat_icons/hand_right.svg;/usr/local/share/gazebo-7/media/gui/arat/arat_icons/pinch_1.jpg;/usr/local/share/gazebo-7/media/gui/arat/arat_icons/pinch_2.jpg;/usr/local/share/gazebo-7/media/gui/arat/arat_icons/settings.png")
  if(CMAKE_WARN_ON_ABSOLUTE_INSTALL_DESTINATION)
    message(WARNING "ABSOLUTE path INSTALL DESTINATION : ${CMAKE_ABSOLUTE_DESTINATION_FILES}")
  endif()
  if(CMAKE_ERROR_ON_ABSOLUTE_INSTALL_DESTINATION)
    message(FATAL_ERROR "ABSOLUTE path INSTALL DESTINATION forbidden (by caller): ${CMAKE_ABSOLUTE_DESTINATION_FILES}")
  endif()
  file(INSTALL DESTINATION "/usr/local/share/gazebo-7/media/gui/arat/arat_icons" TYPE FILE FILES
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
endif()


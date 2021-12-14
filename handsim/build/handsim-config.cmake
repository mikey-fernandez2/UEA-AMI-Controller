if (HANDSIM_CONFIG_INCLUDED)
  return()
endif()
set(HANDSIM_CONFIG_INCLUDED TRUE)

list(APPEND HANDSIM_INCLUDE_DIRS /usr/local/include)

list(APPEND HANDSIM_LIBRARY_DIRS /usr/local/lib)

list(APPEND HANDSIM_CFLAGS -I/usr/local/include)

list(APPEND HANDSIM_CXX_FLAGS -std=c++11)
if ("${CMAKE_CXX_COMPILER_ID}" MATCHES "Clang")
  set(HANDSIM_CXX_FLAGS "${HANDSIM_CXX_FLAGS} -stdlib=libc++")
endif ()

foreach(lib HaptixTracking)
  set(onelib "${lib}-NOTFOUND")
  find_library(onelib ${lib}
    PATHS /usr/local/lib
    NO_DEFAULT_PATH
    )
  if(NOT onelib)
    message(FATAL_ERROR "Library '${lib}' in package HANDSIM is not installed properly")
  endif()
  list(APPEND HANDSIM_LIBRARIES ${onelib})
endforeach()

foreach(dep )
  if(NOT ${dep}_FOUND)
    find_package(${dep} REQUIRED)
  endif()
  string(TOUPPER ${dep} dep_upper)
  list(APPEND HANDSIM_INCLUDE_DIRS ${${dep_upper}_INCLUDE_DIRS})
  list(APPEND HANDSIM_LIBRARIES ${${dep_upper}_LIBRARIES})
endforeach()

list(APPEND HANDSIM_LDFLAGS -L/usr/local/lib)

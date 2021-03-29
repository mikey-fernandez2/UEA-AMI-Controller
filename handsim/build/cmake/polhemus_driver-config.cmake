if (HANDSIM_CONFIG_INCLUDED)
  return()
endif()
set(HANDSIM_CONFIG_INCLUDED TRUE)

list(APPEND HANDSIM_INCLUDE_DIRS /usr/local/include)
list(APPEND HANDSIM_INCLUDE_DIRS /usr/local/include/HANDSIM)

list(APPEND HANDSIM_LIBRARY_DIRS /usr/local/lib/x86_64-linux-gnu)

list(APPEND HANDSIM_CFLAGS -I/usr/local/include)
list(APPEND HANDSIM_CFLAGS -I/usr/local/include/HANDSIM)

foreach(lib HaptixTracking)
  set(onelib "${lib}-NOTFOUND")
  find_library(onelib ${lib}
    PATHS /usr/local/lib/x86_64-linux-gnu
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
  list(APPEND HANDSIM_INCLUDE_DIRS ${${dep}_INCLUDE_DIRS})

  # Protobuf needs to be capitalized to match PROTOBUF_LIBRARIES
  if (${dep} STREQUAL "Protobuf")
    string (TOUPPER ${dep} dep_lib)
  else()
    set (dep_lib ${dep})
  endif()
    
  list(APPEND HANDSIM_LIBRARIES ${${dep_lib}_LIBRARIES})
endforeach()

list(APPEND HANDSIM_LDFLAGS -Wl,-rpath,/usr/local/lib/x86_64-linux-gnu)
list(APPEND HANDSIM_LDFLAGS -L/usr/local/lib/x86_64-linux-gnu)

cmake_minimum_required(VERSION 3.0)

set(PROJECT_NAME trainpilot-client)

set(CMAKE_BINARY_DIR ${CMAKE_SOURCE_DIR}/bin)  # bin/

set(EXECUTABLE_OUTPUT_PATH ${CMAKE_BINARY_DIR})
set(LIBRARY_OUTPUT_PATH ${CMAKE_BINARY_DIR})

set(CMAKE_CXX_STANDARD 11)
set(CMAKE_PREFIX_PATH ${QTDIR})
set(CMAKE_AUTOMOC ON)
set(CMAKE_AUTOUIC ON)
set(CMAKE_AUTORCC ON)


#########################################################
# FIND GLUT
#########################################################
find_package(GLUT REQUIRED)
include_directories(${GLUT_INCLUDE_DIRS})
link_directories(${GLUT_LIBRARY_DIRS})
add_definitions(${GLUT_DEFINITIONS})
#if(NOT GLUT_FOUND)
#    MESSAGE(ERROR " GLUT not found!")
#endif(NOT GLUT_FOUND)
#########################################################
# FIND OPENGL
#########################################################
find_package(OpenGL REQUIRED)
include_directories(${OpenGL_INCLUDE_DIRS})
link_directories(${OpenGL_LIBRARY_DIRS})
add_definitions(${OpenGL_DEFINITIONS})
#if(NOT OPENGL_FOUND)
#	MESSAGE(ERROR " OPENGL not found!")
#endif(NOT OPENGL_FOUND)



find_package(Qt5 REQUIRED Widgets Core Gui Network OpenGL)
find_package(OpenCV REQUIRED COMPONENTS core highgui imgproc features2d calib3d )
find_package(Protobuf REQUIRED) # 3.6.1
find_package(VTK REQUIRED) # 8.1.2
include(${VTK_USE_FILE})

MESSAGE( [Main] " VTK_INCLUDE_DIRS = ${VTK_INCLUDE_DIRS}")
MESSAGE( [Main] " VTK_LIBRARIES = ${VTK_LIBRARIES}")

find_package(PCL REQUIRED) # 1.9.1
#find_package(PCL REQUIRED COMPONENTS common io filters visualization)

link_directories(${PCL_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})

message ([main] "PCL_DIR = ${PCL_DIR}")
message ([main] "PCL_FOUND = ${PCL_FOUND}")
message ([main] "PCL_INCLUDE_DIRS = ${PCL_INCLUDE_DIRS}")
message ([main] "PCL_LIBRARIES = ${PCL_LIBRARIES}")

message ([main] "PCL_LIBRARY_DIRS = ${PCL_LIBRARY_DIRS}")
message ([main] "PCL_COMMON_LIBRARIES = ${PCL_COMMON_LIBRARIES}")
message ([main] "PCL_IO_LIBRARIES = ${PCL_IO_LIBRARIES}")
message ([main] "PCL_FILTERS_LIBRARIES = ${PCL_FILTERS_LIBRARIES}")
message ([main] "PCL_VISUALIZATION_LIBRARIES = ${PCL_VISUALIZATION_LIBRARIES}")

include_directories(
	./ # current folder

#	${GFLAGS_INCLUDE_DIRS}
#	${GLOG_INCLUDE_DIRS}
#	${GTEST_INCLUDE_DIRS}
	${PROTOBUF_INCLUDE_DIRS}
	${Boost_INCLUDE_DIRS} 
	${OpenCV_INCLUDE_DIRS}
	${VTK_INCLUDE_DIRS}
	${PCL_INCLUDE_DIRS}
)

set(PROTO_DIR ${PROJECT_SOURCE_DIR}/src/proto)

add_subdirectory(src)
add_subdirectory(showdemo)
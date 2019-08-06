# This file specifies the build process for the VNaoBridge.
# In order to build the binary, create a directory for all intermediate files, and run cmake from there:
#   $ mkdir -p build
#   $ cd build
#   $ cmake ..
#   $ make

# Require a minimum version for the cmake build tool.
# I (Alexander Breckel) am however not sure about the exact version that we depend on.
# At the time of writing (2018-12-18) I'm using version 3.13.1.
cmake_minimum_required(VERSION 2.8.11)


# Specify a cmake Project. The name will be used as the filename.
project(vnaobridge)


################################################################
# Global config flags

SET(CMAKE_BUILD_WITH_INSTALL_RPATH TRUE)
if(APPLE)
	SET(CMAKE_INSTALL_RPATH "@loader_path/../")
else()
	SET(CMAKE_INSTALL_RPATH "$ORIGIN/../lib")
endif()

# Use the C++ standard C++11.
set(CMAKE_CXX_STANDARD 11)

# These options will ease debugging.
set(CMAKE_VERBOSE_MAKEFILE ON)
set(CMAKE_BUILD_TYPE Release)

# The external files we link against were all compiled with the "old" C++ ABI.
# The following definition makes sure that we use the same ABI.
add_compile_definitions(_GLIBCXX_USE_CXX11_ABI=0)
################################################################


################################################################
# NaoQI (the client part, not Simulator SDK) links in Boost,
# which requires some additional setup under windows.
# I don't know why, but this must come before add_executable().
if(WIN32)
	add_compile_definitions(BOOST_ALL_DYN_LINK)
	link_directories(${SIMSDK_PATH}/lib/)
endif(WIN32)
################################################################


################################################################
# Executable

# build an executable
add_executable(${PROJECT_NAME})

# Specify our own sources and headers
target_sources(${PROJECT_NAME} PUBLIC ${PROJECT_SOURCE_DIR}/source/main.cpp)
################################################################


################################################################
# Dependency: PThread
set(THREADS_PREFER_PTHREAD_FLAG ON)
find_package(Threads REQUIRED)
target_link_libraries(${PROJECT_NAME} Threads::Threads)
################################################################


################################################################
# Dependency: Coppelia Robotics V-Rep Library

set(VREP_PATH "" CACHE PATH "Path to V-Rep installation")

# add V-Rep sources, headers and libraries
include_directories(${VREP_PATH}/programming/remoteApi)
target_sources(${PROJECT_NAME} PUBLIC
	${VREP_PATH}/programming/remoteApi/extApi.c
	${VREP_PATH}/programming/remoteApi/extApiPlatform.c)
add_compile_definitions(NON_MATLAB_PARSING)
add_compile_definitions(MAX_EXT_API_CONNECTIONS=255)
add_compile_definitions(DO_NOT_USE_SHARED_MEMORY) # using shared memory could speed things up a bit...

if(LINUX)
	target_link_libraries(${PROJECT_NAME} rt)
endif()

include_directories(${VREP_PATH}/programming/include)
################################################################


################################################################
# Dependency: Aldebaran Robotics Simulator SDK

set(SIMSDK_PATH "" CACHE PATH "Path to Simulator SDK")

include_directories(${SIMSDK_PATH}/include/)
if(WIN32)
	target_link_libraries(${PROJECT_NAME}
		${SIMSDK_PATH}/lib/alnaosim.lib
		${SIMSDK_PATH}/lib/alrobotmodel.lib
		${SIMSDK_PATH}/lib/qi.lib

		${SIMSDK_PATH}/lib/alproxies.lib
		${SIMSDK_PATH}/lib/alcommon.lib
		${SIMSDK_PATH}/lib/alvalue.lib
		${SIMSDK_PATH}/lib/alerror.lib

		${SIMSDK_PATH}/lib/qimessaging.lib
		${SIMSDK_PATH}/lib/qitype.lib)
elseif(APPLE)
	target_link_libraries(${PROJECT_NAME}
		${SIMSDK_PATH}/lib/libalnaosim.dylib
		${SIMSDK_PATH}/lib/libalrobotmodel.dylib
		${SIMSDK_PATH}/lib/libqi.dylib

		${SIMSDK_PATH}/lib/libalproxies.dylib
		${SIMSDK_PATH}/lib/libalcommon.dylib
		${SIMSDK_PATH}/lib/libalvalue.dylib
		${SIMSDK_PATH}/lib/libalerror.dylib

		${SIMSDK_PATH}/lib/libqitype.dylib
		${SIMSDK_PATH}/lib/libboost_system.dylib)
else()
	target_link_libraries(${PROJECT_NAME}
		${SIMSDK_PATH}/lib/libalnaosim.so
		${SIMSDK_PATH}/lib/libalrobotmodel.so
		${SIMSDK_PATH}/lib/libqi.so

		${SIMSDK_PATH}/lib/libalproxies.so
		${SIMSDK_PATH}/lib/libalcommon.so
		${SIMSDK_PATH}/lib/libalvalue.so
		${SIMSDK_PATH}/lib/libalerror.so

		${SIMSDK_PATH}/lib/libqitype.so
		${SIMSDK_PATH}/lib/libboost_system.so)
endif()
################################################################

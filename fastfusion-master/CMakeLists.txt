cmake_minimum_required(VERSION 2.8)

set(OpenCV_LIBS opencv_core opencv_imgproc opencv_highgui opencv_calib3d)

####set(CMAKE_BUILD_TYPE Release)
#set(CMAKE_BUILD_TYPE RelWithDebInfo)
#set(ROS_BUILD_TYPE RelWithDebInfo)
set(CMAKE_BUILD_TYPE Debug)
#set(ROS_BUILD_TYPE Debug)

#FIND_PACKAGE(Qt4 REQUIRED)
#find_package(OpenCV REQUIRED) # ---- orig
find_package(OpenCV 2.4.8 REQUIRED) # ---- mod
#find_package(TBB)

#if(NOT TBB_FOUND)
#MESSAGE(STATUS "TBB not found!");
#else(NOT TBB_FOUND)
#include_directories(${TBB_INCLUDE_DIRS})
#link_directories(${TBB_LIBRARY_DIRS})
#endif(NOT TBB_FOUND)

set(CMAKE_C_FLAGS
-frounding-math 
#-Wall
)
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -Wall -msse4 -funroll-loops -fopenmp")
set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} -Wall -msse4 -funroll-loops -fopenmp")

set(EXECUTABLE_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/bin)
set(LIBRARY_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/lib)

IF(${CMAKE_SYSTEM_NAME} MATCHES "Darwin")
    # build with clang fails.  clang doesn't support -lgomp. Use gcc in stread.
    # http://stackoverflow.com/questions/17275348/how-to-specify-new-gcc-path-for-cmake
    # to used gcc46,
    #export CC=/usr/local/Cellar/gcc46/4.6.4/bin/gcc-4.6
    #export CXX=/usr/local/Cellar/gcc46/4.6.4/bin/g++-4.6
    #cmake .
    #make sure generated CMakeCache.txt contains gcc46 lines

    # for gcc49
    #export CC=/usr/local/Cellar/gcc49/4.9.3/bin/gcc-4.9
    #export CXX=/usr/local/Cellar/gcc49/4.9.3/bin/c++-4.9
    #cmake .

    #FIND_PACKAGE() for OpenCV already done.  See above.
    FIND_PACKAGE(Boost REQUIRED COMPONENTS thread system)

    # fixme: doesn't link unless manually adding dylib's into link.txt
    # ->modified fusion/CMakeLists.txt to have target_link_libraries(geometryfusion_aos ...
    # fixme: memory sys calls commented in memory.cpp
    # ->just return 0's in #ifdef __APPLE__

    INCLUDE_DIRECTORIES(
      include
      ${OpenCV_INCLUDE_DIRS}
      ${Boost_INCLUDE_DIRS}
    )
    LIST(APPEND LINK_LIBS
      ${OpenCV_LIBRARIES}
      ${Boost_LIBRARIES}
    )
    message(STATUS "@@@ opencv inc " ${OpenCV_INCLUDE_DIRS})
    message(STATUS "@@@ opencv lib " ${OpenCV_LIBRARIES})
    message(STATUS "@@@ boost inc " ${Boost_INCLUDE_DIRS})
    message(STATUS "@@@ boost lib " ${Boost_LIBRARIES})
ENDIF(${CMAKE_SYSTEM_NAME} MATCHES "Darwin")


include_directories(${PROJECT_SOURCE_DIR}/src)
message(STATUS "PROJECT_SOURCE_DIR is " ${PROJECT_SOURCE_DIR})

#set(Boost_DEBUG 0)
#set(BOOST_USE_STATIC_LIBS OFF)
#set(BOOST_USE_MULTITHREADED ON)
#set(BOOST_USE_STATIC_RUNTIME OFF)
#find_package(Boost COMPONENTS thread system tr1)
#include_directories(${Boost_INCLUDE_DIRS})

#find_package(Eigen3 REQUIRED)
#include_directories(EIGEN3_INCLUDE_DIR)


IF(${CMAKE_SYSTEM_NAME} MATCHES "Darwin")
    include_directories("/opt/local/include/eigen3")  # -------- for osx
ELSE()
    include_directories("/usr/include/eigen3")
ENDIF()



## CUDA

#set(CUDA_TOOLKIT_ROOT_DIR "/work/sdks/cudaversions/cuda50/")
# find_package(CUDA 5.0 REQUIRED)
# if(CUDA_FOUND)
#     message("Found CUDA")
# else(CUDA_FOUND)
#     message("CUDA is not installed on this system.")
# endif()
# execute_process( COMMAND nvcc --version )
# include_directories(${CUDA_TOOLKIT_ROOT_DIR}/include)
# set(TEST_CUDA_ARCH $ENV{CUDA_ARCH})
# if(TEST_CUDA_ARCH)
# message("The GPU Architecture is set to $ENV{CUDA_ARCH} in an Environment Variable")
# set(CUDA_ARCH $ENV{CUDA_ARCH})
# else(TEST_CUDA_ARCH)
# message("No GPU Architecture set, setting to sm_13")
# set(CUDA_ARCH sm_11)
# endif(TEST_CUDA_ARCH)
# #if(${CUDA_ARCH} matches sm_11)
# #set(CUDAARCHNUMBER 11)
# #elseif(${CUDA_ARCH} matches sm_13)
# #set(CUDAARCHNUMBER 13)
# #elseif(${CUDA_ARCH} matches sm_20)
# #set(CUDAARCHNUMBER 20)
# #endif(${CUDA_ARCH} matches sm_11)
# set(CUDA_NVCC_FLAGS ${CUDA_NVCC_FLAGS} -arch=${CUDA_ARCH})






#include_directories(${PROJECT_SOURCE_DIR}/external)
#include_directories(${PROJECT_SOURCE_DIR}/external/math)
#include_directories(${PROJECT_SOURCE_DIR}/include)
#include_directories(${PROJECT_SOURCE_DIR}/build/src)
#include_directories("/usr/include/nvidia-current/cuda")

# find_package(PCL 1.5 REQUIRED COMPONENTS common io filters)
# include_directories(${PCL_INCLUDE_DIRS})
# link_directories(${PCL_LIBRARY_DIRS})
# add_definitions(${PCL_DEFINITIONS})



#include(${QT_USE_FILE})
#add_definitions(${QT_DEFINITIONS})

# Doxygen integration
find_package(Doxygen)
if(DOXYGEN_FOUND)
#this adds the correct folder to INPUT= :
configure_file(${CMAKE_CURRENT_SOURCE_DIR}/Doxyfile.in ${CMAKE_CURRENT_BINARY_DIR}/Doxyfile @ONLY)

#add doc target
add_custom_target(doc
${DOXYGEN_EXECUTABLE} ${CMAKE_CURRENT_BINARY_DIR}/Doxyfile
WORKING_DIRECTORY ${CMAKE_CURRENT_BINARY_DIR}
COMMENT "generating docs with Doxygen" VERBATIM
)
endif(DOXYGEN_FOUND)

add_subdirectory(src)


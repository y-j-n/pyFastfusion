set(camerautils_SOURCES camerautils.cpp)

include_directories(${PROJECT_SOURCE_DIR}/src/camerautils)
include_directories("/usr/include/eigen3")

MESSAGE(STATUS @@@ make shared lib for ctypes)
#add_library(camerautils ${camerautils_SOURCES})
add_library(camerautils SHARED ${camerautils_SOURCES})
target_link_libraries(camerautils
)

IF(${CMAKE_SYSTEM_NAME} MATCHES "Darwin")
    target_link_libraries(camerautils
        -L/Users/me/local/opencv-2.4.8/libstdcpp/lib
        -lopencv_calib3d -lopencv_contrib -lopencv_core -lopencv_features2d -lopencv_flann -lopencv_gpu -lopencv_highgui -lopencv_imgproc -lopencv_legacy -lopencv_ml -lopencv_nonfree -lopencv_objdetect -lopencv_photo -lopencv_stitching -lopencv_superres -lopencv_ts -lopencv_video -lopencv_videostab
        ${Boost_LIBRARIES}
    )
ENDIF()

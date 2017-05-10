MESSAGE(STATUS @@@ make shared lib for ctypes)
#add_library(auxiliary ${CPU_LIBRARY_TYPE}
add_library(auxiliary SHARED ${CPU_LIBRARY_TYPE}
debug.cpp memory.cpp threadpool.cpp plywriter.cpp ocv_tools.cpp)
#cuda_add_library(auxiliaryGPU ${GPU_LIBRARY_TYPE}
#cuda_basic.cu
#OPTIONS ${CUDAOPTIONS})

IF(${CMAKE_SYSTEM_NAME} MATCHES "Darwin")
    target_link_libraries(auxiliary
        -L/Users/me/local/opencv-2.4.8/libstdcpp/lib
        -lopencv_calib3d -lopencv_contrib -lopencv_core -lopencv_features2d -lopencv_flann -lopencv_gpu -lopencv_highgui -lopencv_imgproc -lopencv_legacy -lopencv_ml -lopencv_nonfree -lopencv_objdetect -lopencv_photo -lopencv_stitching -lopencv_superres -lopencv_ts -lopencv_video -lopencv_videostab
        ${Boost_LIBRARIES}
    )
ENDIF()


include_directories("/usr/include/nvidia-current/cuda")
include_directories("/work/sdks/cudaversions/cuda50/cuda/include")
include_directories("/home/steinbrf/sdks/cudaversions/cuda50/cuda/include")
#find_package(Qt REQUIRED)

#INCLUDE(${QT_USE_FILE})
include_directories("/usr/include/qt4/QtXml")
include_directories("/usr/include/qt4")
include_directories("/usr/include/qt4/Qt")
include_directories("/usr/include/qt4/QtCore")
include_directories("/usr/include/qt4/QtXml")
include_directories("/usr/include/qt4/QtGui")
include_directories("/usr/include/qt4/QtOpenGL")
ADD_DEFINITIONS(${QT_DEFINITIONS})

include_directories("..")

MESSAGE(STATUS @@@ make shared lib for ctypes)
#add_library(geometryfusion_aos STATIC
add_library(geometryfusion_aos SHARED
geometryfusion_aos.cpp
treeandbrick.cpp
treeandbrick_indexed.cpp
treeandbrick_incremental.cpp
treeandbrick_incremental_recursive.cpp
mesh.cpp
#meshcelltraversal.cpp
mesh_interleaved.cpp
mesh_interleaved_meshcell.cpp
)

IF(${CMAKE_SYSTEM_NAME} MATCHES "Darwin")
    target_link_libraries(geometryfusion_aos
        -L/Users/me/local/opencv-2.4.8/libstdcpp/lib
        -lopencv_calib3d -lopencv_contrib -lopencv_core -lopencv_features2d -lopencv_flann -lopencv_gpu -lopencv_highgui -lopencv_imgproc -lopencv_legacy -lopencv_ml -lopencv_nonfree -lopencv_objdetect -lopencv_photo -lopencv_stitching -lopencv_superres -lopencv_ts -lopencv_video -lopencv_videostab
        ${Boost_LIBRARIES}
    )
ENDIF()

MESSAGE(STATUS @@@ make shared lib for ctypes)
#add_library(geometryfusion_mipmap_cpu STATIC
add_library(geometryfusion_mipmap_cpu SHARED
geometryfusion_mipmap_cpu.cpp
loopclosure.cpp
)

IF(${CMAKE_SYSTEM_NAME} MATCHES "Darwin")
    target_link_libraries(geometryfusion_mipmap_cpu
        geometryfusion_aos
        camerautils
        auxiliary
        -L/Users/me/local/opencv-2.4.8/libstdcpp/lib
        -lopencv_calib3d -lopencv_contrib -lopencv_core -lopencv_features2d -lopencv_flann -lopencv_gpu -lopencv_highgui -lopencv_imgproc -lopencv_legacy -lopencv_ml -lopencv_nonfree -lopencv_objdetect -lopencv_photo -lopencv_stitching -lopencv_superres -lopencv_ts -lopencv_video -lopencv_videostab
        ${Boost_LIBRARIES}
    )
ENDIF()

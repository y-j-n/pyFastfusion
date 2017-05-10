add_subdirectory(camerautils)
add_subdirectory(auxiliary)
#add_subdirectory(auxiliaryGPU)
#add_subdirectory(libelas)
#add_subdirectory(output)
add_subdirectory(fusion)
#add_subdirectory(fusionGPU)
#add_subdirectory(sophus)
#add_subdirectory(filesystem) 

IF(${CMAKE_SYSTEM_NAME} MATCHES "Darwin")
    include_directories("/opt/local/include/QtXml")
    include_directories("/opt/local/include/")
    include_directories("/opt/local/include/Qt")
    include_directories("/opt/local/include/QtCore")
    include_directories("/opt/local/include/QtXml")
    include_directories("/opt/local/include/QtGui")
    include_directories("/opt/local/include/QtOpenGL")
ELSE()
    include_directories("/usr/include/qt4/QtXml")
    include_directories("/usr/include/qt4")
    include_directories("/usr/include/qt4/Qt")
    include_directories("/usr/include/qt4/QtCore")
    include_directories("/usr/include/qt4/QtXml")
    include_directories("/usr/include/qt4/QtGui")
    include_directories("/usr/include/qt4/QtOpenGL")
ENDIF()
ADD_DEFINITIONS(${QT_DEFINITIONS})
set(QGLVIEWER qglviewer-qt4)

# add_executable(cameracalibration cameracalib_main.cpp)
# target_link_libraries(cameracalibration 
# opencv_core opencv_imgproc opencv_highgui opencv_features2d opencv_calib3d
# )
# 
# add_executable(rectify_known rectify_stereo_known_main.cpp)
# target_link_libraries(rectify_known 
# opencv_core opencv_imgproc opencv_highgui opencv_features2d opencv_calib3d
# )


FIND_PACKAGE(Qt4 REQUIRED)
QT4_WRAP_CPP(onlinefusion_HEADERS_MOC onlinefusionviewer.hpp onlinefusionviewer_if.hpp)

add_executable(onlinefusion
  onlinefusionviewer_main.cpp
  onlinefusionviewer.cpp
  onlinefusionviewer_if.cpp
  ${onlinefusion_HEADERS_MOC}
)
IF(${CMAKE_SYSTEM_NAME} MATCHES "Darwin")
    target_link_libraries(onlinefusion
      geometryfusion_mipmap_cpu geometryfusion_aos
      camerautils
      auxiliary
      -L/Users/me/local/opencv-2.4.8/libstdcpp/lib
      -lopencv_calib3d -lopencv_contrib -lopencv_core -lopencv_features2d -lopencv_flann -lopencv_gpu -lopencv_highgui -lopencv_imgproc -lopencv_legacy -lopencv_ml -lopencv_nonfree -lopencv_objdetect -lopencv_photo -lopencv_stitching -lopencv_superres -lopencv_ts -lopencv_video -lopencv_videostab
      /opt/local/lib/libQGLViewer.dylib
      -L/opt/local/lib QtCore QtGui QtOpenGL
      -L/opt/X11/lib GL GLU glut GLEW
      ${Boost_LIBRARIES}
    )
ELSE()
    target_link_libraries(onlinefusion
      geometryfusion_mipmap_cpu geometryfusion_aos
      camerautils
      auxiliary
      ${OpenCV_LIBS}
      ${QGLVIEWER}
      QtCore QtGui QtOpenGL
      GL GLU glut GLEW
      boost_thread
    )
ENDIF()

MESSAGE(STATUS @@@ make shared lib for ctypes)
add_library(onlinefusionctypes SHARED
  onlinefusionviewer_main.cpp
  onlinefusionviewer.cpp
  onlinefusionviewer_if.cpp
  ${onlinefusion_HEADERS_MOC}
)

IF(${CMAKE_SYSTEM_NAME} MATCHES "Darwin")
    target_link_libraries(onlinefusionctypes
      geometryfusion_mipmap_cpu geometryfusion_aos
      camerautils
      auxiliary
      -L/Users/me/local/opencv-2.4.8/libstdcpp/lib
      -lopencv_calib3d -lopencv_contrib -lopencv_core -lopencv_features2d -lopencv_flann -lopencv_gpu -lopencv_highgui -lopencv_imgproc -lopencv_legacy -lopencv_ml -lopencv_nonfree -lopencv_objdetect -lopencv_photo -lopencv_stitching -lopencv_superres -lopencv_ts -lopencv_video -lopencv_videostab
      /opt/local/lib/libQGLViewer.dylib
      -L/opt/local/lib QtCore QtGui QtOpenGL
      -L/opt/X11/lib GL GLU glut GLEW
      ${Boost_LIBRARIES}
    )
ELSE()
    target_link_libraries(onlinefusionctypes
      geometryfusion_mipmap_cpu geometryfusion_aos
      camerautils
      auxiliary
      ${OpenCV_LIBS}
      ${QGLVIEWER}
      QtCore QtGui QtOpenGL
      GL GLU glut GLEW
      boost_thread
    )
ENDIF()

/*
* libfastfusion is comipiled by gcc and so requires libstdc++ version of opencv (not using osx's /usr/lib/libc++.1.dylib)
SEE ~/local/opencv-2.4.8/libstdcpp/memo.org

** ok (clang)
g++ `pkg-config --cflags opencv` -c opencv.cpp -o cv.o
g++ cv.o `pkg-config --libs opencv`

** ng /usr/local/Cellar/gcc46/4.6.4/bin/g++-4.6
# (because the default libopencv_* are using libc++.dylib)
$CXX `pkg-config --cflags opencv` -c opencv.cpp -o cv.o
$CXX cv.o `pkg-config --libs opencv`
Undefined symbols for architecture x86_64:
  "cv::imread(std::basic_string<char, std::char_traits<char>, std::allocator<char> > const&, int)", referenced from:
      _main in cv.o
ld: symbol(s) not found for architecture x86_64
collect2: ld returned 1 exit status

** ok (gcc & opencv with libstdc++)
export OPENCV_LOCAL=/Users//local/opencv-2.4.8
$CXX -I$OPENCV_LOCAL/include/opencv -I$OPENCV_LOCAL/include -c opencv.cpp -o cv.o
$CXX cv.o -L$OPENCV_LOCAL/libstdcpp/lib -lopencv_calib3d -lopencv_contrib -lopencv_core -lopencv_features2d -lopencv_flann -lopencv_gpu -lopencv_highgui -lopencv_imgproc -lopencv_legacy -lopencv_ml -lopencv_nonfree -lopencv_objdetect -lopencv_photo -lopencv_stitching -lopencv_superres -lopencv_ts -lopencv_video -lopencv_videostab

export DYLD_LIBRARY_PATH=$OPENCV_LOCAL/libstdcpp/lib
./a.out
*/
#include <opencv2/opencv.hpp>

using namespace std;

int main() {
    cv::imread("test", 0);
    return 0;
}

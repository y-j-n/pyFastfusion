cmake_minimum_required(VERSION 2.8 FATAL_ERROR)

project(planar_segmentation)

# this not working; failes to have correct PCL_ROOT in share/pcl-1.8/PCLConfig.cmake
#set(PCL_DIR "../pcl-master/build/local/share/pcl-1.8")

# this works
set(CMAKE_PREFIX_PATH "../pcl-master/build/local")

find_package(PCL 1.2 REQUIRED)

include_directories(${PCL_INCLUDE_DIRS})
link_directories(${PCL_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})

# ==================
# http://www.pointclouds.org/documentation/tutorials/

# http://www.pointclouds.org/documentation/tutorials/planar_segmentation.php
add_executable(planar_segmentation planar_segmentation.cpp)
target_link_libraries(planar_segmentation ${PCL_LIBRARIES})

# http://www.pointclouds.org/documentation/tutorials/extract_indices.php
add_executable(extract_indices extract_indices.cpp)
target_link_libraries(extract_indices ${PCL_LIBRARIES})

# http://pointclouds.org/documentation/tutorials/pcl_visualizer.php
add_executable (pcl_visualizer_demo pcl_visualizer_demo.cpp)
target_link_libraries (pcl_visualizer_demo ${PCL_LIBRARIES})

# http://www.pointclouds.org/documentation/tutorials/range_image_border_extraction.php
# !!!!
add_executable (range_image_border_extraction range_image_border_extraction.cpp)
target_link_libraries (range_image_border_extraction ${PCL_LIBRARIES})

# http://www.pointclouds.org/documentation/tutorials/random_sample_consensus.php
add_executable (random_sample_consensus random_sample_consensus.cpp)
target_link_libraries (random_sample_consensus ${PCL_LIBRARIES})

# -----------
# http://www.pointclouds.org/documentation/tutorials/correspondence_grouping.php
# !!!! Hough-voting corresp clustering, keypoints by downsampling with the SHOT 3d descriptor,
# !!!! find corresps by KdTreeFLANN, then Hough3DGrouping(default)/GeometricConsistencyGrouping
add_executable (correspondence_grouping correspondence_grouping.cpp)
target_link_libraries (correspondence_grouping ${PCL_LIBRARIES})

# http://www.pointclouds.org/documentation/tutorials/global_hypothesis_verification.php
# !!!! this is a follow-up of the above
# ./global_hypothesis_verification ../milk.pcd ../milk_cartoon_all_small_clorox.pcd
add_executable (global_hypothesis_verification global_hypothesis_verification.cpp)
target_link_libraries (global_hypothesis_verification ${PCL_LIBRARIES})
# -----------
# primer readings:
# http://www.pointclouds.org/documentation/tutorials/normal_estimation.php
# http://www.pointclouds.org/documentation/tutorials/pfh_estimation.php
# http://www.pointclouds.org/documentation/tutorials/fpfh_estimation.php
# http://www.pointclouds.org/documentation/tutorials/vfh_estimation.php
# https://github.com/PointCloudLibrary/pcl/wiki/Overview-and-Comparison-of-Features
#---
# http://docs.pointclouds.org/trunk/group__features.html
## k closest point features
## octrees or kD-trees
## curvature and normal: the most widely used geometric point features
## obtained by eigendecomposition:
## the eigenvector of the smallest eigenvalue -> surface normal
## lambda0/(lambda0+lambda1+lambda2), lambda0 < lambda1 < lambda2 -> the surface curvature change

# http://www.pointclouds.org/documentation/tutorials/cluster_extraction.php
# This Euclidean Cluster Extraction is preprocessing of the following vfh_recog
# This is on top of the Plain Model Segmentation (SAC) article.
# ./cluster_extraction
# (cloud_cluster_{0,1,2,3,4}.pcd are generated)
# pcl_viewer cloud_cluster_{0,1,2,3,4}.pcd
add_executable (cluster_extraction cluster_extraction.cpp)
target_link_libraries (cluster_extraction ${PCL_LIBRARIES})
#---
# http://www.pointclouds.org/documentation/tutorials/vfh_recognition.php
# for unit test: ./nearest_neighbors -k 16 -thresh 50 ../data/000.580.67/1258730231333_cluster_0_nxyz_vfh.pcd
# for test: cd build && ../test_vfh_nn.sh
find_package(HDF5)
if(HDF5_FOUND)
  find_package(FLANN)
  include_directories(${FLANN_INCLUDE_DIRS})
  include_directories(${HDF5_INCLUDE_DIR})

  add_executable(build_tree build_tree.cpp)
  target_link_libraries(build_tree ${PCL_LIBRARIES} ${Boost_LIBRARIES} ${FLANN_LIBRARIES} ${HDF5_hdf5_LIBRARY})

  add_executable(nearest_neighbors nearest_neighbors.cpp)
  target_link_libraries(nearest_neighbors ${PCL_LIBRARIES} ${Boost_LIBRARIES} ${FLANN_LIBRARIES} ${HDF5_hdf5_LIBRARY})
endif(HDF5_FOUND)
# -----------==================
# http://www.pointclouds.org/documentation/tutorials/ground_based_rgbd_people_detection.php
#add_executable (ground_based_rgbd_people_detector MACOSX_BUNDLE main_ground_based_people_detection.cpp)
#target_link_libraries (ground_based_rgbd_people_detector ${PCL_LIBRARIES})
# -----------==================
add_library(pcl_ctypes SHARED pcl_ctypes.cpp)
target_link_libraries(pcl_ctypes ${PCL_LIBRARIES})
# -----------==================
# MESHING....
# http://pointclouds.org/documentation/tutorials/greedy_projection.php
add_executable (greedy_projection greedy_projection.cpp)
target_link_libraries (greedy_projection ${PCL_LIBRARIES})
# ----
# http://pointclouds.org/documentation/tutorials/using_kinfu_large_scale.php
# this requires GPU..... ?
# -----------==================
# how to use getPointsIndices()
# https://sites.google.com/site/usingpcl/home/mausudezhidingshitalingyunodianqunwoderupclvisualizer
add_executable (get_points_indices get_points_indices.cpp)
target_link_libraries (get_points_indices ${PCL_LIBRARIES})
# -----------==================
# TODOS........
# - test_save_frames_ popup current img !!!!
# - color/depth frames, assoc -> dvo -> ff :: dvo is with ros....
# - plane component selection !!!!
# -
# -
# -


# http://www.pointclouds.org/documentation/overview/visualization.php
# ~/slam/pcl-master/build/local/bin/pcl_viewer

# special issues, potential  --------------
# http://www.journals.elsevier.com/pattern-recognition-letters/call-for-papers/special-issue-on-efficient-shape-representation-matching/
# expired:  http://www.journals.elsevier.com/robotics-and-autonomous-systems/call-for-papers/3d-robot-perception-with-the-point-cloud-library/
# journals distilled from http://vision.in.tum.de/publications  --------------
# https://www.siam.org/journals/siims.php
# http://www.journals.elsevier.com/journal-of-visual-communication-and-image-representation/
# http://www.springer.com/computer/image+processing/journal/10851
# http://www.journals.elsevier.com/image-and-vision-computing/
# http://ieeexplore.ieee.org/xpl/RecentIssue.jsp?punumber=6979  Intelligent Transportation Systems, IEEE Transactions on
# http://ieeexplore.ieee.org/xpl/RecentIssue.jsp?punumber=83  Image Processing, IEEE Transactions on
# http://link.springer.com/journal/138  Machine Vision and Applications

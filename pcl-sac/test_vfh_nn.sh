#!/bin/bash

echo "@@@ run me from dir of slam/pcl-sac/build"

# Example directory containing _vfh.pcd files
DATA=../data

# Inlier distance threshold
thresh=50

# Get the closest K nearest neighbors
k=16

for i in `find $DATA -type d -name "*"`
do
  echo $i
  for j in `find $i -type f \( -iname "*cluster*_vfh.pcd" \) | gsort -R`
  do
    echo $j
    ./nearest_neighbors -k $k -thresh $thresh $j -cam "0.403137,0.868471/0,0,0/-0.0932051,-0.201608,-0.518939/-0.00471487,-0.931831,0.362863/1464,764/6,72"
  done
done


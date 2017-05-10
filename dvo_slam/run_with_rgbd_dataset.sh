#!/bin/sh

echo usage: $0 [dataset=rgbd_dataset]

echo 'do this in .bashrc: source /opt/ros/fuerte/setup.bash'

export ROS_PACKAGE_PATH=$ROS_PACKAGE_PATH:.
env | grep ROS

# DATA_FF should be a full path, otherwise, it fails at run time
##DATA_FF=`pwd`/../recorder/data-fastfusion-tum
DATA_FF=`pwd`/../OpenNI2-master-2.2.0.33/Bin/x64-Release/data-fastfusion-tum
DATASET_DIR=rgbd_dataset
if [ $# -eq 1 ]; then
    $DATASET_DIR=$1
fi
DATASET_PATH=$DATA_FF/$DATASET_DIR
echo this programs will generate assoc_opt_traj_final.txt and associate.txt for FastFusion.
echo DATASET_PATH: $DATASET_PATH
echo press enter to continue:
read key

roslaunch --local dvo_benchmark benchmark.launch dataset:=$DATA_FF/$DATASET_DIR 

echo generating associate.txt using traj...
associate.py $DATASET_PATH/assoc_opt_traj_final.txt $DATASET_PATH/depth.txt > tmp.txt
associate.py tmp.txt $DATASET_PATH/rgb.txt > $DATASET_PATH/associate.txt
rm -f tmp.txt





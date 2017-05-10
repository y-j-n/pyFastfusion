#!/bin/bash 

##DATASET=../data-fastfusion-tum/rgbd_dataset
DATASET=../data-fastfusion-tum/rgbd_dataset_freiburg3_long_office_household
if [ $# != 1 ]; then
    if [ ! -d $DATASET ]; then
        echo "Usage: $0 <rgbd_dataset_dir>"
        exit 1
    fi
else
    DATASET=$1
fi


ASSOC_FF=$DATASET/associate.txt
if [ ! -f $ASSOC_FF ]; then
    echo "$ASSOC_FF not found, generating it..."
    associate.py $DATASET/assoc_opt_traj_final.txt $DATASET/depth.txt > tmp.txt
    associate.py tmp.txt $DATASET/rgb.txt > $ASSOC_FF
    echo done.
else
    echo "$ASSOC_FF found."
fi

# for 'read -p' to work, need #!/bin/bash
##read -p "Press ENTER to continue"

rm -f .qglviewer.xml  # avoid displaying problem!!!!

#### ./bin/onlinefusion $DATASET/associate.txt --thread-fusion  # default
OPT_THREAD_FUSION='--thread-fusion'
#OPT_THREAD_FUSION=''
echo @@@ OPT_THREAD_FUSION: $OPT_THREAD_FUSION
#./bin/onlinefusion $DATASET/associate.txt $OPT_THREAD_FUSION --thread-image 
./bin/onlinefusion $DATASET/associate.txt $OPT_THREAD_FUSION


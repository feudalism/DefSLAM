# echo Recompiling DefSLAM...
# cd ../build
# cmake .. -DCMAKE_BUILD_TYPE=Release
# make -j 6
# cd ../Apps
# echo

orb_voc=/home/user3/slam/DefSLAM/Vocabulary/ORBvoc.txt
yaml=/home/user3/slam/datasets/defslam-f5phantom/hamlyn.yaml
vid=/home/user3/slam/datasets/defslam-f5phantom/f5_dynamic_deint_L.avi

## DefSLAM
# Process a sequence
# Usage: ./DefSLAM ORBvocabulary calibrationFile video
# or     ./DefSLAM ORBvocabulary calibrationFile

# echo '    Run camera'
# camera=/home/user3/slam/DefSLAM/calibration_files/logitechc922.yaml
# echo ./DefSLAM $orb_voc $camera
# echo '    Running ...'
# echo
# ./DefSLAM $orb_voc $camera
# # Outputs:
# # [ WARN:0] global /home/user3/slam/opencv-4.4.0/modules/videoio/src/cap_v4l.cpp (893) open VIDEOIO(V4L2:/dev/video0): can't open camera by index

echo Starting DefSLAM...
echo '    Run one video without ground truth'
echo ./DefSLAM $orb_voc $yaml $vid
echo '    Running ...'
echo
/home/user3/slam/DefSLAM/Apps/DefSLAM $orb_voc $yaml $vid

# ### DefSLAMGTCT
# # Sequences with depth image for ground truth. (Used for CT phantom dataset)
# # Usage: ./DefSLAMCTGT ORBvocabulary calibrationFile video CTfiles(heartDepthMap_ from hamlyn)
# echo Sequences with depth image for ground truth

# depthmap=/home/user3/slam/defslam-f5phantom/f5/heartDepthMap_
# echo ./DefSLAMGTCT $orb_voc $yaml $vid $depthmap
# ./DefSLAMGTCT $orb_voc $yaml $vid $depthmap

# ### DefSLAMGT
# # Sequences with stereo for ground truth.
# # Usage: ./stereo_groundtruth path_to_vocabulary path_to_settings path_to_left_folder path_to_right_folder path_to_times_file

# yaml=/home/user3/slam/defslam-md0/stereo0.yaml
# timestamps=/home/user3/slam/defSLAM-md0/Mandala0/timestamps/timestamps.txt
# img_left=/home/user3/slam/defSLAM-md0/Mandala0/images
# img_right=/home/user3/slam/defSLAM-md0/Mandala0/images
# # ./DefSLAMGT $orb_voc $yaml $img_left $img_right $timestamps
# echo Sequences with stereo for ground truth
# echo s. stereo_groundtruth.cc
# ./DefSLAMGT $orb_voc $yaml $img_left $img_right $timestamps
# echo done

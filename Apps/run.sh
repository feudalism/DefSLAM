
orb_voc=/home/user3/slam/DefSLAM/Vocabulary/ORBvoc.txt

## DATASETS
# Hamlyn
yaml=/home/user3/slam/datasets/defslam-f5phantom/hamlyn.yaml
vid=/home/user3/slam/datasets/defslam-f5phantom/f5_dynamic_deint_L.avi

# # Mandala
# yaml=/home/user3/slam/datasets/mandala0/stereo0.yaml
# vid=/home/user3/slam/datasets/mandala0/images


## DefSLAM
# Process a sequence
# Usage: ./DefSLAM ORBvocabulary calibrationFile video
# or     ./DefSLAM ORBvocabulary calibrationFile

echo Starting DefSLAM...
echo '    Run one video without ground truth'
echo ./DefSLAM $orb_voc $yaml $vid
echo '    Running ...'
echo
./DefSLAM $orb_voc $yaml $vid

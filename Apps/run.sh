### Args for DefSLAM ###
orb_voc=/home/user3/slam/DefSLAM/Vocabulary/ORBvoc.txt

## Datasets
# # Hamlyn
# yaml=/home/user3/slam/datasets/defslam-f5phantom/hamlyn.yaml
# vid=/home/user3/slam/datasets/defslam-f5phantom/f5_dynamic_deint_L.avi

# # Mandala
yaml=/home/user3/slam/datasets/mandala0/stereo0_2.yaml
imgs=/home/user3/slam/datasets/mandala0/images
ts="/home/user3/slam/datasets/mandala0/timestamps/timestamps_copy.txt"


### Run DefSLAM ###
# Process a sequence
# Usage: ./DefSLAM ORBvocabulary calibrationFile video
echo Starting DefSLAM...
echo $orb_voc $yaml $imgs $ts
echo

## Run type
# # Debug
# gdb --command=./debug.sh --args ./DefSLAM $orb_voc $yaml $imgs $ts
gdb --command=./debug.sh --args ./DefSLAMGT $orb_voc $yaml $imgs $imgs $ts

# # Normal run
# ./DefSLAM $orb_voc $yaml $imgs $ts
# ./DefSLAMGT $orb_voc $yaml $imgs $imgs $ts

# # Valgrind
# valgrind --leak-check=yes --log-file="valgrindoutput.txt" ./DefSLAM $orb_voc $yaml $vid $ts


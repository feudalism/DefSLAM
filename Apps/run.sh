### Args for DefSLAM ###
orb_voc=~/git/DefSLAM/Vocabulary/ORBvoc.txt

## Datasets
# # Hamlyn
# yaml=~/datasets/f5phantom/hamlyn.yaml
# vid=~/datasets/f5phantom/f5_dynamic_deint_L.avi

# # Mandala
# yaml=~/datasets/mandala0/stereo0_2.yaml
# imgs=~/datasets/mandala0/images
# ts=~/datasets/mandala0/timestamps/timestamps_copy.txt

# # GRK
yaml=~/datasets/grk/endo.yaml
vid=~/datasets/grk/bubblewrap.webm

### Run DefSLAM ###
# Process a sequence
# Usage: ./DefSLAM ORBvocabulary calibrationFile video
echo Starting DefSLAM...
echo $orb_voc $yaml $vid
# echo $orb_voc $yaml $imgs $ts
echo

## Run type
# # Debug
gdb --command=./debug.sh --args ./DefSLAM $orb_voc $yaml
#gdb --command=./debug.sh --args ./DefSLAM $orb_voc $yaml $vid
# gdb --command=./debug.sh --args ./DefSLAMVI $orb_voc $yaml $imgs $ts
# gdb --command=./debug.sh --args ./DefSLAMGT $orb_voc $yaml $imgs $imgs $ts

# # Normal run
# ./DefSLAM $orb_voc $yaml
# ./DefSLAM $orb_voc $yaml $vid
# ./DefSLAMVI $orb_voc $yaml $imgs $ts
# ./DefSLAMGT $orb_voc $yaml $imgs $imgs $ts

# # Valgrind
# valgrind --leak-check=yes --log-file="valgrindoutput.txt" ./DefSLAM $orb_voc $yaml $vid $ts

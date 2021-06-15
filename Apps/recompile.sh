echo Recompiling DefSLAM...
cd ../build
cmake .. -DCMAKE_BUILD_TYPE=Debug
make -j 6
cd ../Apps
echo

echo Recompiling DefSLAM...
cd ../build
cmake .. -DCMAKE_BUILD_TYPE=Debug
make -j 4
cd ../Apps

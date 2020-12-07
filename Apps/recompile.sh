echo Recompiling DefSLAM...
cd ../build
cmake .. -DCMAKE_BUILD_TYPE=Release
make -j 6
cd ../Apps
echo Recompiling DefSLAM...
cd ../build
cmake .. -DCMAKE_BUILD_TYPE=Debug -DBUILD_SHARED_LIBS=ON
make -j 4
cd ../Apps
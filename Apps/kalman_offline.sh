echo "Compiling"

if g++ kalman_offline.cpp -o kalman_offline -I /usr/local/include/opencv4 -lopencv_core -lopencv_video -g ; then
    echo "Running"
    ./kalman_offline
    # gdb ./kalman_offline
else
    echo "Run failed."
fi

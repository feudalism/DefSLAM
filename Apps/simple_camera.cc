#include <opencv2/core/core.hpp>
#include <System.h>

int main(int argc, char **argv)
{
    // variable declarations for the arguments
    string orbVocab = argv[1];  
    string calibFile = argv[2];    
    string videoFile = argv[3];
    
	// prints the number of arguments
    std::cout << argc << std::endl;
    
	// prints usage if wrong number of arguments
    if ((argc != 4) and (argc != 3))
    {
        cerr << endl
             << "Usage: ./DefSLAM ORBvocabulary calibrationFile video" << endl
             << " or    ./DefSLAM ORBvocabulary calibrationFile  " << endl
             << endl;
        return 1;
    }

	// VideoCapture: class for video capturing (from video files, image seqs, cameras)
	// initialise VideoCapture
    cv::VideoCapture cap;
	
    if (argc == 3)
    {
		// open the default camera using the default API, for video capturing
        std::cout << "Opening the default camera..." << std::endl;
        cap.open(0);
    }
    else
    {
        // video file / capturing device / video stream
        std::cout << "Opening video file: " << videoFile << std::endl;
        cap.open(videoFile);
    }

    // check if we succeeded in initialising video capturing
    if (!cap.isOpened()) 
    {
        std::cout << "Failed to open camera/video." << std::endl;
        return -1;
    }
    else
        std::cout << "Camera/video opened successfully." << std::endl;


    // Create SLAM system. It initializes all system threads (local mapping, loop closing, viewer)
    // and gets ready to process frames.
    // args: ORB vocab, calibration file, use viewer
    defSLAM::System SLAM(orbVocab, calibFile, true);

    uint i(0);
    cap.set(cv::CAP_PROP_FRAME_WIDTH, 640);
    cap.set(cv::CAP_PROP_FRAME_HEIGHT, 480);

    while (cap.isOpened())
    {
        // gets the capture as a matrix
        cv::Mat imLeft;
        cap >> imLeft;

        if (imLeft.empty())
        {
            cerr << endl
                 << "Failed to load image at: " << to_string(i) << endl;
            return 1;
        }

        SLAM.TrackMonocular(imLeft, i);
        i++;
    }

    SLAM.Shutdown();

    return 0;
}

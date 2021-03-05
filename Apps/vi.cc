#include<fstream>

#include <opencv2/core/core.hpp>
#include <System.h>

void LoadMandalaImgs(const string &strImagePath, const string &strPathTimes,
                vector<string> &vstrImageFilenames, vector<double> &vTimeStamps);

int main(int argc, char **argv)
{
    // variable declarations for the arguments
    string orbVocab = argv[1];  
    string calibFile = argv[2];    
    string imgFolder = argv[3];   
    string tsCamFile = argv[4];
    
	// prints usage if wrong number of arguments
    if (argc != 5)
    {
        cerr << endl
             << "Usage: ./DefSLAM ORBvocabulary calibrationFile imgFolder timestampsFile" << endl
             << endl;
        return 1;
    }
    
    // initialise data containers, load data
    vector<string> vstrImageFilenames;
    vector<double> vTimeStampsCam;

    cout << "Loading images..." << endl;
    LoadMandalaImgs(imgFolder, tsCamFile, vstrImageFilenames, vTimeStampsCam);
    cout << "Loaded images!" << endl;

    const int nImages = vstrImageFilenames.size();
    
    // Create SLAM system. It initializes all system threads (local mapping, loop closing, viewer)
    // and gets ready to process frames.
    // args: ORB vocab, calibration file, use viewer
    defSLAM::System SLAM(orbVocab, calibFile, false);
    
    // file for saving trajectory
    ofstream f;
    f.open("./trajectory.txt");
    f << fixed;
    cout << endl << "Saving camera trajectory to trajectory.txt" << endl;

    cv::Mat im;
    size_t start = 200;
    
    for(int ni=start; ni<nImages; ni++)
    {
        std::cout << vstrImageFilenames[ni] << " i:  " << ni << std::endl;
        
        // Read image from file
        im = cv::imread(vstrImageFilenames[ni],cv::IMREAD_UNCHANGED);
        if(im.empty())
        {
            cerr << endl << "Failed to load image at: "
                 <<  vstrImageFilenames[ni] << endl;
            return 1;
        }
        
        double tframe = vTimeStampsCam[ni];

        // Pass the image to the SLAM system
        SLAM.TrackMonocular(im,ni);
        SLAM.SaveTrajectory(f);
    }

    SLAM.Shutdown();
    f.close();

    return 0;
}

void LoadMandalaImgs(const string &strImagePath, const string &strPathTimes,
                vector<string> &vstrImageFilenames, vector<double> &vTimeStamps)
{
    ifstream fTimes;
    fTimes.open(strPathTimes.c_str());
    vTimeStamps.reserve(5000);
    vstrImageFilenames.reserve(5000);
    
    while(!fTimes.eof())
    {
        string s;
        getline(fTimes,s);
        
        if(!s.empty())
        {
            stringstream ss;
            ss << s;
            
            double t;
            ss >> t;
            vTimeStamps.push_back(t * 1e-6);
            
            // image
            string filename = ss.str();
            filename.erase(filename.length() - 6);            
            vstrImageFilenames.push_back(strImagePath + "/stereo_im_l_" + filename + ".png");

        }
    }
}

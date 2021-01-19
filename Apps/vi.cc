#include <opencv2/core/core.hpp>
#include <System.h>
#include "ImuTypes.h"

void LoadMandalaImgs(const string &strImagePath, const string &strPathTimes,
                vector<string> &vstrImageFilenames, vector<double> &vTimeStamps);
                
void LoadIMU(const string &strImuPath, vector<double> &vTimeStamps,
            vector<cv::Point3f> &vAcc, vector<cv::Point3f> &vGyro);


int main(int argc, char **argv)
{
    // variable declarations for the arguments
    string orbVocab = argv[1];  
    string calibFile = argv[2];    
    string imgFolder = argv[3];   
    string imuFile = argv[4];   
    string tsCamFile = argv[5];
    
	// prints usage if wrong number of arguments
    if (argc != 6)
    {
        cerr << endl
             << "Usage: ./DefSLAM ORBvocabulary calibrationFile imgFolder imuFile timestampsImgFile" << endl
             << endl;
        return 1;
    }
    
    // initialise data containers, load data
    vector<string> vstrImageFilenames;
    vector<double> vTimestampsCam;
    vector<double> vTimestampsImu;
    vector<cv::Point3f> vAcc, vGyro;
    int firstImu = 0;

    cout << "Loading images...";
    LoadMandalaImgs(imgFolder, tsCamFile, vstrImageFilenames, vTimestampsCam);
    cout << "Loaded images!" << endl;
    
    cout << "Loading IMU data... ";
    LoadIMU(imuFile, vTimestampsImu, vAcc, vGyro);
    cout << "Loaded IMU!" << endl;

    const int nImages = vstrImageFilenames.size();
    const int nImu = vTimestampsImu.size();

    if((nImages<=0)||(nImu<=0))
    {
        cerr << "ERROR: Failed to load images or IMU." << endl;
        return 1;
    }
    
    // Find first imu to be considered, supposing imu measurements start first
    while(vTimestampsImu[firstImu]<=vTimestampsCam[0])
        firstImu++;
    firstImu--;
    
    // Create SLAM system. It initializes all system threads (local mapping, loop closing, viewer)
    // and gets ready to process frames.
    // args: ORB vocab, calibration file, use viewer
    defSLAM::System SLAM(orbVocab, calibFile, defSLAM::System::IMU_MONOCULAR, true);

    cv::Mat im;
    vector<ORB_SLAM3::IMU::Point> vImuMeas;
    for(int ni=0; ni<nImages; ni++)
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
        double tframe = vTimestampsCam[ni];
        
        // Load imu measurements from previous frame
        vImuMeas.clear();
        while(vTimestampsImu[firstImu]<=vTimestampsCam[ni])
        {
            vImuMeas.push_back(ORB_SLAM3::IMU::Point(vAcc[firstImu].x,vAcc[firstImu].y,vAcc[firstImu].z,
                                                     vGyro[firstImu].x,vGyro[firstImu].y,vGyro[firstImu].z,
                                                     vTimestampsImu[firstImu]));
            firstImu++;
        }

        // Pass the image to the SLAM system
        SLAM.TrackMonocularIMU(im, tframe, vImuMeas);
    }

    SLAM.Shutdown();

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

void LoadIMU(const string &strImuPath, vector<double> &vTimeStamps,
            vector<cv::Point3f> &vAcc, vector<cv::Point3f> &vGyro)
{
    ifstream fImu;
    fImu.open(strImuPath.c_str());
    vTimeStamps.reserve(5000);
    vAcc.reserve(5000);
    vGyro.reserve(5000);

    while(!fImu.eof())
    {
        string s;
        getline(fImu,s);

        if(!s.empty())
        {
            string item;
            size_t pos = 0;
            double data[7];
            int count = 0;
            
            while ((pos = s.find(',')) != string::npos) {
                item = s.substr(0, pos);
                data[count++] = stod(item);
                s.erase(0, pos + 1);
            }
            item = s.substr(0, pos);
            data[6] = stod(item);

            vTimeStamps.push_back(data[0]);
            vAcc.push_back(cv::Point3f(data[4],data[5],data[6]));
            vGyro.push_back(cv::Point3f(data[1],data[2],data[3]));
        }
    }
}

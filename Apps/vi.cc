#include<fstream>

#include <opencv2/core/core.hpp>
#include <System.h>

#include <string.h> 
#include <unistd.h> 

void LoadMandalaImgs(const string &strImagePath, const string &strPathTimes,
                vector<string> &vstrImageFilenames, vector<double> &vTimeStamps);

void readGTData(const string &strGTTrajPath, vector<double> &vTimeStampsGT,
                vector<double> &vX, vector<double> &vY, vector<double> &vZ,
                vector<double> &vq1, vector<double> &vq2, vector<double> &vq3, vector<double> &vq4);

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
                
    // initialise data containers
    string strGTTrajPath = "/home/user3/slam/DefSLAM/Apps/traj_mandala0_gt.txt";
    vector<string> vstrImageFilenames;
    vector<double> vTimeStampsCam;
    vector<double> vTimeStampsGT;
    vector<double> vX, vY, vZ, vqx, vqy, vqz, vqw;
    
    // load GT data
    std::cout << "Loading GT data... ";
    readGTData(strGTTrajPath, vTimeStampsGT, vX,  vY, vZ, vqx, vqy, vqz, vqw);
    std::cout << "Loaded GT data!" << std::endl;

    std::cout << "Loading images...";
    LoadMandalaImgs(imgFolder, tsCamFile, vstrImageFilenames, vTimeStampsCam);
    std::cout << "Loaded images!" << std::endl;

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
        
        // Force update trajectory
        if((ni >= 230) && (ni < 240))
        {
            std::cout << "------------- --------------" << std::endl;
            int idx = ni - start;
            double x = vX[idx];
            double y = vY[idx];
            double z = vZ[idx];
            double qx = vqx[idx];
            double qy = vqy[idx];
            double qz = vqz[idx];
            double qw = vqw[idx];

            SLAM.updateTrajectory(x, y, z, qx, qy, qz, qw);
        }
        
        // Save trajectory to text file
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

void readGTData(const string &strGTTrajPath, vector<double> &vTimeStampsGT,
                vector<double> &vX, vector<double> &vY, vector<double> &vZ,
                vector<double> &vqx, vector<double> &vqy, vector<double> &vqz, vector<double> &vqw)
{
    ifstream fTraj;
    fTraj.open(strGTTrajPath.c_str());
    
    while(!fTraj.eof())
    {
        string s;
        getline(fTraj,s);
        
        if(!s.empty())
        {
            string item;
            size_t pos = 0;
            double data[8];
            int count = 0;

            while ((pos = s.find(' ')) != string::npos) {
                item = s.substr(0, pos);
                data[count++] = stod(item);
                s.erase(0, pos + 1);
            }
            item = s.substr(0, pos);
            data[7] = stod(item);

            vTimeStampsGT.push_back(data[0]);
            vX.push_back(data[1]);
            vY.push_back(data[2]);
            vZ.push_back(data[3]);
            vqx.push_back(data[4]);
            vqy.push_back(data[5]);
            vqz.push_back(data[6]);
            vqw.push_back(data[7]);
        }
        
    }
    
    fTraj.close();
    
}
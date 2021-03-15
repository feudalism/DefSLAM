#include<fstream>

#include <opencv2/core/core.hpp>
#include <System.h>

#include <string.h> 
#include <unistd.h> 

#include <opencv2/video/video.hpp>
#include <random>

void LoadMandalaImgs(const string &strImagePath, const string &strPathTimes,
                vector<string> &vstrImageFilenames, vector<double> &vTimeStamps);

void readGTData(const string &strGTTrajPath, vector<double> &vTimeStampsGT,
                vector<double> &vX, vector<double> &vY, vector<double> &vZ,
                vector<double> &vq1, vector<double> &vq2, vector<double> &vq3, vector<double> &vq4);

void loadIMU(const std::string &strImuPath,
                std::vector<float> &vTimeStampsIMU,
                std::vector<float> &vX, std::vector<float> &vY, std::vector<float> &vZ,
                std::vector<float> &vq1, std::vector<float> &vq2, std::vector<float> &vq3, std::vector<float> &vq4);
                
cv::KalmanFilter createKF();

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
    string strGTTrajPath = "traj_mandala0_gt_noisy.txt";
    vector<string> vstrImageFilenames;
    vector<double> vTimeStampsCam;
    vector<double> vTimeStampsGT;
    vector<double> vX, vY, vZ, vqx, vqy, vqz, vqw;

    // load noisy imu data
    std::string strImuPath = "traj_mandala0_gt_imu_noisy.txt";
    std::vector<float> vTimeStampsIMU;
    std::vector<float> vxmeas, vymeas, vzmeas, vqxmeas, vqymeas, vqzmeas, vqwmeas;
    loadIMU(strImuPath, vTimeStampsIMU,
            vxmeas,  vymeas, vzmeas, vqxmeas, vqymeas, vqzmeas, vqwmeas);

    // kalman filter
    cv::KalmanFilter kf = createKF();
    
    // load GT data
    std::cout << "Loading GT data... ";
    readGTData(strGTTrajPath, vTimeStampsGT, vX,  vY, vZ, vqx, vqy, vqz, vqw);
    std::cout << "Loaded GT data!" << std::endl;

    std::cout << "Loading images...";
    LoadMandalaImgs(imgFolder, tsCamFile, vstrImageFilenames, vTimeStampsCam);
    std::cout << "Loaded images!" << std::endl;

    const int nImages = vstrImageFilenames.size();
    
    // Kalman Filter
    int stateSize = 3;
    int measSize = 3;
    int contrSize = 3;
    cv::KalmanFilter kf(stateSize, measSize, contrSize, CV_32F);

    cv::Mat state(stateSize, 1, CV_32F);  // [x,y,z]
    cv::Mat meas(measSize, 1, CV_32F);    // [zx, zy, zz]
    cv::Mat control(contrSize, 1, CV_32F);    // [ux, uy, uz]

    cv::setIdentity(kf.transitionMatrix);
    cv::setIdentity(kf.controlMatrix);
    cv::setIdentity(kf.measurementMatrix);

    const double mean = 0.0;
    const double stddev = 0.05;
    std::default_random_engine generator;
    std::normal_distribution<double> distribution(mean, stddev);

    // kf.processNoiseCov.at<float>(0) = distribution(generator);
    // kf.processNoiseCov.at<float>(4) = distribution(generator);
    // kf.processNoiseCov.at<float>(8) = distribution(generator);
    // std::cout << kf.processNoiseCov << std::endl;

    setIdentity(kf.processNoiseCov, cv::Scalar(0.1));
    setIdentity(kf.measurementNoiseCov, cv::Scalar(0.05));

    // Create SLAM system. It initializes all system threads (local mapping, loop closing, viewer)
    // and gets ready to process frames.
    // args: ORB vocab, calibration file, use viewer
    defSLAM::System SLAM(orbVocab, calibFile, false);
    
    // file for saving trajectory
    ofstream f;
    ofstream f_rw;
    f.open("./trajectory.txt");
    f_rw.open("./trajectory_rw.txt");
    f << fixed;
    f_rw << fixed;

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
        if((ni >= 220) && (ni < 240))
        {
            std::cout << "------------- --------------" << std::endl;
        
            // Initialise Kalman filter
            if(ni == 220)
            {
                std::cout << "Kalman initial state (at frame " << ni << ") : " << std::endl;
                kf.statePre = SLAM.getCoordinates();
                std::cout << kf.statePre << std::endl;
            }

            // predict
            control.at<float>(0) = distribution(generator);
            control.at<float>(1) = distribution(generator);
            control.at<float>(2) = distribution(generator);
            state = kf.predict(control);
            std::cout << "ni: " << ni << " predicted" << std::endl;
            std::cout << state << std::endl;
            SLAM.savePredictedTrajectory(f_rw, state);

            // measure
            int idx = ni - start;
            meas.at<float>(0) = vX[idx];
            meas.at<float>(1) = vY[idx];
            meas.at<float>(2) = vZ[idx];
            kf.correct(meas);
            std::cout << "ni: " << ni << " measured" << std::endl;
            std::cout << meas << std::endl;

            double qx = vqx[idx];
            double qy = vqy[idx];
            double qz = vqz[idx];
            double qw = vqw[idx];

            SLAM.updateTrajectory(kf.statePost.at<float>(0), kf.statePost.at<float>(1),
                    kf.statePost.at<float>(2),
                    qx, qy, qz, qw);
        }
        
        // Save trajectory to text file
        SLAM.SaveTrajectory(f);
    }

    SLAM.Shutdown();
    f.close();
    f_rw.close();

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

void loadIMU(const std::string &strGTTrajPath, std::vector<float> &vTimeStampsIMU,
                std::vector<float> &vxmeas, std::vector<float> &vymeas, std::vector<float> &vzmeas,
                std::vector<float> &vqxmeas, std::vector<float> &vqymeas, std::vector<float> &vqzmeas, std::vector<float> &vqwmeas)
{
    std::ifstream fTraj;
    fTraj.open(strGTTrajPath.c_str());

    while(!fTraj.eof())
    {
        std::string s;
        getline(fTraj,s);

        if(!s.empty())
        {
            std::string item;
            size_t pos = 0;
            double data[8];
            int count = 0;

            while ((pos = s.find(' ')) != std::string::npos) {
                item = s.substr(0, pos);
                data[count++] = stod(item);
                s.erase(0, pos + 1);
            }
            item = s.substr(0, pos);
            data[7] = stod(item);

            vTimeStampsIMU.push_back(data[0]);
            vxmeas.push_back(data[1]);
            vymeas.push_back(data[2]);
            vzmeas.push_back(data[3]);
            vqxmeas.push_back(data[4]);
            vqymeas.push_back(data[5]);
            vqzmeas.push_back(data[6]);
            vqwmeas.push_back(data[7]);
        }

    }

    fTraj.close();

}

cv::KalmanFilter createKF()
{
    int stateSize = 3;
    int measSize = 3;
    int controlSize = 3;
    cv::KalmanFilter kf(stateSize, measSize, controlSize, CV_32F);

    cv::setIdentity(kf.transitionMatrix);
    cv::setIdentity(kf.controlMatrix);
    cv::setIdentity(kf.measurementMatrix);

    return kf;
}
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
                
void saveTrajectory(std::ofstream &f, const float &t, const float &x, const float &y, const float &z,
        const float &q1, const float &q2, const float &q3, const float &q4);

size_t loadImuQueue(std::vector<float> &vImuQueue, 
        size_t firstImuIndex,
        std::vector<float> &vTimeStampsImu,
        const float &currentFrameTs, std::vector<float> &vMeasurement);

cv::KalmanFilter createKF();
void initialiseKF(cv::KalmanFilter &kf);
void setPostValsToOptimVals(cv::KalmanFilter &kf, const cv::Mat &state);

std::ofstream openFile(const std::string &filepath);

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
                
    // load GT data
    // string strGTTrajPath = "traj_mandala0_gt_noisy.txt";
    // vector<double> vTimeStampsGT;
    vector<string> vstrImageFilenames;
    vector<double> vTimeStampsMono;
    // vector<double> vX, vY, vZ, vqx, vqy, vqz, vqw;
    // std::cout << "Loading GT data... ";
    // readGTData(strGTTrajPath, vTimeStampsGT, vX,  vY, vZ, vqx, vqy, vqz, vqw);
    // std::cout << "Loaded GT data!" << std::endl;

    // load noisy imu data
    std::string strImuPath = "traj_mandala0_gt_imu_noisy.txt";
    std::vector<float> vTimeStampsIMU;
    std::vector<float> vxmeas, vymeas, vzmeas, vqxmeas, vqymeas, vqzmeas, vqwmeas;
    loadIMU(strImuPath, vTimeStampsIMU, vxmeas,  vymeas, vzmeas, vqxmeas, vqymeas, vqzmeas, vqwmeas);

    std::cout << "Loading images...";
    LoadMandalaImgs(imgFolder, tsCamFile, vstrImageFilenames, vTimeStampsMono);
    std::cout << "Loaded images!" << std::endl;

    // kalman filter
    cv::KalmanFilter kf = createKF();

    // random walk
    const double mean = 0.0;
    const double stddev = 0.005;
    std::default_random_engine generator;
    std::normal_distribution<double> distribution(mean, stddev);

    // Create SLAM system. It initializes all system threads (local mapping, loop closing, viewer)
    // and gets ready to process frames.
    // args: ORB vocab, calibration file, use viewer
    defSLAM::System SLAM(orbVocab, calibFile, false);

    cv::Mat im;
    const size_t nImages = vstrImageFilenames.size();
    size_t start = 200;
    size_t kalman_start = 220;
    size_t kalman_end = 260;

    size_t stateSize = 3;
    size_t measSize = 3;
    size_t controlSize = 3;

    cv::Mat state(stateSize, 1, CV_32F);
    cv::Mat quats(stateSize, 1, CV_32F);
    cv::Mat statePost(stateSize, 1, CV_32F);
    cv::Mat processNoise(stateSize, 1, CV_32F);
    cv::Mat control(controlSize, 1, CV_32F);

    cv::Mat meas(measSize, 1, CV_32F);
    cv::Mat rw(stateSize, 1, CV_32F);

    std::vector<float> vImuXQueue, vImuYQueue, vImuZQueue;
    // find index of first IMU data that appears after or during the first frame
    int firstImuIndex = 0;
    int imuQueueStartIndex = 0;
    // while(vTimeStampsIMU[firstImuIndex] <= vTimeStampsMono[0])
      // firstImuIndex++;
    // firstImuIndex--;

    // file for saving trajectory
    std::ofstream f_mono = openFile("./trajectory.txt");
    std::ofstream f_kalman = openFile("./trajectory_kalman.txt");
    std::ofstream f_rw = openFile("./trajectory_rw.txt");
    
    for(int ni=start; ni<(nImages + start); ni++)
    {
        std::cout << vstrImageFilenames[ni] << std::endl
            << " i:  " << ni << std::endl;
        const int idx = ni - start;
        const float currentFrameTs = ni;

        // Read image from file
        im = cv::imread(vstrImageFilenames[ni],cv::IMREAD_UNCHANGED);
        if(im.empty())
        {
            cerr << endl << "Failed to load image at: "
                 <<  vstrImageFilenames[ni] << endl;
            return 1;
        }

        // Pass the image to the SLAM system
        SLAM.TrackMonocular(im,ni);
        state = SLAM.getCoordinates();
        std::vector<float> q = SLAM.getQuaternions();
        quats.at<float>(0) = q[0];
        quats.at<float>(1) = q[1];
        quats.at<float>(2) = q[2];
        quats.at<float>(3) = q[3];

        if(ni == (kalman_start - 1))
            initialiseKF(kf);

        setPostValsToOptimVals(kf, state);

        // load IMU measurements from prev fraem to current
        if(idx > 0)
        {
            imuQueueStartIndex = firstImuIndex;
            loadImuQueue(vImuXQueue, firstImuIndex,
                vTimeStampsIMU, currentFrameTs, vxmeas);
            loadImuQueue(vImuYQueue, firstImuIndex,
                vTimeStampsIMU, currentFrameTs, vymeas);
            firstImuIndex = loadImuQueue(vImuZQueue, firstImuIndex,
                vTimeStampsIMU, currentFrameTs, vzmeas);
        }
        
        // kalman implementation
        if((ni >= kalman_start) && (ni < kalman_end))
        {
            std::cout << "------------- KALMAN --------------" << std::endl;
            
            // imu queue from prev frame up till current
            for(size_t iimu=0; iimu<vImuXQueue.size(); iimu++)
            {
                const float currentImuTs = vTimeStampsIMU[imuQueueStartIndex + iimu];
                
                // predict next position from random walk model
                cv::randn(control, mean, stddev);
                kf.predict(control);

                saveTrajectory(f_rw, currentImuTs,
                    state.at<float>(0), state.at<float>(1), state.at<float>(2),
                    quats.at<float>(0), quats.at<float>(1), quats.at<float>(2), quats.at<float>(3));

                // correct prediction using measurements
                meas.at<float>(0) = vxmeas[imuQueueStartIndex + iimu];
                meas.at<float>(1) = vymeas[imuQueueStartIndex + iimu];
                meas.at<float>(2) = vzmeas[imuQueueStartIndex + iimu];
                kf.correct(meas);

                saveTrajectory(f_kalman, currentImuTs,
                    state.at<float>(0), state.at<float>(1), state.at<float>(2),
                    quats.at<float>(0), quats.at<float>(1), quats.at<float>(2), quats.at<float>(3));

                // SLAM.updateTrajectory(kf.statePost.at<float>(0), kf.statePost.at<float>(1),
                    // kf.statePost.at<float>(2),
                    // quats.at<float>(0), quats.at<float>(1), quats.at<float>(2), quats.at<float>(3));
                SLAM.updateCoordinates(kf.statePost.at<float>(0), kf.statePost.at<float>(1),
                    kf.statePost.at<float>(2));
            }
        }
        
        // Save trajectory to text file
        SLAM.SaveTrajectory(f_mono);
    }

    SLAM.Shutdown();
    f_mono.close();
    f_kalman.close();
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


void initialiseKF(cv::KalmanFilter &kf)
{
    setIdentity(kf.processNoiseCov, cv::Scalar(0.05));
    setIdentity(kf.measurementNoiseCov, cv::Scalar(0.05));
    setIdentity(kf.errorCovPost, cv::Scalar(1));
}

void setPostValsToOptimVals(cv::KalmanFilter &kf, const cv::Mat &state)
{
    kf.statePost = state;
    kf.errorCovPost = kf.transitionMatrix * kf.errorCovPost.t() * kf.transitionMatrix
            + kf.processNoiseCov;
}

std::ofstream openFile(const std::string &filepath)
{
    std::ofstream f;
    f.open(filepath.c_str());
    f << std::fixed;
    return f;
}

size_t loadImuQueue(std::vector<float> &vImuQueue, 
        size_t firstImuIndex,
        std::vector<float> &vTimeStampsImu,
        const float &currentFrameTs, std::vector<float> &vMeasurement)
{
    vImuQueue.clear();
    while(vTimeStampsImu[firstImuIndex] <= currentFrameTs)
    {
        vImuQueue.push_back(vMeasurement[firstImuIndex]);

        // prevent out of bound indexing when reaching end of IMU input
        if(firstImuIndex < vTimeStampsImu.size())
            firstImuIndex++;
        else
            return firstImuIndex;
    }

    return firstImuIndex;

}

void saveTrajectory(std::ofstream &f, const float &t, const float &x, const float &y, const float &z,
        const float &q1, const float &q2, const float &q3, const float &q4)
{
    f <<
      // frame times
      std::setprecision(6) << t << " "
      // pose
      <<  std::setprecision(9) << x
                          << " " << y
                          << " " << z << " "
                          << q1 << " " << q2 << " " << q3 << " " << q4
      <<std::endl;
}


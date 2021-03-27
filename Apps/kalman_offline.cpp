#include <iostream>
#include <iomanip>
#include <fstream>
#include <string.h>
#include <vector>

#include <opencv2/video/video.hpp>
#include <random>

void loadTrajectory(const std::string &strGTTrajPath,
                std::vector<float> &vTimeStampsIMU,
                std::vector<float> &vX, std::vector<float> &vY, std::vector<float> &vZ,
                std::vector<float> &vq1, std::vector<float> &vq2, std::vector<float> &vq3, std::vector<float> &vq4);
void loadImuRaw(const std::string &sstrImuRawTrajPath,
                std::vector<float> &vTimeStampsImuRaw,
                std::vector<float> &vaxmeas,
                std::vector<float> &vaymeas,
                std::vector<float> &vazmeas,
                std::vector<float> &vgxmeas,
                std::vector<float> &vgymeas,
                std::vector<float> &vgzmeas);
            
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
    // load monocular path
    std::string strMonoTrajPath = "traj_mandala0_mono.txt";
    std::vector<float> vTimeStampsMono;
    std::vector<float> vxmono, vymono, vzmono, vqxmono, vqymono, vqzmono, vqwmono;
    loadTrajectory(strMonoTrajPath, vTimeStampsMono,
            vxmono,  vymono, vzmono, vqxmono, vqymono, vqzmono, vqwmono);

    // load noisy imu data
    std::string strGTTrajPath = "traj_mandala0_gt_imu_noisy.txt";
    std::vector<float> vTimeStampsIMU;
    std::vector<float> vxmeas, vymeas, vzmeas, vqxmeas, vqymeas, vqzmeas, vqwmeas;
    loadTrajectory(strGTTrajPath, vTimeStampsIMU,
            vxmeas,  vymeas, vzmeas, vqxmeas, vqymeas, vqzmeas, vqwmeas);

    // load noisy imu data
    std::string strImuRawTrajPath = "traj_mandala0_gt_imuraw_noisy.txt";
    std::vector<float> vTimeStampsImuRaw;
    std::vector<float> vaxmeas, vaymeas, vazmeas, vgxmeas, vgymeas, vgzmeas;
    loadImuRaw(strImuRawTrajPath, vTimeStampsImuRaw,
            vaxmeas, vaymeas, vazmeas, vgxmeas, vgymeas, vgzmeas);

    // kalman filter
    cv::KalmanFilter kf = createKF();

    // random walk
    const double mean = 0.0;
    const double stddev = 0.005;
    std::default_random_engine generator;
    std::normal_distribution<double> distribution(mean, stddev);

    // file for saving trajectory
    std::ofstream f = openFile("./trajectory_offline.txt");
    std::ofstream f_rw = openFile("./trajectory_offline_rw.txt");

    // loop
    const int nImages = vTimeStampsMono.size();
    size_t start = 200;
    size_t kalman_start = 220;
    size_t kalman_end = 310;

    size_t stateSize = 6;
    size_t measSize = 6;
    size_t controlSize = 6;

    cv::Mat state(stateSize, 1, CV_32F);
    cv::Mat statePost(stateSize, 1, CV_32F);
    cv::Mat processNoise(stateSize, 1, CV_32F);
    cv::Mat control(controlSize, 1, CV_32F);

    cv::Mat meas(measSize, 1, CV_32F);
    cv::Mat rw(stateSize, 1, CV_32F);

    std::vector<float> vImuXQueue, vImuYQueue, vImuZQueue;
    // find index of first IMU data that appears after or during the first frame
    int firstImuIndex = 0;
    int imuQueueStartIndex = 0;
    while(vTimeStampsIMU[firstImuIndex] <= vTimeStampsMono[0])
      firstImuIndex++;
    firstImuIndex--;

    for(size_t ni=start; ni<(nImages + start); ni++)
    {
        std::cout << "ni " << ni << " -- " << std::endl;
        const int idx = ni - start;
        const float currentFrameTs = vTimeStampsMono[idx];

        // monocular pose from optimisation
        state.at<float>(0) = vxmono[idx];
        state.at<float>(1) = vymono[idx];
        state.at<float>(2) = vzmono[idx];
        meas.at<float>(0) = vxmono[idx];
        meas.at<float>(1) = vymono[idx];
        meas.at<float>(2) = vzmono[idx];
        float qx = vqxmono[idx];
        float qy = vqymono[idx];
        float qz = vqzmono[idx];
        float qw = vqwmono[idx];

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
            // imu queue from prev frame up till current
            for(size_t iimu=0; iimu<vImuXQueue.size(); iimu++)
            {
                const float currentImuTs = vTimeStampsIMU[imuQueueStartIndex + iimu];

                // predict next position from random walk model
                cv::randn(control, mean, stddev);
                kf.predict(control);

                saveTrajectory(f_rw, currentImuTs,
                    state.at<float>(0), state.at<float>(1), state.at<float>(2),
                    qx, qy, qz, qw);

                // correct prediction using measurements
                meas.at<float>(3) = vgxmeas[imuQueueStartIndex + iimu];
                meas.at<float>(4) = vgymeas[imuQueueStartIndex + iimu];
                meas.at<float>(5) = vgzmeas[imuQueueStartIndex + iimu];
                kf.correct(meas);

                saveTrajectory(f, currentImuTs,
                    state.at<float>(0), state.at<float>(1), state.at<float>(2),
                    qx, qy, qz, qw);
            }
        }
    }

    f.close();
    f_rw.close();
}

void loadTrajectory(const std::string &strGTTrajPath, std::vector<float> &vTimeStampsIMU,
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

void loadImuRaw(const std::string &sstrImuRawTrajPath,
                std::vector<float> &vTimeStampsImuRaw,
                std::vector<float> &vaxmeas,
                std::vector<float> &vaymeas,
                std::vector<float> &vazmeas,
                std::vector<float> &vgxmeas,
                std::vector<float> &vgymeas,
                std::vector<float> &vgzmeas)
{
    std::ifstream fTraj;
    fTraj.open(sstrImuRawTrajPath.c_str());

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

            vTimeStampsImuRaw.push_back(data[0]);
            vaxmeas.push_back(data[1]);
            vaymeas.push_back(data[2]);
            vazmeas.push_back(data[3]);
            vgxmeas.push_back(data[4]);
            vgymeas.push_back(data[5]);
            vgzmeas.push_back(data[6]);
        }

    }

    fTraj.close();

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

cv::KalmanFilter createKF()
{
    int stateSize = 6;
    int measSize = 6;
    int controlSize = 6;
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
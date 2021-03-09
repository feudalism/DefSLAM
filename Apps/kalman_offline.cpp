#include <iostream>
#include <iomanip>
#include <fstream>
#include <string.h>
#include <vector>

#include <opencv2/video/video.hpp>
#include <random>

void loadTrajectory(const std::string &strGTTrajPath,
                std::vector<float> &vTimeStampsGT,
                std::vector<float> &vX, std::vector<float> &vY, std::vector<float> &vZ,
                std::vector<float> &vq1, std::vector<float> &vq2, std::vector<float> &vq3, std::vector<float> &vq4);

void saveTrajectory(std::ofstream &f, const float &t, const float &x, const float &y, const float &z,
        const float &q1, const float &q2, const float &q3, const float &q4);

cv::KalmanFilter createKF();
void initialiseKF(cv::KalmanFilter &kf, const cv::Mat state);

std::ofstream openFile(const std::string &filepath);

int main(int argc, char **argv)
{
    // load monocular path
    std::string strMonoTrajPath = "traj_mandala0_mono.txt";
    std::vector<float> vTimeStampsMono;
    std::vector<float> vxmono, vymono, vzmono, vqxmono, vqymono, vqzmono, vqwmono;
    loadTrajectory(strMonoTrajPath, vTimeStampsMono,
            vxmono,  vymono, vzmono, vqxmono, vqymono, vqzmono, vqwmono);

    // load noisy GT data
    std::string strGTTrajPath = "traj_mandala0_gt_noisy.txt";
    std::vector<float> vTimeStampsGT;
    std::vector<float> vxmeas, vymeas, vzmeas, vqxmeas, vqymeas, vqzmeas, vqwmeas;
    loadTrajectory(strGTTrajPath, vTimeStampsGT,
            vxmeas,  vymeas, vzmeas, vqxmeas, vqymeas, vqzmeas, vqwmeas);

    // kalman filter
    cv::KalmanFilter kf = createKF();

    // random walk
    const double mean = 0.0;
    const double stddev = 0.025;
    std::default_random_engine generator;
    std::normal_distribution<double> distribution(mean, stddev);

    // file for saving trajectory
    std::ofstream f = openFile("./trajectory_offline.txt");
    std::ofstream f_rw = openFile("./trajectory_offline_rw.txt");

    // loop
    const int nImages = vTimeStampsGT.size();
    size_t start = 200;
    size_t kalman_start = 220;
    size_t kalman_end = 250;

    size_t stateSize = 3;
    size_t measSize = 3;
    size_t controlSize = 3;

    cv::Mat state(stateSize, 1, CV_32F);
    cv::Mat statePost(stateSize, 1, CV_32F);
    cv::Mat processNoise(stateSize, 1, CV_32F);
    cv::Mat control(controlSize, 1, CV_32F);

    cv::Mat meas(measSize, 1, CV_32F);
    cv::Mat rw(stateSize, 1, CV_32F);

    for(int ni=start; ni<(nImages + start); ni++)
    {
        int idx = ni - start;

        // monocular coordinates
        state.at<float>(0) = vxmono[idx];
        state.at<float>(1) = vymono[idx];
        state.at<float>(2) = vzmono[idx];

        // monocular quaternions
        float qx = vqxmono[idx];
        float qy = vqymono[idx];
        float qz = vqzmono[idx];
        float qw = vqwmono[idx];

        kf.statePost = state;

        if(ni == (kalman_start - 1))
            initialiseKF(kf, state.clone());

        // kalman implementation
        if((ni >= kalman_start))// && (ni < kalman_end))
        {
            // predict
            // random walk          
            cv::randn(control, mean, stddev);

            // set coordinates from DefSLAM to be 
            rw = kf.predict(control);
            saveTrajectory(f_rw, ni,
                rw.at<float>(0), rw.at<float>(1), rw.at<float>(2),
                qx, qy, qz, qw);

            // correct
            // noisy stereo gt coordinates
            meas.at<float>(0) = vxmeas[idx];
            meas.at<float>(1) = vymeas[idx];
            meas.at<float>(2) = vzmeas[idx];
            kf.correct(meas);

        }

        saveTrajectory(f, ni, kf.statePost.at<float>(0), kf.statePost.at<float>(1), kf.statePost.at<float>(2),
            qx, qy, qz, qw);
    }

    f.close();
    f_rw.close();
}

void loadTrajectory(const std::string &strGTTrajPath, std::vector<float> &vTimeStampsGT,
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

            vTimeStampsGT.push_back(data[0]);
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
    int stateSize = 3;
    int measSize = 3;
    int controlSize = 3;
    cv::KalmanFilter kf(stateSize, measSize, controlSize, CV_32F);

    cv::setIdentity(kf.transitionMatrix);
    cv::setIdentity(kf.controlMatrix);
    cv::setIdentity(kf.measurementMatrix);

    return kf;
}

void initialiseKF(cv::KalmanFilter &kf, const cv::Mat state)
{
    setIdentity(kf.processNoiseCov, cv::Scalar(0.05));
    setIdentity(kf.measurementNoiseCov, cv::Scalar(0.05));

    kf.statePost = state; // same pointers
    setIdentity(kf.errorCovPre, cv::Scalar(1));
}

std::ofstream openFile(const std::string &filepath)
{
    std::ofstream f;
    f.open(filepath.c_str());
    f << std::fixed;
    return f;
}
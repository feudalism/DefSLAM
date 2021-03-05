#include<fstream>

#include <opencv2/core/core.hpp>
#include <System.h>

#include <sys/socket.h> 
#include <netinet/in.h> 
#include <string.h> 

#define PORT 8080 

void LoadMandalaImgs(const string &strImagePath, const string &strPathTimes,
                vector<string> &vstrImageFilenames, vector<double> &vTimeStamps);
                
void createSocket(int server_fd, struct sockaddr_in address);

int main(int argc, char **argv)
{
    int server_fd = socket(AF_INET, SOCK_STREAM, 0);
    struct sockaddr_in address; 
    
    createSocket(server_fd, address);
  
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

    cout << "Loading images...";
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
    
    // creates new connected socket from first connection request
    int accept_socket;
    int addrlen = sizeof(address); 
    if ((accept_socket = accept(server_fd, (struct sockaddr *)&address,  
                       (socklen_t*)&addrlen))<0) 
    { 
        perror("accept"); 
        exit(EXIT_FAILURE); 
    }
    
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
        
        // Transmit trajectory via socket
        std::string message = SLAM.getDataAsString(); 
        const char *message_c = message.c_str();
        
        std::cout << "Sending message..." << std::endl;
        send(accept_socket, message_c, strlen(message_c), 0);
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

void createSocket(int server_fd, struct sockaddr_in address)
{
    std::cout << "Creating socket..." << std::endl;
    
    // char buffer[1024] = {0}; 
       
    // Creating socket file descriptor 
    if (server_fd < 0) 
    { 
        perror("socket failed"); 
        exit(EXIT_FAILURE); 
    } 
       
    int opt = 1; 
    if (setsockopt(server_fd, SOL_SOCKET, SO_REUSEADDR | SO_REUSEPORT, 
                                                  &opt, sizeof(opt))) 
    { 
        perror("setsockopt"); 
        exit(EXIT_FAILURE); 
    } 
    address.sin_family = AF_INET; 
    address.sin_addr.s_addr = INADDR_ANY; 
    address.sin_port = htons( PORT ); 
       
    // Forcefully attaching socket to the port 8080 
    if (bind(server_fd, (struct sockaddr *)&address,  
                                 sizeof(address))<0) 
    { 
        perror("bind failed"); 
        exit(EXIT_FAILURE); 
    } 
    
    // puts server socket in passive mode: waits for client to talk to server
    if (listen(server_fd, 3) < 0) 
    { 
        perror("listen"); 
        exit(EXIT_FAILURE); 
    }
    
    return;
}

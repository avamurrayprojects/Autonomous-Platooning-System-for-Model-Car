#include <opencv2/opencv.hpp>
#include <opencv2/aruco.hpp>
#include <opencv2/video/tracking.hpp> // Kalman and other tracking algorithms
#include <iostream>
#include <deque>
#include <fcntl.h>  // file control operations
#include <unistd.h> // POSIX os API (r/w)
#include <sys/ioctl.h> // ioctl function
#include <termios.h>

// Configuration structure holds program parameters
struct Config {
    // Desired processing resolution (downscaled image size)
    int frameWidth = 320;
    int frameHeight = 240;
    double frameRate = 30.0;
   
    // ArUco marker dictionary
    int arucoDictId = cv::aruco::DICT_4X4_250;
   
    // Kalman filter parameters
    int kalmanStateSize = 4; // x, y, vx, vy
    int kalmanMeasSize  = 2; // measuring only x and y
    int kalmanContrSize = 0;
    unsigned int kalmanType = CV_32F;
    float processNoisePos = 1e-2;
    float processNoiseVel = 5.0f;
    float measNoisePos    = 1e-1;
    float errorCovPos     = 1.0f;
    float errorCovVel     = 1.0f;
   
    // Buffer size for smoothing detections
    int bufferSize = 1;
   
    // Image processing parameters
    cv::Size gaussianKernelSize = cv::Size(5, 5);
    double gaussianSigmaX = 0;
    int adaptiveThreshBlockSize = 11;
    double adaptiveThreshC = 2.0;
   
    // Virtual serial port parameters
    std::string uartDevice = "/dev/serial0";
    int baudRate = 115200;
    
    int noMarkerThreshold = 4; //amount of consecutive frames without detection
};

// Check if GUI mode is enabled via the "--gui" argument
bool is_gui_mode(int argc, char** argv) {
    for (int i = 1; i < argc; ++i) {
        if (std::string(argv[i]) == "--gui") {
            return true;
        }
    }
    return false;
}

int main(int argc, char** argv) {
    Config config;
    bool gui_enabled = is_gui_mode(argc, argv);

    // Open UART device (virtual serial port)
    int serial_fd = open(config.uartDevice.c_str(), O_RDWR | O_NOCTTY | O_NDELAY);
    if (serial_fd < 0) {
        std::cerr << "Error opening UART device: " << config.uartDevice << std::endl;
        return -1;
    }
   
    // Configure the serial port
    struct termios tty;
    if (tcgetattr(serial_fd, &tty) != 0) {
        std::cerr << "Error from tcgetattr\n";
        close(serial_fd);
        return -1;
    }
    cfsetispeed(&tty, B115200);
    cfsetospeed(&tty, B115200);
    tty.c_cflag &= ~PARENB;  // no parity
    tty.c_cflag &= ~CSTOPB;  // one stop bit
    tty.c_cflag &= ~CSIZE;
    tty.c_cflag |= CS8;      // 8 data bits
    tty.c_cflag &= ~CRTSCTS; // disable hardware flow control
    tty.c_cflag |= CREAD | CLOCAL; // enable receiver, local mode
    tty.c_lflag &= ~(ICANON | ECHO | ECHOE | ISIG); // raw input mode
    tty.c_oflag &= ~OPOST;   // raw output
    tty.c_iflag &= ~(IXON | IXOFF | IXANY); // disable software flow control
    if (tcsetattr(serial_fd, TCSANOW, &tty) != 0) {
        std::cerr << "Error from tcsetattr\n";
        close(serial_fd);
        return -1;
    }

    // Capture Setup
    // Use an open-ended pipeline that doesn't force a resolution.
    // This lets the camera output its native full resolution.
    std::string pipeline = "libcamerasrc ! videoconvert ! appsink";

    // Open camera using the pipeline string
    cv::VideoCapture cap(pipeline, cv::CAP_GSTREAMER);
    if (!cap.isOpened()) {
        std::cerr << "Could not open camera using pipeline:\n" << pipeline << std::endl;
        close(serial_fd);
        return -1;
    }
   
    cv::Mat fullFrame, frame, gray;
    double ticks = 0;
    double dt = 1.0 / config.frameRate;
   
    // Initialize ArUco detection and Kalman filter
    cv::Ptr<cv::aruco::Dictionary> dictionary =
        cv::aruco::getPredefinedDictionary(config.arucoDictId);

    cv::KalmanFilter kf(config.kalmanStateSize,
                        config.kalmanMeasSize,
                        config.kalmanContrSize,
                        config.kalmanType);
    cv::setIdentity(kf.transitionMatrix);
    kf.measurementMatrix = cv::Mat::zeros(config.kalmanMeasSize,
                                          config.kalmanStateSize,
                                          config.kalmanType);
    kf.measurementMatrix.at<float>(0, 0) = 1.0f;
    kf.measurementMatrix.at<float>(1, 1) = 1.0f;

    kf.processNoiseCov = cv::Mat::zeros(config.kalmanStateSize,
                                        config.kalmanStateSize,
                                        config.kalmanType);
    kf.processNoiseCov.at<float>(0, 0) = config.processNoisePos;
    kf.processNoiseCov.at<float>(1, 1) = config.processNoisePos;
    kf.processNoiseCov.at<float>(2, 2) = config.processNoiseVel;
    kf.processNoiseCov.at<float>(3, 3) = config.processNoiseVel;

    kf.measurementNoiseCov = cv::Mat::zeros(config.kalmanMeasSize,
                                            config.kalmanMeasSize,
                                            config.kalmanType);
    kf.measurementNoiseCov.at<float>(0, 0) = config.measNoisePos;
    kf.measurementNoiseCov.at<float>(1, 1) = config.measNoisePos;

    cv::Mat state(config.kalmanStateSize, 1, config.kalmanType, cv::Scalar(0));
    cv::Mat meas(config.kalmanMeasSize, 1, config.kalmanType, cv::Scalar(0));

    std::deque<cv::Point2f> buffer;
    bool found = false;
    
    int noMarkerFrames = 0;
    bool markerRecentlyVisible = true;

    // Main loop
    while (true) {
        // Capture the full resolution frame from the camera
        if (!cap.read(fullFrame) || fullFrame.empty()) {
            std::cerr << "Error reading frame\n";
            break;
        }
       
        // Downscale the full resolution image to the processing resolution.
        // This way, you preserve the full field of view.
        cv::resize(fullFrame, frame, cv::Size(config.frameWidth, config.frameHeight));

        double precTick = ticks;
        ticks = static_cast<double>(cv::getTickCount());
        double dT = (ticks - precTick) / cv::getTickFrequency();
        if (dT == 0) dT = dt; // fallback
        kf.transitionMatrix.at<float>(0, 2) = dT;
        kf.transitionMatrix.at<float>(1, 3) = dT;
        // test save
       
        cv::cvtColor(frame, gray, cv::COLOR_BGR2GRAY);
       
        // Detect ArUco markers in the downscaled (and grayscale) frame
        std::vector<int> ids;
        std::vector<std::vector<cv::Point2f>> corners;
        cv::aruco::detectMarkers(gray, dictionary, corners, ids);
       
        if (gui_enabled) {
            cv::imshow("Thresholded", gray);
            if (cv::waitKey(1) == 27) { // Exit on ESC key
                break;
            }
        }
       
        if (!ids.empty()) {
            cv::aruco::drawDetectedMarkers(frame, corners, ids);
            // Compute the center of the first detected marker
            cv::Point2f marker_center(0.f, 0.f);
            for (auto& corner : corners[0]) {
                marker_center += corner;
            }
            marker_center *= 0.25f; // average of the four corners
           
            meas.at<float>(0) = marker_center.x;
            meas.at<float>(1) = marker_center.y;
           
            if (!found) {
                kf.errorCovPre.at<float>(0, 0) = config.errorCovPos;
                kf.errorCovPre.at<float>(1, 1) = config.errorCovPos;
                kf.errorCovPre.at<float>(2, 2) = config.errorCovVel;
                kf.errorCovPre.at<float>(3, 3) = config.errorCovVel;
                state.at<float>(0) = marker_center.x;
                state.at<float>(1) = marker_center.y;
                state.at<float>(2) = 0.f;
                state.at<float>(3) = 0.f;
                kf.statePost = state;
                found = true;
            } else {
                kf.correct(meas);
            }
            buffer.push_back(marker_center);
            if ((int)buffer.size() > config.bufferSize) {
                buffer.pop_front();
            }
        } else {
            if (found) {
                kf.predict();
            }
            if (!buffer.empty()) {
                cv::Point2f avg_point(0.f, 0.f);
                for (auto& pt : buffer) {
                    avg_point += pt;
                }
                avg_point *= (1.0f / buffer.size());
                meas.at<float>(0) = avg_point.x;
                meas.at<float>(1) = avg_point.y;
                kf.correct(meas);
            }
        }
                
       
        state = kf.predict();
        float x_loc = state.at<float>(0);
       
        if (!ids.empty()) {
            std::cout << "Detected: X = " << x_loc << std::endl;
        } else {
            std::cout << "No Aruco detected" << std::endl;
        }
       
        char outBuffer[32];
        
//detection handling with grace period        
if (!ids.empty()) {
    //Reset counter if marker is detected
    noMarkerFrames = 0;
    markerRecentlyVisible = true;
    std::snprintf(outBuffer, sizeof(outBuffer), "%.2f\n", x_loc);
        } else {
            noMarkerFrames++;
        
        if (noMarkerFrames >= config.noMarkerThreshold) {
            if (markerRecentlyVisible) {
                std::cout << "[INFO] Marker lost after" << config.noMarkerThreshold << " frames." << std::endl;
            }
            markerRecentlyVisible = false;
            std::strncpy(outBuffer, "NO_MARKER\n", sizeof(outBuffer));
        } else { 
            //Still within the threshold - send last known position
            std::snprintf(outBuffer,sizeof(outBuffer), "%.2f\n", x_loc);
        }
}

//write to UART
ssize_t write_count = write(serial_fd, outBuffer, std::strlen(outBuffer));
if (write_count < 0) {
    std::cerr << "UART write error\n";
}        
       
        int scaled_x = static_cast<int>(x_loc * 100.0f);
        std::cout << "X pos: " << x_loc << " (scaled int: " << scaled_x << ")" << std::endl;
       
        if (gui_enabled) {
            cv::circle(frame, cv::Point2f(x_loc, state.at<float>(1)), 5, cv::Scalar(0, 0, 255), -1);
            cv::imshow("Frame", frame);
            if (cv::waitKey(1) == 27) break;
        }
    }
   
    // Clean up resources
    cap.release();
    close(serial_fd);
    return 0;
}

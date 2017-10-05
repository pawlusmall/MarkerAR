//
//  MarkerDetector.hpp
//  MarkerARExample
//
//  Created by Pablo Soto on 27/9/17.
//  Copyright © 2017 Neosentec. All rights reserved.
//

#ifndef MarkerAR_hpp
#define MarkerAR_hpp

#include <opencv2/opencv.hpp>

using namespace std;
using namespace cv;

    
class MarkerAR {
     
private:
        
    const int CAMERA_WIDTH = 640;
    const int CAMERA_HEIGHT = 480;
     
    Mat K;                                          // Camera intrinsics
    Mat P;                                          // Projection matrix
    Mat cv2gl;                                      // OpenCV to OpenGL transformation matrix
    Mat Pcw;                                        // Pose camera world (OpenGL coordinates)
    vector<cv::Point> lastMarkerCorners;            // Last detected marker corners
        
    void checkMarkerCorners(vector<cv::Point>& marker);
        
public:
        
    MarkerAR();
    void processFrame(Mat& frame);
    Mat getCameraProjection() { return P; }
    Mat getCameraPose() { return Pcw; }
    
};

#endif

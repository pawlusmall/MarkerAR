//
//  MarkerDetector.cpp
//  MarkerARExample
//
//  Created by Pablo Soto on 27/9/17.
//  Copyright © 2017 Neosentec. All rights reserved.
//

#include "MarkerAR.hpp"

    
MarkerAR::MarkerAR() {
    
    // Set camera intrinsics (K)
    K = Mat::zeros(3, 3, CV_64F);
    K.at<double>(0,0) = CAMERA_WIDTH;
    K.at<double>(0,2) = CAMERA_WIDTH / 2;
    K.at<double>(1,1) = CAMERA_WIDTH;
    K.at<double>(1,2) = CAMERA_HEIGHT / 2;
    K.at<double>(2,2) = 1;
    
    double fx = K.at<double>(0,0);
    double fy = K.at<double>(1,1);
    double cx = K.at<double>(0,2);
    double cy = K.at<double>(1,2);
    double near = 0.1, far = 1000.0;
    
    // Set projection matrix
    P = Mat::zeros(4,4, CV_64F);
    P.at<double>(0,0) = fx/cx;
    P.at<double>(1,1) = fy/cy;
    P.at<double>(2,2) = -(far+near)/(far-near);
    P.at<double>(2,3) = -2.0*far*near / (far-near);
    P.at<double>(3,2) = -1.0;
    P = P.t();
    
    // Set transformation matrix between OpenCV and OpenGL coordinate systems
    cv2gl = Mat::zeros(4, 4, CV_64F);
    cv2gl.at<double>(0,0) = 1;
    cv2gl.at<double>(1,1) = -1;
    cv2gl.at<double>(2,2) = -1;
    cv2gl.at<double>(3,3) = 1;
}

void MarkerAR::processFrame(Mat& frame) {
    
    // Convert to gray to improve speed/memory
    Mat grayFrame;
    cvtColor(frame, grayFrame, CV_BGR2GRAY);
    
    // Blur image for a better edge detection
    blur(grayFrame, grayFrame, Size(5,5));
    // Detect edges
    Canny(grayFrame, grayFrame, 100, 300, 5);
    // Dilate edges to avoid small holes between paths
    dilate(grayFrame, grayFrame, Mat(), Point(-1,-1));
    
    // Lets transform edges in contours and store them into an array
    vector<vector<Point>> contours;
    findContours(grayFrame, contours, RETR_LIST, CHAIN_APPROX_SIMPLE);
    
    
    vector<Point> approx;
    vector<Point> markerCorners;
    int markerArea = 0;
    for( int i = 0; i < contours.size(); i++ ) {
        // We want to find a rectangular shape, so we need to transform contours into polygons
        approxPolyDP(Mat(contours[i]), approx, arcLength(Mat(contours[i]), true)  *0.02, true);
        // rectangular contours should have 4 vertices after approximation, relatively large area (to filter out noisy contours) and be convex.
        int area = fabs(contourArea(Mat(approx)));
        if( approx.size() == 4 && area > 10000 && area > markerArea && isContourConvex(Mat(approx)))
        {
            // if more than one rectangular shape is found we retrieve the biggest one
            markerCorners = approx;
        }
    }
    
    // If a marker was found
    if (markerCorners.size() > 0) {
        // Order corners so they are in the same position as previous detected marker
        checkMarkerCorners(markerCorners);
        lastMarkerCorners = markerCorners;
        // Draw marker border
        line(frame, markerCorners[0], markerCorners[1], Scalar(0, 255, 0), 3);
        line(frame, markerCorners[1], markerCorners[2], Scalar(0, 255, 0), 3);
        line(frame, markerCorners[2], markerCorners[3], Scalar(0, 255, 0), 3);
        line(frame, markerCorners[3], markerCorners[0], Scalar(0, 255, 0), 3);
        
        // Convert to float
        vector<Point2f> markerCorners2f;
        for ( int i = 0; i < 4; i++) {
            markerCorners2f.push_back(markerCorners[i]);
        }
        
        // Initialize desired initial camera (a 480x480px marker image in center of the 640x480 screen)
        vector<Point2f> initialCorners2f;
        initialCorners2f.push_back(Point2f(80, 0));
        initialCorners2f.push_back(Point2f(560, 0));
        initialCorners2f.push_back(Point2f(560, 480));
        initialCorners2f.push_back(Point2f(80, 480));
        
        /**
         You may want to analyze detected marker, if so, it is very useful to warp it with an homography
         
         Mat warped;
         Mat homography;
         homography = findHomography(markerCorners2f, initialCorners2f, RANSAC);
         warpPerspective(frame, warped, homography, frame.size());
         */
        
        // Convert initial corners into 3d points to compute camera
        vector<Point3f> initialPoints3d;
        for (int i = 0; i < initialCorners2f.size(); i++) {
            Point2f p = initialCorners2f[i];
            initialPoints3d.push_back(Point3f(p.x - frame.cols/2, p.y - frame.rows/2, 0) * 1/frame.cols);
        }
        
        // Estimate object pose
        Mat rvec, tvec;
        solvePnP(initialPoints3d, markerCorners2f, K, Mat(), rvec, tvec, !rvec.empty());
        
        // Build [R | t] matrix
        Mat Rcw;
        Mat Tcw;
        rvec.convertTo(Rcw, CV_32F);
        tvec.convertTo(Tcw, CV_64F);
        Rodrigues(rvec, Rcw);
        
        // Generate OpenGL pose camera world
        Mat newPcw = Mat::eye(4,4, CV_64F);
        Rcw.convertTo(newPcw(Rect(0,0,3,3)),CV_64F);
        Tcw.copyTo(newPcw(Rect(3,0,1,3)));
        newPcw = (cv2gl * newPcw).t();
        
        if (!Pcw.empty()) {
            // Lowpass filter
            Pcw = Pcw + 0.3 * (newPcw - Pcw);
        } else {
            Mat(newPcw).copyTo(Pcw);
        }
        
    }
    
    
}

void MarkerAR::checkMarkerCorners(vector<Point>& markerCorners) {
    vector<Point> newMarkerCorners(4);
    double sumLeastDistances = 0;
    if (lastMarkerCorners.size() > 0) {
        for (int i = 0; i < markerCorners.size(); i++) {
            int leastDistanceIdx = 0;
            double leastDistance = 10000;
            for (int j = 0; j < lastMarkerCorners.size(); j++) {
                double distance = norm(Mat(lastMarkerCorners[j]), Mat(markerCorners[i]));
                if (distance < leastDistance) {
                    leastDistance = distance;
                    leastDistanceIdx = j;
                }
            }
            newMarkerCorners[leastDistanceIdx] = markerCorners[i];
            sumLeastDistances += leastDistance;
        }
        if (sumLeastDistances < 100) {
            markerCorners = newMarkerCorners;
        }
    }
}

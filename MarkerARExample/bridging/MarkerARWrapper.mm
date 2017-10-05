//
//  MarkerARWrapper.m
//  MarkerARExample
//
//  Created by Pablo Soto on 27/9/17.
//  Copyright © 2017 Neosentec. All rights reserved.
//

#import "MarkerARExample-Bridging-Header.h"
#import <opencv2/opencv.hpp>
#import "UIImage+OpenCV.h"
#import "MarkerAR.hpp"

@implementation MarkerARWrapper: NSObject

MarkerAR *markerAR = NULL;

- (id) init
{
    self = [super init];
    markerAR = new MarkerAR::MarkerAR();
    return self;
}

- (UIImage *)processFrame:(UIImage *)frame
{
    cv::Mat matFrame = [frame CVMat3];
    markerAR->processFrame(matFrame);
    return [UIImage imageWithCVMat:matFrame];
}

- (SCNMatrix4) getCameraPose {
    GLKMatrix4 glkMatrix;
    if (!markerAR->getCameraPose().empty()) {
        for(int i = 0; i < 16; i++) {
            glkMatrix.m[i] = markerAR->getCameraPose().at<double>(i);
        }
    }
    return SCNMatrix4Invert(SCNMatrix4FromGLKMatrix4(glkMatrix));
}

- (SCNMatrix4) getCameraProjection {
    GLKMatrix4 glkMatrix;
    for (int i = 0; i < 16; i++) {
        glkMatrix.m[i] = markerAR->getCameraProjection().at<double>(i);
    }
    return SCNMatrix4FromGLKMatrix4(glkMatrix);
}

@end

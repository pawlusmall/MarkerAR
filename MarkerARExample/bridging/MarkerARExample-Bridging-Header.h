#import <Foundation/Foundation.h>
#import <UIKit/UIKit.h>
#import <QuartzCore/QuartzCore.h>
#import <SceneKit/SceneKit.h>

@interface MarkerARWrapper: NSObject

- (id) init;
- (UIImage *) processFrame:(UIImage *)frame;
- (SCNMatrix4) getCameraProjection;
- (SCNMatrix4) getCameraPose;

@end

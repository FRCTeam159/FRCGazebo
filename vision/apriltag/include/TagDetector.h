#pragma "once"

#include <iostream>
#include <vector>

#include <apriltag.h>
#include <apriltag_pose.h>
#include <tag16h5.h>
#include <tag36h11.h>

#include "common/image_u8.h"
#include "common/zarray.h"

class TagDetector {
public:    
    apriltag_detector_t *td;
    apriltag_family_t *tf;

     int type;
     TagDetector();
     TagDetector(int);
     ~TagDetector();
     zarray_t *detect(uint8_t *, int, int);
     void test_image(image_u8_t *im, bool dopose, bool timing);
};
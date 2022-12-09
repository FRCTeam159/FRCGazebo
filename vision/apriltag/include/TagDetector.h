#pragma "once"

#include <iostream>
#include <vector>

#include <apriltag.h>
#include <apriltag_pose.h>
#include <tag16h5.h>
#include <tag36h11.h>

#include "common/image_u8.h"
#include "common/pjpeg.h"
#include "common/zarray.h"

class TagDetector {
    apriltag_detector_t *td;
    apriltag_family_t *tf;
public:
     int type;
     TagDetector();
     TagDetector(int);
     ~TagDetector();
     zarray_t *detect(uint8_t *, int, int);
     void test(const char *, bool dopose);
};
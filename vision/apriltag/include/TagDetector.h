#pragma "once"

#include <iostream>
#include <vector>

#include <apriltag.h>
#include <tag36h11.h>
#include "common/image_u8.h"
#include "common/pjpeg.h"
#include "common/zarray.h"

class TagResult {
    public:
    int id;
    int tag_id;
    double margin;
    double centerX, centerY;
    double corners[4][2];
    TagResult(int n,apriltag_detection_t *det);
    double getWidth();
    double getHeight();
    void print();
};
class TagDetector {
    apriltag_detector_t *td;
    apriltag_family_t *tf;
    std::vector<TagResult> result;
public:
     int type;
     TagDetector();
     TagDetector(int);
     ~TagDetector();
     std::vector<TagResult> detect(uint8_t *, int, int);
     void test(const char *);
};
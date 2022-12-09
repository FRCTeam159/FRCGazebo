#include "TagDetector.h"

using namespace std;

static bool debug=false;


TagDetector::TagDetector(){
    TagDetector(0);
}
TagDetector::TagDetector(int t) : type(t){
    td=apriltag_detector_create();
    switch(type){
        case 0: tf=tag16h5_create(); break;
        case 1: tf=tag36h11_create(); break;
    }
    apriltag_detector_add_family_bits(td, tf, 1);

    cout<<"TagDetector::TagDetector("<<type<<")"<<endl;
}
TagDetector::~TagDetector(){
    cout<<"TagDetector::~TagDetector("<<type<<")"<<endl;
    switch(type){
        case 0: tag16h5_destroy(tf); break;
        case 1: tag36h11_destroy(tf); break;
    }
    
    apriltag_detector_destroy(td);
}

void TagDetector::test(const char *path, bool dopose){
    cout<<"TagDetector::test("<<path<<","<<dopose<<")"<<endl;
    int err = 0;
    pjpeg_t *pjpeg = pjpeg_create_from_file(path, 0, &err);

    if (pjpeg == NULL) {
        cout<<"pjpeg failed to load:"<<path<< " error "<<err<<endl;
        return;
    }
    cout<<"pjpeg loaded :"<<path<<endl;
    image_u8_t *im=pjpeg_to_u8_baseline(pjpeg);

    zarray_t *detections = apriltag_detector_detect(td, im);
    
    // from photon-vision (same inputs) 
    // Transform3d(Translation3d(X: 6.74, Y: 2.00, Z: 0.18), Rotation3d(Quaternion(-0.008917661766202317, 0.010451058345968461, 0.0018763825497193909, 0.9999038603168806)))
    // Transform3d(Translation3d(X: 6.67, Y: -1.43, Z: 0.18), Rotation3d(Quaternion(-0.02109792541616773, -0.0010678894230088387, -1.4532908145981072E-4, 0.999776833115657)))
    // Object x -0.02 y 0.00 z 0.02
    // from this test
    // pose1: -2.01,-0.18,6.78,1.5448e-05  pose2: -2.01,-0.18,6.78,9.25975e-05
    // pose1: 1.43,-0.18,6.67,1.23121e-05  pose2: 1.44,-0.18,6.72,8.25468e-05

    // note: The pose differences probably are because of the way opencl and FRC (gazebo) interpret world axis directions
    // in opencv (also opengl) forward eye direction is +z with left=-x right=+x up=-y down= +y
    // in the frc field robot moves forward along +x turns left (+y) turns right (-y) up(+z)
    // so z(opencv)->x(frc), x(opencv)->-y(frc), y(opencv)->-z(frc)  
    apriltag_pose_t pose1 = { 0 };
    apriltag_pose_t pose2 = { 0 };
    double err1 = 0; //Should get overwritten if pose estimation is happening
    double err2 = 0;
    if(dopose){
        for (int i = 0; i < zarray_size(detections); i++) {
            apriltag_detection_t *det;
            zarray_get(detections, i, &det);
            apriltag_detection_info_t info { det, 0.4, 879, 895, 320, 240 };
             estimate_tag_pose_orthogonal_iteration(&info, &err1, &pose1, &err2, &pose2, 1);       
        }
        timeprofile_stamp(td->tp, "dopose");
    }
    for (int i = 0; i < zarray_size(detections); i++) {
        apriltag_detection_t *det;
        zarray_get(detections, i, &det);
         printf("detection %3d: id (%2dx%2d)-%d hamming %d margin %8.3f center x:%d y:%d w:%d h:%d\n",
                            i, det->family->nbits, det->family->h, det->id,  det->hamming, det->decision_margin,(int)det->c[0],(int)det->c[1],
                            (int)(det->p[1][0]-det->p[0][0]),(int)(det->p[0][1]-det->p[2][1])); 
        if(dopose){
            printf("pose1: %-1.2f,%-1.2f,%-1.2f,%g  pose2: %-1.2f,%-1.2f,%-1.2f,%g\n",
                pose1.t->data[0],pose1.t->data[1],pose1.t->data[2],err1,pose2.t->data[0],pose2.t->data[1],pose2.t->data[2],err2
            );
        }
    }
   
    timeprofile_display(td->tp);
    fflush(stdout);
    pjpeg_destroy(pjpeg);
}

zarray_t *TagDetector::detect(uint8_t * buff, int height, int width){
    image_u8_t img={ .width = width, .height = height, .stride = 640, .buf = buff };   
    zarray_t *detections = apriltag_detector_detect(td, &img);
    return detections;
}

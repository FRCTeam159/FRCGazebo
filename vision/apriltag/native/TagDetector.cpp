#include "TagDetector.h"
//#include "pjpeg.h"

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

/*
void TagDetector::test(const char *path, bool dopose, bool timing){
    cout<<"TagDetector::test("<<path<<","<<dopose<<")"<<endl;
    int err = 0;
    pjpeg_t *pjpeg = pjpeg_create_from_file(path, 0, &err);

    if (pjpeg == NULL) {
        cout<<"pjpeg failed to load:"<<path<< " error "<<err<<endl;
        return;(dopose,timing);
    }
    cout<<"pjpeg loaded :"<<path<<endl;
    image_u8_t *im=pjpeg_to_u8_baseline(pjpeg);
    test_image(im,dopose,timing);
    
    pjpeg_destroy(pjpeg);
}
*/

void TagDetector::test_image(image_u8_t *im, bool dopose, bool timing){
    zarray_t *detections = apriltag_detector_detect(td, im);
    for (int i = 0; i < zarray_size(detections); i++) {
        apriltag_pose_t pose1 = { 0 };
        apriltag_pose_t pose2 = { 0 };
    
        double err1 = HUGE_VAL; //Should get overwritten if pose estimation is happening
        double err2 = HUGE_VAL;

        apriltag_pose_t *best_pose=0;
        apriltag_detection_t *det;
        zarray_get(detections, i, &det);
        if(dopose){
            apriltag_detection_info_t info={ det, 0.4, 879, 895, 320, 240 };
            estimate_tag_pose_orthogonal_iteration(&info, &err1, &pose1, &err2, &pose2, 1);
            if(i==0 && timing)   
                timeprofile_stamp(td->tp, "dopose");
        }
        printf("detection %3d: id (%2dx%2d)-%d hamming %d margin %8.3f center x:%d y:%d w:%d h:%d\n",
                i, det->family->nbits, det->family->h, det->id,  det->hamming, det->decision_margin,(int)det->c[0],(int)det->c[1],
                (int)(det->p[1][0]-det->p[0][0]),(int)(det->p[0][1]-det->p[2][1]));
        if(dopose){
            double err=err1<err2?err1:err2;
            if(pose1.t && !pose2.t)
                best_pose=&pose1;
            else if(!pose1.t && pose2.t)
                best_pose=&pose2;
            else
                best_pose=err1<err2?&pose1:&pose2;
            if(best_pose)  
                printf("\tpose: %-1.2f,%-1.2f,%-1.2f err:%g\n",best_pose->t->data[0],best_pose->t->data[1],best_pose->t->data[2],err1);           
            else
                printf("no valid poses found !!\n");        
        }      
    }
    if(timing)
        timeprofile_display(td->tp);
    fflush(stdout);
}
    
zarray_t *TagDetector::detect(uint8_t * buff, int height, int width){
    image_u8_t img={width, height, width, buff };   
    zarray_t *detections = apriltag_detector_detect(td, &img);
    return detections;
}

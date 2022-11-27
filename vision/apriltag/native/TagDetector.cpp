#include "TagDetector.h"

using namespace std;

static bool debug=false;

TagResult::TagResult(int n,apriltag_detection_t *det){
    id=n;
    tag_id = det->id;
    margin = det->decision_margin;
    centerX = det->c[0];
    centerY = det->c[1];
    for(int i=0;i<4;i++)
    for(int j=0;j<2;j++)
        corners[i][j]=det->p[i][j];
}
void TagResult::print(){
    printf("detection %3d: margin %8.3f center x:%d y:%d w:%d h:%d\n",
                            id, margin,(int)centerX,(int)centerX,
                            (int)(corners[1][0]-corners[0][0]),(int)(corners[0][1]-corners[2][1]));
}

double TagResult::getWidth(){
    return corners[1][0]-corners[0][0];
}
double TagResult::getHeight(){
    return corners[0][1]-corners[2][1];
}

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

void TagDetector::test(const char *path){
    cout<<"TagDetector::test("<<path<<")"<<endl;
    int err = 0;
    pjpeg_t *pjpeg = pjpeg_create_from_file(path, 0, &err);

    if (pjpeg == NULL) {
        cout<<"pjpeg failed to load:"<<path<< " error "<<err<<endl;
        return;
    }
    cout<<"pjpeg loaded :"<<path<<endl;
    image_u8_t *im=pjpeg_to_u8_baseline(pjpeg);

    zarray_t *detections = apriltag_detector_detect(td, im);
    for (int i = 0; i < zarray_size(detections); i++) {
        apriltag_detection_t *det;
        zarray_get(detections, i, &det);
        printf("detection %3d: id (%2dx%2d)-%d hamming %d margin %8.3f center x:%d y:%d w:%d h:%d\n",
                            i, det->family->nbits, det->family->h, det->id,  det->hamming, det->decision_margin,(int)det->c[0],(int)det->c[1],
                            (int)(det->p[1][0]-det->p[0][0]),(int)(det->p[0][1]-det->p[2][1]));
    }
    timeprofile_display(td->tp);
    fflush(stdout);
    pjpeg_destroy(pjpeg);
}


std::vector<TagResult> TagDetector::detect(uint8_t * buff, int height, int width){
    image_u8_t img={ .width = width, .height = height, .stride = 640, .buf = buff };   
    zarray_t *detections = apriltag_detector_detect(td, &img);
    
    result.clear();
    for (int i = 0; i < zarray_size(detections); i++) {
        apriltag_detection_t *det;
        zarray_get(detections, i, &det);
        TagResult tag(i,det);
        result.push_back(tag);   
    }
    if(debug){
        for(int i=0;i<result.size();i++){
            TagResult tag=result[i];
            tag.print();
        }
        fflush(stdout);
    }

   return result;
}
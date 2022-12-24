// AprilTag Detector

#include "utils_TagDetectorJNI.h"
#include "TagDetector.h"

jfieldID getPtrFieldId(JNIEnv * env, jobject obj){
  // In Cmake builds java jni will delete java class instances (obj) after each native call
  // unless saved as a global reference - for some reason this doesn't work in gradle projects
  // (leads to access violation crash)
  // cmake needs this:
  // jobject ref=env->NewGlobalRef(obj);
  // jclass cls=env->GetObjectClass(ref);
  // jfieldID fld =env->GetFieldID(cls, "detector", "J");
  // gradle needs this:
  static jfieldID ptrFieldId = 0;
    if (!ptrFieldId) {
        jclass c = env->GetObjectClass(obj);
        ptrFieldId = env->GetFieldID(c, "detector", "J");
        env->DeleteLocalRef(c);
    }
    return ptrFieldId;
}

/*
 * Class:     TagDetectorJNI
 * Method:    detector_create
 * Signature: ()J
 */
JNIEXPORT void JNICALL Java_utils_TagDetectorJNI_detector_1create
  (JNIEnv *env, jobject obj, jint type){
    jfieldID fld=getPtrFieldId(env, obj);
    TagDetector *cobj= new TagDetector(type);
    env->SetLongField(obj, fld, (jlong) cobj); 
  }

/*
 * Class:     utils_TagDetectorJNI
 * Method:    detector_detect
 * Signature: (JIIDDDDD)[Lutils/TagResult;
 */
JNIEXPORT jobjectArray JNICALL Java_utils_TagDetectorJNI_detector_1detect
  (JNIEnv *env, jobject obj, jlong data, jint rows, jint cols, jdouble tw, jdouble fx, jdouble fy, jdouble cx, jdouble cy)
  {
    TagDetector *detector = (TagDetector *)env->GetLongField(obj, getPtrFieldId(env, obj));

    if(!detector){
        std::cout<<"error: couldn't obtain detector in java class"<<std::endl;
        return 0;
    }

    zarray_t *detections = detector->detect((uint8_t *)data, rows, cols);

    jclass jcls = env->FindClass("utils/TagResult");
    if(!jcls){
      std::cout<<"error: could not find TagResult class"<<std::endl;
      return 0;
    }
    int ntags=zarray_size(detections);

    jobjectArray ret = env->NewObjectArray( ntags, jcls, NULL);
    if(!ret){
       std::cout<<"error: failed to create jobjectArray of size:"<<ntags<<std::endl;
       return 0;
    }
    //jmethodID constructor=env->GetMethodID(jcls, "<init>", "(IIDDD[D)V");
    jmethodID constructor=env->GetMethodID(jcls, "<init>", "(IIDDD[D[D[D[DD)V");

    if(!constructor){
       std::cout<<"error: failed to create TagResult constructor:"<<ntags<<std::endl;
       return 0;
    }

    for (int t = 0; t < ntags; t++){
      apriltag_detection_t *detect;
      zarray_get(detections, t, &detect);

      jdouble corners[8]; // = new jdouble[8]{};
      for (int i = 0; i < 4; i++){
        corners[i * 2] = detect->p[i][0];
        corners[i * 2 + 1] = detect->p[i][1];
      }
      jdoubleArray carr = env->NewDoubleArray(8);
      env->SetDoubleArrayRegion( carr, 0, 8, &corners[0] );

      jdouble h[9];
      for (int i = 0; i < 9; i++) {
        h[i] = detect->H->data[i];
      }
      jdoubleArray harr = env->NewDoubleArray(9);
      env->SetDoubleArrayRegion( harr, 0, 9, &h[0] );

      jdouble pose[3]={0};
      jdouble rot[9]={1,0,0,0,1,0,0,0,1};
      jdouble perr=HUGE_VAL;
      if(tw>0){
        apriltag_pose_t pose1 = { 0 };
        apriltag_pose_t pose2 = { 0 };
        apriltag_pose_t *best_pose=0;
        double err1 = HUGE_VAL; //Should get overwritten if pose estimation is happening
        double err2 = HUGE_VAL;
        apriltag_detection_info_t info {detect, tw, fx, fy, cx, cy };

        estimate_tag_pose_orthogonal_iteration(&info, &err1, &pose1, &err2, &pose2, 1);
        perr=err1<err2?err1:err2;

        if (pose1.t && !pose2.t)
          best_pose = &pose1;
        else if (!pose1.t && pose2.t)
          best_pose = &pose2;
        else
          best_pose = err1 < err2 ? &pose1 : &pose2;
        if (best_pose){
          for (int i = 0; i < 3; i++)
            pose[i] = best_pose->t->data[i];
          for (int i = 0; i < 9; i++)
            rot[i] = best_pose->R->data[i];
        }
      }
      
      jdoubleArray parr = env->NewDoubleArray(3);
      env->SetDoubleArrayRegion( parr, 0, 3, &pose[0] );

      jdoubleArray rarr = env->NewDoubleArray(9);
      env->SetDoubleArrayRegion( rarr, 0, 9, &rot[0] );

      int tag_id = detect->id;
      double margin = detect->decision_margin;
      double centerX = detect->c[0];
      double centerY = detect->c[1];

      //std::cout<<tag_id<<" ";
      jobject obj = env->NewObject(jcls, constructor,
       t,       // detection number
       tag_id,  // apriltag id
       margin,  // detection confidence
       centerX, // target center x
       centerY, // target center y
       carr,    // corners (4x2)
       harr,    // homog   (3x3)
       parr,    // pose-translation (3x1)
       rarr,    // pose-rotation (3x3)
       perr     // pose error
      ); 
       env->SetObjectArrayElement( ret, t, obj);     
     }
    apriltag_detections_destroy(detections);
    return ret;
  }

 /*
 * Class:     utils_TagDetectorJNI
 * Method:    detector_image_test
 * Signature: (JIIZZ)V
 */
JNIEXPORT void JNICALL Java_utils_TagDetectorJNI_detector_1image_1test
  (JNIEnv *env, jobject obj, jlong data, jint rows, jint cols, jboolean pose, jboolean timing){

  TagDetector * detector = (TagDetector *) env->GetLongField(obj, getPtrFieldId(env, obj));
  if(detector){
      image_u8_t im = {static_cast<int32_t>(cols), static_cast<int32_t>(rows),
                   static_cast<int32_t>(cols),
                   reinterpret_cast<uint8_t*>(data)};
      detector->test_image(&im,pose,timing);
    }
  }
  
 /*
 * Class:     utils_TagDetectorJNI
 * Method:    detector_image_test
 * Signature: (Ljava/lang/String;)V
 */
/*
JNIEXPORT void JNICALL Java_utils_TagDetectorJNI_detector_1test
  (JNIEnv *env, jobject obj, jstring inJNIStr, jboolean pose, jboolean timing){
    const char *inCStr = env->GetStringUTFChars(inJNIStr, NULL);
    TagDetector * detector = (TagDetector *) env->GetLongField(obj, getPtrFieldId(env, obj));
    if(detector){
        detector->test(inCStr,pose,timing);
    }
    env->ReleaseStringUTFChars(inJNIStr, inCStr);  // release resources
  }
*/
/*
 * Class:     TagDetectorJNI
 * Method:    detector_destroy
 * Signature: (J)V
 */
JNIEXPORT void JNICALL Java_utils_TagDetectorJNI_detector_1destroy
  (JNIEnv *env, jobject obj){
  //std::cout<<"destroy called"<<std::endl;

  TagDetector * detector = (TagDetector *) env->GetLongField(obj, getPtrFieldId(env, obj));
  if(detector)
     delete detector;
  }


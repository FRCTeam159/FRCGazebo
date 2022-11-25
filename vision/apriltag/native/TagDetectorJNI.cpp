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
 * Class:     TagDetectorJNI
 * Method:    detector_detect
 * Signature: (JJII)[Ljava/lang/Object;
 *   (JNIEnv *, jclass, jlong, jint, jint);
 */
JNIEXPORT jobjectArray JNICALL Java_utils_TagDetectorJNI_detector_1detect
  (JNIEnv *env, jobject obj, jlong a, jint r, jint c){
   
    TagDetector *detector = (TagDetector *)env->GetLongField(obj, getPtrFieldId(env, obj));

    if(!detector){
        std::cout<<"error: couldn't obtain detector in java class"<<std::endl;
        return 0;
    }
    uint8_t *ptr=(uint8_t*)a;
    std::vector<TagResult> tags = detector->detect(ptr, r, c);
    
    jclass jcls = env->FindClass("utils/TagResult");
    if(!jcls){
      std::cout<<"error: could not find TagResult class"<<std::endl;
      return 0;
    }
    int ntags=tags.size();
    jobjectArray ret = env->NewObjectArray( ntags, jcls, NULL);
    if(!ret){
       std::cout<<"error: failed to create jobjectArray of size:"<<ntags<<std::endl;
       return 0;
    }
    //std::cout<<"created jobjectArray of size:"<<tags.size()<<std::endl;
    //midIntegerInit = (*env)->GetMethodID(env, classInteger, "<init>", "(IIDDD[[D)V");
    jmethodID constructor=env->GetMethodID(jcls, "<init>", "(IIDDDDDDDDDDD)V");
    if(!constructor){
       std::cout<<"error: failed to create constructor:"<<ntags<<std::endl;
       return 0;
    }

     for (int i = 0; i < ntags; i++){
       TagResult tag=tags[i];
       double w=tag.getWidth();
       double h=tag.getHeight();

       jobject obj = env->NewObject(jcls, constructor,
       tag.id,
       tag.tag_id,
       tag.margin,
       tag.centerX,
       tag.centerY,
       tag.corners[0][0],
       tag.corners[0][1],
       tag.corners[1][0],
       tag.corners[1][1],
       tag.corners[2][0],
       tag.corners[2][1],
       tag.corners[3][0],
       tag.corners[3][1]
       );

       env->SetObjectArrayElement( ret, i, obj);
      
    //   jobject object =env->NewObject(env, cls, constructor, obj, 5, 6);
     
     }

    //std::cout<<myClass<<std::endl;
    return ret;
  }

  

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

/*
 * Class:     utils_TagDetectorJNI
 * Method:    detector_test
 * Signature: (Ljava/lang/String;)V
 */
JNIEXPORT void JNICALL Java_utils_TagDetectorJNI_detector_1test
  (JNIEnv *env, jobject obj, jstring inJNIStr){

    const char *inCStr = env->GetStringUTFChars(inJNIStr, NULL);

    TagDetector * detector = (TagDetector *) env->GetLongField(obj, getPtrFieldId(env, obj));
     if(detector)
        detector->test(inCStr);

    env->ReleaseStringUTFChars(inJNIStr, inCStr);  // release resources

  }
 

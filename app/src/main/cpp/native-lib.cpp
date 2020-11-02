#include <jni.h>
#include <math.h>

#include <exception>
#include <string>
#include <iostream>

#include <android/bitmap.h>
#include <android/log.h>

#include "apriltag.h"
#include "tag36h11.h"
#include "tag25h9.h"
#include "tag16h5.h"

#include "apriltag_pose.h"

#include "common/getopt.h"
#include "common/matd.h"
#include "common/getopt.h"
#include "common/image_u8.h"
#include "common/image_u8x4.h"
#include "common/pjpeg.h"
#include "common/zarray.h"

#include <opencv2/imgproc.hpp>
#include "opencv2/opencv.hpp"
#include "opencv2/core.hpp"
#include "opencv2/imgproc.hpp"
#include "opencv2/core/mat.inl.hpp"

#include "opencv2/core/base.hpp"
#include "opencv2/stitching.hpp"
#include "opencv2/imgcodecs.hpp"

#include "Eigen/Dense"

#include "trackerForTarmac.h"

#include <opencv2/opencv.hpp>
#include <opencv2/tracking/tracker.hpp>
#include <fstream>
#include <time.h>
#include "fdssttracker.hpp"
#include <cstring>

extern "C"
{
#include "libavcodec/avcodec.h"
#include "libavformat/avformat.h"
#include "libavfilter/avfilter.h"
}

#include <stdlib.h>
#include <libyuv.h>

using namespace libyuv;
using namespace cv;
using namespace std;
using namespace Eigen;

#define TAG "jni-log-libyuv" // 这个是自定义的LOG的标识
#define LOGD(...) __android_log_print(ANDROID_LOG_DEBUG,TAG ,__VA_ARGS__) // 定义LOGD类型
#define LOGI(...) __android_log_print(ANDROID_LOG_INFO,TAG ,__VA_ARGS__) // 定义LOGI类型
#define LOGW(...) __android_log_print(ANDROID_LOG_WARN,TAG ,__VA_ARGS__) // 定义LOGW类型
#define LOGE(...) __android_log_print(ANDROID_LOG_ERROR,TAG ,__VA_ARGS__) // 定义LOGE类型
#define LOGF(...) __android_log_print(ANDROID_LOG_FATAL,TAG ,__VA_ARGS__) // 定义LOGF类型

#define ASSERT(status, ret)     if (!(status)) { return ret; }
#define ASSERT_FALSE(status)    ASSERT(status, false)

char *input_src_data,
        *output_processed_data,
        *src_y_data, *src_u_data, *src_v_data,
        *dst_y_data, *dst_u_data, *dst_v_data;

int len_src_rgb,
        len_src, len_scale;

static struct {
    apriltag_detector_t *td;
    apriltag_family_t *tf;

    void (*tf_destroy)(apriltag_family_t *);

    apriltag_detection_info_t info;

    jclass al_cls;
    jmethodID al_constructor, al_add;

    jclass ad_cls;
    jmethodID ad_constructor;

    jint rec_id;

    jclass ap_cls;
    jmethodID ap_constructor;

    jfieldID ap_r_field, ap_t_field;

    jfieldID ad_id_field, ad_c_field, ad_p_field;
} state;

static struct {

    jclass al_cls;
    jmethodID al_constructor, al_add;

    jclass tr_cls;
    jmethodID tr_constructor;

    jfieldID x;
    jfieldID y;
    jfieldID width;
    jfieldID height;
    jfieldID isTarmac;

} tarmac;


int isFFmpegInitialized;

AVFrame *m_pYUVFrame;
AVCodecContext *m_pCodecCtx;
AVCodec *m_pAVCodec;
AVCodecParserContext *m_pCodecPaser;

jmethodID dataCallbackMID;

//FIX
struct URLProtocol;

extern "C" JNIEXPORT jstring JNICALL
Java_com_liyang_droneplus_MainActivity_stringFromJNI(
        JNIEnv *env,
        jobject /* this */) {
    std::string hello = "Hello from C++";
    return env->NewStringUTF(hello.c_str());
}

/**
 * Invoke the java callback method
 */
void invokeFrameDataCallback(JNIEnv *env, jobject obj, uint8_t *buf, int size, int frameNum,
                             int isKeyFrame, int width, int height) {
    jbyte *buff = (jbyte *) buf;
    jbyteArray jarray = (env)->NewByteArray(size);
    (env)->SetByteArrayRegion(jarray, 0, size, buff);
    (env)->CallVoidMethod(obj, dataCallbackMID, jarray, size, frameNum, isKeyFrame != 0, width,
                          height);
}

/**
 * Initialize the ffmpeg and software decoder.
 */
extern "C"
JNIEXPORT jboolean JNICALL
Java_com_liyang_droneplus_media_NativeHelper_init(JNIEnv *env, jobject obj) {

    jclass clazz = (env)->GetObjectClass(obj);
    dataCallbackMID = (env)->GetMethodID(clazz, "onFrameDataRecv", "([BIIZII)V");
    if (isFFmpegInitialized == 0) {
        avcodec_register_all();
        av_register_all();
        isFFmpegInitialized = 1;
    }
    m_pAVCodec = avcodec_find_decoder(AV_CODEC_ID_H264);
    m_pCodecCtx = avcodec_alloc_context3(m_pAVCodec);
    m_pCodecPaser = av_parser_init(AV_CODEC_ID_H264);
    if (m_pAVCodec == NULL || m_pCodecCtx == NULL) {
        __android_log_write(ANDROID_LOG_INFO, "ffmpeg_jni",
                            " m_pAVCodec == NULL||m_pCodecCtx == NULL ");
        return 0;
    }

//    if (m_pAVCodec->capabilities & CODEC_CAP_TRUNCATED)
//        m_pCodecCtx->flags |= CODEC_FLAG_TRUNCATED;

    m_pCodecCtx->thread_count = 4;
    m_pCodecCtx->thread_type = FF_THREAD_FRAME;

    if (avcodec_open2(m_pCodecCtx, m_pAVCodec, NULL) < 0) {
        m_pAVCodec = NULL;
        return 0;
    }

    m_pYUVFrame = av_frame_alloc();

    if (m_pYUVFrame == NULL) {
        __android_log_write(ANDROID_LOG_INFO, "ffmpeg_jni",
                            " CDecoder avcodec_alloc_frame() == NULL ");
        return 0;
    }
    __android_log_write(ANDROID_LOG_INFO, "ffmpeg_jni",
                        " CDecoder::prepare()2 ");
    return 1;
}

/**
 * Framing the raw data from camera using the av parser.
 */
int parse(JNIEnv *env, jobject obj, uint8_t *pBuff, int videosize, uint64_t pts) {
    int paserLength_In = videosize;
    int paserLen;
    int decode_data_length;
    int got_picture = 0;
    uint8_t *pFrameBuff = (uint8_t *) pBuff;
    while (paserLength_In > 0) {
        AVPacket packet;
        av_init_packet(&packet);
        if (m_pCodecPaser == NULL) {
            __android_log_write(ANDROID_LOG_INFO, "ffmpeg_jni",
                                " m_pCodecPaser == NULL ");
//            Java_com_dji_videostreamdecodingsample_media_NativeHelper_init(env, obj);
        }
        if (m_pCodecCtx == NULL) {
            __android_log_write(ANDROID_LOG_INFO, "ffmpeg_jni",
                                " m_pCodecCtx == NULL ");
//            Java_com_dji_videostreamdecodingsample_media_NativeHelper_init(env, obj);
        }
        paserLen = av_parser_parse2(m_pCodecPaser, m_pCodecCtx, &packet.data, &packet.size,
                                    pFrameBuff,
                                    paserLength_In, AV_NOPTS_VALUE, AV_NOPTS_VALUE, AV_NOPTS_VALUE);

        //LOGD("paserLen = %d",paserLen);
        paserLength_In -= paserLen;
        pFrameBuff += paserLen;

        if (packet.size > 0) {

            // LOGD(
            // 	"packet size=%d, pts=%lld, width_in_pixel=%d, height_in_pixel=%d, key_frame=%d, frame_has_sps=%d, frame_has_pps=%d, frame_num=%d",
            // 	packet.size,
            // 	pts,
            // 	m_pCodecPaser->width_in_pixel,
            // 	m_pCodecPaser->height_in_pixel,
            // 	m_pCodecPaser->key_frame,
            // 	m_pCodecPaser->frame_has_sps,
            // 	m_pCodecPaser->frame_has_pps,
            // 	m_pCodecPaser->frame_num
            // 	);
            invokeFrameDataCallback(
                    env,
                    obj,
                    packet.data,
                    packet.size,
                    m_pCodecPaser->output_picture_number,
                    m_pCodecPaser->key_frame,
                    m_pCodecPaser->width,
                    m_pCodecPaser->height
            );

        }
        av_free_packet(&packet);
    }

    return 0;
}

uint8_t audbuffer2[] = {0x00, 0x00, 0x00, 0x01, 0x09, 0x10};
uint8_t audsize2 = 6;
uint8_t fillerbuffer2[] = {0x00, 0x00, 0x00, 0x01, 0x0C, 0x00, 0x00, 0x00, 0x01, 0x09, 0x10};
uint8_t fillersize2 = 11;
uint8_t audaudbuffer2[] = {0x00, 0x00, 0x00, 0x01, 0x09, 0x10, 0x00, 0x00, 0x00, 0x01, 0x09, 0x10};
uint8_t audaudsize2 = 12;
/**
 * Framing the raw data from camera.
 */
extern "C"
JNIEXPORT jboolean JNICALL
Java_com_liyang_droneplus_media_NativeHelper_parse(JNIEnv *env, jobject obj, jbyteArray pBuff,
                                                   jint size) {
    jbyte *jBuff = (jbyte *) ((env)->GetByteArrayElements(pBuff, 0));
    uint8_t *buff = (uint8_t *) jBuff;
    uint64_t pts = 0;
    jbyte *jBuff2;

    // LOGD("pts=%llu", pts);

    // Removing the aud bytes.
    if (size >= fillersize2 && memcmp(fillerbuffer2, buff + size - fillersize2, fillersize2) == 0) {
        __android_log_write(ANDROID_LOG_INFO, "ffmpeg_jni",
                            " Remove filler+AUD ");
        parse(env, obj, buff, size - fillersize2, pts);
    } else if (size >= audaudsize2 &&
               memcmp(audaudbuffer2, buff + size - audaudsize2, audaudsize2) == 0) {
        __android_log_write(ANDROID_LOG_INFO, "ffmpeg_jni",
                            " Remove AUD+AUD ");
        parse(env, obj, buff, size - audaudsize2, pts);
    } else if (size >= audsize2 && memcmp(audbuffer2, buff + size - audsize2, audsize2) == 0) {
        __android_log_write(ANDROID_LOG_INFO, "ffmpeg_jni",
                            " Remove AUD ");
        parse(env, obj, buff, size - audsize2, pts);
    } else {
        // LOGD("Remove Nothing");
        parse(env, obj, buff, size, pts);
    }
    (env)->ReleaseByteArrayElements(pBuff, jBuff, 0);

    return 1;
}

extern "C"
JNIEXPORT jboolean JNICALL
Java_com_liyang_droneplus_media_NativeHelper_release(JNIEnv *env, jobject instance) {

    if (m_pCodecCtx) {
        avcodec_close(m_pCodecCtx);
        m_pCodecCtx = NULL;
    }

    av_free(m_pYUVFrame);
    av_free(m_pCodecCtx);
    av_parser_close(m_pCodecPaser);

    return 1;

}

extern "C"
JNIEXPORT void JNICALL
Java_com_liyang_droneplus_apriltags_ApriltagR_nativeInit(JNIEnv *env, jclass clazz) {
    // TODO: implement nativeInit()
    // Just do method lookups once and cache the results

    // Get ArrayList methods
    jclass al_cls = env->FindClass("java/util/ArrayList");
    if (!al_cls) {
        __android_log_write(ANDROID_LOG_ERROR, "apriltag_jni",
                            "couldn't find ArrayList class");
        return;
    }
    state.al_cls = static_cast<jclass>(env->NewGlobalRef(al_cls));
    // env->GetMethodID获取类的方法，有3个参数，第一个是类句柄，第二个是方法名，第三个则是参数的签名
    state.al_constructor = env->GetMethodID(al_cls, "<init>", "()V");
    state.al_add = env->GetMethodID(al_cls, "add", "(Ljava/lang/Object;)Z");
    if (!state.al_constructor || !state.al_add) {
        __android_log_write(ANDROID_LOG_ERROR, "apriltag_jni",
                            "couldn't find ArrayList methods");
        return;
    }

    // Get ApriltagPoseEstimation methods
    jclass ap_cls = env->FindClass("com/liyang/droneplus/apriltags/ApriltagPoseEstimation");

    if (!ap_cls) {
        __android_log_write(ANDROID_LOG_ERROR, "apriltag_jni",
                            "couldn't find ApriltagDetection class");
        return;
    }
    state.ap_cls = static_cast<jclass>(env->NewGlobalRef(ap_cls));

    state.ap_constructor = env->GetMethodID(ap_cls, "<init>", "()V");
    if (!state.ap_constructor) {
        __android_log_write(ANDROID_LOG_ERROR, "apriltag_jni",
                            "couldn't find ApriltagDetection constructor");
        return;
    }

    state.ad_id_field = env->GetFieldID(ap_cls, "id", "I");

    state.ad_c_field = env->GetFieldID(ap_cls, "c", "[D");
    state.ad_p_field = env->GetFieldID(ap_cls, "p", "[D");

    state.ap_r_field = env->GetFieldID(ap_cls, "R", "[D");
    state.ap_t_field = env->GetFieldID(ap_cls, "t", "[D");

    if (!state.ad_id_field ||
        !state.ad_c_field ||
        !state.ad_p_field) {
        __android_log_write(ANDROID_LOG_ERROR, "apriltag_jni",
                            "couldn't find ApriltagDetection fields");
        return;
    }
}

extern "C"
JNIEXPORT void JNICALL
Java_com_liyang_droneplus_apriltags_ApriltagR_apriltagInit(JNIEnv *env, jclass cls, jstring _tfname,
                                                           jint errorbits, jdouble decimate,
                                                           jdouble sigma, jint nthreads,
                                                           jdouble tagsize, jdouble fx, jdouble fy,
                                                           jdouble cx,
                                                           jdouble cy) {
    // TODO: implement apriltagInit()
    __android_log_write(ANDROID_LOG_INFO, "apriltag_jni",
                        "enter_detect_jni1");    // Do cleanup in case we're already initialized

    if (state.td) {
        apriltag_detector_destroy(state.td);
        state.td = NULL;
    }
    if (state.tf) {
        state.tf_destroy(state.tf);
        state.tf = NULL;
    }

    // Initialize state
    const char *tfname = env->GetStringUTFChars(_tfname, NULL);

    if (!strcmp(tfname, "tag36h11")) {
        state.tf = tag36h11_create();
        state.tf_destroy = tag36h11_destroy;
    } else if (!strcmp(tfname, "tag25h9")) {
        state.tf = tag25h9_create();
        state.tf_destroy = tag25h9_destroy;
    } else if (!strcmp(tfname, "tag16h5")) {
        state.tf = tag16h5_create();
        state.tf_destroy = tag16h5_destroy;
    } else {
        __android_log_print(ANDROID_LOG_ERROR, "apriltag_jni",
                            "invalid tag family: %s", tfname);
        env->ReleaseStringUTFChars(_tfname, tfname);
        return;
    }
    env->ReleaseStringUTFChars(_tfname, tfname);

    state.td = apriltag_detector_create();
    apriltag_detector_add_family_bits(state.td, state.tf, errorbits);
    state.td->quad_decimate = decimate;
    state.td->quad_sigma = sigma;
    state.td->nthreads = nthreads;

    state.info.tagsize = tagsize;
    state.info.fx = fx;
    state.info.fy = fy;
    state.info.cx = cx;
    state.info.cy = cy;
}

/*
 * Class:     edu_umich_eecs_april_apriltag_ApriltagNative
 * Method:    apriltag_detect_yuv
 * Signature: ([BII)Ljava/util/ArrayList;
 */
extern "C"
JNIEXPORT jobject JNICALL
Java_com_liyang_droneplus_apriltags_ApriltagR_apriltagDetectYuv(JNIEnv *env, jclass clazz,
                                                                jbyteArray _buf, jint width,
                                                                jint height) {
    // TODO: implement apriltagDetectYuv()
    // If not initialized, init with default settings
    if (!state.td) {
        state.tf = tag36h11_create();
        state.td = apriltag_detector_create();
        apriltag_detector_add_family_bits(state.td, state.tf, 2);
        state.td->quad_decimate = 2.0;
        state.td->quad_sigma = 0.0;
        state.td->nthreads = 4;
        state.info.tagsize = 0.005184;
        state.info.fx = 5423.612353784232;
        state.info.fy = 5519.056520316598;
        state.info.cx = 1315.1967405060602;
        state.info.cy = 838.5860437396254;
        __android_log_write(ANDROID_LOG_INFO, "apriltag_jni",
                            "using default parameters");
    }

//    image_u8_t *im = NULL;
//    if (str_ends_with(path, "pnm") || str_ends_with(path, "PNM") ||
//        str_ends_with(path, "pgm") || str_ends_with(path, "PGM"))
//        im = image_u8_create_from_pnm(path);
//    else if (str_ends_with(path, "jpg") || str_ends_with(path, "JPG")) {
//        int err = 0;
//        pjpeg_t *pjpeg = pjpeg_create_from_file(path, 0, &err);
//        if (pjpeg == NULL) {
//            printf("pjpeg error %d\n", err);
//            continue;
//        }
//
//        if (1) {
//            im = pjpeg_to_u8_baseline(pjpeg);
//        } else {
//            printf("illumination invariant\n");
//
//            image_u8x3_t *imc =  pjpeg_to_u8x3_baseline(pjpeg);
//
//            im = image_u8_create(imc->width, imc->height);
//
//            for (int y = 0; y < imc->height; y++) {
//                for (int x = 0; x < imc->width; x++) {
//                    double r = imc->buf[y*imc->stride + 3*x + 0] / 255.0;
//                    double g = imc->buf[y*imc->stride + 3*x + 1] / 255.0;
//                    double b = imc->buf[y*imc->stride + 3*x + 2] / 255.0;
//
//                    double alpha = 0.42;
//                    double v = 0.5 + log(g) - alpha*log(b) - (1-alpha)*log(r);
//                    int iv = v * 255;
//                    if (iv < 0)
//                        iv = 0;
//                    if (iv > 255)
//                        iv = 255;
//
//                    im->buf[y*im->stride + x] = iv;
//                }
//            }
//            image_u8x3_destroy(imc);
//            if (td->debug)
//                image_u8_write_pnm(im, "debug_invariant.pnm");
//        }
//
//        pjpeg_destroy(pjpeg);
//    }

    // Use the luma channel (the first width*height elements)
    // as grayscale input image
    jbyte *buf = env->GetByteArrayElements(_buf, NULL);
    image_u8_t im = {
            .buf = (uint8_t *) buf,
            .height = height,
            .width = width,
            .stride = width
    };
    zarray_t *detections = apriltag_detector_detect(state.td, &im);
    env->ReleaseByteArrayElements(_buf, buf, 0);

    // al = new ArrayList();
    jobject al = env->NewObject(state.al_cls, state.al_constructor);
    for (int i = 0; i < zarray_size(detections); i += 1) {
        apriltag_detection_t *det;
        zarray_get(detections, i, &det);

        state.info.det = det;
        apriltag_pose_t pose;
        double err = estimate_tag_pose(&(state.info), &pose);

        // ap = new ApriltagPoseEstimation();
        jobject ap = env->NewObject(state.ap_cls, state.ap_constructor);
        env->SetIntField(ap, state.ad_id_field, det->id);

        jdoubleArray ad_c = static_cast<jdoubleArray>(env->GetObjectField(ap, state.ad_c_field));
        env->SetDoubleArrayRegion(ad_c, 0, 2, det->c);
        jdoubleArray ad_p = static_cast<jdoubleArray>(env->GetObjectField(ap, state.ad_p_field));
        env->SetDoubleArrayRegion(ad_p, 0, 8, (double *) det->p);

        jdoubleArray ap_r = static_cast<jdoubleArray>(env->GetObjectField(ap, state.ap_r_field));
        env->SetDoubleArrayRegion(ap_r, 0, 9, (double *) pose.R->data);
        jdoubleArray ap_t = static_cast<jdoubleArray>(env->GetObjectField(ap, state.ap_t_field));
        env->SetDoubleArrayRegion(ap_t, 0, 3, (double *) pose.t->data);

        // al.add(ap);
        env->CallBooleanMethod(al, state.al_add, ap);

        // Need to respect the local reference limit
        env->DeleteLocalRef(ap);
        env->DeleteLocalRef(ad_c);
        env->DeleteLocalRef(ad_p);
        env->DeleteLocalRef(ap_r);
        env->DeleteLocalRef(ap_t);
    }

    // Cleanup
    apriltag_detections_destroy(detections);

    return al;
}

extern "C"
JNIEXPORT jdoubleArray JNICALL
Java_com_liyang_droneplus_apriltags_ApriltagR_angle(JNIEnv *env, jclass clazz,
                                                    jdoubleArray array_) {
    // TODO: implement angle()
    jdouble *array = env->GetDoubleArrayElements(array_, NULL);
    jdoubleArray result;
    result = env->NewDoubleArray(3);
    if (result == NULL) {
        return NULL; /* out of memory error thrown */
    }

    jdouble array1[3];

    Matrix3d R;
    R << array[0], array[1], array[2], array[3], array[4], array[5], array[6], array[7], array[8];

    Vector3d eular_angle;
//    eular_angle = R.eulerAngles(0, 1, 2); //radian
    eular_angle = R.eulerAngles(0, 1, 2) * 180.f / M_PI; //angle

//    Matrix m = eular_angle;

    array1[0] = eular_angle[0];
    array1[1] = eular_angle[1];
    array1[2] = eular_angle[2];
    env->ReleaseDoubleArrayElements(array_, array, 0);

    env->SetDoubleArrayRegion(result, 0, 3, array1);
    return result;
}

extern "C"
JNIEXPORT void JNICALL
Java_com_liyang_droneplus_graduationproject_utils_YuvUtils_allocateMemo(JNIEnv *env, jclass clazz,
                                                                        jint src_yuv_length,
                                                                        jint src_argb_length,
                                                                        jint dst_length) {
    len_src = src_yuv_length;
    len_src_rgb = src_argb_length;
    len_scale = dst_length;

    input_src_data = static_cast<char *>(malloc(sizeof(char) * len_src));

    src_y_data = static_cast<char *>(malloc(sizeof(char) * (len_src * 2 / 3)));
    src_u_data = static_cast<char *>(malloc(sizeof(char) * (len_src / 6)));
    src_v_data = static_cast<char *>(malloc(sizeof(char) * (len_src / 6)));

    dst_y_data = static_cast<char *>(malloc(sizeof(char) * (len_src * 2 / 3)));
    dst_u_data = static_cast<char *>(malloc(sizeof(char) * (len_src / 6)));
    dst_v_data = static_cast<char *>(malloc(sizeof(char) * (len_src / 6)));

    output_processed_data = static_cast<char *>(malloc(sizeof(char) * len_scale));

}

extern "C"
JNIEXPORT void JNICALL
Java_com_liyang_droneplus_graduationproject_utils_YuvUtils_rgbToYuvBylibyuv(JNIEnv *env,
                                                                            jclass clazz,
                                                                            jobject src_bitmap,
                                                                            jbyteArray dst_yuv) {
    AndroidBitmapInfo infocolor;
    int ret;

    LOGI("convertToGray");
    if ((ret = AndroidBitmap_getInfo(env, src_bitmap, &infocolor)) < 0) {
        LOGE("AndroidBitmap_getInfo() failed ! error=%d", ret);
        return;
    }

    LOGI("color image :: width is %d; height is %d; stride is %d; format is %d;flags is %d",
         infocolor.width, infocolor.height, infocolor.stride, infocolor.format, infocolor.flags);

    if (infocolor.format != ANDROID_BITMAP_FORMAT_RGBA_8888) {
        LOGE("Bitmap format is not RGBA_8888 ! infocolor.format = %d", infocolor.format);
        return;
    } else {
        LOGE("Bitmap format is RGBA_8888 ! infocolor.format = %d", infocolor.format);
    }

    const uint8_t *argb_array = NULL;

    if ((ret = AndroidBitmap_lockPixels(env, src_bitmap, (void **) &argb_array)) < 0) {
        LOGE("AndroidBitmap_lockPixels() failed ! error=%d", ret);
        return;
    }

    LOGD("########## start do yuv convert #############\n");

    libyuv::ARGBToI420(static_cast<const uint8_t *>(argb_array), infocolor.stride,
                       reinterpret_cast<uint8_t *>(dst_y_data), infocolor.width,
                       reinterpret_cast<uint8_t *>(dst_u_data), infocolor.width / 2,
                       reinterpret_cast<uint8_t *>(dst_v_data), infocolor.width / 2,
                       infocolor.width, infocolor.height);


    // ARGBToI420 function format:
    // int ARGBToI420(const uint8* src_frame, int src_stride_frame,
    //                uint8* dst_y, int dst_stride_y,
    //                uint8* dst_u, int dst_stride_u,
    //                uint8* dst_v, int dst_stride_v,
    //                int width, int height);

    // merge y plane to output_data
    memcpy(output_processed_data, dst_y_data, (len_src * 2 / 3));
    // merge v plane to output_data
    memcpy(output_processed_data + (len_src * 2 / 3), dst_u_data, (len_src / 6));
    // merge u plane to output_data
    memcpy(output_processed_data + (len_src * 5 / 6), dst_v_data, (len_src / 6));

    LOGI("unlocking pixels");
    AndroidBitmap_unlockPixels(env, src_bitmap);
    (env)->SetByteArrayRegion(dst_yuv, 0, len_src, (jbyte *) (output_processed_data));
}

extern "C"
JNIEXPORT void JNICALL
Java_com_liyang_droneplus_graduationproject_utils_YuvUtils_releaseMemo(JNIEnv *env, jclass clazz) {
    // TODO: implement releaseMemo()
    LOGD("########## Release START#############\n");
    free(input_src_data);
    free(src_y_data);
    free(src_u_data);
    free(src_v_data);
    free(dst_y_data);
    free(dst_u_data);
    free(dst_v_data);
    free(output_processed_data);

    input_src_data = NULL;
    src_y_data = NULL;
    src_u_data = NULL;
    src_v_data = NULL;
    dst_y_data = NULL;
    dst_u_data = NULL;
    dst_v_data = NULL;
    output_processed_data = NULL;

    LOGD("########## Release OVER#############\n");
}

void BitmapToMat2(JNIEnv *env, jobject &bitmap, Mat &mat, jboolean needUnPremultiplyAlpha) {
    AndroidBitmapInfo info;
    void *pixels = 0;
    Mat &dst = mat;

    try {
        LOGD("nBitmapToMat");
        CV_Assert(AndroidBitmap_getInfo(env, bitmap, &info) >= 0);
        CV_Assert(info.format == ANDROID_BITMAP_FORMAT_RGBA_8888 ||
                  info.format == ANDROID_BITMAP_FORMAT_RGB_565);
        CV_Assert(AndroidBitmap_lockPixels(env, bitmap, &pixels) >= 0);
        CV_Assert(pixels);
        dst.create(info.height, info.width, CV_8UC4);
        if (info.format == ANDROID_BITMAP_FORMAT_RGBA_8888) {
            LOGD("nBitmapToMat: RGBA_8888 -> CV_8UC4");
            Mat tmp(info.height, info.width, CV_8UC4, pixels);
            if (needUnPremultiplyAlpha) cvtColor(tmp, dst, COLOR_mRGBA2RGBA);
            else tmp.copyTo(dst);
        } else {
            // info.format == ANDROID_BITMAP_FORMAT_RGB_565
            LOGD("nBitmapToMat: RGB_565 -> CV_8UC4");
            Mat tmp(info.height, info.width, CV_8UC2, pixels);
            cvtColor(tmp, dst, COLOR_BGR5652RGBA);
        }
        AndroidBitmap_unlockPixels(env, bitmap);
        return;
    } catch (const cv::Exception &e) {
        AndroidBitmap_unlockPixels(env, bitmap);
        LOGE("nBitmapToMat catched cv::Exception: %s", e.what());
        jclass je = env->FindClass("org/opencv/core/CvException");
        if (!je) je = env->FindClass("java/lang/Exception");
        env->ThrowNew(je, e.what());
        return;
    } catch (...) {
        AndroidBitmap_unlockPixels(env, bitmap);
        LOGE("nBitmapToMat catched unknown exception (...)");
        jclass je = env->FindClass("java/lang/Exception");
        env->ThrowNew(je, "Unknown exception in JNI code {nBitmapToMat}");
        return;
    }
}

void MatToBitmap2
        (JNIEnv *env, Mat &mat, jobject &bitmap, jboolean needPremultiplyAlpha) {
    AndroidBitmapInfo info;
    void *pixels = 0;
    Mat &src = mat;

    try {
        CV_Assert(AndroidBitmap_getInfo(env, bitmap, &info) >= 0);
        CV_Assert(info.format == ANDROID_BITMAP_FORMAT_RGBA_8888 ||
                  info.format == ANDROID_BITMAP_FORMAT_RGB_565);
        CV_Assert(src.dims == 2 && info.height == (uint32_t) src.rows &&
                  info.width == (uint32_t) src.cols);
        CV_Assert(src.type() == CV_8UC1 || src.type() == CV_8UC3 || src.type() == CV_8UC4);
        CV_Assert(AndroidBitmap_lockPixels(env, bitmap, &pixels) >= 0);
        CV_Assert(pixels);
        if (info.format == ANDROID_BITMAP_FORMAT_RGBA_8888) {
            Mat tmp(info.height, info.width, CV_8UC4, pixels);
            if (src.type() == CV_8UC1) {
                cvtColor(src, tmp, COLOR_GRAY2RGBA);
            } else if (src.type() == CV_8UC3) {
                cvtColor(src, tmp, COLOR_RGB2RGBA);
            } else if (src.type() == CV_8UC4) {
                if (needPremultiplyAlpha)
                    cvtColor(src, tmp, COLOR_RGBA2mRGBA);
                else
                    src.copyTo(tmp);
            }
        } else {
            // info.format == ANDROID_BITMAP_FORMAT_RGB_565
            Mat tmp(info.height, info.width, CV_8UC2, pixels);
            if (src.type() == CV_8UC1) {
                cvtColor(src, tmp, COLOR_GRAY2BGR565);
            } else if (src.type() == CV_8UC3) {
                cvtColor(src, tmp, COLOR_RGB2BGR565);
            } else if (src.type() == CV_8UC4) {
                cvtColor(src, tmp, COLOR_RGBA2BGR565);
            }
        }
        AndroidBitmap_unlockPixels(env, bitmap);
        return;
    } catch (const cv::Exception &e) {
        AndroidBitmap_unlockPixels(env, bitmap);
        jclass je = env->FindClass("org/opencv/core/CvException");
        if (!je) je = env->FindClass("java/lang/Exception");
        env->ThrowNew(je, e.what());
        return;
    } catch (...) {
        AndroidBitmap_unlockPixels(env, bitmap);
        jclass je = env->FindClass("java/lang/Exception");
        env->ThrowNew(je, "Unknown exception in JNI code {nMatToBitmap}");
        return;
    }
}

void BitmapToMat(JNIEnv *env, jobject &bitmap, Mat &mat) {
    BitmapToMat2(env, bitmap, mat, false);
}

void MatToBitmap(JNIEnv *env, Mat &mat, jobject &bitmap) {
    MatToBitmap2(env, mat, bitmap, false);
}

bool BitmapToMatrix(JNIEnv *env, jobject obj_bitmap, cv::Mat &matrix) {
    void *bitmapPixels;                                            // 保存图片像素数据
    AndroidBitmapInfo bitmapInfo;                                   // 保存图片参数

    ASSERT_FALSE(AndroidBitmap_getInfo(env, obj_bitmap, &bitmapInfo) >= 0);        // 获取图片参数

    ASSERT_FALSE(bitmapInfo.format == ANDROID_BITMAP_FORMAT_RGBA_8888
                 || bitmapInfo.format ==
                    ANDROID_BITMAP_FORMAT_RGB_565);          // 只支持 ARGB_8888 和 RGB_565

    ASSERT_FALSE(AndroidBitmap_lockPixels(env, obj_bitmap, &bitmapPixels) >= 0);  // 获取图片像素（锁定内存块）

    ASSERT_FALSE(bitmapPixels);

    if (bitmapInfo.format == ANDROID_BITMAP_FORMAT_RGBA_8888) {
        cv::Mat tmp(bitmapInfo.height, bitmapInfo.width, CV_8UC4, bitmapPixels);    // 建立临时 mat
        tmp.copyTo(matrix);                                                         // 拷贝到目标 matrix
    } else {
        cv::Mat tmp(bitmapInfo.height, bitmapInfo.width, CV_8UC2, bitmapPixels);
        cv::cvtColor(tmp, matrix, cv::COLOR_BGR5652RGB);
    }

    AndroidBitmap_unlockPixels(env, obj_bitmap);            // 解锁
    return true;
}

bool MatrixToBitmap(JNIEnv *env, cv::Mat &matrix, jobject obj_bitmap) {
    void *bitmapPixels;                                            // 保存图片像素数据
    AndroidBitmapInfo bitmapInfo;                                   // 保存图片参数

    ASSERT_FALSE(AndroidBitmap_getInfo(env, obj_bitmap, &bitmapInfo) >= 0);        // 获取图片参数
    ASSERT_FALSE(bitmapInfo.format == ANDROID_BITMAP_FORMAT_RGBA_8888
                 || bitmapInfo.format ==
                    ANDROID_BITMAP_FORMAT_RGB_565);          // 只支持 ARGB_8888 和 RGB_565
    ASSERT_FALSE(matrix.dims == 2
                 && bitmapInfo.height == (uint32_t) matrix.rows
                 && bitmapInfo.width == (uint32_t) matrix.cols);                   // 必须是 2 维矩阵，长宽一致
    ASSERT_FALSE(matrix.type() == CV_8UC1 || matrix.type() == CV_8UC3 || matrix.type() == CV_8UC4);
    ASSERT_FALSE(AndroidBitmap_lockPixels(env, obj_bitmap, &bitmapPixels) >= 0);  // 获取图片像素（锁定内存块）
    ASSERT_FALSE(bitmapPixels);

    if (bitmapInfo.format == ANDROID_BITMAP_FORMAT_RGBA_8888) {
        cv::Mat tmp(bitmapInfo.height, bitmapInfo.width, CV_8UC4, bitmapPixels);
        switch (matrix.type()) {
            case CV_8UC1:
                cv::cvtColor(matrix, tmp, cv::COLOR_GRAY2RGBA);
                break;
            case CV_8UC3:
                cv::cvtColor(matrix, tmp, cv::COLOR_RGB2RGBA);
                break;
            case CV_8UC4:
                matrix.copyTo(tmp);
                break;
            default:
                AndroidBitmap_unlockPixels(env, obj_bitmap);
                return false;
        }
    } else {
        cv::Mat tmp(bitmapInfo.height, bitmapInfo.width, CV_8UC2, bitmapPixels);
        switch (matrix.type()) {
            case CV_8UC1:
                cv::cvtColor(matrix, tmp, cv::COLOR_GRAY2BGR565);
                break;
            case CV_8UC3:
                cv::cvtColor(matrix, tmp, cv::COLOR_RGB2BGR565);
                break;
            case CV_8UC4:
                cv::cvtColor(matrix, tmp, cv::COLOR_RGBA2BGR565);
                break;
            default:
                AndroidBitmap_unlockPixels(env, obj_bitmap);
                return false;
        }
    }
    AndroidBitmap_unlockPixels(env, obj_bitmap);                // 解锁
    return true;
}

extern "C"
JNIEXPORT void JNICALL
Java_com_liyang_droneplus_apriltags_ApriltagR_JniBitmapExec(JNIEnv *env, jclass clazz,
                                                            jobject obj_bitmap, jlong mat) {
    // TODO: implement JniBitmapExec()

    cv::Mat matBitmap;
    bool ret = BitmapToMatrix(env, obj_bitmap, matBitmap);          // Bitmap 转 cv::Mat
    if (ret == false) {
        return;
    }

    Mat mat_image_src;
    BitmapToMat(env, obj_bitmap, mat_image_src);//图片转化成mat

    __android_log_print(ANDROID_LOG_ERROR, "mat_jni",
                        "mat_image_src.rows: %d, mat_image_src.cols: %d, mat_image_src.type(): %d",
                        mat_image_src.rows, mat_image_src.cols, mat_image_src.type());

    LOGI("开始获取mat...");
    Mat *res = (Mat *) mat;
    res->create(mat_image_src.rows, mat_image_src.cols, mat_image_src.type());
    memcpy(res->data, mat_image_src.data, mat_image_src.rows * mat_image_src.step);
    LOGI("获取成功");


//    cvtColor(mat_image_src, mat_image_src, COLOR_BGR2GRAY);
//    blur(mat_image_src, mat_image_src, Size(3, 3));
//    Canny(mat_image_src, mat_image_src, 20, 40, 4);
//
//    MatToBitmap(env, mat_image_src, obj_bitmap);


//    Rect2d matchMat;
//    bool findTarmac = FindTargetInFrame(mat_image_src, 2, matchMat, 0);
//    __android_log_print(ANDROID_LOG_ERROR, "findTarmac", ": %d", findTarmac);

    // 对 mat 进行 opencv 处理

//    ret = MatrixToBitmap(env, matBitmap, obj_bitmapOut);       // Bitmap 转 cv::Mat
//    if (ret == false) {
//        return;
//    }
}

extern "C"
JNIEXPORT void JNICALL
Java_com_liyang_droneplus_apriltags_ApriltagR_tarmacInit(JNIEnv *env, jclass clazz) {
    // TODO: implement tarmacInit()
    // Get ArrayList methods
    jclass al_cls = env->FindClass("java/util/ArrayList");
    if (!al_cls) {
        __android_log_write(ANDROID_LOG_ERROR, "tarmac_jni",
                            "couldn't find ArrayList class");
        return;
    }
    tarmac.al_cls = static_cast<jclass>(env->NewGlobalRef(al_cls));

    // env->GetMethodID获取类的方法，有3个参数，第一个是类句柄，第二个是方法名，第三个则是参数的签名
    tarmac.al_constructor = env->GetMethodID(al_cls, "<init>", "()V");
    tarmac.al_add = env->GetMethodID(al_cls, "add", "(Ljava/lang/Object;)Z");
    if (!tarmac.al_constructor || !tarmac.al_add) {
        __android_log_write(ANDROID_LOG_ERROR, "tarmac_jni",
                            "couldn't find ArrayList methods");
        return;
    }

    // Get ApriltagPoseEstimation methods
    jclass tr_cls = env->FindClass("com/liyang/droneplus/TarmacResult");

    if (!tr_cls) {
        __android_log_write(ANDROID_LOG_ERROR, "tarmac_jni",
                            "couldn't find TarmacResult class");
        return;
    }
    tarmac.tr_cls = static_cast<jclass>(env->NewGlobalRef(tr_cls));

    tarmac.tr_constructor = env->GetMethodID(tr_cls, "<init>", "()V");
    if (!tarmac.tr_constructor) {
        __android_log_write(ANDROID_LOG_ERROR, "tarmac_jni",
                            "couldn't find TarmacResult constructor");
        return;
    }

    tarmac.x = env->GetFieldID(tr_cls, "x", "D");

    tarmac.y = env->GetFieldID(tr_cls, "y", "D");
    tarmac.width = env->GetFieldID(tr_cls, "width", "D");
    tarmac.height = env->GetFieldID(tr_cls, "height", "D");
    tarmac.isTarmac = env->GetFieldID(tr_cls, "isTarmac", "Z");


    if (!tarmac.x ||
        !tarmac.y ||
        !tarmac.width ||
        !tarmac.height ||
        !tarmac.isTarmac) {
        __android_log_write(ANDROID_LOG_ERROR, "tarmac_jni",
                            "couldn't find TarmacResult fields");
        return;
    }
}

extern "C"
JNIEXPORT jobject JNICALL
Java_com_liyang_droneplus_apriltags_ApriltagR_findTargetInFrame(JNIEnv *env, jclass clazz,
                                                                jobject obj_bitmap, jint n_height,
                                                                jint width, jint height) {
    // TODO: implement findTargetInFrame()

    Mat mat_image_src;
    BitmapToMat(env, obj_bitmap, mat_image_src);//图片转化成mat

    __android_log_print(ANDROID_LOG_ERROR, "mat_jni",
                        "mat_image_src.rows: %d, mat_image_src.cols: %d, mat_image_src.type(): %d",
                        mat_image_src.rows, mat_image_src.cols, mat_image_src.type());

    Rect2d matchRect;
    bool findTarmac = FindTargetInFrame(mat_image_src, n_height, matchRect, 0);
    __android_log_print(ANDROID_LOG_ERROR, "findTarmac", ": %d", findTarmac);

    // al = new ArrayList();
    jobject al = env->NewObject(tarmac.al_cls, tarmac.al_constructor);

    // tr = new TarmacResult();
    jobject tr = env->NewObject(tarmac.tr_cls, tarmac.tr_constructor);

    env->SetDoubleField(tr, tarmac.x, matchRect.x);
    env->SetDoubleField(tr, tarmac.y, matchRect.y);
    env->SetDoubleField(tr, tarmac.width, matchRect.width);
    env->SetDoubleField(tr, tarmac.height, matchRect.height);
    env->SetBooleanField(tr, tarmac.isTarmac, findTarmac);

    // al.add(ap);
    env->CallBooleanMethod(al, state.al_add, tr);

    // Need to respect the local reference limit
    env->DeleteLocalRef(tr);

    return al;

}


/**********************************************KCF*****************************************************/
//初始化跟踪器
Ptr<Tracker> tracker;
Rect2d bbox;

int countKcf = 0;

extern "C"
JNIEXPORT void JNICALL
Java_com_liyang_droneplus_graduationproject_jni_NativeHelper_initKcf(JNIEnv *env, jobject thiz,
//                                                                     jintArray input_array,
//                                                                     jobject src_mat,
                                                                     jobject src_bitmap,
                                                                     jfloat left, jfloat top,
                                                                     jfloat right, jfloat bottom,
                                                                     jint width,
                                                                     jint height) {
    // TODO: implement initKcf()
    LOGE("tracker initing in JNI...");
//    int *inputArrayNew = env->GetIntArrayElements(input_array, NULL);
//    if (inputArrayNew == NULL) {
//        LOGE("inputArray is null,check input.");
//    }
//    cv::Mat frame(height, width, CV_8UC3, (unsigned char *) inputArrayNew);

//    Mat frame = reinterpret_cast<Mat &>(src_mat);

    Mat frame;
    BitmapToMat(env, src_bitmap, frame);//图片转化成mat
//    cvtColor(frame, frame, COLOR_BGRA2BGR);
    cvtColor(frame, frame, COLOR_BGRA2RGB);

    bbox.x = left;
    bbox.y = top;
    bbox.width = right - left;
    bbox.height = bottom - top;

    rectangle(frame, bbox, Scalar(255, 0, 0), 2, 1);
    imwrite("/storage/emulated/0/result/readYuvkcf.jpg", frame);
    __android_log_print(ANDROID_LOG_ERROR, "mat_jni",
                        "frame.rows: %d, frame.cols: %d, frame.type(): %d",
                        frame.rows, frame.cols, frame.type());
    tracker = TrackerKCF::create();
    LOGE("tracker init finished in JNI...");

    //跟踪器初始化
    tracker->init(frame, bbox);
}

extern "C"
JNIEXPORT jobject JNICALL
Java_com_liyang_droneplus_graduationproject_jni_NativeHelper_usingKcf(JNIEnv *env, jobject thiz,
//                                                                      jintArray input_array,
//                                                                      jobject src_mat,
                                                                      jobject src_bitmap,
                                                                      jint width, jint height) {
    // TODO: implement usingKcf()

    jclass cSructInfo = env->FindClass(
            "com/liyang/droneplus/graduationproject/tracking/KCFResultFormJNI");
    jfieldID cXLoc = env->GetFieldID(cSructInfo, "x", "I");
    jfieldID cYLoc = env->GetFieldID(cSructInfo, "y", "I");
    jfieldID cWidthLoc = env->GetFieldID(cSructInfo, "width", "I");
    jfieldID cHeightLoc = env->GetFieldID(cSructInfo, "height", "I");
    //新建Jni类对象
    jobject oStructInfo = env->AllocObject(cSructInfo);

//    int *inputArrayNew = env->GetIntArrayElements(input_array, NULL);
//    if (inputArrayNew == NULL) {
//        LOGE("inputArray is null,check input.");
//    }

//    cv::Mat frame(height, width, CV_8UC3, (unsigned char *) inputArrayNew);

//    Mat frame = reinterpret_cast<Mat &>(src_mat);

    Mat frame;
    BitmapToMat(env, src_bitmap, frame);//图片转化成mat
    cvtColor(frame, frame, COLOR_BGRA2RGB);
    __android_log_print(ANDROID_LOG_ERROR, "mat_jni",
                        "frame.rows: %d, frame.cols: %d, frame.type(): %d",
                        frame.rows, frame.cols, frame.type());
    bool ok = tracker->update(frame, bbox);

    if (ok) {
        rectangle(frame, bbox, Scalar(255, 0, 0), 2, 1);
        ostringstream oss;
        oss << "/storage/emulated/0/result/readYuvrectangle" << countKcf++ << ".jpg";
        cout << oss.str() << endl;

        imwrite(oss.str(), frame);
        LOGE("update is finish,status is ok.");
        int x = bbox.x;
        int y = bbox.y;
        int width = bbox.width;
        int height = bbox.height;
        env->SetIntField(oStructInfo, cXLoc, x);
        env->SetIntField(oStructInfo, cYLoc, y);
        env->SetIntField(oStructInfo, cWidthLoc, width);
        env->SetIntField(oStructInfo, cHeightLoc, height);
        //释放数组
//        env->ReleaseIntArrayElements(input_array, inputArrayNew, 0);
    } else {
        LOGE("update is finish,status is not ok.");
        env->SetIntField(oStructInfo, cXLoc, 0);
        env->SetIntField(oStructInfo, cYLoc, 0);
        env->SetIntField(oStructInfo, cWidthLoc, 0);
        env->SetIntField(oStructInfo, cHeightLoc, 0);
        //释放数组
//        env->ReleaseIntArrayElements(input_array, inputArrayNew, 0);
    }

    return oStructInfo;
}
/**********************************************KCF*****************************************************/

/**********************************************FDSST*****************************************************/
//初始化跟踪器
Rect2d bboxForF;
Point center;
//动态数组存储坐标点
vector<Point2d> points;
Mat tmp, dst;

int countFdsst = 0;

bool HOG = true;
bool FIXEDWINDOW = false;
bool MULTISCALE = true;
bool SILENT = true;
bool LAB = false;
// Create DSSTTracker tracker object
FDSSTTracker trackerForF(HOG, FIXEDWINDOW, MULTISCALE, LAB);

extern "C"
JNIEXPORT void JNICALL
Java_com_liyang_droneplus_graduationproject_jni_NativeHelper_initFdsst(JNIEnv *env, jobject thiz,
                                                                       jobject src_bitmap,
                                                                       jfloat left, jfloat top,
                                                                       jfloat right, jfloat bottom,
                                                                       jint width,
                                                                       jint height) {
    // TODO: implement initFdsst()
    LOGE("tracker initing in JNI...");

    Mat frame;
    BitmapToMat(env, src_bitmap, frame);//图片转化成mat
    cvtColor(frame, frame, COLOR_BGRA2RGB);

    __android_log_print(ANDROID_LOG_ERROR, "mat_jni",
                        "frame.rows: %d, frame.cols: %d, frame.type(): %d",
                        frame.rows, frame.cols, frame.type());
    tracker = TrackerKCF::create();
    LOGE("tracker init finished in JNI...");
    bboxForF.x = left;
    bboxForF.y = top;
    bboxForF.width = right - left;
    bboxForF.height = bottom - top;
    rectangle(frame, bboxForF, Scalar(255, 0, 0), 2, 1);
    imwrite("/storage/emulated/0/result/readYuvfdsst.jpg", frame);
    //跟踪器初始化
    cvtColor(frame, dst, CV_BGR2GRAY);
    trackerForF.init(bboxForF, dst);
}

extern "C"
JNIEXPORT jobject JNICALL
Java_com_liyang_droneplus_graduationproject_jni_NativeHelper_usingFdsst(JNIEnv *env, jobject thiz,
                                                                        jobject src_bitmap,
                                                                        jint width, jint height) {
    // TODO: implement usingFdsst()

    jclass cSructInfo = env->FindClass(
            "com/liyang/droneplus/graduationproject/tracking/FDSSTResultFormJNI");
    jfieldID cXLoc = env->GetFieldID(cSructInfo, "x", "I");
    jfieldID cYLoc = env->GetFieldID(cSructInfo, "y", "I");
    jfieldID cWidthLoc = env->GetFieldID(cSructInfo, "width", "I");
    jfieldID cHeightLoc = env->GetFieldID(cSructInfo, "height", "I");
    //新建Jni类对象
    jobject oStructInfo = env->AllocObject(cSructInfo);


    Mat frame;
    BitmapToMat(env, src_bitmap, frame);//图片转化成mat
    cvtColor(frame, frame, COLOR_BGRA2RGB);

    __android_log_print(ANDROID_LOG_ERROR, "mat_jni",
                        "frame.rows: %d, frame.cols: %d, frame.type(): %d",
                        frame.rows, frame.cols, frame.type());

    cvtColor(frame, dst,CV_RGB2GRAY);
    bboxForF = trackerForF.update(dst);

    LOGE("update is finish,status is ok.");
    int bboxx = bboxForF.x;
    int bboxy = bboxForF.y;
    int bboxwidth = bboxForF.width;
    int bboxheight = bboxForF.height;
    env->SetIntField(oStructInfo, cXLoc, bboxx);
    env->SetIntField(oStructInfo, cYLoc, bboxy);
    env->SetIntField(oStructInfo, cWidthLoc, bboxwidth);
    env->SetIntField(oStructInfo, cHeightLoc, bboxheight);

    ostringstream oss;
    oss << "/storage/emulated/0/result/readFdsstRectangle" << countFdsst++ << ".jpg";
    cout << oss.str() << endl;
    imwrite(oss.str(), frame);

    return oStructInfo;
}
/**********************************************FDSST*****************************************************/

/**********************************************TEST*****************************************************/
extern "C"
JNIEXPORT jintArray JNICALL
Java_com_liyang_droneplus_graduationproject_test_PictureConversionTestActivity_gray(JNIEnv *env,
                                                                                    jobject thiz,
                                                                                    jintArray pix_,
                                                                                    jint w,
                                                                                    jint h) {
    // TODO: implement gray()
    jint *pix = env->GetIntArrayElements(pix_, NULL);
    if (pix == NULL) {
        return 0;
    }
#if 1
    //将c++图片转成Opencv图片
    Mat imgData(h, w, CV_8UC4, (unsigned char *) pix);
    uchar *ptr = imgData.ptr(0);
    for (int i = 0; i < w * h; i++) {
        //计算公式：Y(亮度) = 0.299*R + 0.587*G + 0.114*B
        //对于一个int四字节，其彩色值存储方式为：BGRA
        int grayScale = (int) (ptr[4 * i + 2] * 0.299 + ptr[4 * i + 1] * 0.587 +
                               ptr[4 * i + 0] * 0.114);
        ptr[4 * i + 1] = grayScale;
        ptr[4 * i + 2] = grayScale;
        ptr[4 * i + 0] = grayScale;
    }
#endif
    int size = w * h;
    jintArray result = env->NewIntArray(size);
    env->SetIntArrayRegion(result, 0, size, pix);
    env->ReleaseIntArrayElements(pix_, pix, 0);
    return result;

}
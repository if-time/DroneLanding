package com.liyang.droneplus.graduationproject.jni;

import com.liyang.droneplus.graduationproject.tracking.FDSSTResultFormJNI;
import com.liyang.droneplus.graduationproject.tracking.KCFResultFormJNI;

/**
 * @author dongsiyuan
 * @date 2020年10月29日
 */
public class NativeHelper {

    static{
        System.loadLibrary("apriltag");
    }

    public static final String TAG = NativeHelper.class.getSimpleName();

    private static NativeHelper instance;

    public static NativeHelper getInstance() {
        if (instance == null) {
            instance = new NativeHelper();
        }
        return instance;
    }

    private NativeHelper() {
    }

    private NativeDataListener dataListener;

    public void setDataListener(NativeDataListener dataListener) {
        this.dataListener = dataListener;
    }

    /**********************************************KCF*****************************************************/
//    public native void initKcf(int[] inputArray, float left, float top, float right, float bottom, int width, int height);
//    public native void initKcf(Object srcMat, float left, float top, float right, float bottom, int width, int height);
    public native void initKcf(Object srcBitmap, float left, float top, float right, float bottom, int width, int height);
//    public native KCFResultFormJNI usingKcf(int[] inputArray, int width, int height);
//    public native KCFResultFormJNI usingKcf(Object srcMat, int width, int height);
    public native KCFResultFormJNI usingKcf(Object srcBitmap, int width, int height);
    /**********************************************KCF*****************************************************/

    /**********************************************FDSST*****************************************************/
    public native void initFdsst(Object srcBitmap, float left, float top, float right, float bottom, int width, int height);

    public native FDSSTResultFormJNI usingFdsst(Object srcBitmap, int width, int height);

    /**********************************************FDSST*****************************************************/

    public interface NativeDataListener {
        /**
         * Callback method for receiving the frame data from NativeHelper.
         * Note that this method will be invoke in framing thread, which means time consuming
         * processing should not in this thread, or the framing process will be blocked.
         * @param data
         * @param size
         * @param frameNum
         * @param isKeyFrame
         * @param width
         * @param height
         */
        void onDataRecv(byte[] data, int size, int frameNum, boolean isKeyFrame, int width, int height);
    }

    /**
     * Invoke by JNI
     * Callback the frame data.
     * @param buf
     * @param size
     * @param frameNum
     * @param isKeyFrame
     * @param width
     * @param height
     */
    public void onFrameDataRecv(byte[] buf, int size, int frameNum, boolean isKeyFrame, int width, int height) {
        if (dataListener != null) {
            dataListener.onDataRecv(buf, size, frameNum, isKeyFrame, width, height);
        }
    }
}

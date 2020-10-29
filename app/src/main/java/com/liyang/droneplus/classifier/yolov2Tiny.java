package com.liyang.droneplus.classifier;
import android.graphics.Bitmap;

public class yolov2Tiny {
    public native boolean Init(byte[] param, byte[] bin);
    public native boolean Clear();
    public native float[] Detect(Bitmap bitmap);
    static {
        System.loadLibrary(yolov2Tiny.class.getName().substring(yolov2Tiny.class.getName().lastIndexOf(".")+1));
//        System.loadLibrary("yolov2_jni");
    }

}

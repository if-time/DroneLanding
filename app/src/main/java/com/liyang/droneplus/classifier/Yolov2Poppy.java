package com.liyang.droneplus.classifier;

import android.graphics.Bitmap;

public class Yolov2Poppy {
    public native boolean Init(byte[] param, byte[] bin);
    public native boolean Clear();
    public native float[] Detect(Bitmap bitmap);
    static {
//        System.loadLibrary(Yolov2Poppy.class.getName().substring(Yolov2Poppy.class.getName().lastIndexOf(".")+1));
        System.loadLibrary("Yolov2Poppy");
        System.out.println("aaaaaaaaaaaaaaaaaaaaaaaaa"+ Yolov2Poppy.class.getName().substring(Yolov2Poppy.class.getName().lastIndexOf(".")+1));
    }
}

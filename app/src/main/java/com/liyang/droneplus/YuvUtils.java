package com.liyang.droneplus;

public class YuvUtils {

    public static native void allocateMemo(int src_yuv_length, int src_argb_length, int dst_length);

    public static native void rgbToYuvBylibyuv(Object srcBitmap, byte[] dst_yuv);

    public static native void releaseMemo();

}

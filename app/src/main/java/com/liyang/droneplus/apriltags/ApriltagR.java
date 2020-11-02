package com.liyang.droneplus.apriltags;

import android.graphics.Bitmap;
import android.os.Environment;
import android.util.Log;

import com.liyang.droneplus.TarmacResult;

import org.opencv.core.Mat;
import org.opencv.imgcodecs.Imgcodecs;

import java.io.File;
import java.io.FileOutputStream;
import java.io.IOException;
import java.util.ArrayList;

public class ApriltagR {
    static {
        System.loadLibrary("apriltag");
        nativeInit();
        tarmacInit();
    }

    //JNI

    public static native void nativeInit();

    //    public static native void yuv_to_rgb(byte[] src, int width, int height, Bitmap dst);

    //    public static native void apriltag_init(String tagFamily, int errorBits, double decimateFactor,
    //                                            double blurSigma, int nthreads);
    public static native void apriltagInit(String tagFamily, int errorBits, double decimateFactor, double blurSigma, int nthreads, double tagsize, double fx, double fy, double cx, double cy);


    public static native ArrayList<ApriltagPoseEstimation> apriltagDetectYuv(byte[] src, int width, int height);

    public static native double[] angle(double[] array);


    public static native void JniBitmapExec(Object srcBitmap, long mat);

    public static native void tarmacInit();

    public static native ArrayList<TarmacResult> findTargetInFrame(Object srcBitmap, int nHeight, int width, int height);


    public static String resultPath = Environment.getExternalStorageDirectory().getAbsolutePath() + "/result";

    public static final void matSave(String fileName, Mat mat) {
        Log.i("MainActivity", "1111111111111");
        if (mat.empty())
            Log.i("dongempty", "matSave: ");
        Imgcodecs.imwrite(fileName, mat);
        Log.i("MainActivity", "保存成功！");
    }

    /**
     * 保存bitmap到本地
     *
     * @param bitmap Bitmap
     */
    public static void saveBitmap(Bitmap bitmap, String path) {
        String savePath;
        File filePic;
        if (Environment.getExternalStorageState().equals(Environment.MEDIA_MOUNTED)) {
            savePath = path;
        } else {
            Log.e("tag", "saveBitmap failure : sdcard not mounted");
            return;
        }
        try {
            filePic = new File(savePath);
            if (!filePic.exists()) {
                filePic.getParentFile().mkdirs();
                filePic.createNewFile();
            }
            FileOutputStream fos = new FileOutputStream(filePic);
            bitmap.compress(Bitmap.CompressFormat.JPEG, 100, fos);
            fos.flush();
            fos.close();
        } catch (IOException e) {
            Log.e("tag", "saveBitmap: " + e.getMessage());
            return;
        }
        Log.i("tag", "saveBitmap success: " + filePic.getAbsolutePath());
    }
}

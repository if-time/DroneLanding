package com.liyang.droneplus.classifier;

import android.graphics.Bitmap;
import android.graphics.RectF;
import android.util.Log;

import com.liyang.droneplus.apriltags.ApriltagPoseEstimation;
import com.liyang.droneplus.apriltags.ApriltagR;
import com.liyang.droneplus.bean.DetectModel;
import com.liyang.droneplus.util.ImageUtils;

import java.util.ArrayList;
import java.util.List;

public class ApriltagsDetectAPIModel implements RecognizerForAprilTags {

    public double center_x, center_y, left_top_x, left_top_y, right_top_x, right_top_y, right_bottom_x, right_bottom_y, left_bottom_x, left_bottom_y;
    private ArrayList<ApriltagPoseEstimation> mPoses;

    long start3;

    public ApriltagsDetectAPIModel() {
    }

    public List<Recognition> predict_image(Bitmap bitmap) {

        List<Recognition> recognitions = new ArrayList<>();
        try {
            long start = System.currentTimeMillis();
//            byte[] data = ImageUtils.bitmapToNv21(bitmap, bitmap.getWidth(), bitmap.getHeight());
//            Log.i("dongpredict_image", "run: tonv21 : " + (System.currentTimeMillis() - start) + "ms");
            byte[] data = ImageUtils.libyuvToNv21(bitmap);
            Log.i("dongpredict_image", "run: libyuvToNv21 : " + (System.currentTimeMillis() - start) + "ms");
//            Log.i("dongwh", "  x: " + "  y: " + "  z: " + "h : " + bitmap.getHeight() + " w : " + bitmap.getWidth());

            long start2 = System.currentTimeMillis();
            mPoses = ApriltagR.apriltagDetectYuv(data, bitmap.getWidth(), bitmap.getHeight());
            Log.i("dongpredict_image", "run: Detect : " + (System.currentTimeMillis() - start2) + "ms");
//            Log.i("dongmPoses", "  mPoses: " + mPoses.size());

            start3 = System.currentTimeMillis();
            float[] points = new float[8];

            double angle_x = 0;
            double angle_y = 0;
            double angle_z = 0;

            double t_x = 0;
            double t_y = 0;
            double t_z = 0;

            double c_x = 0;
            double c_y = 0;
            for (ApriltagPoseEstimation pose : mPoses) {
                double[] f = ApriltagR.angle(pose.R);

//                Log.i("dongangle", "  x: " + f[0] + "  y: " + f[1] + "  z: " + f[2]);
                Log.i("dongT", "  x: " + pose.t[0] * 1000 + "  y: " + pose.t[1] * 1000 + "  z: " + pose.t[2]);

                c_x = pose.c[0];
                c_y = pose.c[1];


                for (int i = 0; i < 4; i += 1) {
                    double x = 0.5 - (pose.p[2 * i + 1] / bitmap.getHeight());
                    double y = 0.5 - (pose.p[2 * i + 0] / bitmap.getWidth());
                    points[2 * i + 0] = (float) x;
                    points[2 * i + 1] = (float) y;
                }

                angle_x = f[0];
                angle_y = f[1];
                angle_z = f[2];

                t_x = pose.t[0] * 1000;
                t_y = pose.t[1] * 1000;
                t_z = pose.t[2];

                left_top_x = pose.p[0];
                left_top_y = pose.p[1];
                right_top_x = pose.p[2];
                right_top_y = pose.p[3];
                right_bottom_x = pose.p[4];
                right_bottom_y = pose.p[5];
                left_bottom_x = pose.p[6];
                left_bottom_y = pose.p[7];

//                Log.i("dongpointss", "processImageForAprilTags: c_x : " + c_x + " c_y : " + c_y + " " + ((right_top_x - left_top_x) / 2 + left_top_x) + " " +
//                        ((right_bottom_y - right_top_y) / 2 + right_top_y));
//
//                Log.i("dongpoints", "  points1: " + pose.p[0] + "  points2: " + pose.p[1] + "  points3: " + pose.p[2] +
//                        "  points4: " + pose.p[3] + "  points5: " + pose.p[4] + "  points6: " + pose.p[5] +
//                        "  points7: " + pose.p[6] + "  points8: " + pose.p[7]);
//
//                Log.i("dongpoint", "  points1: " + points[0] + "  points2: " + points[1] + "  points3: " + points[2] +
//                        "  points4: " + points[3] + "  points5: " + points[4] + "  points6: " + points[5] +
//                        "  points7: " + points[6] + "  points8: " + points[7]);

                recognitions.add(new Recognition(pose.id,
                        c_x, c_y,
                        left_top_x, left_top_y, right_top_x, right_top_y,
                        right_bottom_x, right_bottom_y, left_bottom_x, left_bottom_y,
                        angle_x, angle_y, angle_z,
                        t_x, t_y, t_z,
                        new RectF((float) left_top_x, (float) left_top_y, (float) right_bottom_x, (float) right_bottom_y)));
            }
        } catch (Exception e) {
            e.printStackTrace();
        }
        Log.i("dongpredict_image", "run: ReturnResult : " + (System.currentTimeMillis() - start3) + "ms");

        return recognitions;
    }

    @Override
    public List<Recognition> recognizeImage(Bitmap bitmap) {
        List<Recognition> res = predict_image(bitmap);
        return res;
    }

    @Override
    public void close() {

    }

    @Override
    public void setModel(ArrayList<DetectModel> models) {

    }
}

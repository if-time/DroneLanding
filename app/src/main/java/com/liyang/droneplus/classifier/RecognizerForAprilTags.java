package com.liyang.droneplus.classifier;

import android.graphics.Bitmap;
import android.graphics.RectF;

import com.liyang.droneplus.bean.DetectModel;

import java.util.ArrayList;
import java.util.List;

public interface RecognizerForAprilTags {

    class Recognition {
        //    public ApriltagDetection apd;
        // The decoded ID of the tag
        public int id;
        // The center of the detection in image pixel coordinates.
        //            public double[] c = new double[2];

        // The corners of the tag in image pixel coordinates. These always
        // wrap counter-clock wise around the tag.
        // Flattened to [x0 y0 x1 y1 ...] for JNI convenience
        //            public double[] p = new double[8];

        private double center_x, center_y;

        private double left_top_x, left_top_y, right_top_x, right_top_y, right_bottom_x, right_bottom_y, left_bottom_x, left_bottom_y;

        private double angle_x, angle_y, angle_z;

        private RectF location;

        private double t_x;
        private double t_y;
        private double t_z;

        public Recognition(int id,
                           double center_x, double center_y,
                           double left_top_x, double left_top_y, double right_top_x, double right_top_y,
                           double right_bottom_x, double right_bottom_y, double left_bottom_x, double left_bottom_y,
                           double angle_x, double angle_y, double angle_z,
                           double t_x , double t_y, double t_z,
                           RectF location) {
            this.id = id;
            this.center_x = center_x;
            this.center_y = center_y;
            this.left_top_x = left_top_x;
            this.left_top_y = left_top_y;
            this.right_top_x = right_top_x;
            this.right_top_y = right_top_y;
            this.right_bottom_x = right_bottom_x;
            this.right_bottom_y = right_bottom_y;
            this.left_bottom_x = left_bottom_x;
            this.left_bottom_y = left_bottom_y;
            this.angle_x = angle_x;
            this.angle_y = angle_y;
            this.angle_z = angle_z;
            this.t_x = t_x;
            this.t_y = t_y;
            this.t_z = t_z;
            this.location = location;
        }

        public RectF getLocation() {
            return location;
        }

        public void setLocation(RectF location) {
            this.location = location;
        }

        public int getId() {
            return id;
        }

        public void setId(int id) {
            this.id = id;
        }

        public double getCenter_x() {
            return center_x;
        }

        public void setCenter_x(double center_x) {
            this.center_x = center_x;
        }

        public double getCenter_y() {
            return center_y;
        }

        public void setCenter_y(double center_y) {
            this.center_y = center_y;
        }

        public double getLeft_top_x() {
            return left_top_x;
        }

        public void setLeft_top_x(double left_top_x) {
            this.left_top_x = left_top_x;
        }

        public double getLeft_top_y() {
            return left_top_y;
        }

        public void setLeft_top_y(double left_top_y) {
            this.left_top_y = left_top_y;
        }

        public double getRight_top_x() {
            return right_top_x;
        }

        public void setRight_top_x(double right_top_x) {
            this.right_top_x = right_top_x;
        }

        public double getRight_top_y() {
            return right_top_y;
        }

        public void setRight_top_y(double right_top_y) {
            this.right_top_y = right_top_y;
        }

        public double getRight_bottom_x() {
            return right_bottom_x;
        }

        public void setRight_bottom_x(double right_bottom_x) {
            this.right_bottom_x = right_bottom_x;
        }

        public double getRight_bottom_y() {
            return right_bottom_y;
        }

        public void setRight_bottom_y(double right_bottom_y) {
            this.right_bottom_y = right_bottom_y;
        }

        public double getLeft_bottom_x() {
            return left_bottom_x;
        }

        public void setLeft_bottom_x(double left_bottom_x) {
            this.left_bottom_x = left_bottom_x;
        }

        public double getLeft_bottom_y() {
            return left_bottom_y;
        }

        public void setLeft_bottom_y(double left_bottom_y) {
            this.left_bottom_y = left_bottom_y;
        }

        public double getAngle_x() {
            return angle_x;
        }

        public void setAngle_x(double angle_x) {
            this.angle_x = angle_x;
        }

        public double getAngle_y() {
            return angle_y;
        }

        public void setAngle_y(double angle_y) {
            this.angle_y = angle_y;
        }

        public double getAngle_z() {
            return angle_z;
        }

        public void setAngle_z(double angle_z) {
            this.angle_z = angle_z;
        }

        public double getT_x() {
            return t_x;
        }

        public void setT_x(double t_x) {
            this.t_x = t_x;
        }

        public double getT_y() {
            return t_y;
        }

        public void setT_y(double t_y) {
            this.t_y = t_y;
        }

        public double getT_z() {
            return t_z;
        }

        public void setT_z(double t_z) {
            this.t_z = t_z;
        }
    }

    List<Recognition> recognizeImage(Bitmap bitmap);

    void close();

    void setModel(ArrayList<DetectModel> models);
}



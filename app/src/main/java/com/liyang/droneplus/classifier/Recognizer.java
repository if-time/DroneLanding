package com.liyang.droneplus.classifier;

import android.graphics.Bitmap;
import android.graphics.RectF;

import com.liyang.droneplus.bean.DetectModel;

import java.util.ArrayList;
import java.util.List;

public interface Recognizer {
    class Recognition {
        private final String name;
        private final Float confidence;
        private RectF location;
        private long time;
        private float left;
        private float top;
        private float right;
        private float bottom;

        public Recognition(String name, Float confidence, RectF location, long time, float left, float top,
                           float right, float bottom) {
            this.name = name;
            this.confidence = confidence;
            this.location = location;
            this.time = time;
            this.left = left;
            this.top = top;
            this.right = right;
            this.bottom = bottom;
        }

        public String getName() {
            return name;
        }

        public Float getConfidence() {
            return confidence;
        }

        public RectF getLocation() {
            return location;
        }

        public void setLocation(RectF location) {
            this.location = location;
        }

        public long getTime() {
            return time;
        }

        public void setTime(long time) {
            this.time = time;
        }

        public float getLeft() {
            return left;
        }

        public void setLeft(float left) {
            this.left = left;
        }

        public float getTop() {
            return top;
        }

        public void setTop(float top) {
            this.top = top;
        }

        public float getRight() {
            return right;
        }

        public void setRight(float right) {
            this.right = right;
        }

        public float getBottom() {
            return bottom;
        }

        public void setBottom(float bottom) {
            this.bottom = bottom;
        }
    }
    List<Recognition> recognizeImage(Bitmap bitmap);

    void close();

    void setModel(ArrayList<DetectModel> models);
}

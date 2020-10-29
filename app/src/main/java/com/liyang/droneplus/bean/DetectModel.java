package com.liyang.droneplus.bean;

public class DetectModel {
    private float left;
    private float top;
    private float right;
    private float bottom;
    private String name;
    private float confidence;

    public DetectModel() {
    }

    public DetectModel(float left, float top, float right, float bottom, String name, float confidence) {
        this.left = left;
        this.top = top;
        this.right = right;
        this.bottom = bottom;
        this.name = name;
        this.confidence = confidence;
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

    public String getName() {
        return name;
    }

    public void setName(String name) {
        this.name = name;
    }

    public float getConfidence() {
        return confidence;
    }

    public void setConfidence(float confidence) {
        this.confidence = confidence;
    }
}

package com.liyang.droneplus.bean;

/**
 * Created by zh931 on 2018/9/3.
 */

public class RecognitionResultItem {

    private String time;
    private String result;
    private String probability;
    private String recognitionTime;
    private String left;
    private String top;
    private String right;
    private String bottom;
    private String time2;

    public RecognitionResultItem(String time, String result, String probability,
                                 String recognitionTime, String left, String top,
                                 String right, String bottom, String time2) {
        this.time = time;
        this.result = result;
        this.probability = probability;
        this.recognitionTime = recognitionTime;
        this.left = left;
        this.top = top;
        this.right = right;
        this.bottom = bottom;
        this.time2 = time2;
    }

    public String getTime() {
        return time;
    }

    public void setTime(String time) {
        this.time = time;
    }

    public String getResult() {
        return result;
    }

    public void setResult(String result) {
        this.result = result;
    }

    public String getProbability() {
        return probability;
    }

    public void setProbability(String probability) {
        this.probability = probability;
    }

    public String getRecognitionTime() {
        return recognitionTime;
    }

    public void setRecognitionTime(String recognitionTime) {
        this.recognitionTime = recognitionTime;
    }

    public String getLeft() {
        return left;
    }

    public void setLeft(String left) {
        this.left = left;
    }

    public String getTop() {
        return top;
    }

    public void setTop(String top) {
        this.top = top;
    }

    public String getRight() {
        return right;
    }

    public void setRight(String right) {
        this.right = right;
    }

    public String getBottom() {
        return bottom;
    }

    public void setBottom(String bottom) {
        this.bottom = bottom;
    }

    public String getTime2() {
        return time2;
    }

    public void setTime2(String time2) {
        this.time2 = time2;
    }
}

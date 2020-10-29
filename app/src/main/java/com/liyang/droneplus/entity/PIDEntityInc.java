package com.liyang.droneplus.entity;

public class PIDEntityInc {

    private float setValue; // 设定值
    private float actualValue; // 实际值（测量值）
    private float err; // 当前偏差
    private float lastErr; // 上一个偏差
    private float beforLastErr; // 再上一个偏差
    private float kp;
    private float ki;
    private float kd;
    private float controlValue;

    public PIDEntityInc() {
    }

    public float getSetValue() {
        return setValue;
    }

    public void setSetValue(float setValue) {
        this.setValue = setValue;
    }

    public float getActualValue() {
        return actualValue;
    }

    public void setActualValue(float actualValue) {
        this.actualValue = actualValue;
    }

    public float getErr() {
        return err;
    }

    public void setErr(float err) {
        this.err = err;
    }

    public float getLastErr() {
        return lastErr;
    }

    public void setLastErr(float lastErr) {
        this.lastErr = lastErr;
    }

    public float getBeforLastErr() {
        return beforLastErr;
    }

    public void setBeforLastErr(float beforLastErr) {
        this.beforLastErr = beforLastErr;
    }

    public float getKp() {
        return kp;
    }

    public void setKp(float kp) {
        this.kp = kp;
    }

    public float getKi() {
        return ki;
    }

    public void setKi(float ki) {
        this.ki = ki;
    }

    public float getKd() {
        return kd;
    }

    public void setKd(float kd) {
        this.kd = kd;
    }

    public float getControlValue() {
        return controlValue;
    }

    public void setControlValue(float controlValue) {
        this.controlValue = controlValue;
    }
}

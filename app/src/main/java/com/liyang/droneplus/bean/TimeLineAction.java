package com.liyang.droneplus.bean;

import dji.common.mission.waypoint.Waypoint;

public class TimeLineAction {
    private Waypoint waypoint;
    private boolean hotpoint;
    private int radius;
    private float hotheight;
    private int action;
    private int gimbalangle;
    private int yawangle;

    public TimeLineAction(Waypoint waypoint, boolean hotpoint, int radius, float hotheight, int action, int gimbalangle, int yawangle) {
        this.waypoint = waypoint;
        this.hotpoint = hotpoint;
        this.radius = radius;
        this.hotheight = hotheight;
        this.action = action;
        this.gimbalangle = gimbalangle;
        this.yawangle = yawangle;
    }

    public Waypoint getWaypoint() {
        return waypoint;
    }

    public void setWaypoint(Waypoint waypoint) {
        this.waypoint = waypoint;
    }

    public boolean isHotpoint() {
        return hotpoint;
    }

    public void setHotpoint(boolean hotpoint) {
        this.hotpoint = hotpoint;
    }

    public int getRadius() {
        return radius;
    }

    public void setRadius(int radius) {
        this.radius = radius;
    }

    public int getAction() {
        return action;
    }

    public void setAction(int action) {
        this.action = action;
    }

    public float getHotheight() {
        return hotheight;
    }

    public void setHotheight(float hotheight) {
        this.hotheight = hotheight;
    }

    public int getGimbalangle() {
        return gimbalangle;
    }

    public void setGimbalangle(int gimbalangle) {
        this.gimbalangle = gimbalangle;
    }

    public int getYawangle() {
        return yawangle;
    }

    public void setYawangle(int yawangle) {
        this.yawangle = yawangle;
    }
}

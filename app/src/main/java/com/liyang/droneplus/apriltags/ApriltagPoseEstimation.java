package com.liyang.droneplus.apriltags;

public class ApriltagPoseEstimation {
    //    public ApriltagMatd R;
    public double R[] = new double[9];
    public double t[] = new double[3];
    //    public ApriltagDetection apd;
    // The decoded ID of the tag
    public int id;
    // The center of the detection in image pixel coordinates.
    public double[] c = new double[2];

    // The corners of the tag in image pixel coordinates. These always
    // wrap counter-clock wise around the tag.
    // Flattened to [x0 y0 x1 y1 ...] for JNI convenience
    public double[] p = new double[8];

}

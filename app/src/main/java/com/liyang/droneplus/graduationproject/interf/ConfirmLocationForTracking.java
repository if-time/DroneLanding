package com.liyang.droneplus.graduationproject.interf;

import android.graphics.RectF;

/**
 * 点击画框区域后，确认是否跟踪
 */
public interface ConfirmLocationForTracking {

    public void confirmForTracking(RectF rectFForFrame);
}

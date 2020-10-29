package com.liyang.droneplus.util;

import android.util.Log;

/**
 * Created by zh931 on 2018/8/23.
 */

public class LogUtil {
    //规定每段显示的长度
    private static int LOG_MAXLENGTH = 2000;

    public static void i(String TAG, String msg) {
        int strLength = msg.length();
        int start = 0;
        int end = LOG_MAXLENGTH;
        for (int i = 0; i < 100; i++) {
            //剩下的文本还是大于规定长度则继续重复截取并输出
            if (strLength > end) {
                Log.i(TAG + i, msg.substring(start, end));
                start = end;
                end = end + LOG_MAXLENGTH;
            } else {
                Log.i(TAG, msg.substring(start, strLength));
                break;
            }
        }
    }

    /**
     * apriltags详情
     * @param sb
     * @param name
     * @param value
     */
    public static void addLineToSB(StringBuffer sb, String name, Object value) {
        if (sb == null)
            return;
        sb.
                append((name == null || "".equals(name)) ? "" : name + ": ").
                append(value == null ? "" : value + "").
                append("\n");
    }
}

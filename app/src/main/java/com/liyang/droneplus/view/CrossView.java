package com.liyang.droneplus.view;

import android.content.Context;
import android.graphics.Canvas;
import android.graphics.Color;
import android.graphics.Paint;
import android.support.annotation.Nullable;
import android.util.AttributeSet;
import android.util.Log;
import android.view.View;

/**
 * GPS 定位 测试 网格
 */
public class CrossView extends View {

    public CrossView(Context context) {
        super(context);
    }

    public CrossView(Context context, @Nullable AttributeSet attrs) {
        super(context, attrs);
    }

    public CrossView(Context context, @Nullable AttributeSet attrs, int defStyleAttr) {
        super(context, attrs, defStyleAttr);

    }

    @Override
    public void draw(Canvas canvas) {
        super.draw(canvas);

        float lineT = 1;
        float pW = getMeasuredWidth() / 10;
        float pH = getMeasuredHeight() / 10;

        Log.i("dongcross", "draw: getMeasuredWidth() " + getMeasuredWidth() + " getMeasuredHeight() " + getMeasuredHeight()
        + " pW: " + pW + " pH: " + pH);


        //设置画笔
        Paint paint = new Paint();
        paint.setAntiAlias(true);
        paint.setStyle(Paint.Style.STROKE);

        paint.setColor(Color.RED);
        canvas.drawPoint(getMeasuredWidth() / 2, getMeasuredHeight() / 2, paint);
        canvas.drawLine(0, pH * 5, getMeasuredWidth(), pH * 5, paint);
        canvas.drawLine(pW * 5, 0, pW * 5, getMeasuredHeight(), paint);

        //画线
        paint.setColor(Color.BLACK);
        // 横
        canvas.drawLine(0, pH, getMeasuredWidth(), pH, paint);
        canvas.drawLine(0, pH * 2, getMeasuredWidth(), pH * 2, paint);
        canvas.drawLine(0, pH * 3, getMeasuredWidth(), pH * 3, paint);
        canvas.drawLine(0, pH * 4, getMeasuredWidth(), pH * 4, paint);

        canvas.drawLine(0, pH * 6, getMeasuredWidth(), pH * 6, paint);
        canvas.drawLine(0, pH * 7, getMeasuredWidth(), pH * 7, paint);
        canvas.drawLine(0, pH * 8, getMeasuredWidth(), pH * 8, paint);
        canvas.drawLine(0, pH * 9, getMeasuredWidth(), pH * 9, paint);


        // 竖
        canvas.drawLine(pW, 0, pW, getMeasuredHeight(), paint);
        canvas.drawLine(pW * 2, 0, pW * 2, getMeasuredHeight(), paint);
        canvas.drawLine(pW * 3, 0, pW * 3, getMeasuredHeight(), paint);
        canvas.drawLine(pW * 4, 0, pW * 4, getMeasuredHeight(), paint);

        canvas.drawLine(pW * 6, 0, pW * 6, getMeasuredHeight(), paint);
        canvas.drawLine(pW * 7, 0, pW * 7, getMeasuredHeight(), paint);
        canvas.drawLine(pW * 8, 0, pW * 8, getMeasuredHeight(), paint);
        canvas.drawLine(pW * 9, 0, pW * 9, getMeasuredHeight(), paint);

    }
}

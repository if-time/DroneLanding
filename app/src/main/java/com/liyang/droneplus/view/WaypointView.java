package com.liyang.droneplus.view;

import android.content.Context;
import android.graphics.Canvas;
import android.graphics.Color;
import android.graphics.Paint;
import android.graphics.Rect;
import android.support.annotation.Nullable;
import android.util.AttributeSet;
import android.util.Log;
import android.view.MotionEvent;
import android.view.View;

public class WaypointView extends View {

    private Paint paint;
    private Rect bound;
    private int count;
    private String TAG = "aaa";

    public WaypointView(Context context, int count) {
        super(context);
        this.count = count;
        paint = new Paint(Paint.ANTI_ALIAS_FLAG);
        bound = new Rect();
    }

    public WaypointView(Context context, @Nullable AttributeSet attrs) {
        super(context, attrs);
        paint = new Paint(Paint.ANTI_ALIAS_FLAG);
        bound = new Rect();
    }

    @Override
    protected void onMeasure(int widthMeasureSpec, int heightMeasureSpec) {
        super.onMeasure(widthMeasureSpec, heightMeasureSpec);
        int widthMode = MeasureSpec.getMode(widthMeasureSpec);   //获取宽的模式
        int heightMode = MeasureSpec.getMode(heightMeasureSpec); //获取高的模式
        int widthSize = MeasureSpec.getSize(widthMeasureSpec);   //获取宽的尺寸
        int heightSize = MeasureSpec.getSize(heightMeasureSpec); //获取高的尺寸
        //        Log.v(TAG, "宽的模式:"+widthMode);
        //        Log.v(TAG, "高的模式:"+heightMode);
        //        Log.v(TAG, "宽的尺寸:"+widthSize);
        //        Log.v(TAG, "高的尺寸:"+heightSize);
        int width;
        int height;
        if (widthMode == MeasureSpec.EXACTLY) {
            //如果match_parent或者具体的值，直接赋值
            width = widthSize;
        } else {
            //如果是wrap_content，我们要得到控件需要多大的尺寸
            float textWidth = 50;   //文本的宽度
            //控件的宽度就是文本的宽度加上两边的内边距。内边距就是padding值，在构造方法执行完就被赋值
            width = (int) (getPaddingLeft() + textWidth + getPaddingRight());
            //            Log.v(TAG, "文本的宽度:"+textWidth + "控件的宽度："+width);
        }
        //高度跟宽度处理方式一样
        if (heightMode == MeasureSpec.EXACTLY) {
            height = heightSize;
        } else {
            float textHeight = 50;
            height = (int) (getPaddingTop() + textHeight + getPaddingBottom());
            Log.v(TAG, "文本的高度:" + textHeight + "控件的高度：" + height);
        }
        //保存测量宽度和测量高度
        setMeasuredDimension(width, height);
    }

    @Override
    protected void onDraw(Canvas canvas) {
        super.onDraw(canvas);
        paint.setColor(Color.BLUE);
        canvas.drawCircle(getWidth() / 2, getHeight() / 2, 22, paint);
        //        canvas.drawRect(0, 0, getWidth(), getHeight(), paint);
        paint.setColor(Color.YELLOW);
        paint.setTextSize(33);
        String test = String.valueOf(count);

        // 获取文字的宽和高
        paint.getTextBounds(test, 0, test.length(), bound);
        float textWidth = bound.width();
        float textHeight = bound.height();

        canvas.drawText(test, getWidth() / 2 - textWidth / 2, getHeight() / 2 + textHeight / 2, paint);
    }

}

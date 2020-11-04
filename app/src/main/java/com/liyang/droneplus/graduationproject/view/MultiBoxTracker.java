/* Copyright 2019 The TensorFlow Authors. All Rights Reserved.

Licensed under the Apache License, Version 2.0 (the "License");
you may not use this file except in compliance with the License.
You may obtain a copy of the License at

    http://www.apache.org/licenses/LICENSE-2.0

Unless required by applicable law or agreed to in writing, software
distributed under the License is distributed on an "AS IS" BASIS,
WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
See the License for the specific language governing permissions and
limitations under the License.
==============================================================================*/

package com.liyang.droneplus.graduationproject.view;

import android.content.Context;
import android.graphics.Canvas;
import android.graphics.Color;
import android.graphics.Matrix;
import android.graphics.Paint;
import android.graphics.Paint.Cap;
import android.graphics.Paint.Join;
import android.graphics.Paint.Style;
import android.graphics.RectF;
import android.text.TextUtils;
import android.util.Pair;
import android.util.TypedValue;

import com.liyang.droneplus.graduationproject.detection.ClassifierFromTensorFlow;
import com.liyang.droneplus.graduationproject.utils.BorderedText;

import java.util.LinkedList;
import java.util.List;
import java.util.Queue;

/**
 * A tracker that handles non-max suppression and matches existing objects to new detections.
 */
public class MultiBoxTracker {
    private static final float TEXT_SIZE_DIP = 18;
    private static final int[] COLORS = {Color.BLUE, Color.RED, Color.GREEN, Color.YELLOW, Color.CYAN,
            Color.MAGENTA, Color.WHITE, Color.parseColor("#55FF55"),
            Color.parseColor("#FFA500"), Color.parseColor("#FF8888"),
            Color.parseColor("#AAAAFF"), Color.parseColor("#FFFFAA"),
            Color.parseColor("#55AAAA"), Color.parseColor("#AA33AA"),
            Color.parseColor("#0D0068")};
    final List<Pair<Float, RectF>> screenRects = new LinkedList<Pair<Float, RectF>>();
    private final Queue<Integer> availableColors = new LinkedList<Integer>();
    private final List<TrackedRecognition> trackedObjects = new LinkedList<TrackedRecognition>();
    private final Paint boxPaint = new Paint();
    private final float textSizePx;
    private final BorderedText borderedText;
    private Matrix frameToCanvasMatrix;
    private int frameWidth;
    private int frameHeight;
    private int sensorOrientation;

    public MultiBoxTracker(final Context context) {
        for (final int color : COLORS) {
            availableColors.add(color);
        }

        boxPaint.setColor(Color.RED);
        boxPaint.setStyle(Style.STROKE);
        boxPaint.setStrokeWidth(10.0f);
        boxPaint.setStrokeCap(Cap.ROUND);
        boxPaint.setStrokeJoin(Join.ROUND);
        boxPaint.setStrokeMiter(100);

        textSizePx = TypedValue.applyDimension(TypedValue.COMPLEX_UNIT_DIP, TEXT_SIZE_DIP, context.getResources().getDisplayMetrics());
        borderedText = new BorderedText(textSizePx);
    }

    public synchronized void setFrameConfiguration(final int width, final int height) {
        frameWidth = width;
        frameHeight = height;
    }

    public synchronized void setFrameConfiguration(final int width, final int height, final int sensorOrientation) {
        frameWidth = width;
        frameHeight = height;
        this.sensorOrientation = sensorOrientation;
    }

    private Matrix getFrameToCanvasMatrix() {
        return frameToCanvasMatrix;
    }

    public synchronized void trackResultsFromTensorFlow(final List<ClassifierFromTensorFlow.Recognition> results) {
        processResultsFromTensorFlow(results);
    }

    private void processResultsFromTensorFlow(final List<ClassifierFromTensorFlow.Recognition> results) {
        final List<Pair<Float, ClassifierFromTensorFlow.Recognition>> rectsToTrack = new LinkedList<Pair<Float, ClassifierFromTensorFlow.Recognition>>();

//        screenRects.clear();
//        final Matrix rgbFrameToScreen = new Matrix(getFrameToCanvasMatrix());

        for (final ClassifierFromTensorFlow.Recognition result : results) {
            if (result.getLocation() == null) {
                continue;
            }
//            final RectF detectionFrameRect = new RectF(result.getLocation());
//
//            final RectF detectionScreenRect = new RectF();
//            rgbFrameToScreen.mapRect(detectionScreenRect, detectionFrameRect);
//
//            screenRects.add(new Pair<Float, RectF>(result.getConfidence(), detectionScreenRect));
//
//            if (detectionFrameRect.width() < MIN_SIZE || detectionFrameRect.height() < MIN_SIZE) {
//
//                continue;
//            }

            rectsToTrack.add(new Pair<Float, ClassifierFromTensorFlow.Recognition>(result.getConfidence(), result));
        }

        trackedObjects.clear();
        if (rectsToTrack.isEmpty()) {
            return;
        }

        for (final Pair<Float, ClassifierFromTensorFlow.Recognition> potential : rectsToTrack) {
            final TrackedRecognition trackedRecognition = new TrackedRecognition();
            trackedRecognition.detectionConfidence = potential.first;
            trackedRecognition.location = new RectF(potential.second.getLocation());
            trackedRecognition.title = potential.second.getTitle();
            trackedRecognition.color = COLORS[trackedObjects.size()];
            trackedObjects.add(trackedRecognition);

            if (trackedObjects.size() >= COLORS.length) {
                break;
            }
        }
    }

    public synchronized void draw(final Canvas canvas) {
//        final boolean rotated = sensorOrientation % 180 == 90;
//        final boolean rotated = false;
//        final float multiplier = Math.min(canvas.getHeight() / (float) (rotated ? frameWidth : frameHeight), canvas.getWidth() / (float) (rotated ? frameHeight : frameWidth));
//        frameToCanvasMatrix = ImageUtils.getTransformationMatrix(frameWidth, frameHeight, (int) (multiplier * (rotated ? frameHeight : frameWidth)), (int) (multiplier * (rotated ? frameWidth : frameHeight)), sensorOrientation, false);
        for (final TrackedRecognition recognition : trackedObjects) {
            final RectF trackedPos = new RectF(recognition.location);

//            getFrameToCanvasMatrix().mapRect(trackedPos);
            boxPaint.setColor(recognition.color);

            float cornerSize = Math.min(trackedPos.width(), trackedPos.height()) / 8.0f;
//            canvas.drawRoundRect(trackedPos, cornerSize, cornerSize, boxPaint);
            canvas.drawRect(trackedPos, boxPaint);

            final String labelString = !TextUtils.isEmpty(recognition.title) ?
                    String.format("%s %.2f", recognition.title, (100 * recognition.detectionConfidence)) : String.format("%.2f", (100 * recognition.detectionConfidence));
            //            borderedText.drawText(canvas, trackedPos.left + cornerSize, trackedPos.top, labelString);
            borderedText.drawText(canvas, trackedPos.left + cornerSize, trackedPos.top, labelString + "%", boxPaint);
        }
    }

    private static class TrackedRecognition {
        RectF location;
        float detectionConfidence;
        int color;
        String title;
    }
}

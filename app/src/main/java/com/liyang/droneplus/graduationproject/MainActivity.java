package com.liyang.droneplus.graduationproject;

import android.annotation.SuppressLint;
import android.content.Context;
import android.graphics.Bitmap;
import android.graphics.Canvas;
import android.graphics.Color;
import android.graphics.Paint;
import android.graphics.Point;
import android.graphics.RectF;
import android.graphics.SurfaceTexture;
import android.os.Build;
import android.os.Handler;
import android.os.HandlerThread;
import android.support.annotation.RequiresApi;
import android.support.v7.app.AppCompatActivity;
import android.os.Bundle;
import android.util.Log;
import android.view.Display;
import android.view.TextureView;
import android.view.View;
import android.view.WindowManager;
import android.widget.Button;
import android.widget.ImageView;
import android.widget.Toast;

import com.liyang.droneplus.R;
import com.liyang.droneplus.application.DemoApplication;
import com.liyang.droneplus.graduationproject.detection.ClassifierFromTensorFlow;
import com.liyang.droneplus.graduationproject.detection.TensorFlowObjectDetectionAPIModel;
import com.liyang.droneplus.graduationproject.view.AutoFitTextureView;
import com.liyang.droneplus.graduationproject.view.TouchPaintView;

import java.io.IOException;
import java.util.List;

import dji.common.camera.SettingsDefinitions;
import dji.common.error.DJIError;
import dji.common.product.Model;
import dji.common.util.CommonCallbacks;
import dji.sdk.base.BaseProduct;
import dji.sdk.camera.Camera;
import dji.sdk.camera.VideoFeeder;
import dji.sdk.codec.DJICodecManager;

public class MainActivity extends AppCompatActivity implements TextureView.SurfaceTextureListener, View.OnClickListener {

    private static final String TAG = MainActivity.class.getName();

    private static final String HANDLE_THREAD_NAME = "CameraBackgroundDetection";

    private static final String TF_OD_API_MODEL_FILE = "file:///android_asset/frozen_inference_graph_v6.pb";
    private static final String TF_OD_API_LABELS_FILE = "file:///android_asset/coco_labels_list.txt";
    private static final float MINIMUM_CONFIDENCE_TF_OD_API = 0.6f;
    private static final int TF_OD_API_INPUT_SIZE = 300;

    private int widthDisplay;
    private int heightDisplay;

    private static float canvasWidth = 0;
    private static float canvasHeight = 0;

    protected VideoFeeder.VideoDataListener mReceivedVideoDataListener = null;
    // Codec for video live view
    protected DJICodecManager mCodecManager = null;
    private Camera mCamera;

    private HandlerThread backgroundThread;
    private Handler backgroundHandler;

    private final Object lock = new Object();

    private ClassifierFromTensorFlow classifierFromTensorFlow;

    private boolean runDetection = false;

    protected AutoFitTextureView mVideoSurface = null;
    private Button btnThermal;
    private Button btnThread;

    private ImageView imageViewForFrame;

    private TouchPaintView touchView;


    @Override
    protected void onCreate(Bundle savedInstanceState) {
        super.onCreate(savedInstanceState);
        setContentView(R.layout.activity_main2);

        initUI();
        getDisplaySize();

        // The callback for receiving the raw H264 video data for camera live view
        mReceivedVideoDataListener = new VideoFeeder.VideoDataListener() {

            @Override
            public void onReceive(byte[] videoBuffer, int size) {
                if (mCodecManager != null) {
                    mCodecManager.sendDataToDecoder(videoBuffer, size);
                }
            }
        };

        try {
            // create either a new ImageClassifierQuantizedMobileNet or an ImageClassifierFloatInception
            classifierFromTensorFlow = TensorFlowObjectDetectionAPIModel.create(getAssets(), TF_OD_API_MODEL_FILE, TF_OD_API_LABELS_FILE, TF_OD_API_INPUT_SIZE);
        } catch (IOException e) {
            Log.e(TAG, "Failed to initialize an image classifier.");
        }

    }

    private void initUI() {
        // init mVideoSurface
        mVideoSurface = (AutoFitTextureView) findViewById(R.id.video_previewer_surface);
        imageViewForFrame = findViewById(R.id.imageView);
        //imageViewForFrame.setOnTouchListener(this);

        if (null != mVideoSurface) {
            mVideoSurface.setSurfaceTextureListener(this);
        }

        btnThermal = findViewById(R.id.btnThermal);
        btnThermal.setOnClickListener(this);
        btnThread = findViewById(R.id.btnThread);
        btnThread.setOnClickListener(this);

        touchView = (TouchPaintView) findViewById(R.id.touch_view);
    }

    /**
     * 获取屏幕大小
     */
    private void getDisplaySize() {
        WindowManager manager = (WindowManager) this.getSystemService(Context.WINDOW_SERVICE);
        Display display = manager.getDefaultDisplay();
        Point point = new Point();
        if (Build.VERSION.SDK_INT < 17) {
            display.getSize(point);
        } else {
            display.getRealSize(point);
        }
        widthDisplay = point.x;
        heightDisplay = point.y;
    }

    private void setThermalConfig() {
        BaseProduct baseProduct = DemoApplication.getProductInstance();
        mCamera = baseProduct.getCameras().get(0);
        mCamera.setDisplayMode(SettingsDefinitions.DisplayMode.MSX, new CommonCallbacks.CompletionCallback() {
            @Override
            public void onResult(DJIError djiError) {
                if (djiError != null) {

                }
            }
        });
        mCamera.setMSXLevel(90, new CommonCallbacks.CompletionCallback() {
            @Override
            public void onResult(DJIError djiError) {

            }
        });
        mCamera.setThermalPalette(SettingsDefinitions.ThermalPalette.FUSION, new CommonCallbacks.CompletionCallback() {
            @Override
            public void onResult(DJIError djiError) {

            }
        });

    }

    @Override
    public void onResume() {
        Log.e(TAG, "onResume");
        super.onResume();
        initPreviewer();
        onProductChange();

        if (mVideoSurface == null) {
            Log.e(TAG, "mVideoSurface is null");
        }
    }

    @Override
    public void onPause() {
        uninitPreviewer();
        stopBackgroundThread();
        super.onPause();
    }

    @Override
    public void onStop() {
        super.onStop();
    }

    public void onReturn(View view) {
        this.finish();
    }

    @Override
    protected void onDestroy() {
        uninitPreviewer();
        super.onDestroy();
    }

    protected void onProductChange() {
        initPreviewer();
    }

    private void initPreviewer() {

        BaseProduct product = DemoApplication.getProductInstance();

        if (product == null || !product.isConnected()) {
            //            showToast(getString(R.string.disconnected));
        } else {
            if (!product.getModel().equals(Model.UNKNOWN_AIRCRAFT)) {
                if (null != mVideoSurface) {
                    mVideoSurface.setSurfaceTextureListener(this);
                }
                VideoFeeder.getInstance().getPrimaryVideoFeed().addVideoDataListener(mReceivedVideoDataListener);
            }
        }
    }

    private void uninitPreviewer() {
        //        Camera camera = DemoApplication.getCameraInstance();
        if (mCamera != null) {
            // Reset the callback
            VideoFeeder.getInstance().getPrimaryVideoFeed().addVideoDataListener(null);
        }
    }

    @Override
    public void onSurfaceTextureAvailable(SurfaceTexture surface, int width, int height) {
        Log.e(TAG, "onSurfaceTextureAvailable");
        if (mCodecManager == null) {
            mCodecManager = new DJICodecManager(this, surface, width, height);
        }
    }

    @Override
    public void onSurfaceTextureSizeChanged(SurfaceTexture surface, int width, int height) {

    }

    @Override
    public boolean onSurfaceTextureDestroyed(SurfaceTexture surface) {
        Log.e(TAG, "onSurfaceTextureDestroyed");
        if (mCodecManager != null) {
            mCodecManager.cleanSurface();
            mCodecManager = null;
        }

        return false;
    }

    @Override
    public void onSurfaceTextureUpdated(SurfaceTexture surface) {
        Log.i(TAG, "onSurfaceTextureUpdated: 1111111111");
    }

    private void showToast(String s) {
        Toast.makeText(getApplicationContext(), s, Toast.LENGTH_SHORT).show();
    }

    /**
     * 启动后台线程
     */
    private void startBackgroundThread() {
        backgroundThread = new HandlerThread(HANDLE_THREAD_NAME);
        backgroundThread.start();
        backgroundHandler = new Handler(backgroundThread.getLooper());
        synchronized (lock) {
            runDetection = true;
        }
        backgroundHandler.post(periodicDetection);
    }

    /**
     * 停止后台线程
     */
    @RequiresApi(api = Build.VERSION_CODES.JELLY_BEAN_MR2)
    private void stopBackgroundThread() {
        backgroundThread.quitSafely();
        try {
            backgroundThread.join();
            backgroundThread = null;
            backgroundHandler = null;
            synchronized (lock) {
                runDetection = false;
            }
        } catch (InterruptedException e) {
            e.printStackTrace();
        }
    }

    /**
     * 定期识别
     */
    private Runnable periodicDetection = new Runnable() {
        @Override
        public void run() {
            synchronized (lock) {
                if (runDetection) {
                    classifyFrame();
                }
            }
            backgroundHandler.post(periodicDetection);
        }
    };

    /**
     * tensorFlow识别
     */
    private void classifyFrame() {
        //detectionForTensorFlow();

        canvasWidth = mVideoSurface.getWidth();
        canvasHeight = mVideoSurface.getHeight();
        imageViewForFrame.getLayoutParams().width = mVideoSurface.getWidth();
        imageViewForFrame.getLayoutParams().height = mVideoSurface.getHeight();

        final Bitmap croppedBitmap = Bitmap.createBitmap((int) canvasWidth, (int) canvasHeight, Bitmap.Config.ARGB_8888);
        final Canvas canvas = new Canvas(croppedBitmap);

        RectF rectF = touchView.getFrameLocation();
        // 截取此区域的bitmap传入fdsst中

        // 获取到识别出的位置并画框

    }

    private void detectionForTensorFlow() {
        if (classifierFromTensorFlow == null) {
            showToast("Uninitialized Classifier or invalid context.");
            return;
        }

        Bitmap bitmap = mVideoSurface.getBitmap(TF_OD_API_INPUT_SIZE, TF_OD_API_INPUT_SIZE);

        if (bitmap == null) {
            showToast("bitmap == null");
            return;
        } else {
            //           showToast("bitmap == getWidth: " + bitmap.getWidth() + " bitmap == getHeight: " + bitmap.getHeight());
            final List<ClassifierFromTensorFlow.Recognition> results = classifierFromTensorFlow.recognizeImage(bitmap);

            canvasWidth = mVideoSurface.getWidth();
            canvasHeight = mVideoSurface.getHeight();
            //           if (mVideoSurface.getWidth() != imageView.getWidth() || mVideoSurface.getHeight() != imageView.getHeight()) {
            //               canvasWidth = mVideoSurface.getWidth();
            //               canvasHeight = mVideoSurface.getHeight();
            imageViewForFrame.getLayoutParams().width = mVideoSurface.getWidth();
            imageViewForFrame.getLayoutParams().height = mVideoSurface.getHeight();
            //           }
            bitmap.recycle();

            final Bitmap croppedBitmap = Bitmap.createBitmap((int) canvasWidth, (int) canvasHeight, Bitmap.Config.ARGB_8888);
            final Canvas canvas = new Canvas(croppedBitmap);

            for (final ClassifierFromTensorFlow.Recognition result : results) {
                final RectF location = result.getLocation();
                Log.i(TAG, "classifyFrame: " + location.bottom);
                if (location != null && result.getConfidence() >= MINIMUM_CONFIDENCE_TF_OD_API) {
                    Paint paint = new Paint();
                    Paint paint1 = new Paint();
                    if (result.getTitle().equals("openeyes")) {
                        paint.setColor(Color.GREEN);
                        paint1.setColor(Color.GREEN);
                    } else if (result.getTitle().equals("closeeyes")) {
                        paint.setColor(Color.RED);
                        paint1.setColor(Color.RED);

                    } else if (result.getTitle().equals("phone")) {
                        paint.setColor(0xFFFF9900);
                        paint1.setColor(0xFFFF9900);

                    } else if (result.getTitle().equals("smoke")) {
                        paint.setColor(Color.YELLOW);
                        paint1.setColor(Color.YELLOW);
                    } else
                        paint.setColor(Color.WHITE);

                    paint.setStyle(Paint.Style.STROKE);
                    paint.setStrokeWidth(5.0f);
                    paint.setAntiAlias(true);
                    paint1.setStyle(Paint.Style.FILL);
                    paint1.setAlpha(125);
                    //                canvas.drawRect(location, paint);
                    //                   canvas.drawText();
                    canvas.drawRect(canvasWidth * location.left / TF_OD_API_INPUT_SIZE, canvasHeight * location.top / TF_OD_API_INPUT_SIZE, canvasWidth * location.right / TF_OD_API_INPUT_SIZE, canvasHeight * location.bottom / TF_OD_API_INPUT_SIZE, paint);
                    canvas.drawRect(canvasWidth * location.left / TF_OD_API_INPUT_SIZE, canvasHeight * location.top / TF_OD_API_INPUT_SIZE, canvasWidth * location.right / TF_OD_API_INPUT_SIZE, canvasHeight * location.bottom / TF_OD_API_INPUT_SIZE, paint1);
                    canvas.drawRect((float) (canvasWidth * 0.5), (float) (canvasHeight * 0.5), (float) (canvasWidth * 0.5), (float) (canvasHeight * 0.5), paint);
                }

            }
            imageViewForFrame.post(new Runnable() {
                @Override
                public void run() {
                    imageViewForFrame.setImageBitmap(croppedBitmap);
                }
            });
        }
    }

    @SuppressLint("NonConstantResourceId")
    @Override
    public void onClick(View v) {
        switch (v.getId()) {
            case R.id.btnThermal:
                setThermalConfig();
                showToast("红外");
                break;
            case R.id.btnThread:
                startBackgroundThread();
                break;
        }
    }



}
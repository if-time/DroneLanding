package com.liyang.droneplus.classifier;

import android.graphics.Bitmap;
import android.graphics.RectF;
import android.util.Log;

import com.liyang.droneplus.bean.DetectModel;

import java.io.BufferedReader;
import java.io.IOException;
import java.io.InputStream;
import java.io.InputStreamReader;
import java.util.ArrayList;
import java.util.List;

public class TarmacDetectAPIModel implements Recognizer{

    private String soName;
    private int[] ddims = {1, 3, 416, 416};
    private static com.liyang.droneplus.classifier.yolov2Tiny yolov2Tiny = new yolov2Tiny();
    private static Yolov2Poppy yolov2Poppy = new Yolov2Poppy();
    private static boolean clear_t_result = false;
    private static boolean clear_p_result = false;

    private static boolean load_result = false;
    //    private static boolean clear_result = false;
//    private static com.liyang.droneplus.classifier.yolov2Tiny yolov2Tiny = new yolov2Tiny();
    public static List<String> resultLabel = new ArrayList<>();

    public TarmacDetectAPIModel() {
    }

    public TarmacDetectAPIModel(String soName) {
        this.soName = soName;
    }

    //  predict image
    public List<Recognition> predict_image(Bitmap bitmap) {
        // picture to float array
        // Bitmap bmp = bitmap;
        Bitmap rgba = bitmap.copy(Bitmap.Config.ARGB_8888, true);
        // resize to 416x416
        Bitmap input_bmp = Bitmap.createScaledBitmap(rgba, ddims[2], ddims[3], false);
        List<Recognition> recognitions = new ArrayList<>();
        try {
            // Data format conversion takes too long
            // Log.d("inputData", Arrays.toString(inputData));
            long start = System.currentTimeMillis();
            // get predict result
            if (soName.equals("yolov2Tiny")) {
                float[] result = yolov2Tiny.Detect(input_bmp);
                long end = System.currentTimeMillis();
                //Log.d(TAG, "origin predict result:" + Arrays.toString(result));
                long time = end - start;
                Log.i("result", "length of result: " + String.valueOf(result.length));
                // show predict result and time
                if (result.length == 0) {
                    Log.i("result", "没有识别");
                    recognitions.add(new Recognition("", 0f,
                            new RectF(10000, 10000, 11000, 11000),
                            time, 0, 0, 0, 0));
                } else {
                    int num_r = result.length / 6;
                    for (int i = 0; i < num_r; i++){
                        recognitions.add(new Recognition(ReadConfigFileUtil.resultLabel.get((int) result[i*6]), result[i*6+1],
                                new RectF(result[i*6+2]*rgba.getWidth(), result[i*6+3]*rgba.getHeight(),
                                        result[i*6+4]*rgba.getWidth(), result[i*6+5]*rgba.getHeight()), time,
                                result[i*6+2]*rgba.getWidth(), result[i*6+3]*rgba.getHeight(), result[i*6+4]*rgba.getWidth(),
                                result[i*6+5]*rgba.getHeight()));
                    }
//                float[] r = get_max_result(result);
                    // Log.i("result", "length of r: " + String.valueOf(r.length));
                    // Log.i("result", "r[0]:" + r[0] + "; r[1]:" + r[1] + "; r[2]:" + r[2] + "; r[3]:" + r[3] + "; r[4]:" + r[4] +"; r[5]:" + r[5]);
                    // Log.i("result", "label:" + "tarmac");
                    // Log.i("result", "left:" + r[2]*rgba.getWidth());
                    // Log.i("result", "top:" + r[3]*rgba.getHeight());
                    // Log.i("result", "right:" + r[4]*rgba.getWidth());
                    // Log.i("result", "bottom:" + r[5]*rgba.getHeight());
//                if (result.length != 0) {
//                    recognitions.add(new Recognition(TaskActivity.resultLabel.get((int) r[0]), r[1],
//                            new RectF(r[2]*rgba.getWidth(), r[3]*rgba.getHeight(),
//                                    r[4]*rgba.getWidth(), r[5]*rgba.getHeight()), time,
//                            r[2]*rgba.getWidth(), r[3]*rgba.getHeight(), r[4]*rgba.getWidth(),
//                            r[5]*rgba.getHeight()));
//                }
                }
            }
            if (soName.equals("Yolov2Poppy")) {
                float[] result = yolov2Poppy.Detect(input_bmp);
                long end = System.currentTimeMillis();
                //Log.d(TAG, "origin predict result:" + Arrays.toString(result));
                long time = end - start;
                Log.i("result", "length of result: " + String.valueOf(result.length));
                // show predict result and time
                if (result.length == 0) {
                    Log.i("result", "没有识别");
                    recognitions.add(new Recognition("", 0f,
                            new RectF(10000, 10000, 11000, 11000),
                            time, 0, 0, 0, 0));
                } else {
                    int num_r = result.length / 6;
                    for (int i = 0; i < num_r; i++){
                        recognitions.add(new Recognition(ReadConfigFileUtil.resultLabel.get((int) result[i*6]), result[i*6+1],
                                new RectF(result[i*6+2]*rgba.getWidth(), result[i*6+3]*rgba.getHeight(),
                                        result[i*6+4]*rgba.getWidth(), result[i*6+5]*rgba.getHeight()), time,
                                result[i*6+2]*rgba.getWidth(), result[i*6+3]*rgba.getHeight(), result[i*6+4]*rgba.getWidth(),
                                result[i*6+5]*rgba.getHeight()));
                    }
//                float[] r = get_max_result(result);
                    // Log.i("result", "length of r: " + String.valueOf(r.length));
                    // Log.i("result", "r[0]:" + r[0] + "; r[1]:" + r[1] + "; r[2]:" + r[2] + "; r[3]:" + r[3] + "; r[4]:" + r[4] +"; r[5]:" + r[5]);
                    // Log.i("result", "label:" + "tarmac");
                    // Log.i("result", "left:" + r[2]*rgba.getWidth());
                    // Log.i("result", "top:" + r[3]*rgba.getHeight());
                    // Log.i("result", "right:" + r[4]*rgba.getWidth());
                    // Log.i("result", "bottom:" + r[5]*rgba.getHeight());
//                if (result.length != 0) {
//                    recognitions.add(new Recognition(TaskActivity.resultLabel.get((int) r[0]), r[1],
//                            new RectF(r[2]*rgba.getWidth(), r[3]*rgba.getHeight(),
//                                    r[4]*rgba.getWidth(), r[5]*rgba.getHeight()), time,
//                            r[2]*rgba.getWidth(), r[3]*rgba.getHeight(), r[4]*rgba.getWidth(),
//                            r[5]*rgba.getHeight()));
//                }
                }
            }
            //Log.i("result", "recognitions:" + recognitions.get(0).getConfidence());
            //Log.i("result", "recognitions:" + recognitions.toString());
            //resultStr = "result：" + Arrays.toString(r) + "\nname：" + resultLabel.get((int) r[0]) + "\nprobability：" + r[1] + "\ntime：" + time + "ms";
        } catch (Exception e) {
            e.printStackTrace();
        }
        return recognitions;
    }

    public void initYolov2Tiny(InputStream paramFile, InputStream lFile, String soFileName) throws IOException {

        byte[] param = null;
        byte[] bin = null;
        {
            // InputStream assetsInputStream = getAssets().open("yolov2-tiny_tarmac.param.bin");
//            InputStream assetsInputStream = paramFile;
            int available = paramFile.available();
            param = new byte[available];
            int byteCode = paramFile.read(param);
            paramFile.close();
        }
        {
//            InputStream assetsInputStream = lFile;
            int available = lFile.available();
            bin = new byte[available];
            int byteCode = lFile.read(bin);
            lFile.close();
        }

        if (soFileName.equals("yolov2Tiny")) {
//            clear_result = yolov2Poppy.Clear();
            load_result = yolov2Tiny.Init(param, bin);
        }
        if (soFileName.equals("Yolov2Poppy")) {
//            clear_result = yolov2Tiny.Clear();
            load_result = yolov2Poppy.Init(param, bin);
        }
        Log.d("load model", "yolov2tiny_load_model_result:" + load_result);
//        Log.d("load model", "yolov2tiny_clear_model_result:" + clear_result);
    }

    // load label's name
    public void readCacheLabelFromLocalFile(InputStream inputStream) {
        try {
            // AssetManager assetManager = getApplicationContext().getAssets();
            BufferedReader reader = new BufferedReader(new InputStreamReader(inputStream));
            String readLine = null;
            while ((readLine = reader.readLine()) != null) {
                resultLabel.add(readLine);
            }
            reader.close();
        } catch (Exception e) {
            Log.e("labelCache", "error " + e);
        }
    }

    // get max probability label
    private float[] get_max_result(float[] result) {
        int num_rs = result.length / 6;
        float maxProp = result[1];
        int maxI = 0;
        for(int i = 1; i<num_rs;i++){
            if(maxProp<result[i*6+1]){
                maxProp = result[i*6+1];
                maxI = i;
            }
        }
        float[] ret = {0,0,0,0,0,0};
        for(int j=0;j<6;j++){
            ret[j] = result[maxI*6 + j];
        }
        return ret;
    }

    @Override
    public List<Recognition> recognizeImage(Bitmap bitmap) {

        Log.i("result length", "length这里执行了");
        List<Recognition> res = predict_image(bitmap);

        return res;
    }

//    @Override
//    public void enableStatLogging(boolean debug) {
//
//    }
//
//    @Override
//    public String getStatString() {
//        return null;
//    }

    @Override
    public void close() {
        clear_t_result = yolov2Tiny.Clear();
        clear_p_result = yolov2Poppy.Clear();
        Log.d("load_model", "yolov2tiny_clear_model_result:" + clear_t_result);
        Log.d("load_model", "yolov2Poppy_clear_model_result:" + clear_p_result);
    }

    @Override
    public void setModel(ArrayList<DetectModel> models) {

    }
}

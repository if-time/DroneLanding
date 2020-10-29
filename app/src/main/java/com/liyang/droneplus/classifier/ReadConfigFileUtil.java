package com.liyang.droneplus.classifier;

import android.util.Log;

import java.io.BufferedReader;
import java.io.IOException;
import java.io.InputStream;
import java.io.InputStreamReader;
import java.util.ArrayList;
import java.util.List;

/**
 * 读取识别配置文件的类
 */
public class ReadConfigFileUtil {

    private static boolean load_result = false;
//    private static boolean clear_result = false;
//    private static com.liyang.droneplus.classifier.yolov2Tiny yolov2Tiny = new yolov2Tiny();
    public static List<String> resultLabel = new ArrayList<>();

    /**
     * 读参数文件和权重文件
     * @param paramFile
     * @param lFile
     * @param soFileName
     * @throws IOException
     */
    public static void initYolov2Tiny(InputStream paramFile, InputStream lFile, String soFileName) throws IOException {

        yolov2Tiny yolov2Tiny = new yolov2Tiny();
        Yolov2Poppy yolov2Poppy = new Yolov2Poppy();
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

    /**
     * 读标签文件
     * @param inputStream
     */
    // load label's name
    public static void readCacheLabelFromLocalFile(InputStream inputStream) {
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

}

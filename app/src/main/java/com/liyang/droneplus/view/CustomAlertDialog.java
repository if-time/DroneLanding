//package com.liyang.droneplus.view;
//
//import android.app.Dialog;
//import android.content.Context;
//import android.os.Bundle;
//import android.support.annotation.NonNull;
//import android.support.annotation.Nullable;
//import android.view.View;
//import android.widget.Button;
//import android.widget.TextView;
//
//import com.liyang.droneplus.R;
//
//public class CustomAlertDialog extends Dialog implements View.OnClickListener {
//
//    public interface OnDialogButtonClickListener {
//        /**
//         * 点击按钮的回调方法
//         *
//         * @param requestCode
//         * @param isPositive
//         */
//        void onDialogButtonClick(int requestCode, boolean isPositive);
//    }
//
//    private Context context;
//    private String title;
//    private String message;
//    private String strPositive;
//    private String strNegative;
//    private int requestCode;
//    private OnDialogButtonClickListener listener;
//
//    public CustomAlertDialog(Context context, String title, String message, int requestCode,
//                             OnDialogButtonClickListener listener) {
//        super(context, R.style.MyDialog);
//        this.context=context;
//        this.title=title;
//        this.message=message;
//        this.requestCode=requestCode;
//        this.listener=listener;
//
//    }
//
//    private TextView tvTitle;
//    private TextView tvMessage;
//    private Button btnPositive;
//    private Button btnNegative;
//
//    @Override
//    protected void onCreate(Bundle savedInstanceState) {
//        super.onCreate(savedInstanceState);
//        setContentView(R.layout.alert_dialog);
//        //  setCancelable(false);//设置点击对话框外部和按返回键都不可以取消
//        //  setCanceledOnTouchOutside(false);//设置点击对话框外部是否可以取消，默认是不可以取消（但是点返回键可以取消）
//
//        tvTitle = (TextView) findViewById(R.id.tvAlertDialogTitle);
//        tvMessage = (TextView) findViewById(R.id.tvAlertDialogMessage);
//        btnPositive = (Button) findViewById(R.id.btnAlertDialogPositive);
//        btnNegative = (Button) findViewById(R.id.btnAlertDialogNegative);
//
//        tvTitle.setText(title);
//        tvMessage.setText(message);
//
//        btnPositive.setOnClickListener(this);
//        btnNegative.setOnClickListener(this);
//    }
//
//    @Override
//    public void onClick(View view) {
//        switch (view.getId()){
//            case R.id.btnAlertDialogPositive:
//                //确定按钮
//                listener.onDialogButtonClick(requestCode,true);
//                break;
//            case R.id.btnAlertDialogNegative:
//                //取消按钮
//                listener.onDialogButtonClick(requestCode,false);
//                break;
//        }
//        dismiss();
//    }
//}
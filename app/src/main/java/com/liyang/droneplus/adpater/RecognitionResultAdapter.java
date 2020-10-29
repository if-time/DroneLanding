package com.liyang.droneplus.adpater;

import android.support.v7.widget.RecyclerView;
import android.view.LayoutInflater;
import android.view.View;
import android.view.ViewGroup;
import android.widget.TextView;

import com.liyang.droneplus.R;
import com.liyang.droneplus.bean.RecognitionResultItem;

import java.util.ArrayList;

/**
 * Created by zh931 on 2018/9/4.
 */

public class RecognitionResultAdapter extends RecyclerView.Adapter {

    private ArrayList<RecognitionResultItem> dataList;

    public RecognitionResultAdapter(ArrayList<RecognitionResultItem> dataList) {
        this.dataList = dataList;
    }

    @Override
    public RecyclerView.ViewHolder onCreateViewHolder(ViewGroup parent, int viewType) {
        View view = LayoutInflater.from(parent.getContext()).inflate(R.layout.recognition_result_list_item, parent, false);
        ResultViewHolder resultViewHolder = new ResultViewHolder(view);
        return resultViewHolder;
    }

    @Override
    public void onBindViewHolder(RecyclerView.ViewHolder holder, int position) {
        if (holder instanceof ResultViewHolder) {
            RecognitionResultItem recognitionResultItem = dataList.get(position);
            ((ResultViewHolder) holder).timeTv.setText(dataList.get(position).getTime());
            ((ResultViewHolder) holder).resultTv.setText(dataList.get(position).getResult());
            ((ResultViewHolder) holder).probability.setText(dataList.get(position).getProbability());
            ((ResultViewHolder) holder).recognitionTimeTv.setText(dataList.get(position).getRecognitionTime());
            //((ResultViewHolder) holder).locationTv.setText(dataList.get(position).getLeft()+" "+dataList.get(position).getTop()+" "+dataList.get(position).getRight()+" "+dataList.get(position).getBottom());
            ((ResultViewHolder) holder).time2Tv.setText(dataList.get(position).getTime2());
        }
    }

    @Override
    public int getItemCount() {
        return dataList.size();
    }

    public class ResultViewHolder extends RecyclerView.ViewHolder {

        private TextView timeTv;
        private TextView resultTv;
        private TextView probability;
        private TextView recognitionTimeTv;
        private TextView locationTv;
        private TextView time2Tv;

        public ResultViewHolder(View itemView) {
            super(itemView);
            timeTv = itemView.findViewById(R.id.recognition_time_tv);
            resultTv = itemView.findViewById(R.id.recognition_result_tv);
            probability = itemView.findViewById(R.id.recognition_probability_tv);
            recognitionTimeTv = itemView.findViewById(R.id.finish_time_tv);
            locationTv = itemView.findViewById(R.id.location_tv);
            time2Tv = itemView.findViewById(R.id.time2_tv);
        }
    }

}

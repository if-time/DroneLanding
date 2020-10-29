package com.liyang.droneplus;

import android.Manifest;
import android.content.BroadcastReceiver;
import android.content.ClipData;
import android.content.Context;
import android.content.DialogInterface;
import android.content.Intent;
import android.content.IntentFilter;
import android.graphics.Bitmap;
import android.graphics.BitmapFactory;
import android.graphics.Color;
import android.graphics.Matrix;
import android.graphics.Point;
import android.graphics.SurfaceTexture;
import android.graphics.drawable.BitmapDrawable;
import android.os.Build;
import android.os.Environment;
import android.os.Handler;
import android.os.HandlerThread;
import android.os.Looper;
import android.support.annotation.NonNull;
import android.support.annotation.Nullable;
import android.support.v4.app.ActivityCompat;
import android.support.v7.app.AlertDialog;
import android.support.v7.app.AppCompatActivity;
import android.os.Bundle;
import android.support.v7.widget.LinearLayoutManager;
import android.support.v7.widget.RecyclerView;
import android.util.Log;
import android.view.Display;
import android.view.LayoutInflater;
import android.view.MotionEvent;
import android.view.TextureView;
import android.view.View;
import android.view.ViewGroup;
import android.view.Window;
import android.view.WindowManager;
import android.view.animation.Animation;
import android.view.animation.LinearInterpolator;
import android.view.animation.RotateAnimation;
import android.widget.Button;
import android.widget.EditText;
import android.widget.ImageButton;
import android.widget.ImageView;
import android.widget.LinearLayout;
import android.widget.RadioButton;
import android.widget.RadioGroup;
import android.widget.RelativeLayout;
import android.widget.SlidingDrawer;
import android.widget.TextView;
import android.widget.Toast;

import com.amap.api.maps.AMap;
import com.amap.api.maps.CameraUpdate;
import com.amap.api.maps.CameraUpdateFactory;
import com.amap.api.maps.CoordinateConverter;
import com.amap.api.maps.MapView;
import com.amap.api.maps.UiSettings;
import com.amap.api.maps.model.BitmapDescriptorFactory;
import com.amap.api.maps.model.CameraPosition;
import com.amap.api.maps.model.LatLng;
import com.amap.api.maps.model.Marker;
import com.amap.api.maps.model.MarkerOptions;
import com.amap.api.maps.model.Polygon;
import com.amap.api.maps.model.PolygonOptions;
import com.amap.api.maps.model.Polyline;
import com.amap.api.maps.model.PolylineOptions;
import com.liyang.droneplus.adpater.RecognitionResultAdapter;
import com.liyang.droneplus.application.DemoApplication;
import com.liyang.droneplus.apriltags.ApriltagPoseEstimation;
import com.liyang.droneplus.apriltags.ApriltagR;
import com.liyang.droneplus.bean.RecognitionResultItem;
import com.liyang.droneplus.bean.TimeLineAction;
import com.liyang.droneplus.classifier.ApriltagsDetectAPIModel;
import com.liyang.droneplus.classifier.RecognizerForAprilTags;
import com.liyang.droneplus.util.GPSTransformUtil;
import com.liyang.droneplus.util.GeneralUtils;
import com.liyang.droneplus.util.ImageUtils;
import com.liyang.droneplus.util.LogUtil;
import com.liyang.droneplus.util.PIDControlIncTest;
import com.liyang.droneplus.util.WriteFileUtil;
import com.liyang.droneplus.view.WaypointView;

import org.opencv.android.OpenCVLoader;
import org.opencv.core.Mat;

import java.io.File;
import java.math.BigDecimal;
import java.text.SimpleDateFormat;
import java.util.ArrayList;
import java.util.Date;
import java.util.HashMap;
import java.util.LinkedList;
import java.util.List;
import java.util.Map;
import java.util.Timer;
import java.util.TimerTask;
import java.util.concurrent.ConcurrentHashMap;

import dji.common.battery.BatteryState;
import dji.common.camera.SystemState;
import dji.common.error.DJIError;
import dji.common.flightcontroller.CompassCalibrationState;
import dji.common.flightcontroller.FlightControllerState;
import dji.common.flightcontroller.FlightMode;
import dji.common.flightcontroller.simulator.SimulatorState;
import dji.common.flightcontroller.virtualstick.FlightControlData;
import dji.common.flightcontroller.virtualstick.FlightCoordinateSystem;
import dji.common.flightcontroller.virtualstick.RollPitchControlMode;
import dji.common.flightcontroller.virtualstick.VerticalControlMode;
import dji.common.flightcontroller.virtualstick.YawControlMode;
import dji.common.gimbal.Attitude;
import dji.common.gimbal.GimbalState;
import dji.common.gimbal.Rotation;
import dji.common.mission.hotpoint.HotpointHeading;
import dji.common.mission.hotpoint.HotpointMission;
import dji.common.mission.hotpoint.HotpointStartPoint;
import dji.common.mission.waypoint.Waypoint;
import dji.common.mission.waypoint.WaypointAction;
import dji.common.mission.waypoint.WaypointActionType;
import dji.common.mission.waypoint.WaypointMission;
import dji.common.mission.waypoint.WaypointMissionDownloadEvent;
import dji.common.mission.waypoint.WaypointMissionExecutionEvent;
import dji.common.mission.waypoint.WaypointMissionFinishedAction;
import dji.common.mission.waypoint.WaypointMissionFlightPathMode;
import dji.common.mission.waypoint.WaypointMissionGotoWaypointMode;
import dji.common.mission.waypoint.WaypointMissionHeadingMode;
import dji.common.mission.waypoint.WaypointMissionUploadEvent;
import dji.common.model.LocationCoordinate2D;
import dji.common.product.Model;
import dji.common.useraccount.UserAccountState;
import dji.common.util.CommonCallbacks;
import dji.sdk.base.BaseProduct;
import dji.sdk.battery.Battery;
import dji.sdk.camera.Camera;
import dji.sdk.camera.VideoFeeder;
import dji.sdk.codec.DJICodecManager;
import dji.sdk.flightcontroller.Compass;
import dji.sdk.flightcontroller.FlightController;
import dji.sdk.gimbal.Gimbal;
import dji.sdk.mission.MissionControl;
import dji.sdk.mission.Triggerable;
import dji.sdk.mission.timeline.TimelineElement;
import dji.sdk.mission.timeline.TimelineEvent;
import dji.sdk.mission.timeline.TimelineMission;
import dji.sdk.mission.timeline.actions.GimbalAttitudeAction;
import dji.sdk.mission.timeline.actions.GoHomeAction;
import dji.sdk.mission.timeline.actions.GoToAction;
import dji.sdk.mission.timeline.actions.HotpointAction;
import dji.sdk.mission.timeline.actions.RecordVideoAction;
import dji.sdk.mission.timeline.actions.ShootPhotoAction;
import dji.sdk.mission.timeline.actions.TakeOffAction;
import dji.sdk.mission.timeline.triggers.AircraftLandedTrigger;
import dji.sdk.mission.timeline.triggers.BatteryPowerLevelTrigger;
import dji.sdk.mission.timeline.triggers.Trigger;
import dji.sdk.mission.timeline.triggers.TriggerEvent;
import dji.sdk.mission.timeline.triggers.WaypointReachedTrigger;
import dji.sdk.mission.waypoint.WaypointMissionOperator;
import dji.sdk.mission.waypoint.WaypointMissionOperatorListener;
import dji.sdk.products.Aircraft;
import dji.sdk.sdkmanager.DJISDKManager;
import dji.sdk.useraccount.UserAccountManager;

import static com.liyang.droneplus.apriltags.ApriltagR.resultPath;

/**
 * 降落部分，去掉分象限
 */
public class MainActivity extends AppCompatActivity implements TextureView.SurfaceTextureListener, View.OnClickListener, View.OnTouchListener, AMap.OnMapClickListener, AMap.OnInfoWindowClickListener, RadioGroup.OnCheckedChangeListener {

    private static final String TAG = MainActivity.class.getSimpleName();

    protected VideoFeeder.VideoDataListener mReceivedVideoDataListener = null;

    private DJICodecManager mCodecManager;      // Codec for video live view

    private String brand;

    private int viewWidth = 2418;
    private int viewHeight = 1600;

    /*-----------------------------------主界面控件----------------------------------*/

    private View vview;
    private View mView;

    private UiSettings uiSettings; // 地图UI设置
    private RelativeLayout mapContainer; // 右下角的布局容器
    private RelativeLayout dcontainer; // 与屏幕大小相同的布局容器
    private RelativeLayout cover; // 放置在右下角的布局容器中，用于点击切换地图和视频界面

    private boolean isMap;
    private AMap aMap;
    private MapView mapView;
    private PolylineOptions polylineOptions;
    private Polyline polyline;

    private CameraPosition cameraPosition;

    private TextureView fpvWidget; // 视频控件，FPVWidget继承自TextureView

    private RecyclerView resultList; // 显示识别结果的列表控件
    private RecognitionResultAdapter adapter;
    private ArrayList<RecognitionResultItem> resultItemArrayList;

    // 下面是自己添加的按钮类控件
    private ImageView disableSimulatorIv; // 关闭虚拟摇杆
    private Button turnBtn;

    private LinearLayout atsrell;
    private LinearLayout atinfoll;
    private LinearLayout speedparall;

    //apriltags详情 抽屉
    private ImageButton mPushDrawerIb;
    private SlidingDrawer mPushInfoSd;
    private TextView ltPushInfoTv;
    private TextView rtPushInfoTv;
    private TextView lbPushInfoTv;
    private TextView rbPushInfoTv;
    private TextView cPushInfoTv;
    private TextView bPushInfoTv;

    private TextView bigangletv;
    private TextView ltangletv;
    private TextView rtangletv;
    private TextView lbangletv;
    private TextView rbangletv;
    private TextView cangletv;
    private TextView caliangletv;

    private TextView timetv;


    // 识别
    private ImageView detectIv;

    private TextView ultrasonicTv;
    private TextView yawTv;
    private TextView pitchTv;

    private LinearLayout timelinell;

    // TimeLine
    private ImageView setWpIv;
    private ImageView wpInitIv;
    private ImageView wpStartIv;
    private ImageView wpStopIv;
    private ImageView wpPauseIv;
    private ImageView wpResumeIv;
    private ImageView wpCleanIv;

    private LinearLayout curTlInfoLl;
    private TextView curHeiText;
    private TextView curYawText;
    private TextView curGPitchText;


//    private LinearLayout tlSettingLl;

//    private LinearLayout settingHLl;
//    private LinearLayout settingAttLl;

//    private TextView heiTextTv;
//    private TextView droneYawTv;
//    private TextView gimbalPitchTv;
    private Button tlWpBtn;
//    private Button wpSettingBtn;
//    private Button wpCancelBtn;

    private RelativeLayout wpTlRl;

    private ImageView heig_M ;
    private EditText heig_text ;
    private ImageView heig_P;

    private RadioGroup hot_set;

    private RadioButton R_is_hot ;
    private RadioButton R_no_hot;

    private LinearLayout ishot_layout ;
    private ImageView hot_M ;
    private EditText radius_text ;
    private ImageView hot_P ;

    private ImageView hot_h_M ;
    private EditText hot_h_text ;
    private ImageView hot_h_P ;

    private RadioButton R_photo ;
    private RadioButton R_video_yes ;
    private RadioButton R_video_no;
    private RadioButton R_no ;

    private RadioGroup camera_type;

    private ImageView camera_M ;
    private EditText camera_text;
    private ImageView camera_P;

    private ImageView flight_M ;
    private EditText flight_text;
    private ImageView flight_P;

    private Button confirmbtn;
    private Button cancelbtn;

    /*-----------------------------------主界面控件----------------------------------*/

    private FlightController mFlightController;
    private Compass compass;
    private Gimbal gimbal;
    private Battery battery;
    private Camera camera;
    // 飞机状态
    private LatLng sourceLatLng; // 无人机位置(真实坐标)
    private FlightMode flightMode; // 飞行模式
    private double roll;
    private double yaw;
    private double pitch;
    private int satelliteCount; // 卫星数
    private float velocityX; // X轴方向的速度
    private float velocityY; // Y轴方向的速度
    private float velocityZ; // Z轴方向的速度
    private float gimbalPitch; // 云台俯仰角
    private float gimbalYaw; // 云台水平角度
    private int batteryRemianing; // 剩余电量
    private int flightTime; // 已飞行的时间

    private String lastUploadTime = "2018-12-12 00:00:00";

    private boolean isTraceEnd;
    private long startTime;
    private float maxAltitude;
    private int imageCount;
    private boolean isRecord;
    private boolean isShot;
    private long videoStartTime;
    private long videoEndTime;
    private int videoLength; // 录像总时长
    private float distance;
    private String endTraceTime;
    private float pointCenterX;
    private double pointCenterY;
    private double hCenterX;
    private double hCenterY;
    private double angle = 0;

    /**
     * 移动设配屏幕的高度
     */
    private int height; // 移动设配屏幕的高度
    private int width; // 移动设配屏幕的宽度
    /**
     * cm
     */
    private float errX = 9;
    private float errY = 9;
    private float errXprecise;
    private float errYprecise;
    private float XDistance;
    private float YDistance;
    private float aircraftHeight;
    private float ultrasonicHeight;

    /*------------------------------下面是timeline飞行控制相关变量--------------------------------*/

    private float cur_height_tl = 0;
    private float cur_pitch_tl = 0;
    private double cur_yaw_tl = 0;

    private double droneGDLocationLat_tl = 36.667094, droneGDLocationLng_tl = 117.140428;

    private float height_tl = 50;

    private boolean hotset_tl = false;
    private int radius_tl = 5;
    private int hot_height_tl = 50;

    private int camera_action_type_tl = 4; // 相机动作：1拍照，2录像，3结束录像，4无动作(如果是录像就继续录像)

    private int pitch_tl = 0;
    private int yaw_tl = 0;

    private int markerNum;

    private static double PI = Math.PI; // 圆周率
    private static double AXIS = 6378245.0;  //轴
    private static double OFFSET = 0.00669342162296594323;  //偏移量 (a^2 - b^2) / a^2

    private double droneGDLocationLat = 36.667094, droneGDLocationLng = 117.140428; // 无人机坐标(高德)
    private double droneLocationLat = 37, droneLocationLng = 117;       // 无人机坐标(大疆)
    private float droneLocationHeight = 30f;

    public static WaypointMission.Builder waypointMissionBuilder;
    private WaypointMissionOperator instance;
    private WaypointMissionFinishedAction mFinishedAction = WaypointMissionFinishedAction.NO_ACTION;
    private WaypointMissionHeadingMode mHeadingMode = WaypointMissionHeadingMode.USING_WAYPOINT_HEADING;
    private WaypointMissionFlightPathMode pathMode = WaypointMissionFlightPathMode.CURVED;
    private WaypointMissionOperator waypointMissionOperator;

    private Marker droneMarker = null;

    private float altitude = 30.0f;
    private float mSpeed = 10.0f;


    private boolean isAdd = false;

    //    private
    private boolean initTimeLine = false;

    private MissionControl missionControl;
    private TimelineEvent preEvent;
    private TimelineElement preElement;
    private DJIError preError;
    private boolean isGetHome;
    protected double homeLatitude = 181; // 真实坐标
    protected double homeLongitude = 181; // 真实坐标
    protected double homeGDLatitude = 181; // 高德坐标
    protected double homeGDLongitude = 181; // 高德坐标

    private GPSTransformUtil gpsTransformUtil;

    private List<Marker> markerList = new ArrayList<>();

    private final Map<Integer, Marker> mMarkers = new ConcurrentHashMap<Integer, Marker>();
    private List<Waypoint> waypointList = new ArrayList<>();

    private List<Waypoint> waypointList_GD = new ArrayList<>();
    private List<TimeLineAction> timeLineActionList = new ArrayList<>();
    private List<Integer> heightList = new ArrayList<>();
    private List<Boolean> hotpointList = new ArrayList<>();
    private List<Integer> radiusList = new ArrayList<>();
    private List<Integer> hotheightList = new ArrayList<>();
    private List<Integer> actionList = new ArrayList<>();
    private List<Integer> gimbalangleList = new ArrayList<>();
    private List<Integer> yawangleList = new ArrayList<>();

    private ArrayList<LatLng> marker_GD_GPS = new ArrayList<>();

    private List<LatLng> points = new ArrayList<>();

    private List<Polyline> polylineList = new ArrayList<>();

    List<Integer> hot_num = new ArrayList<>();
    List<Integer> record_num = new ArrayList<>();
    private boolean isRecordTimeLine = false;

    /*-----------------------------上面是timeline飞行控制相关变量--------------------------------*/

    private ImageView landingIm;

    private ArrayList<TarmacResult> tarmacResults;

    private Handler handlerForTarmac;
    private HandlerThread handlerThreadForTarmac;

    private boolean isTarmacRec;
    private boolean tarmacFlag = false;

    public double tar_x;
    public double tar_y;
    public double tar_width;
    public double tar_height;
    public boolean isTarmacJni = false;

    public double tar_dx;
    public double tar_dy;

    private boolean firstLanding = false;
    private boolean secondLanding = false;
    private boolean thirdLanding = false;

    private Bitmap captureBitmap;
    /*---------------------------AprilTag识别----------------------------*/

    // 改变速度参数
    private float oneSpeedPara = 0.0009f;   // 7m以上
    private float twoSpeedPara = 0.0008f;   // 5m以上
    private float threeSpeedPara = 0.00041f; // 3m以上
    private float fourSpeedPara = 0.00011f; // 3m以下

    private float turnSpeedPara = 0.55f;

    private TextView speedParaTv;
    private ImageView speedParaIv;

    public boolean isDetection = false;

    //    private int id;
    //    public double center_x, center_y, left_top_x, left_top_y, right_top_x, right_top_y, right_bottom_x, right_bottom_y, left_bottom_x, left_bottom_y;
    double t_x = 0;
    double t_y = 0;
    double angle_z;
    double tagCenterX = 0;
    double tagCenterY = 0;

    float tagErrX = 300;
    float tagErrY = 300;
    private ArrayList<ApriltagPoseEstimation> mPoses;

    private double last_t_x = 0;
    private double last_t_y = 0;
    private double last_tagCenterX = 0;
    private double last_tagCenterY = 0;

    private boolean tagfirstLanding = false;
    private boolean tagsecondLanding = false;
    private boolean tagthirdLanding = false;
    private boolean tagforthLanding = false;

    private volatile boolean tagFlag = false;
    Thread tagThread;

    private Handler handlerForAprilTags;
    private HandlerThread handlerThreadForAprilTags;
    private RecognizerForAprilTags recognizerForAprilTags;

    // 借助TimerTask来定时发数据，timePeriod设为200ms  官方文档推荐app与遥控的通信频率在50Hz
    private float leftRight, frontBack, turnLeftRight, upDown;
    private Timer sendVirtualStickDataTimer;
    private SendVirtualStickDataTask sendVirtualStickDataTask;

    private float boxX;
    private float boxY;
    private float screenX;
    private float screenY;
    private float dX;
    private float dY;

    private boolean isSimulator = false;
    private boolean isLandingOpen = true;
    private boolean a = true;
    private boolean isLandingOpenToast = false;
    private boolean isTurnOpen = false;

    private int adjustTimes = 0;
    private int secondAdjustTimes = 0;

    private float controlValueIncX;
    private float controlValueIncY;
    private float controlValueIncTurn;

    // 3米以下
    private double cx, cy, ltx, lty, lbx, lby, rtx, rty, rbx, rby;      // 五个标签的中心点像素坐标
    private double ltdx, ltdy, lbdx, lbdy, rtdx, rtdy, rbdx, rbdy, cdx, cdy;    // 与屏幕中心的距离

    private double ltscreendx, ltscreendy, lbscreendx, lbscreendy, rtscreendx, rtscreendy, rbscreendx, rbscreendy; // 与中心标签距离

    private int center_id;
    private double c_t_x, c_t_y;     // 中心标签的水平位移
    private double cangle_z;

    private int left_top_id;
    private double left_top_t_x, left_top_t_y;
    private double ltangle_z;
    private double lt_c_tx, lt_c_ty;    // 标签间的距离

    private int right_top_id;
    private double right_top_t_x, right_top_t_y;
    private double rtangle_z;
    private double rt_c_tx, rt_c_ty;

    private int left_bottom_id;
    private double left_bottom_t_x, left_bottom_t_y;
    private double lbangle_z;
    private double lb_c_tx, lb_c_ty;

    private int right_bottom_id;
    private double right_bottom_t_x, right_bottom_t_y;
    private double rbangle_z;
    private double rb_c_tx, rb_c_ty;

    // 10米以下3米以上
    // 标签ID：10
    private int ten_center_id;
    private double ten_center_x, ten_center_y;
    private double ten_center_t_x, ten_center_t_y;
    private double ten_center_dx, ten_center_dy;
    private double ten_angle_z;


    // 判断是否之前已经识别过ID为0的标签
    // 每一次for循环是否都识别这几个标签
    private boolean notfirstRec0 = false;
    private boolean notfirstRec10 = false;
    private boolean notfirstRec1 = false;
    private boolean notfirstRec2 = false;
    private boolean notfirstRec3 = false;
    private boolean notfirstRec4 = false;

    // 每一次for循环记录识别的标签
    private HashMap<Integer, Integer> recResult = new HashMap<>();
    private int index = 0;

    private boolean firstaprilinit = false;
    private boolean secondaprilinit = false;

    private boolean virtualStickModeEnabled = true;

    public void writeAprilTagsStatus(final int id, final double t_x, final double t_y, final float dX, final float dY, final float ultrasonicHeight, final float controlValueIncX, final float controlValueIncY, final double angle_z) {
        // 将数据写入文件，包括每次识别后：停机坪框的中心与屏幕中心在x轴和y轴的距离差、无人机高度、x方向和y方向的控制量、当前时间
        runOnUiThread(new Runnable() {
            @Override
            public void run() {
                if (WriteFileUtil.isExternalStorageWritable()) {
                    String status = Environment.getExternalStorageState();
                    if (status.equals(Environment.MEDIA_MOUNTED)) {
                        File dir = new File(Environment.getExternalStorageDirectory().getAbsolutePath() + "/droneplus/");
                        if (!dir.exists()) {
                            dir.mkdir();
                        }
                        WriteFileUtil.putStringToExternalStorage(t_x + "\r\n", dir, "t_x_tag_" + id + ".txt", true);
                        WriteFileUtil.putStringToExternalStorage(t_y + "\r\n", dir, "t_y_tag_" + id + ".txt", true);
                        WriteFileUtil.putStringToExternalStorage(dX + "\r\n", dir, "dx_tag_" + id + ".txt", true);
                        WriteFileUtil.putStringToExternalStorage(dY + "\r\n", dir, "dy_tag_" + id + ".txt", true);
                        WriteFileUtil.putStringToExternalStorage(ultrasonicHeight + "\r\n", dir, "height_tag_" + id + ".txt", true);
                        WriteFileUtil.putStringToExternalStorage(controlValueIncX + "\r\n", dir, "PIDx_tag_" + id + ".txt", true);
                        WriteFileUtil.putStringToExternalStorage(controlValueIncY + "\r\n", dir, "PIDy_tag_" + id + ".txt", true);
                        //                                        WriteFileUtil.putStringToExternalStorage(controlValueIncTurn+"\r\n", dir, "PID_turn.txt", true);
                        WriteFileUtil.putStringToExternalStorage(currentTime() + "\r\n", dir, "time_tag_" + id + ".txt", true);
                        WriteFileUtil.putStringToExternalStorage(angle_z + "\r\n", dir, "angle_z_tag_" + id + ".txt", true);
                    }
                }
            }
        });
    }

    /**
     * AprilTags 线程
     */
    Runnable tagRunnable = new Runnable() {
        @Override
        public void run() {

            runInBackgroundForAprilTags(new Runnable() {
                @Override
                public void run() {
                    while (tagFlag) {

                        long start = System.currentTimeMillis();
                        // 如果低于0.6米直接降落, 反之计算
                        if (ultrasonicHeight >= 0.4) {   //0.4

                            // 初始化aprilTags
                            if (ultrasonicHeight > 3.1) {
                                if (!firstaprilinit) {
                                    ApriltagR.apriltagInit("tag36h11", 2, 4, 0.0, 8, 0.04, 5423.612353784232, 5519.056520316598, 1315.1967405060602, 838.5860437396254);
                                    Log.i("apriltagInit", "run: notfirstRec10");
                                }
                                firstaprilinit = true;
                            }
                            if (ultrasonicHeight <= 3.2) {
                                if (!secondaprilinit) {
                                    Log.i("apriltagInit", "run: notfirstRec0");
                                    ApriltagR.apriltagInit("tag36h11", 2, 4, 0.0, 8, 0.005184, 5423.612353784232, 5519.056520316598, 1315.1967405060602, 838.5860437396254);
                                }
                                secondaprilinit = true;
                            }

                            Log.i("dongrun", "run: ");
                            List<RecognizerForAprilTags.Recognition> results = null;

                            if (recognizerForAprilTags != null) {

                                long startgetBitamp = System.currentTimeMillis();
                                Bitmap bitmap = fpvWidget.getBitmap();
                                Log.i("dongpredict_image", "run: getBitamp : " + (System.currentTimeMillis() - startgetBitamp) + "ms");
                                results = recognizerForAprilTags.recognizeImage(bitmap);
                            }
                            long start1 = System.currentTimeMillis();

                            runOnUiThread(new Runnable() {
                                @Override
                                public void run() {
                                    timetv.setText("Time : " + new SimpleDateFormat("yyyy-MM-dd HH:mm:ss").format(System.currentTimeMillis()));
                                }
                            });

                            if (results != null) {

                                // 如果有识别结果
                                if (results.size() > 0) {
                                    Log.i("dongnotfindmark", "if  results != null : results.size(): " + results.size());
                                    long start2 = System.currentTimeMillis();
                                    // 获取标签信息 遍历识别有几个结果
                                    for (final RecognizerForAprilTags.Recognition result : results) {

                                        resultItemArrayList.add(new RecognitionResultItem(String.valueOf(new BigDecimal(result.getAngle_z()).setScale(2, BigDecimal.ROUND_HALF_UP).doubleValue()) + "°", "x:" + String.valueOf(new BigDecimal(result.getT_x()).setScale(2, BigDecimal.ROUND_HALF_UP).doubleValue()) + "cm", "y:" + String.valueOf(new BigDecimal(result.getT_y()).setScale(2, BigDecimal.ROUND_HALF_UP).doubleValue()) + "cm", "id:" + result.id, "", "", "", "", ""));
                                        while (resultItemArrayList.size() > 6) {
                                            resultItemArrayList.remove(0);
                                        }

                                        runOnUiThread(new Runnable() {
                                            @Override
                                            public void run() {
                                                adapter.notifyDataSetChanged();
                                            }
                                        });

                                        StringBuffer sb = new StringBuffer();

                                        switch (result.id) {
                                            case 10:
//                                                recResult.put(index++, 10);
                                                notfirstRec10 = true;

                                                ten_center_x = result.getCenter_x();
                                                ten_center_y = result.getCenter_y();

                                                ten_center_dx = screenX - ten_center_x;
                                                ten_center_dy = screenY - ten_center_y;

                                                ten_center_t_x = result.getT_x();
                                                ten_center_t_y = result.getT_y();

                                                ten_angle_z = result.getAngle_z();

                                                runOnUiThread(new Runnable() {
                                                    @Override
                                                    public void run() {
                                                        bigangletv.setText("ID 10: " + (double) Math.round(ten_angle_z * 100) / 100 + "°");
                                                    }
                                                });

                                                if (ultrasonicHeight > 3.0) {

                                                    // 在3米以上， 无法识别时 保持之前的值
                                                    last_t_x = ten_center_t_x;
                                                    last_t_y = ten_center_t_y;

                                                    last_tagCenterX = ten_center_x;
                                                    last_tagCenterY = ten_center_y;
                                                }

                                                LogUtil.addLineToSB(sb, "右下标签", result.id == 10 ? 10 : "");
                                                LogUtil.addLineToSB(sb, "center_x: ", Math.round(result.getCenter_x() * 100) / 100);
                                                LogUtil.addLineToSB(sb, "center_y: ", Math.round(result.getCenter_y() * 100) / 100);
                                                LogUtil.addLineToSB(sb, "t_x: ", Math.round(result.getT_x() * 100) / 100);
                                                LogUtil.addLineToSB(sb, "t_y: ", Math.round(result.getT_y() * 100) / 100);
                                                LogUtil.addLineToSB(sb, "距离屏幕中心 x(px): ", Math.round((screenX - result.getCenter_x()) * 100) / 100);
                                                LogUtil.addLineToSB(sb, "距离屏幕中心 y(px): ", Math.round((screenY - result.getCenter_y()) * 100) / 100);
                                                LogUtil.addLineToSB(sb, "z轴方向倾角: ", result.getAngle_z());
                                                LogUtil.addLineToSB(sb, "与中心标签距离 x(px): ", Math.round((rbx - cx) * 100) / 100);
                                                LogUtil.addLineToSB(sb, "与中心标签距离 y(px): ", Math.round((rby - cy) * 100) / 100);

                                                writeAprilTagsStatus(result.id, result.getT_x(), result.getT_y(), (float) (screenX - result.getCenter_x()), (float) (screenY - result.getCenter_y()), ultrasonicHeight, 0, 0, result.getAngle_z());
                                                break;
                                            case 1:
                                                recResult.put(index++, 1);
                                                notfirstRec1 = true;

                                                ltx = result.getCenter_x();
                                                lty = result.getCenter_y();

                                                ltdx = Math.abs(ltx - screenX);
                                                ltdy = Math.abs(lty - screenY);

                                                left_top_t_x = result.getT_x();
                                                left_top_t_y = result.getT_y();

                                                lt_c_tx = Math.abs(left_top_t_x - c_t_x);
                                                lt_c_ty = Math.abs(left_top_t_y - c_t_y);

                                                ltscreendx = Math.round(ltx - cx);
                                                ltscreendy = Math.round(lty - cy);

                                                ltangle_z = result.getAngle_z();

                                                runOnUiThread(new Runnable() {
                                                    @Override
                                                    public void run() {
                                                        ltangletv.setText("ID 1 : " + (double) Math.round(ltangle_z * 100) / 100 + "°");
                                                    }
                                                });

                                                LogUtil.addLineToSB(sb, "左上标签: ", result.id == 1 ? 1 : "");
                                                LogUtil.addLineToSB(sb, "center_x: ", Math.round(result.getCenter_x() * 100) / 100);
                                                LogUtil.addLineToSB(sb, "center_y: ", Math.round(result.getCenter_y() * 100) / 100);
                                                LogUtil.addLineToSB(sb, "t_x: ", Math.round(result.getT_x() * 100) / 100);
                                                LogUtil.addLineToSB(sb, "t_y: ", Math.round(result.getT_y() * 100) / 100);
                                                LogUtil.addLineToSB(sb, "距离屏幕中心 x(px): ", Math.round((screenX - result.getCenter_x()) * 100) / 100);
                                                LogUtil.addLineToSB(sb, "距离屏幕中心 y(px): ", Math.round((screenY - result.getCenter_y()) * 100) / 100);
                                                LogUtil.addLineToSB(sb, "z轴方向倾角: ", result.getAngle_z());
                                                LogUtil.addLineToSB(sb, "与中心标签距离 x(px): ", Math.round((ltx - cx) * 100) / 100);
                                                LogUtil.addLineToSB(sb, "与中心标签距离 y(px): ", Math.round((lty - cy) * 100) / 100);

                                                writeAprilTagsStatus(result.id, result.getT_x(), result.getT_y(), (float) (screenX - result.getCenter_x()), (float) (screenY - result.getCenter_y()), ultrasonicHeight, 0, 0, result.getAngle_z());
                                                break;
                                            case 4:
                                                recResult.put(index++, 4);
                                                notfirstRec4 = true;

                                                lbx = result.getCenter_x();
                                                lby = result.getCenter_y();

                                                lbdx = Math.abs(lbx - screenX);
                                                lbdy = Math.abs(lby - screenY);

                                                left_bottom_t_x = result.getT_x();
                                                left_bottom_t_y = result.getT_y();

                                                lb_c_tx = Math.abs(left_bottom_t_x - c_t_x);
                                                lb_c_ty = Math.abs(left_bottom_t_y - c_t_x);

                                                lbscreendx = Math.round(lbx - cx);
                                                lbscreendy = Math.round(lby - cy);

                                                lbangle_z = result.getAngle_z();

                                                runOnUiThread(new Runnable() {
                                                    @Override
                                                    public void run() {
                                                        lbangletv.setText("ID 4 : " + (double) Math.round(lbangle_z * 100) / 100 + "°");
                                                    }
                                                });

                                                LogUtil.addLineToSB(sb, "左下标签", result.id == 4 ? 4 : "");
                                                LogUtil.addLineToSB(sb, "center_x: ", Math.round(result.getCenter_x() * 100) / 100);
                                                LogUtil.addLineToSB(sb, "center_y: ", Math.round(result.getCenter_y() * 100) / 100);
                                                LogUtil.addLineToSB(sb, "t_x: ", Math.round(result.getT_x() * 100) / 100);
                                                LogUtil.addLineToSB(sb, "t_y: ", Math.round(result.getT_y() * 100) / 100);
                                                LogUtil.addLineToSB(sb, "距离屏幕中心 x(px): ", Math.round((screenX - result.getCenter_x()) * 100) / 100);
                                                LogUtil.addLineToSB(sb, "距离屏幕中心 y(px): ", Math.round((screenY - result.getCenter_y()) * 100) / 100);
                                                LogUtil.addLineToSB(sb, "z轴方向倾角: ", result.getAngle_z());
                                                LogUtil.addLineToSB(sb, "与中心标签距离 x(px): ", Math.round((lbx - cx) * 100) / 100);
                                                LogUtil.addLineToSB(sb, "与中心标签距离 y(px): ", Math.round((lby - cy) * 100) / 100);

                                                writeAprilTagsStatus(result.id, result.getT_x(), result.getT_y(), (float) (screenX - result.getCenter_x()), (float) (screenY - result.getCenter_y()), ultrasonicHeight,  0, 0, result.getAngle_z());


                                                break;
                                            case 2:
                                                recResult.put(index++, 2);
                                                notfirstRec2 = true;

                                                rtx = result.getCenter_x();
                                                rty = result.getCenter_y();

                                                rtdx = Math.abs(rtx - screenX);
                                                rtdy = Math.abs(rty - screenY);

                                                right_top_t_x = result.getT_x();
                                                right_top_t_y = result.getT_y();

                                                rt_c_tx = Math.abs(right_top_t_x - c_t_x);
                                                rt_c_ty = Math.abs(right_top_t_y - c_t_x);

                                                rtscreendx = Math.round(rtx - cx);
                                                rtscreendy = Math.round(rty - cy);

                                                rtangle_z = result.getAngle_z();

                                                runOnUiThread(new Runnable() {
                                                    @Override
                                                    public void run() {
                                                        rtangletv.setText("ID 2 : " + (double) Math.round(rtangle_z * 100) / 100 + "°");
                                                    }
                                                });

                                                LogUtil.addLineToSB(sb, "右上标签: ", result.id == 2 ? 2 : "");
                                                LogUtil.addLineToSB(sb, "center_x: ", Math.round(result.getCenter_x() * 100) / 100);
                                                LogUtil.addLineToSB(sb, "center_y: ", Math.round(result.getCenter_y() * 100) / 100);
                                                LogUtil.addLineToSB(sb, "t_x: ", Math.round(result.getT_x() * 100) / 100);
                                                LogUtil.addLineToSB(sb, "t_y: ", Math.round(result.getT_y() * 100) / 100);
                                                LogUtil.addLineToSB(sb, "距离屏幕中心 x(px): ", Math.round((screenX - result.getCenter_x()) * 100) / 100);
                                                LogUtil.addLineToSB(sb, "距离屏幕中心 y(px): ", Math.round((screenY - result.getCenter_y()) * 100) / 100);
                                                LogUtil.addLineToSB(sb, "z轴方向倾角: ", result.getAngle_z());
                                                LogUtil.addLineToSB(sb, "与中心标签距离 x(px): ", Math.round((rtx - cx) * 100) / 100);
                                                LogUtil.addLineToSB(sb, "与中心标签距离 y(px): ", Math.round((rty - cy) * 100) / 100);

                                                writeAprilTagsStatus(result.id, result.getT_x(), result.getT_y(), (float) (screenX - result.getCenter_x()), (float) (screenY - result.getCenter_y()), ultrasonicHeight, 0, 0, result.getAngle_z());
                                                break;
                                            case 3:
                                                recResult.put(index++, 3);
                                                notfirstRec3 = true;

                                                rbx = result.getCenter_x();
                                                rby = result.getCenter_y();

                                                rbdx = Math.abs(rbx - screenX);
                                                rbdy = Math.abs(rby - screenY);

                                                right_bottom_t_x = result.getT_x();
                                                right_bottom_t_y = result.getT_y();

                                                rb_c_tx = Math.abs(right_bottom_t_x - c_t_x);
                                                rb_c_ty = Math.abs(right_bottom_t_y - c_t_x);

                                                rbscreendx = Math.round(rbx - cx);
                                                rbscreendy = Math.round(rby - cy);

                                                rbangle_z = result.getAngle_z();

                                                runOnUiThread(new Runnable() {
                                                    @Override
                                                    public void run() {
                                                        rbangletv.setText("ID 3 : " + (double) Math.round(rbangle_z * 100) / 100 + "°");
                                                    }
                                                });

                                                LogUtil.addLineToSB(sb, "右下标签", result.id == 3 ? 3 : "");
                                                LogUtil.addLineToSB(sb, "center_x: ", Math.round(result.getCenter_x() * 100) / 100);
                                                LogUtil.addLineToSB(sb, "center_y: ", Math.round(result.getCenter_y() * 100) / 100);
                                                LogUtil.addLineToSB(sb, "t_x: ", Math.round(result.getT_x() * 100) / 100);
                                                LogUtil.addLineToSB(sb, "t_y: ", Math.round(result.getT_y() * 100) / 100);
                                                LogUtil.addLineToSB(sb, "距离屏幕中心 x(px): ", Math.round((screenX - result.getCenter_x()) * 100) / 100);
                                                LogUtil.addLineToSB(sb, "距离屏幕中心 y(px): ", Math.round((screenY - result.getCenter_y()) * 100) / 100);
                                                LogUtil.addLineToSB(sb, "z轴方向倾角: ", result.getAngle_z());
                                                LogUtil.addLineToSB(sb, "与中心标签距离 x(px): ", Math.round((rbx - cx) * 100) / 100);
                                                LogUtil.addLineToSB(sb, "与中心标签距离 y(px): ", Math.round((rby - cy) * 100) / 100);

                                                writeAprilTagsStatus(result.id, result.getT_x(), result.getT_y(), (float) (screenX - result.getCenter_x()), (float) (screenY - result.getCenter_y()), ultrasonicHeight, 0, 0, result.getAngle_z());
                                                break;
                                            case 0:
                                                recResult.put(index++, 0);
                                                notfirstRec0 = true;

                                                cx = result.getCenter_x();
                                                cy = result.getCenter_y();
                                                c_t_x = result.getT_x();
                                                c_t_y = result.getT_y();

                                                cdx = screenX - cx;
                                                cdy = screenY - cy;

                                                angle_z = result.getAngle_z();

                                                cangle_z = result.getAngle_z();

                                                runOnUiThread(new Runnable() {
                                                    @Override
                                                    public void run() {
                                                        cangletv.setText("ID 0 : " + (double) Math.round(cangle_z * 100) / 100 + "°");
                                                    }
                                                });

                                                LogUtil.addLineToSB(sb, "中间标签:", result.id == 0 ? 0 : "");
                                                LogUtil.addLineToSB(sb, "center_x: ", Math.round(result.getCenter_x() * 100) / 100);
                                                LogUtil.addLineToSB(sb, "center_y: ", Math.round(result.getCenter_y() * 100) / 100);
                                                LogUtil.addLineToSB(sb, "t_x: ", Math.round(result.getT_x() * 100) / 100);
                                                LogUtil.addLineToSB(sb, "x轴方向倾角: ", result.getAngle_x());
                                                LogUtil.addLineToSB(sb, "t_y: ", Math.round(result.getT_y() * 100) / 100);
                                                LogUtil.addLineToSB(sb, "y轴方向倾角: ", result.getAngle_y());
                                                LogUtil.addLineToSB(sb, "z轴方向倾角: ", result.getAngle_z());
                                                LogUtil.addLineToSB(sb, "距离屏幕中心 x(px): ", Math.round((screenX - result.getCenter_x()) * 100) / 100);
                                                LogUtil.addLineToSB(sb, "距离屏幕中心 y(px): ", Math.round((screenY - result.getCenter_y()) * 100) / 100);

                                                writeAprilTagsStatus(result.id, result.getT_x(), result.getT_y(), (float) (screenX - result.getCenter_x()), (float) (screenY - result.getCenter_y()), ultrasonicHeight, 0, 0, result.getAngle_z());

                                                runOnUiThread(new Runnable() {
                                                    @Override
                                                    public void run() {
                                                        if (WriteFileUtil.isExternalStorageWritable()) {
                                                            String status = Environment.getExternalStorageState();
                                                            if (status.equals(Environment.MEDIA_MOUNTED)) {
                                                                File dir = new File(Environment.getExternalStorageDirectory().getAbsolutePath() + "/droneplus/");
                                                                if (!dir.exists()) {
                                                                    dir.mkdir();
                                                                }
                                                                WriteFileUtil.putStringToExternalStorage(result.getAngle_x() + "\r\n", dir, "angle_x_tag.txt", true);
                                                                WriteFileUtil.putStringToExternalStorage(result.getAngle_y() + "\r\n", dir, "angle_y_tag.txt", true);
                                                            }
                                                        }
                                                    }
                                                });

                                                break;
                                        }

                                        setResultToText(sb.toString(), result.id);

                                    } // for   results

                                    Log.i("dongpredict_image", "run: result_for : " + (System.currentTimeMillis() - start2) + "ms");

                                    // 0和10都识别后计算角度
                                    if (notfirstRec10 && notfirstRec0) {
                                        if (cx > ten_center_x && cy < ten_center_y) { // 第一象限

                                            angle = Math.atan((cx - ten_center_x) / (ten_center_y - cy)) * 180 / Math.PI; // angle > 0

                                        } else if (cx > ten_center_x && cy > ten_center_y) { // 第四象限

                                            angle = Math.atan((cy - ten_center_y) / (cx - ten_center_x)) * 180 / Math.PI + 90; // angle > 0

                                        } else if (cx < ten_center_x && cy > ten_center_y) { // 第三象限

                                            angle = -(Math.atan((cy - hCenterY) / (ten_center_x - cx)) * 180 / Math.PI + 90); // angle < 0

                                        } else if (cx < ten_center_x && cy < ten_center_y) { // 第二象限

                                            angle = -Math.atan((ten_center_x - cx) / (ten_center_y - cy)) * 180 / Math.PI; // angle < 0

                                        }
                                    }
                                    Log.i("dongsuanangle", "angle: " + angle + " angle_z:" + cangle_z);

                                    controlValueIncTurn = (float) (angle * turnSpeedPara);
                                    if (ultrasonicHeight > 1.9) {
                                        if (Math.abs(angle) > 1) {
                                            if (angle > 15 && angle <= 180) {
                                                // 向右转
                                                flyControl(0, 0, Math.abs(controlValueIncTurn), 0);
                                                controlValueIncTurn = Math.abs(controlValueIncTurn);
                                            }
                                            if (angle < -15 && angle >= -180) {
                                                // 向左转
                                                flyControl(0, 0, (-1) * Math.abs(controlValueIncTurn), 0);
                                                controlValueIncTurn = (-1) * Math.abs(controlValueIncTurn);
                                            }
                                        }
                                    }
                                    runOnUiThread(new Runnable() {
                                        @Override
                                        public void run() {
                                            caliangletv.setText("caliangle: " + (double) Math.round(angle * 100) / 100 + "°");
                                        }
                                    });

                                    if (ultrasonicHeight > 3.0) {

                                        if (ultrasonicHeight > 7.7) {
                                            if (Math.abs(ten_center_t_x) >= errX) {
                                                controlValueIncX = (float) (ten_center_dx * oneSpeedPara);
                                            }
                                            if (Math.abs(ten_center_t_y) >= errY) {
                                                controlValueIncY = (float) (ten_center_dy * oneSpeedPara);
                                            }
                                        }
                                        if (ultrasonicHeight <= 7.7 && ultrasonicHeight > 5.2) {
                                            if (Math.abs(ten_center_t_x) >= errX) {
                                                controlValueIncX = (float) (ten_center_dx * twoSpeedPara);
                                            }
                                            if (Math.abs(ten_center_t_y) >= errY) {
                                                controlValueIncY = (float) (ten_center_dy * twoSpeedPara);
                                            }
                                        }
                                        if (ultrasonicHeight <= 5.2 && ultrasonicHeight > 3.0) {
                                            if (Math.abs(ten_center_dx) < 500) {
                                                ten_center_dx = ten_center_dx * 3;
                                            }
                                            if (Math.abs(ten_center_dy) < 500) {
                                                ten_center_dy = ten_center_dy * 3;
                                            }
                                            if (Math.abs(ten_center_t_x) >= errX) {
                                                controlValueIncX = (float) (ten_center_dx * threeSpeedPara);
                                            }
                                            if (Math.abs(ten_center_t_y) >= errY) {
                                                controlValueIncY = (float) (ten_center_dy * threeSpeedPara);
                                            }
                                        }

                                        Log.i("dongnotfindmark10", "run: controlValueIncX: " + controlValueIncX + " controlValueIncY: " + controlValueIncY + " controlValueIncTurn: " + controlValueIncTurn);
                                    }

                                    // 调整 降落
                                    // 3米以上 使用标签10
                                    if (ultrasonicHeight > 7.7) {
                                        if ((Math.abs(ten_center_t_x) <= 200 && Math.abs(ten_center_t_y - 20) <= 200)) {
                                            Log.i("dongislanding10", "run > " + ultrasonicHeight + "m: Math.abs(ten_center_t_x) <= 250 && Math.abs(ten_center_t_y) <= 200");
                                            //                                            flyControl(0, 0, 0, -1.0f);
                                            droneReadyLanding(ten_center_t_x, ten_center_t_y - 20, 200, 200, controlValueIncX, controlValueIncY, -1.0f, -0.4f);
                                        } else {
                                            Log.i("dongislanding10", "run > " + ultrasonicHeight + "m: Math.abs(ten_center_t_x) > 250 && Math.abs(ten_center_t_y) > 200");
                                            droneAdjustment(ten_center_t_x, ten_center_t_y - 20, 200, 200, false, controlValueIncX, controlValueIncY, 0, -0.01f, -0.01f, -0.7f);
                                        }
                                    }

                                    if (ultrasonicHeight <= 7.7 && ultrasonicHeight > 5.2) {
                                        if ((Math.abs(ten_center_t_x) <= 120 && Math.abs(ten_center_t_y - 20) <= 100)) {
                                            Log.i("dongislanding10", "run <= " + ultrasonicHeight + "m && > 5.2m: Math.abs(ten_center_t_x) <= 150 && Math.abs(ten_center_t_y) <= 100");
                                            droneReadyLanding(ten_center_t_x, ten_center_t_y - 20, 100, 100, controlValueIncX, controlValueIncY, -0.6f, -0.3f);
                                        } else {
                                            Log.i("dongislanding10", "run <= " + ultrasonicHeight + "m && > 5.2m: Math.abs(ten_center_t_x) > 150 && Math.abs(ten_center_t_y) > 100");
                                            droneAdjustment(ten_center_t_x, ten_center_t_y - 20, 100, 100, false, controlValueIncX, controlValueIncY, 0, -0.01f, -0.01f, -0.6f);
                                        }
                                    }

                                    if (ultrasonicHeight <= 5.2 && ultrasonicHeight > 3.0) {
                                        if ((Math.abs(ten_center_t_x) <= 35 && Math.abs(ten_center_t_y - 20) <= 35)) {
                                            Log.i("dongislanding10", "run <= " + ultrasonicHeight + "m && > 3.0m: Math.abs(ten_center_t_x) <= 50 && Math.abs(ten_center_t_y) <= 40");
                                            if (Math.abs(ten_center_t_y - 20) <= 25) {
                                                Log.i("dongislanding10", "run <= " + ultrasonicHeight + "m && > 3.0m: Math.abs(ten_center_t_x) <= 35 && Math.abs(ten_center_t_y) <= 25");
                                                flyControl(0, 0, 0, -0.6f);
                                            } else {
                                                Log.i("dongislanding10", "run <= " + ultrasonicHeight + "m && > 3.0m: Math.abs(ten_center_t_y) >= 25");
                                                droneReadyLanding(ten_center_t_x, ten_center_t_y - 20, 25, 25, controlValueIncX, controlValueIncY, -0.5f, -0.4f);
                                            }

                                        } else {
                                            Log.i("dongislanding10", "run <= " + ultrasonicHeight + "m && > 3.0m: Math.abs(ten_center_t_x) > 50 && Math.abs(ten_center_t_y) > 40");
                                            droneAdjustment(ten_center_t_x, ten_center_t_y - 20, 30, 30, false, controlValueIncX, controlValueIncY, controlValueIncTurn, -0.01f, -0.1f, -0.4f);
                                        }
                                    }

                                    // 3米以下
                                    if (ultrasonicHeight <= 3.1) {

                                        // 判断中心标签的偏移
                                        if (ultrasonicHeight >= 2.5) {
                                            if (cy < screenY / 5 * 1.5 || cy > screenY * 2 / 10 * 8.5 || cx < screenX / 5 * 1.5 || cx > screenX * 2 / 10 * 8.5) {
                                                // 第一象限
                                                if (c_t_x > 0 && c_t_y < 0) {
                                                    cdx = cdx - lbdx;
                                                    cdy = cdy + lbdy;
                                                }
                                                // 第二象限
                                                if (c_t_x < 0 && c_t_y < 0) {
                                                    cdx = cdx + rbdx;
                                                    cdy = cdy + rbdy;
                                                }
                                                // 第三象限
                                                if (c_t_x < 0 && c_t_y > 0) {
                                                    cdx = cdx + rtdx;
                                                    cdy = cdy - rtdy;
                                                }
                                                // 第四象限
                                                if (c_t_x > 0 && c_t_y > 0) {
                                                    cdx = cdx - ltdx;
                                                    cdy = cdy - ltdy;
                                                }
                                            }
                                        } else if (ultrasonicHeight < 2.5 && ultrasonicHeight >= 1.5) {
                                            if (cy < screenY / 5 * 2.5 || cy > screenY * 2 / 10 * 7.5 || cx < screenX / 5 * 2.5 || cx > screenX * 2 / 10 * 7.5) {
                                                // 第一象限
                                                if (c_t_x > 0 && c_t_y < 0) {

                                                    cdx = cdx - lbdx;
                                                    cdy = cdy + lbdy;

                                                }
                                                // 第二象限
                                                if (c_t_x < 0 && c_t_y < 0) {

                                                    cdx = cdx + rbdx;
                                                    cdy = cdy + rbdy;

                                                }
                                                // 第三象限
                                                if (c_t_x < 0 && c_t_y > 0) {

                                                    cdx = cdx + rtdx;
                                                    cdy = cdy - rtdy;

                                                }
                                                // 第四象限
                                                if (c_t_x > 0 && c_t_y > 0) {

                                                    cdx = cdx - ltdx;
                                                    cdy = cdy - ltdy;

                                                }
                                            }
                                        } else if (ultrasonicHeight < 1.5 && ultrasonicHeight > 0.9) {
                                            if (cy < screenY / 5 * 3 || cy > screenY * 2 / 10 * 7 || cx < screenX / 5 * 3 || cx > screenX * 2 / 10 * 7) {
                                                Log.i("dong3ge", "run: ");

                                                // 第一象限
                                                if (c_t_x > 0 && c_t_y < 0) {
                                                    // 左下标签在第二、三象限用rb
                                                    if (Math.abs(cdx) < 10000) {
                                                        cdx = cdx - lbdx;
                                                    } else {
                                                        cdx = -10000;
                                                    }

                                                    if (Math.abs(cdy) < 10000) {
                                                        cdy = cdy + lbdy;
                                                    } else {
                                                        cdy = 10000;
                                                    }
                                                    Log.i("dngdXdY1", "cdx : " + cdx + " cdy: " + cdy);
                                                }
                                                // 第二象限
                                                if (c_t_x < 0 && c_t_y < 0) {
                                                    // 右下标签在第一象限用lb
                                                    if (Math.abs(cdx) < 10000) {
                                                        cdx = cdx + rbdx;
                                                    } else {
                                                        cdx = 10000;
                                                    }

                                                    if (Math.abs(cdy) < 10000) {
                                                        cdy = cdy + rbdy;
                                                    } else {
                                                        cdy = 10000;
                                                    }
                                                    Log.i("dngdXdY2", "cdx : " + cdx + " cdy: " + cdy);
                                                }
                                                // 第三象限
                                                if (c_t_x < 0 && c_t_y > 0) {
                                                    //右上标签在第四象限用lt
                                                    if (Math.abs(cdx) < 10000) {
                                                        cdx = cdx + rtdx;
                                                    } else {
                                                        cdx = 10000;
                                                    }

                                                    if (Math.abs(cdy) < 10000) {
                                                        cdy = cdy - rtdy;
                                                    } else {
                                                        cdy = -10000;
                                                    }
                                                    Log.i("dngdXdY3", "cdx : " + cdx + " cdy: " + cdy);
                                                }
                                                // 第四象限
                                                if (c_t_x > 0 && c_t_y > 0) {
                                                    //左上标签在第三象限用rt
                                                    if (Math.abs(cdx) < 10000) {
                                                        cdx = cdx - ltdx;
                                                    } else {
                                                        cdx = -10000;
                                                    }

                                                    if (Math.abs(cdy) < 10000) {
                                                        cdy = cdy - ltdy;
                                                    } else {
                                                        cdy = -10000;
                                                    }
                                                    Log.i("dngdXdY4", "cdx : " + cdx + " cdy: " + cdy);
                                                }
                                            }
                                        }

                                        // 是否识别出中心标签
                                        if (recResult.containsValue(0)) {
                                            // X轴方向，方框中心与屏幕中心的距离差
                                            dX = (float) cdx;
                                            dY = (float) cdy;
                                            if (recResult.size() <= 3) {
                                                if (ultrasonicHeight > 0.6) {
                                                    if (Math.abs(cdx) < 1000) {
                                                        dX = (float) (dX * 2.5);
                                                    }
                                                    if (Math.abs(cdy) < 1000) {
                                                        dY = (float) (dY * 2.5);
                                                    }
                                                }
                                            }
//                                            Log.i("dongrecResult", "run if : is containsValue(0)");
                                            t_x = c_t_x;
                                            t_y = c_t_y;


                                        } else if (recResult.size() > 0 ) {    // 否则使用其他标签做辅助   && !recResult.containsValue(10)

                                            // 中心标签上一次出现时所在位置
                                            if (cy < screenY / 5 * 2.5 || cy > screenY * 2 / 10 * 7.5 || cx < screenX / 5 * 2.5 || cx > screenX * 2 / 10 * 7.5) {

                                                // 第一象限
                                                if (t_x >= 0 && t_y <= 0) {
                                                    cdx = cdx - lbscreendx;
                                                    cdy = cdy + lbscreendy;

                                                    t_x = left_bottom_t_x + lb_c_tx;
                                                    t_y = left_bottom_t_x - lb_c_ty;
                                                }
                                                // 第二象限
                                                if (t_x <= 0 && t_y <= 0) {
                                                    cdx = cdx + rbscreendx;
                                                    cdy = cdy + rbscreendy;

                                                    t_x = right_bottom_t_x - rb_c_tx;
                                                    t_y = right_bottom_t_y - rb_c_ty;
                                                }
                                                // 第三象限
                                                if (t_x <= 0 && t_y >= 0) {
                                                    cdx = cdx + rtscreendx;
                                                    cdy = cdy - rtscreendy;

                                                    t_x = right_top_t_x - rt_c_tx;
                                                    t_y = right_top_t_y + rt_c_ty;
                                                }
                                                // 第四象限
                                                if (t_x >= 0 && t_y >= 0) {
                                                    cdx = cdx - ltscreendx;
                                                    cdy = cdy - ltscreendy;

                                                    t_x = left_top_t_x + lt_c_tx;
                                                    t_y = left_top_t_y + lt_c_ty;
                                                }
                                            }

                                            dX = (float) cdx;
                                            dY = (float) cdy;

                                        }
                                        // 在3米以下， 无法识别时 保持之前的值 保留最后一次的误差
                                        last_t_x = t_x;
                                        last_t_y = t_y;
                                        last_tagCenterX = dX;
                                        last_tagCenterY = dY;

                                        if (ultrasonicHeight < 3.1 && ultrasonicHeight > 2.0) {

                                            if (Math.abs(dX) < 500) {
                                                dX = (float) (dX * 3.5);
                                            }

                                            if (Math.abs(dY) < 500) {
                                                dY = (float) (dY * 3.5);
                                            }
                                        }

                                        // 增量式PID算法，计算x轴方向的控制量
                                        // 距离差>=3cm时，需要调整水平位置，计算得到控制量
                                        if (Math.abs(t_x) >= errX - 6) {
                                            controlValueIncX = dX * fourSpeedPara;
                                        }
                                        if (Math.abs(t_y) >= errY - 6) {
                                            controlValueIncY = dY * fourSpeedPara;
                                        }

                                        Log.i("dongfindmark3", "run: t_x: " + t_x + " t_y: " + t_y);
                                        Log.i("dongfindmark3", "run: cdx: " + cdx + " cdy: " + cdy);
                                        Log.i("dongfindmark3", "run: dx: " + dX + " dy: " + dY);
                                        Log.i("dongfindmark3", "run: controlValueIncX: " + controlValueIncX + " controlValueIncY: " + controlValueIncY + " controlValueIncTurn : " + controlValueIncTurn);

                                        runOnUiThread(new Runnable() {
                                            @Override
                                            public void run() {
                                                if (WriteFileUtil.isExternalStorageWritable()) {
                                                    String status = Environment.getExternalStorageState();
                                                    if (status.equals(Environment.MEDIA_MOUNTED)) {
                                                        File dir = new File(Environment.getExternalStorageDirectory().getAbsolutePath() + "/droneplus/");
                                                        if (!dir.exists()) {
                                                            dir.mkdir();
                                                        }
                                                        WriteFileUtil.putStringToExternalStorage(dX + "\r\n", dir, "cdX_3.txt", true);
                                                        WriteFileUtil.putStringToExternalStorage(dY + "\r\n", dir, "cdY_3.txt", true);
                                                        WriteFileUtil.putStringToExternalStorage(t_x + "\r\n", dir, "t_x_3.txt", true);
                                                        WriteFileUtil.putStringToExternalStorage(t_y + "\r\n", dir, "t_y_3.txt", true);
                                                        WriteFileUtil.putStringToExternalStorage(angle + "\r\n", dir, "angle_3.txt", true);
                                                        WriteFileUtil.putStringToExternalStorage(ultrasonicHeight + "\r\n", dir, "height_3.txt", true);
                                                        WriteFileUtil.putStringToExternalStorage(currentTime() + "\r\n", dir, "time_3.txt", true);
                                                        WriteFileUtil.putStringToExternalStorage(controlValueIncX + "\r\n", dir, "controlValueIncX_3.txt", true);
                                                        WriteFileUtil.putStringToExternalStorage(controlValueIncY + "\r\n", dir, "controlValueIncY_3.txt", true);
                                                    }
                                                }
                                            }
                                        });

                                        if ((Math.abs(t_x) <= 10 && Math.abs(t_y) <= 10)) {
                                            if (ultrasonicHeight >= 2.4) {
                                                if (Math.abs(t_x) <= 5 && Math.abs(t_y) <= 5) {
                                                    Log.i("dongislanding", "run 3.0m-2.4m: Math.abs(t_x) <= 7 && Math.abs(t_y) <= 7");
                                                    adjustTimes++;
                                                    if (adjustTimes >= 3)
                                                        flyControl(0, 0, 0, -0.7f);
                                                } else {
                                                    adjustTimes = 0;
                                                    flyControl(0, 0, 0, -0.2f);
                                                }
                                            } else if (ultrasonicHeight <= 2.5 && ultrasonicHeight >= 0.5) {
                                                Log.i("dongislanding", "run 2.5m-0.4m: Math.abs(t_x) <= 10 && Math.abs(t_y) <= 10");
                                                adjustTimes++;
                                                if (adjustTimes >= 2) {
                                                    flyControl(0, 0, 0, -0.5f);
                                                } else {
                                                    adjustTimes = 0;
                                                    flyControl(0, 0, 0, -0.15f);
                                                }
                                            }
                                        } else {
                                            if (ultrasonicHeight <= 3.2 && ultrasonicHeight > 2.0) { // 3m-2.0m调整一次

                                                Log.i("donglanding", "run - 3m-2.0m: t_x: " + t_x + "t_y: " + t_y);
                                                if ((Math.abs(t_x) <= 20 && Math.abs(t_y) <= 20)) {
                                                    Log.i("dongislanding", "run 3m-2.0m: Math.abs(t_x) <= 20 && Math.abs(t_y) <= 20");
                                                    if ((Math.abs(t_x) <= 9 && Math.abs(t_y) <= 9)) {
                                                        flyControl(0, 0, 0, -0.5f);
                                                    } else
                                                        flyControl((-1) * controlValueIncX, controlValueIncY, 0, -0.15f);
                                                } else {
                                                    Log.i("dongislanding", "run 3m-2.0m: Math.abs(t_x) > 20 && Math.abs(t_y) > 20");
                                                    flyControl((-1) * controlValueIncX, controlValueIncY, 0, -0.05f);
                                                }

                                            }

                                            if (ultrasonicHeight <= 2.2 && ultrasonicHeight > 1.0) { // 2.0m调整一次

                                                Log.i("donglanding", "run 2.0m : t_x: " + t_x + "t_y: " + t_y);
                                                if ((Math.abs(t_x) <= 17 && Math.abs(t_y) <= 17)) {
                                                    Log.i("dongislanding", "run 2.0m: Math.abs(t_x) <= 17 && Math.abs(t_y) <= 17");
                                                    adjustTimes++;
                                                    if (adjustTimes >= 2) {
                                                        if ((Math.abs(t_x) <= 12 && Math.abs(t_y) <= 12)) {
                                                            flyControl(0, 0, 0, -0.3f);
                                                        } else {
                                                            flyControl((-1) * controlValueIncX, controlValueIncY, 0, -0.1f);
                                                        }
                                                    }

                                                } else {
                                                    adjustTimes = 0;
                                                    flyControl((-1) * controlValueIncX, controlValueIncY, 0, -0.02f);
                                                }

                                            }

                                            if (ultrasonicHeight <= 1.2 && ultrasonicHeight > 0.3) { // 1.0m调整一次

                                                Log.i("donglanding", "run 1.0m : t_x: " + t_x + "t_y: " + t_y);
                                                if (Math.abs(t_x) <= 14.0 && Math.abs(t_y) <= 14.0) {

                                                    // 降落过程中，在0.9米处缩小误差范围
                                                    if (ultrasonicHeight < 0.9 && ultrasonicHeight > 0.6) {

                                                        if (Math.abs(t_x) <= 12.0 && Math.abs(t_y) <= 12.0) {
                                                            Log.i("dongislanding", "run " + "h" + ultrasonicHeight + "m: Math.abs(t_x) <= 12 && Math.abs(t_y) <= 12");
                                                            secondAdjustTimes++;
                                                            if (secondAdjustTimes >= 1)
                                                                flyControl(0, 0, 0, -0.4f);
                                                        } else {
                                                            Log.i("dongislanding", "run " + "h" + ultrasonicHeight + "m: Math.abs(t_x) > 12 || Math.abs(t_y) > 12");
                                                            secondAdjustTimes = 0;
                                                            flyControl((-1) * controlValueIncX, controlValueIncY, 0, -0.3f);
                                                        }

                                                    } else {

                                                        Log.i("dongislanding", "run " + "m: Math.abs(t_x) <= 15 && Math.abs(t_y) <= 15");
                                                        adjustTimes++;
                                                        if (adjustTimes >= 2)
                                                            flyControl(0, 0, 0, -0.2f);
                                                    }

                                                } else {
                                                    Log.i("dongislanding", "run " + "m: Math.abs(t_x) > 15 && Math.abs(t_y) > 15");
                                                    adjustTimes = 0;
                                                    flyControl((-1) * controlValueIncX, controlValueIncY, 0, 0);
                                                }
                                            }
                                        }
                                    }
                                } else {
                                    Log.i("dongnotfindmark", "else   results != null : results.size(): " + results.size());
                                    firstaprilinit = false;
                                    secondaprilinit = false;

                                    if ((ultrasonicHeight > 3.0 && notfirstRec10) || (ultrasonicHeight < 3.0 && notfirstRec0)) {
                                        // 无人机已经无法获取到mark位置, 控制无人机朝最后一次记录的位置移动

                                        //                                    if (notfirstRec0 && results.size() < 3) {
                                        if (ultrasonicHeight > 0.4) {

                                            // Xy轴方向，方框中心与屏幕中心的距离差
                                            dX = (float) last_tagCenterX;
                                            dY = (float) last_tagCenterY;

                                            Log.i("dongnotfindmark", "dX : " + dX + " dY: " + dY);
                                            Log.i("dongnotfindmark", "size==0 run: last_t_x: " + last_t_x + " last_t_y: " + last_t_y);

                                            // 计算xy轴方向的控制量

                                            if (ultrasonicHeight > 7.7) {
                                                if (Math.abs(last_t_x) >= errX) {
                                                    controlValueIncX = (float) (dX * oneSpeedPara);
                                                }
                                                if (Math.abs(last_t_y) >= errY) {
                                                    controlValueIncY = (float) (dY * oneSpeedPara);
                                                }
                                            }
                                            if (ultrasonicHeight <= 7.7 && ultrasonicHeight > 5.2) {
                                                if (Math.abs(last_t_x) >= errX) {
                                                    controlValueIncX = (float) (dX * twoSpeedPara);
                                                }
                                                if (Math.abs(last_t_y) >= errY) {
                                                    controlValueIncY = (float) (dY * twoSpeedPara);
                                                }
                                            }
                                            if (ultrasonicHeight <= 5.2 && ultrasonicHeight > 3.0) {
                                                if (Math.abs(last_t_x) < 500) {
                                                    dX = dX * 3;
                                                }
                                                if (Math.abs(last_t_y) < 500) {
                                                    dY = dY * 3;
                                                }
                                                if (Math.abs(last_t_x) >= errX) {
                                                    controlValueIncX = (float) (dX * threeSpeedPara);
                                                }
                                                if (Math.abs(last_t_y) >= errY) {
                                                    controlValueIncY = (float) (dY * threeSpeedPara);
                                                }
                                            }

                                            if (ultrasonicHeight < 3.2) {
                                                if (Math.abs(last_t_x) >= errX - 6) {
                                                    controlValueIncX = dX * fourSpeedPara;
                                                }
                                                if (Math.abs(last_t_y) >= errY - 6) {
                                                    controlValueIncY = dY * fourSpeedPara;
                                                }
                                            }

                                            Log.i("dongnotfindmark", "size==0 run: controlValueIncX: " + controlValueIncX + " controlValueIncY: " + controlValueIncY);

                                            if (dX > 1500 || dY > 1500) {
                                                // 相对于标签，相机所在位置
                                                if (last_t_x < 0 && last_t_y > 0) {
                                                    if (ultrasonicHeight < 2.0) {
                                                        flyControl(0, 0, 0, 0.3f);
                                                    } else {
                                                        flyControl((-1) * controlValueIncX, controlValueIncY, 0, 0.2f);
                                                    }
                                                }
                                            } else {
                                                // 相对于标签，相机所在位置
                                                if (last_t_x < 0 && last_t_y > 0) {
                                                    if (ultrasonicHeight > 0 && ultrasonicHeight < 2.0) {
                                                        flyControl((-1) * controlValueIncX, controlValueIncY, 0, 0.2f);
                                                    } else {
                                                        flyControl((-1) * controlValueIncX, controlValueIncY, 0, 0);
                                                    }
                                                }
                                            }
                                        }
                                    }
                                } // else 没有识别结果  results.size() == 0

                            }
                            // run() 执行完一次初始化
                            Log.i("run()init", " run() 执行完一次初始化");
                            recResult.clear();

                            Log.i("dongpredict_image", "run: landing: " + (System.currentTimeMillis() - start1) + "ms");

                            // notfirstRec0 = false;
                            // notfirstRec10 = false;
                        } else {
                            firstaprilinit = false;
                            secondaprilinit = false;
                            notfirstRec0 = false;
                            notfirstRec10 = false;
                            flyControl(0, 0, 0, -0.4f);
                            BaseProduct product = DJISDKManager.getInstance().getProduct();
                            if (product != null && product.getModel() != null) {
                                product.getName(new CommonCallbacks.CompletionCallbackWith<String>() {
                                    @Override
                                    public void onSuccess(String s) {
                                        Log.i("product.getName", "onSuccess: " + s);
                                    }

                                    @Override
                                    public void onFailure(DJIError djiError) {

                                    }
                                });
                                switch (product.getModel()) {
                                    case MAVIC_2:
                                    case MAVIC_2_PRO:
                                    case MAVIC_2_ZOOM:
                                        Log.i("dongstartLanding", "run: MAVIC_2_ZOOM");
                                        mFlightController.startLanding(new CommonCallbacks.CompletionCallback() {
                                            @Override
                                            public void onResult(DJIError djiError) {
                                                Log.i("dongstartLanding", "djiError: ");
                                            }
                                        });
                                        break;
                                    case PHANTOM_4:
                                    case PHANTOM_4_PRO: // p4p
                                        Log.i("dongstartLanding", "run: PHANTOM_4_PRO");
                                        flyControl(0, 0, 0, -0.3f);
                                        break;
                                    default:
                                        Log.i("dongstartLanding", "run: default");
                                        flyControl(0, 0, 0, -0.3f);
                                        break;
                                }
                            }
                        } //else 降落
                        // 若无人机螺旋桨的电机已停止转动
                        //                        if (!mFlightController.getState().areMotorsOn()) {
                        //                            runOnUiThread(new Runnable() {
                        //                                @Override
                        //                                public void run() {
                        //                                    setResultToToast("已安全降落!");
                        //                                    if (tagThread != null && tagThread.isAlive()) {
                        //                                        tagThread.interrupt();
                        //                                        tagFlag = false;
                        //                                        isDetection = false;
                        //                                        recognizerForAprilTags.close();
                        //                                    }
                        //                                    detectIv.setImageResource(R.mipmap.detect_close_u);
                        //                                    // 关闭虚拟摇杆
                        //                                    if (mFlightController != null) {
                        //                                        mFlightController.setVirtualStickModeEnabled(false, new CommonCallbacks.CompletionCallback() {
                        //                                            @Override
                        //                                            public void onResult(DJIError djiError) {
                        //                                                if (djiError != null) {
                        //                                                    setResultToToast(djiError.getDescription());
                        //                                                } else {
                        //                                                    isSimulator = false;
                        //                                                    if (tagFlag) {
                        //                                                        tagFlag = false;
                        //                                                        isDetection = false;
                        //                                                        tagThread.interrupt();
                        //                                                    }
                        //
                        //                                                    runOnUiThread(new Runnable() {
                        //                                                        @Override
                        //                                                        public void run() {
                        //                                                            setResultToToast("若无人机螺旋桨的电机已停止转动, 虚拟摇杆关闭");
                        //                                                            disableSimulatorIv.setImageResource(R.mipmap.rc_mode);
                        //                                                        }
                        //                                                    });
                        //                                                }
                        //                                            }
                        //                                        });
                        //                                    }
                        //                                }
                        //                            });
                        //                        }
                        Log.i("dongpredict_image", "run: programdone: " + (System.currentTimeMillis() - start) + "ms");
                    }
                }
            });
        }
    };

    /**
     * 无人机调整降落 3米以上 调整
     *
     * @param t_x
     * @param t_y
     * @param error_t_x
     * @param error_t_y
     * @param isLanding
     * @param controlValueIncX
     * @param controlValueIncY
     * @param controlValueIncTurn
     * @param landingSpeed1
     * @param landingSpeed2
     * @param landingSpeed3
     */
    public void droneAdjustment(double t_x, double t_y, double error_t_x, double error_t_y, boolean isLanding, float controlValueIncX, float controlValueIncY, float controlValueIncTurn, float landingSpeed1, float landingSpeed2, float landingSpeed3) {

        if ((Math.abs(t_x) <= error_t_x && Math.abs(t_y) <= error_t_y) && !isLanding) {
            flyControl((-1) * controlValueIncX, controlValueIncY, controlValueIncTurn, landingSpeed1);
        } else if ((Math.abs(t_x) > error_t_x || Math.abs(t_y) > error_t_y) && !isLanding) {
            adjustTimes = 0;
            flyControl((-1) * controlValueIncX, controlValueIncY, controlValueIncTurn, landingSpeed2);
        }
        if ((Math.abs(t_x) < error_t_x - 1 && Math.abs(t_y) < error_t_y - 1) && !isLanding) {
            adjustTimes++;
            if (adjustTimes >= 2) {
                isLanding = true;
                adjustTimes = 0;
            }
        }
        if (isLanding) {
            Log.i("dongtag10isLanding", "run: " + isLanding);
            if (ultrasonicHeight >= 0) {
                flyControl(0, 0, 0, landingSpeed3);
            }
        }

    }

    /**
     * 无人机降落 3米以上 降落
     *
     * @param t_x
     * @param t_y
     * @param error_t_x
     * @param error_t_y
     * @param controlValueIncX
     * @param controlValueIncY
     * @param landingSpeed
     * @param adjustSpeed
     */
    private void droneReadyLanding(double t_x, double t_y, double error_t_x, double error_t_y, float controlValueIncX, float controlValueIncY, float landingSpeed, float adjustSpeed) {

        Log.i("donglanding", "run " + ultrasonicHeight + "m : t_x: " + t_x + " t_y: " + t_y);
        if (Math.abs(t_x) <= error_t_x && Math.abs(t_y) <= error_t_y) {
            adjustTimes++;
            if (adjustTimes >= 2)
                flyControl((-1) * controlValueIncX, controlValueIncY, 0, landingSpeed);
        } else {
            adjustTimes = 0;
            flyControl((-1) * controlValueIncX, controlValueIncY, 0, adjustSpeed);
        }
    }

    /**
     * -
     * @param view
     */
    public void paraD(View view) {
        fourSpeedPara -= 0.000005;
        speedParaTv.setText(Float.toString(fourSpeedPara));
    }

    /**
     * +
     * @param view
     */
    public void paraA(View view) {
        fourSpeedPara += 0.000005;
        speedParaTv.setText(Float.toString(fourSpeedPara));

    }

    /**
     * 虚拟摇杆功能，可通过程序控制无人机的前、后、左、右、上升、下降的飞行动作
     * <p>
     * leftRight     正数为右，负数为左
     * frontBack     正数为前，负数为后
     * turnLeftRight
     * upDown        正数为上升，负数为下降
     */
    private class SendVirtualStickDataTask extends TimerTask {

        @Override
        public void run() {
            mFlightController.sendVirtualStickFlightControlData(new FlightControlData(leftRight, frontBack, turnLeftRight, upDown), new CommonCallbacks.CompletionCallback() {
                @Override
                public void onResult(DJIError djiError) {

                }
            });
        }
    }

    /**
     * 虚拟摇杆功能，可通过程序控制无人机的前、后、左、右、上升、下降的飞行动作
     *
     * @param leftRight     正数为右，负数为左
     * @param frontBack     正数为前，负数为后
     * @param turnLeftRight
     * @param upDown        正数为上升，负数为下降
     */
    private void flyControl(float leftRight, float frontBack, float turnLeftRight, float upDown) {
        if (mFlightController == null) {
            BaseProduct product = DemoApplication.getProductInstance();
            if (product == null || !product.isConnected()) {
                setResultToToast("未连接到无人机！");
                missionControl = null;
            } else {
                missionControl = MissionControl.getInstance(); // 任务控制器实例
                if (product instanceof Aircraft) {
                    Aircraft aircraft = (Aircraft) DJISDKManager.getInstance().getProduct();
                    mFlightController = aircraft.getFlightController();
                }
            }
        }

        mFlightController.sendVirtualStickFlightControlData(new FlightControlData(leftRight, frontBack, turnLeftRight, upDown), new CommonCallbacks.CompletionCallback() {
            @Override
            public void onResult(DJIError djiError) {
                if (djiError != null) {
                    setResultToToast("flyControl_DJIError: " + djiError.getDescription());
                }
            }
        });
    }

    private String currentTime() {
        SimpleDateFormat sdf = new SimpleDateFormat("HH:mm:ss");
        Date curDate = new Date(System.currentTimeMillis());
        return sdf.format(curDate);
    }

    @Override
    protected void onCreate(Bundle savedInstanceState) {
//                        loginAccount();
        super.onCreate(savedInstanceState);
        // When the compile and target version is higher than 22, please request the
        // following permissions at runtime to ensure the
        // SDK work well.
        if (Build.VERSION.SDK_INT >= Build.VERSION_CODES.M) {
            ActivityCompat.requestPermissions(this, new String[]{Manifest.permission.WRITE_EXTERNAL_STORAGE, Manifest.permission.VIBRATE, Manifest.permission.INTERNET, Manifest.permission.ACCESS_WIFI_STATE, Manifest.permission.WAKE_LOCK, Manifest.permission.ACCESS_COARSE_LOCATION, Manifest.permission.ACCESS_NETWORK_STATE, Manifest.permission.ACCESS_FINE_LOCATION, Manifest.permission.CHANGE_WIFI_STATE, Manifest.permission.MOUNT_UNMOUNT_FILESYSTEMS, Manifest.permission.READ_EXTERNAL_STORAGE, Manifest.permission.SYSTEM_ALERT_WINDOW, Manifest.permission.READ_PHONE_STATE,}, 1);
        }
        setContentView(R.layout.activity_main);

        if (!OpenCVLoader.initDebug()) {
            Log.e(this.getClass().getSimpleName(), "  OpenCVLoader.initDebug(), not working.");
            Log.i("dongOpenCV", "onCreate: OpenCVLoader.initDebug(), not working.");
        } else {
            Log.i("dongOpenCV", "onCreate: OpenCVLoader.initDebug(), working.");
            Log.d(this.getClass().getSimpleName(), "  OpenCVLoader.initDebug(), working.");
        }

        initUi(savedInstanceState);
    }

    private void initUi(Bundle savedInstanceState) {

        // 获取屏幕大小
        WindowManager manager = (WindowManager) this.getSystemService(Context.WINDOW_SERVICE);
        Display display = manager.getDefaultDisplay();
        Point point = new Point();
        if (Build.VERSION.SDK_INT < 17) {
            display.getSize(point);
        } else {
            display.getRealSize(point);
        }
        width = point.x;
        height = point.y;
        Log.i("dongsc", "initUi: width:" + width + " height:" + height);

        // width: 1088 height: 720

        errXprecise = (float) (width * 0.012); // width = 2560时，errX = 30.72
        errYprecise = (float) (height * 0.02); // height = 1600时，errY = 32

        screenX = width / 2;
        screenY = height / 2;

        brand = android.os.Build.BRAND;
        Log.e("dongbrand", "手机厂商：" + brand);

//        if (brand == "HUAWEI") {
//            viewWidth = 2418;
//            viewHeight = 1600;
////            Toast.makeText(this, brand, Toast.LENGTH_LONG).show();
//        } else {
//            viewWidth = 2176;
//            viewHeight = 1440;
//        }

        gpsTransformUtil = new GPSTransformUtil();

        mapContainer = (RelativeLayout) findViewById(R.id.map_container);
        dcontainer = (RelativeLayout) findViewById(R.id.video_container);
        cover = new RelativeLayout(this);

        mView = LayoutInflater.from(this).inflate(R.layout.mapview, null);
        vview = LayoutInflater.from(this).inflate(R.layout.vidview, null);

        //        fpvWidget = findViewById(R.id.video);
        fpvWidget = vview.findViewById(R.id.video);
        mapView = mView.findViewById(R.id.map);
        mapView.onCreate(savedInstanceState);

        mapContainer.addView(mView, new LinearLayout.LayoutParams(ViewGroup.LayoutParams.MATCH_PARENT, ViewGroup.LayoutParams.MATCH_PARENT));
        mapContainer.addView(cover, new LinearLayout.LayoutParams(ViewGroup.LayoutParams.MATCH_PARENT, ViewGroup.LayoutParams.MATCH_PARENT));
//        dcontainer.addView(vview, new LinearLayout.LayoutParams(ViewGroup.LayoutParams.MATCH_PARENT, ViewGroup.LayoutParams.MATCH_PARENT))

        dcontainer.addView(vview, new LinearLayout.LayoutParams(viewWidth, viewHeight));   // 2176 1440 三星   2418 1600 华为
        cover.setOnClickListener(new View.OnClickListener() {
            @Override
            public void onClick(View v) {
                if (!isMap) {
                    uiSettings = aMap.getUiSettings();
                    uiSettings.setZoomControlsEnabled(true);
                    uiSettings.setZoomInByScreenCenter(true);
                    uiSettings.setAllGesturesEnabled(true);
                    uiSettings.setCompassEnabled(true);
                    mapView.setClickable(true);

                    mapContainer.removeAllViews();
                    dcontainer.removeAllViews();
                    dcontainer.addView(mView, new LinearLayout.LayoutParams(ViewGroup.LayoutParams.MATCH_PARENT, ViewGroup.LayoutParams.MATCH_PARENT));
                    mapContainer.addView(fpvWidget);
                    mapContainer.addView(cover);
                    isMap = true;
                } else {
                    uiSettings = aMap.getUiSettings();
                    uiSettings.setZoomControlsEnabled(false);
                    uiSettings.setZoomInByScreenCenter(true);
                    uiSettings.setAllGesturesEnabled(true);
                    uiSettings.setCompassEnabled(true);
                    mapView.setClickable(true);

                    mapContainer.removeAllViews();
                    dcontainer.removeAllViews();
//                    dcontainer.addView(vview, new LinearLayout.LayoutParams(ViewGroup.LayoutParams.MATCH_PARENT, ViewGroup.LayoutParams.MATCH_PARENT));
                    dcontainer.addView(vview, new LinearLayout.LayoutParams(viewWidth, viewHeight));
                    mapContainer.addView(mapView);
                    mapContainer.addView(cover);
                    isMap = false;
                }
            }
        });

        atsrell = findViewById(R.id.at_s_re_ll);
        atinfoll = findViewById(R.id.at_info_ll);
        speedparall = findViewById(R.id.llspeedpara);
        //apriltags详情 抽屉
        mPushDrawerIb = (ImageButton) findViewById(R.id.tracking_drawer_control_ib);
        mPushInfoSd = (SlidingDrawer) findViewById(R.id.tracking_drawer_sd);

        ltPushInfoTv = (TextView) findViewById(R.id.lt_tracking_push_tv);
        lbPushInfoTv = (TextView) findViewById(R.id.lb_tracking_push_tv);
        rtPushInfoTv = (TextView) findViewById(R.id.rt_tracking_push_tv);
        rbPushInfoTv = (TextView) findViewById(R.id.rb_tracking_push_tv);
        cPushInfoTv = (TextView) findViewById(R.id.c_tracking_push_tv);
        bPushInfoTv = (TextView) findViewById(R.id.b_tracking_push_tv);

        bigangletv = (TextView) findViewById(R.id.bigangle_tv);
        ltangletv = (TextView) findViewById(R.id.ltangle_tv);
        rtangletv = (TextView) findViewById(R.id.rtangle_tv);
        lbangletv = (TextView) findViewById(R.id.lbangle_tv);
        rbangletv = (TextView) findViewById(R.id.rbangle_tv);
        cangletv = (TextView) findViewById(R.id.cangle_tv);
        caliangletv = findViewById(R.id.caliangle_tv);

        speedParaTv = (TextView) findViewById(R.id.speedPara_tv);
        speedParaTv.setText(Float.toString(fourSpeedPara));

        speedParaIv = findViewById(R.id.speedPara_iv);
        speedParaIv.setOnClickListener(this);

        timetv = (TextView) findViewById(R.id.time_tv);

        detectIv = (ImageView) findViewById(R.id.detect_button);
        disableSimulatorIv = (ImageView) findViewById(R.id.simulator_stop_iv);

        ultrasonicTv = (TextView) findViewById(R.id.ultrasonic_height);

        yawTv = (TextView) findViewById(R.id.drone_yaw_tv);
        pitchTv = (TextView) findViewById(R.id.drone_pitch_tv);

        //        turnBtn = (Button) findViewById(R.id.turn_btn);
        //        turnBtn.setOnClickListener(this);

        detectIv.setOnTouchListener(this);
        disableSimulatorIv.setOnClickListener(this);
        mPushDrawerIb.setOnClickListener(this);

        timelinell = findViewById(R.id.timeline_ll);

        setWpIv = findViewById(R.id.check_wp_button);
        wpInitIv = findViewById(R.id.wp_init_button);
        wpStartIv = findViewById(R.id.wp_start_button);
        wpStopIv = findViewById(R.id.wp_stop_button);
        wpPauseIv = findViewById(R.id.wp_pause_button);
        wpResumeIv = findViewById(R.id.wp_resume_button);
        wpCleanIv = findViewById(R.id.wp_clean_button);

        curTlInfoLl = findViewById(R.id.cur_tl_info_ll);
        curHeiText = findViewById(R.id.cur_hei_text);
        curYawText = findViewById(R.id.cur_yaw_text);
        curGPitchText = findViewById(R.id.cur_g_pitch_text);

//        tlSettingLl = findViewById(R.id.tl_setting_ll);
//        settingHLl = findViewById(R.id.setting_h_ll);
//        settingAttLl = findViewById(R.id.setting_att_ll);
//        heiTextTv = findViewById(R.id.hei_text);
//        droneYawTv = findViewById(R.id.yaw_text);
//        gimbalPitchTv = findViewById(R.id.g_pitch_text);
        tlWpBtn = findViewById(R.id.tl_wp_btn);
//        wpSettingBtn = findViewById(R.id.wp_setting_btn);
//        wpCancelBtn = findViewById(R.id.wp_cancel_btn);

        wpTlRl = findViewById(R.id.wp_tl_rl);

        heig_M = findViewById(R.id.height_M);
        heig_text = (EditText) findViewById(R.id.height_text);
        heig_P = findViewById(R.id.height_P);

        hot_set = findViewById(R.id.is_hot_set);

        R_is_hot = findViewById(R.id.is_hot);
        R_no_hot = findViewById(R.id.no_hot);

        ishot_layout = findViewById(R.id.is_add_hot);
        hot_M = findViewById(R.id.hot_M);
        radius_text = (EditText) findViewById(R.id.radius_text);
        hot_P = findViewById(R.id.hot_P);

        hot_h_M = findViewById(R.id.hot_h_M);
        hot_h_text = (EditText) findViewById(R.id.hot_h_text);
        hot_h_P = findViewById(R.id.hot_h_P);

        R_photo = findViewById(R.id.photo);
        R_video_yes = findViewById(R.id.video_yes);
        R_video_no = findViewById(R.id.video_no);
        R_no = findViewById(R.id.no);

        camera_type = findViewById(R.id.camera_type);

        camera_M = findViewById(R.id.camera_M);
        camera_text = (EditText) findViewById(R.id.camera_text);
        camera_P = findViewById(R.id.camera_P);

        flight_M = findViewById(R.id.flight_M);
        flight_text = (EditText) findViewById(R.id.flight_text);
        flight_P = findViewById(R.id.flight_P);

        confirmbtn = findViewById(R.id.btn_confirm);
        cancelbtn = findViewById(R.id.btn_cancel);

        heig_M.setOnClickListener(this);
        heig_P.setOnClickListener(this);
        hot_h_M.setOnClickListener(this);
        hot_h_P.setOnClickListener(this);
        hot_set.setOnCheckedChangeListener(this);
        hot_M.setOnClickListener(this);
        hot_P.setOnClickListener(this);
        camera_type.setOnCheckedChangeListener(this);
        camera_M.setOnClickListener(this);
        camera_P.setOnClickListener(this);
        flight_M.setOnClickListener(this);
        flight_P.setOnClickListener(this);
        confirmbtn.setOnClickListener(this);
        cancelbtn.setOnClickListener(this);

        setWpIv.setOnClickListener(this);
        wpInitIv.setOnClickListener(this);
        wpStartIv.setOnClickListener(this);
        wpStopIv.setOnClickListener(this);
        wpPauseIv.setOnClickListener(this);
        wpResumeIv.setOnClickListener(this);
        wpCleanIv.setOnClickListener(this);
        tlWpBtn.setOnClickListener(this);
//        wpSettingBtn.setOnClickListener(this);
//        wpCancelBtn.setOnClickListener(this);

        if (null != fpvWidget) {
            fpvWidget.setSurfaceTextureListener(this);
        }


        // 注册无人机监听广播
        initFlightController();
        IntentFilter filter = new IntentFilter();
        filter.addAction(DemoApplication.FLAG_CONNECTION_CHANGE);
        registerReceiver(mReceiver, filter);

        resultList = (RecyclerView) findViewById(R.id.recognition_result_list);
        resultData();
        resultList.setLayoutManager(new LinearLayoutManager(this, LinearLayoutManager.VERTICAL, false));
        adapter = new RecognitionResultAdapter(resultItemArrayList);
        resultList.setAdapter(adapter);
        resultList.setOnTouchListener(this);

        initMapView(); // 初始化地图

        // 10秒后设置监听（临时
        Timer t = new Timer();
        t.schedule(new TimerTask() {
            @Override
            public void run() {
                //                addListener();
                updateDroneLocation();
                Log.i("dongaddListener", "run: ");
            }
        }, 10000);


    }

    /**
     * 初始化地图
     */
    private void initMapView() {
        LatLng softwarePark = new LatLng(36.667094, 117.140428); // 软件园校区经纬度
        cameraPosition = new CameraPosition.Builder().target(softwarePark).zoom(18).tilt(70).build();
        if (aMap == null) {
            aMap = mapView.getMap();
            uiSettings = aMap.getUiSettings();
            uiSettings.setZoomControlsEnabled(false);
            uiSettings.setZoomInByScreenCenter(true);
            uiSettings.setAllGesturesEnabled(true);
            uiSettings.setCompassEnabled(true);
            mapView.setClickable(true);

            aMap.setOnMapClickListener(this);// add the listener for click for amap object

            aMap.setOnMarkerClickListener(markerClickListener);
        }
        aMap.moveCamera(CameraUpdateFactory.newCameraPosition(cameraPosition));
//        aMap.addMarker(new MarkerOptions().position(softwarePark).title("Marker in softwarePark"));
        polylineOptions = new PolylineOptions().width(15).geodesic(true).color(Color.GREEN);
    }

    @Override
    public void onMapClick(LatLng point) {
        if (isAdd == true) {
//            markWaypoint(point);
            LatLng DJIPoint2 = getDJILatLng(point.latitude, point.longitude);
            Log.i("dongwapoint", "onMapClick: waypointgetLatitude: " + DJIPoint2.latitude + " waypointgetLongitude: " + DJIPoint2.longitude);

            Waypoint mWaypoint = new Waypoint(point.latitude, point.longitude, altitude);
//            waypointList_GD.add(mWaypoint);

//            TimeLineAction timeLineAction = new TimeLineAction(mWaypoint, false, 5, 4, 0);
//            timeLineActionList.add(timeLineAction);
//
//            heightList.add(50);
//            hotpointList.add(false);
//            radiusList.add(5);
//            actionList.add(4);
//            angleList.add(0);
        } else {
//            setResultToToast("Cannot Add Waypoint");
        }
    }

    private void markWaypoint(LatLng point) {
        marker_GD_GPS.add(point);

        // 未修改航点信息时使用当前无人机的信息
        TimeLineAction timeLineAction = new TimeLineAction(new Waypoint(point.latitude, point.longitude, droneLocationHeight), false, 5, 4, 4, (int) gimbalPitch, (int) yaw);
        timeLineActionList.add(timeLineAction);

        heightList.add((int) droneLocationHeight);
        hotpointList.add(false);
        radiusList.add(5);
        hotheightList.add(5);
        actionList.add(4);
        gimbalangleList.add((int) gimbalPitch);
        yawangleList.add((int) yaw);

        Waypoint mWaypoint = new Waypoint(point.latitude, point.longitude, droneLocationHeight);
        waypointList_GD.add(mWaypoint);

        //Create MarkerOptions object
        MarkerOptions markerOptions = new MarkerOptions();
        markerOptions.position(point);
//        markerOptions.title("lat:" + point.latitude + "\n" + "lon: " + point.longitude);
//        markerOptions.icon(BitmapDescriptorFactory.defaultMarker(BitmapDescriptorFactory.HUE_MAGENTA));

        markerOptions.icon(BitmapDescriptorFactory.fromView(new WaypointView(this, markerNum + 1)));
        markerNum++;

//        markerOptions.draggable(true);//设置Marker可拖动

        Marker marker = aMap.addMarker(markerOptions);
        mMarkers.put(mMarkers.size(), marker);

        Log.i("dongmarkerNum", "markWaypoint: mMarkers.size() : " + mMarkers.size());

//        marker

//        marker.getObject();

//        Waypoint mWaypoint = new Waypoint(marker.getPosition().latitude, marker.getPosition().longitude, 40);
//        Waypoint mWaypoint = new Waypoint(point.latitude, point.longitude, 30);
//        waypointList_GD.add(mWaypoint);
    }

    private void deleteMarkWaypoint(LatLng point) {

    }

    @Override
    public void onInfoWindowClick(Marker marker) {

    }


    AMap.OnMarkerClickListener markerClickListener = new AMap.OnMarkerClickListener() {
        @Override
        public boolean onMarkerClick(final Marker marker) {
            String number = marker.getId().substring(6);
            int position = 0;
            Log.i("dongmc", "onMarkerClick: " + marker.getId());
            if (!number.equals("") && number != null) {
                position = Integer.parseInt(number) - 1;
                Log.i("dongmc", "onMarkerClick: " + marker.getId().length() + " position: " + position);
            }
            final int finalPosition = position;

            Log.i("dongmapclick", "onMarkerClick: " + marker.getPosition().latitude + " lon: " + marker.getPosition().longitude);


            return false;
        }
    };


    // 监听与无人机连接状态的广播
    protected BroadcastReceiver mReceiver = new BroadcastReceiver() {

        @Override
        public void onReceive(Context context, Intent intent) {
            onProductConnectionChange();
        }
    };

    private void onProductConnectionChange() {
        initFlightController();
        initPreviewer();
    }

    /**
     * 初始化无人机控制，并获取无人机位置、各种状态
     */
    private void initFlightController() {
        BaseProduct product = DemoApplication.getProductInstance();
        if (product == null || !product.isConnected()) {
            setResultToToast("未连接到无人机！");
            missionControl = null;
        } else {
            missionControl = MissionControl.getInstance(); // 任务控制器实例
            if (product instanceof Aircraft) {
                //                mFlightController = ((Aircraft) product).getFlightController();
                Aircraft aircraft = (Aircraft) DJISDKManager.getInstance().getProduct();
                mFlightController = aircraft.getFlightController();
                gimbal = product.getGimbal();
                battery = product.getBattery();
                camera = product.getCamera();
            }
        }

        // 云台
        if (gimbal != null) {
            gimbal.setStateCallback(new GimbalState.Callback() {
                @Override
                public void onUpdate(@NonNull final GimbalState gimbalState) {
                    new Handler(Looper.getMainLooper()).post(new Runnable() {
                        @Override
                        public void run() {
                            gimbalYaw = gimbalState.getAttitudeInDegrees().getYaw();
                            gimbalPitch = gimbalState.getAttitudeInDegrees().getPitch();
                            runOnUiThread(new Runnable() {
                                @Override
                                public void run() {
                                    pitchTv.setText("gimbalPitch: " + gimbalPitch + "°");
                                }
                            });
                        }
                    });
                }
            });
        }
        // 相机
        if (camera != null) {
            camera.setSystemStateCallback(new SystemState.Callback() {
                @Override
                public void onUpdate(@NonNull SystemState systemState) {
                    if (systemState.isShootingSinglePhoto() && !isShot) {
                        imageCount++;
                        Log.i("Timeline", "imageCount:" + imageCount);
                        isShot = true;
                    } else if (!systemState.isShootingSinglePhoto() && isShot) {
                        Log.i("Timeline", "Shooting Single Photo is End.imageCount:" + imageCount);
                        isShot = false;
                    }
                    if (systemState.isRecording() && !isRecord) {
                        videoStartTime = System.currentTimeMillis();
                        Log.i("Timeline", "videoTimeStart:" + videoStartTime);
                        isRecord = true;
                    } else if (!systemState.isRecording() && isRecord) {
                        videoEndTime = System.currentTimeMillis();
                        videoLength += (int) (videoEndTime - videoStartTime);
                        Log.i("Timeline", "videoTimeEnd:" + videoEndTime);
                        Log.i("Timeline", "videoLength:" + videoLength);
                        isRecord = false;
                    }
                }
            });

            camera.getLensInformation(new CommonCallbacks.CompletionCallbackWith<String>() {
                @Override
                public void onSuccess(String s) {
                    Log.i("donggetLensInformation", "onSuccess: " + s);
                }

                @Override
                public void onFailure(DJIError djiError) {
                    Log.i("donggetLensInformation", "onFailure: ");
                }
            });
        }
        // 电池
        if (battery != null) {
            battery.setStateCallback(new BatteryState.Callback() {
                @Override
                public void onUpdate(BatteryState batteryState) {
                    batteryRemianing = batteryState.getChargeRemaining();
                }
            });
        }

        // 无人机
        if (mFlightController != null) {

            compass = mFlightController.getCompass();
            // 指南针
            Log.i("dongcompass", "initFlightController: " + compass.isCalibrating());

            compass.setCalibrationStateCallback(new CompassCalibrationState.Callback() {
                @Override
                public void onUpdate(@NonNull CompassCalibrationState compassCalibrationState) {

                }
            });

            mFlightController.setRollPitchControlMode(RollPitchControlMode.VELOCITY);
            mFlightController.setYawControlMode(YawControlMode.ANGULAR_VELOCITY);
            //            mFlightController.setYawControlMode(YawControlMode.ANGLE);
            mFlightController.setVerticalControlMode(VerticalControlMode.VELOCITY);
            mFlightController.setRollPitchCoordinateSystem(FlightCoordinateSystem.BODY);
            mFlightController.getSimulator().setStateCallback(new SimulatorState.Callback() {
                @Override
                public void onUpdate(final SimulatorState stateData) {
                    new Handler(Looper.getMainLooper()).post(new Runnable() {
                        @Override
                        public void run() {

                            String yaw = String.format("%.2f", stateData.getYaw());
                            String pitch = String.format("%.2f", stateData.getPitch());
                            String roll = String.format("%.2f", stateData.getRoll());
                            String positionX = String.format("%.2f", stateData.getPositionX());
                            String positionY = String.format("%.2f", stateData.getPositionY());
                            String positionZ = String.format("%.2f", stateData.getPositionZ());
                            Log.i("Timeline", "simulator yaw:" + yaw + " |pitch:" + pitch + " |roll" + roll + " |positionX:" + positionX + " |positionY" + positionY + " |positionZ:" + positionZ);
                        }
                    });
                }
            });
            mFlightController.setStateCallback(new FlightControllerState.Callback() {
                @Override
                public void onUpdate(FlightControllerState djiFlightControllerCurrentState) {
                    sourceLatLng = new LatLng(djiFlightControllerCurrentState.getAircraftLocation().getLatitude(), djiFlightControllerCurrentState.getAircraftLocation().getLongitude());
                    aircraftHeight = djiFlightControllerCurrentState.getAircraftLocation().getAltitude();
                    ultrasonicHeight = djiFlightControllerCurrentState.getUltrasonicHeightInMeters();
                    runOnUiThread(new Runnable() {
                        @Override
                        public void run() {
                            ultrasonicTv.setText("ultrasonicHeight:" + ultrasonicHeight + " m");
                        }
                    });
                    if (maxAltitude < aircraftHeight) {
                        maxAltitude = aircraftHeight;
                    }
                    Log.i("aircraftHeight", "aircraftHeight:" + aircraftHeight);
                    flightMode = djiFlightControllerCurrentState.getFlightMode();
                    roll = djiFlightControllerCurrentState.getAttitude().roll;
                    yaw = djiFlightControllerCurrentState.getAttitude().yaw;        // 偏航角
//                    runOnUiThread(new Runnable() {
//                        @Override
//                        public void run() {
//                            yawTv.setText("yaw: " + yaw + "°");
//                        }
//                    });
                    pitch = djiFlightControllerCurrentState.getAttitude().pitch;
                    satelliteCount = djiFlightControllerCurrentState.getSatelliteCount();
                    velocityX = djiFlightControllerCurrentState.getVelocityX();
                    velocityY = djiFlightControllerCurrentState.getVelocityY();
                    velocityZ = djiFlightControllerCurrentState.getVelocityZ();
                    flightTime = djiFlightControllerCurrentState.getFlightTimeInSeconds();
                    Log.i("dongyaw", "yaw: " + yaw);

                    Log.i("dongupdateDroneState", "1   onUpdate: droneGDLocationLat: " + sourceLatLng.latitude + " droneGDLocationLat: " + sourceLatLng.longitude);
                    CoordinateConverter converter = new CoordinateConverter(getApplicationContext());
                    converter.from(CoordinateConverter.CoordType.GPS);
                    // sourceLatLng 待转换坐标点 LatLng类型
                    converter.coord(sourceLatLng);
                    // 执行转换操作，转换为高德坐标
                    LatLng desLatLng = converter.convert();
                    droneGDLocationLat = desLatLng.latitude;
                    droneGDLocationLng = desLatLng.longitude;
                    Log.i("dongTimeline", "2   onUpdate: droneGDLocationLat: " + droneGDLocationLat + " droneGDLocationLng: " + droneGDLocationLng);
//                    Log.i("dongupdateDroneState", "3   onUpdate: droneGDLocationLat: " + sourceLatLng.latitude + " droneGDLocationLng: " + sourceLatLng.longitude);
                    //                    if (!isGetHome) { // 获取home点坐标
                    //                        homeLatitude = sourceLatLng.latitude;
                    //                        homeLongitude = sourceLatLng.longitude;
                    //                        Log.i("dongupdateDroneState", "4    onUpdate: droneGDLocationLat: " + sourceLatLng.latitude + " droneGDLocationLng: " + sourceLatLng.longitude);
                    //                        homeGDLatitude = desLatLng.latitude;
                    //                        homeGDLongitude = desLatLng.longitude;
                    //                        isGetHome = true;
                    //                    }
                    // droneLocationHeight = djiFlightControllerCurrentState.getAircraftLocation().getAltitude();

                    droneLocationLat = djiFlightControllerCurrentState.getAircraftLocation().getLatitude();
                    droneLocationLng = djiFlightControllerCurrentState.getAircraftLocation().getLongitude();
                    droneLocationHeight = djiFlightControllerCurrentState.getAircraftLocation().getAltitude();
                    Log.i("dongTimeline", "onUpdate: gimbalPitch : " + gimbalPitch + " droneYaw: " + yaw + " droneLocationLat: " + droneLocationLat + " droneLocationLng: " +
                            droneLocationLng + " droneLocationHeight: " + droneLocationHeight);
                    updateDroneLocation();

                    // TimeLine设置 显示无人机信息
                    runOnUiThread(new Runnable() {
                        @Override
                        public void run() {
                            curHeiText.setText(droneLocationHeight + "");
                            curYawText.setText(yaw + "");
                            curGPitchText.setText(gimbalPitch + "");
                        }
                    });

                }
            });
        }
    }

    /**
     * Update the drone location based on states from MCU.
     */
    private void updateDroneLocation() {
        Log.i("dongsoft", "run if: droneLocationLat: " + droneGDLocationLat + " droneLocationLng: " + droneGDLocationLng);
        LatLng pos = new LatLng(droneGDLocationLat, droneGDLocationLng);
        //Create MarkerOptions object
        final MarkerOptions markerOptions = new MarkerOptions();
        markerOptions.position(pos);

        markerOptions.icon(BitmapDescriptorFactory.fromBitmap(ImageUtils.rotate(getResources(), (float) yaw)));

//        markerOptions.icon(BitmapDescriptorFactory.fromResource(R.drawable.aircraft));

        runOnUiThread(new Runnable() {
            @Override
            public void run() {
                if (true) {

                    if (droneMarker != null) {
                        droneMarker.remove();
                    }
                    //                Log.i("dongsoft", "run: droneLocationLat: " + droneLocationLat + " droneLocationLng: " + droneLocationLng);
                    if (checkGpsCoordination(droneGDLocationLat, droneGDLocationLng)) {
                        Log.i("dongsoft", "run if: droneLocationLat: " + droneGDLocationLat + " droneLocationLng: " + droneGDLocationLng);
                        droneMarker = aMap.addMarker(markerOptions);
                    }
                }
            }
        });
    }

    private void cameraUpdate() {
        LatLng pos = new LatLng(droneGDLocationLat, droneGDLocationLng);
        float zoomlevel = (float) 18.0;
        CameraUpdate cu = CameraUpdateFactory.newLatLngZoom(pos, zoomlevel);
        aMap.moveCamera(cu);

        if (droneMarker != null) {
            droneMarker.remove();
        }
        MarkerOptions markerOptions = new MarkerOptions();
        markerOptions.position(pos);
        markerOptions.icon(BitmapDescriptorFactory.fromBitmap(ImageUtils.rotate(getResources(), (float) yaw)));
        droneMarker = aMap.addMarker(markerOptions);
    }

    public static boolean checkGpsCoordination(double latitude, double longitude) {
        return (latitude > -90 && latitude < 90 && longitude > -180 && longitude < 180) && (latitude != 0f && longitude != 0f);
    }

    @Override
    public boolean onTouch(View v, MotionEvent motionEvent) {
        switch (v.getId()) {
            case R.id.detect_button:
                if (motionEvent.getAction() == MotionEvent.ACTION_DOWN) {
                    detectIv.setImageResource(R.mipmap.detect_close_p);
                }
                if (motionEvent.getAction() == MotionEvent.ACTION_UP) {

                    // 如果没有开始识别，打开选择识别模式的对话框
                    if (!isDetection) {
                        final AlertDialog alertDialog = new AlertDialog.Builder(this).create();
                        alertDialog.show();

                        Window window = alertDialog.getWindow();
                        //实现提示窗体透明的代码
                        WindowManager.LayoutParams attributes = window.getAttributes();
                        attributes.alpha = 0.6f;
                        window.setAttributes(attributes);
                        window.setContentView(R.layout.detection_dialog);

                        Button btn_detection = window.findViewById(R.id.btn_detection);
                        Button btn_cancel = window.findViewById(R.id.btn_cancel);

                        TextView name_tv = window.findViewById(R.id.name_text);
                        TextView waring_tv = window.findViewById(R.id.warning_text);

                        name_tv.setText("识别停机坪");
                        waring_tv.setText("是否开始自动降落");

                        btn_detection.setOnClickListener(new View.OnClickListener() {
                            @Override
                            public void onClick(View view) {
                                //                                Toast.makeText(MainActivity.this, "确定", Toast.LENGTH_SHORT).show();

                                if (ultrasonicHeight > 7.7) {
                                    oneSpeedPara = 0.0009f;
                                }
                                if (ultrasonicHeight <= 7.7 && ultrasonicHeight > 5.2) {
                                    twoSpeedPara = 0.0008f;
                                }
                                if (ultrasonicHeight <= 5.2 && ultrasonicHeight > 3.0) {
                                    threeSpeedPara = 0.00041f;
                                }
                                if (ultrasonicHeight < 3.0) {
                                    fourSpeedPara = 0.00011f;
                                }

                                if (mFlightController != null) {
                                    mFlightController.setVirtualStickModeEnabled(true, new CommonCallbacks.CompletionCallback() {
                                        @Override
                                        public void onResult(DJIError djiError) {
                                            if (djiError != null) {
                                                setResultToToast(djiError.getDescription());
                                            } else {
                                                isSimulator = true;
                                                runOnUiThread(new Runnable() {
                                                    @Override
                                                    public void run() {
                                                        disableSimulatorIv.setImageResource(R.mipmap.aircraft_mode);
                                                        //                                                                    setResultToToast("虚拟摇杆开启");
                                                    }
                                                });
                                            }
                                        }
                                    });
                                }

                                if (!tagFlag) {
                                    Log.i("dongtagFlag", "onClick: tagRunnable");
                                    tagFlag = true;
                                    isDetection = true;
                                    tagThread = new Thread(tagRunnable);
                                    tagThread.start();
                                    detectIv.setImageResource(R.mipmap.detect_open);
                                    recognizerForAprilTags = new ApriltagsDetectAPIModel();

                                    detectIv.setVisibility(View.VISIBLE);
                                    mPushDrawerIb.setVisibility(View.VISIBLE);
                                    speedParaIv.setVisibility(View.VISIBLE);
                                    atsrell.setVisibility(View.VISIBLE);
                                    atinfoll.setVisibility(View.VISIBLE);
                                    speedparall.setVisibility(View.VISIBLE);
                                }
                                Log.i("dongtagThread", "if onClick: tagThread.isAlive(): " + tagThread.isAlive());
                                alertDialog.dismiss();
                            }
                        });

                        btn_cancel.setOnClickListener(new View.OnClickListener() {
                            @Override
                            public void onClick(View view) {
                                detectIv.setImageResource(R.mipmap.detect_close_u);

                                detectIv.setVisibility(View.GONE);
                                mPushDrawerIb.setVisibility(View.GONE);
                                speedParaIv.setVisibility(View.GONE);
                                atsrell.setVisibility(View.GONE);
                                atinfoll.setVisibility(View.GONE);
                                speedparall.setVisibility(View.GONE);

                                alertDialog.dismiss();
                            }
                        });
                    } else {
                        AlertDialog.Builder builder = new AlertDialog.Builder(this);
                        builder.setTitle("是否关闭识别功能？");
                        builder.setPositiveButton("确定", new DialogInterface.OnClickListener() {
                            @Override
                            public void onClick(DialogInterface dialog, int which) {
                                detectIv.setImageResource(R.mipmap.detect_close_u);
                                Log.i("dongtagThread", "else onClick: tagThread.isAlive(): " + tagThread.isAlive());
                                if (tagThread != null) { // && tagThread.isAlive()
                                    Log.i("dongtagThread", "onClick: tagThread.isAlive(): " + tagThread.isAlive());
                                    tagThread.interrupt();
                                    tagFlag = false;
                                    isDetection = false;
                                    recognizerForAprilTags.close();
                                }

                                if (mFlightController != null) {
                                    mFlightController.setVirtualStickModeEnabled(false, new CommonCallbacks.CompletionCallback() {
                                        @Override
                                        public void onResult(DJIError djiError) {
                                            if (djiError != null) {
                                                setResultToToast(djiError.getDescription());
                                            } else {
                                                isSimulator = false;
                                                if (tagFlag) {
                                                    tagFlag = false;
                                                    isDetection = false;
                                                    tagThread.interrupt();
                                                }

                                                runOnUiThread(new Runnable() {
                                                    @Override
                                                    public void run() {
                                                        setResultToToast("AlertDialog  关闭识别功能, 虚拟摇杆关闭");
                                                        disableSimulatorIv.setImageResource(R.mipmap.rc_mode);
                                                    }
                                                });
                                            }
                                        }
                                    });
                                }

                                dialog.dismiss();
                            }
                        });
                        builder.setNegativeButton("取消", new DialogInterface.OnClickListener() {
                            @Override
                            public void onClick(DialogInterface dialog, int which) {
                                dialog.dismiss();
                            }
                        });
                        builder.show();
                    }
                }
                break;
        }
        return true;
    }

    @Override
    public void onClick(View view) {
        switch (view.getId()) {

            case R.id.wp_init_button:
                initTimeline();
                wpStartIv.setVisibility(View.VISIBLE);
                break;
            case R.id.wp_start_button:
                startTimeline();
                wpStartIv.setVisibility(View.GONE);
                wpPauseIv.setVisibility(View.VISIBLE);
                wpStopIv.setVisibility(View.VISIBLE);

                break;

            case R.id.wp_pause_button:
                pauseTimeline();
                wpResumeIv.setVisibility(View.VISIBLE);
                wpPauseIv.setVisibility(View.GONE);
                break;

            case R.id.wp_resume_button:
                resumeTimeline();
                wpResumeIv.setVisibility(View.GONE);
                wpPauseIv.setVisibility(View.VISIBLE);
                break;

            case R.id.wp_stop_button:
                stopTimeline();
                wpStartIv.setVisibility(View.VISIBLE);
                wpPauseIv.setVisibility(View.GONE);
                wpResumeIv.setVisibility(View.GONE);
                wpStopIv.setVisibility(View.GONE);
                wpCleanIv.setVisibility(View.VISIBLE);
                break;

            case R.id.wp_clean_button:
                cleanTimelineDataAndLog();
                wpCleanIv.setVisibility(View.GONE);
                break;

            case R.id.check_wp_button:

                final AlertDialog alertDialog = new AlertDialog.Builder(this).create();
                alertDialog.show();

                Window window = alertDialog.getWindow();
                //实现提示窗体透明的代码
                WindowManager.LayoutParams attributes = window.getAttributes();
                attributes.alpha = 0.6f;
                window.setAttributes(attributes);
                window.setContentView(R.layout.detection_dialog);

                Button btn_detection = window.findViewById(R.id.btn_detection);
                Button btn_cancel = window.findViewById(R.id.btn_cancel);

                TextView name_tv = window.findViewById(R.id.name_text);
                TextView waring_tv = window.findViewById(R.id.warning_text);

                name_tv.setText("TimeLine设置");

                if (isAdd) {
                    waring_tv.setText("是否结束标定航点");
                } else {
                    waring_tv.setText("是否开始标定航点?" + "\n" + "注意！点击确认按钮后无人机起飞点与返航点将是当前位置。");
                }

                btn_detection.setOnClickListener(new View.OnClickListener() {
                    @Override
                    public void onClick(View view) {

                        if (isAdd == false) {

                            if (DemoApplication.getProductInstance() instanceof Aircraft) {
                                mFlightController.getHomeLocation(new CommonCallbacks.CompletionCallbackWith<LocationCoordinate2D>() {
                                    @Override
                                    public void onSuccess(LocationCoordinate2D locationCoordinate2D) {
                                        homeLatitude = locationCoordinate2D.getLatitude();
                                        homeLongitude = locationCoordinate2D.getLongitude();
                                        if (GeneralUtils.checkGpsCoordinate(homeLatitude, homeLongitude)) {
                                            //                                setTimelinePlanToText("home point latitude: " + homeLatitude + "\nhome point longitude: " + homeLongitude);
                                            Log.i("donghomepoint", "onSuccess: latitude: " + homeLatitude + " home point longitude: " + homeLongitude);
                                        } else {
                                            setResultToToast("Failed to get home coordinates: Invalid GPS coordinate");
                                        }
                                    }

                                    @Override
                                    public void onFailure(DJIError djiError) {
                                        setResultToToast("Failed to get home coordinates: " + djiError.getDescription());
                                    }
                                });
                            }

//                            tlSettingLl.setVisibility(View.VISIBLE);
                            curTlInfoLl.setVisibility(View.VISIBLE);

                            isAdd = true;
                            setResultToToast("添加航点");

                            timeLineActionList.clear();
                            heightList.clear();
                            hotpointList.clear();
                            radiusList.clear();
                            hotheightList.clear();
                            actionList.clear();
                            gimbalangleList.clear();
                            yawangleList.clear();

                            // （隐藏的第一个点） 起飞点 和 返航点
                            TimeLineAction timeLineAction = new TimeLineAction(new Waypoint(homeLatitude, homeLongitude, 50), false, 5, 50, 4, (int) gimbalPitch, (int) yaw);
                            timeLineActionList.add(timeLineAction);

                            heightList.add(50);
                            hotpointList.add(false);
                            radiusList.add(5);
                            hotheightList.add(50);
                            actionList.add(4);
                            gimbalangleList.add((int) gimbalPitch);
                            yawangleList.add((int) yaw);

                        } else {
                            isAdd = false;
                            timelinell.setVisibility(View.VISIBLE);
                            curTlInfoLl.setVisibility(View.INVISIBLE);
                            setResultToToast("不能添加航点");
                        }
                        cameraUpdate();
                        alertDialog.dismiss();
                    }
                });

                btn_cancel.setOnClickListener(new View.OnClickListener() {
                    @Override
                    public void onClick(View view) {
                        alertDialog.dismiss();
                    }
                });
                break;

            case R.id.tl_wp_btn:    // 取点
                markWaypoint(new LatLng(droneGDLocationLat, droneGDLocationLng));

                droneGDLocationLat_tl = droneGDLocationLat;
                droneGDLocationLng_tl = droneGDLocationLng;
                //                wpSettingBtn.setVisibility(View.VISIBLE);

                wpTlRl.setVisibility(View.VISIBLE);

                // 航点修改界面的参数
                height_tl = droneLocationHeight;
                pitch_tl = (int) gimbalPitch;
                yaw_tl = (int) yaw;

                heig_text.setText(height_tl + "");
                camera_text.setText(pitch_tl + "");
                flight_text.setText(yaw_tl + "");


//                settingHLl.setVisibility(View.VISIBLE);
//                settingAttLl.setVisibility(View.VISIBLE);

//                heiTextTv.setText(height_tl + "");
//                droneYawTv.setText(yaw_tl + "");
//                gimbalPitchTv.setText(pitch_tl + "");
                break;

            /*case R.id.wp_setting_btn:
                wpTlRl.setVisibility(View.VISIBLE);
                heig_text.setText(height_tl + "");
                camera_text.setText(pitch_tl + "");
                flight_text.setText(yaw_tl + "");

                break;*/

            case R.id.height_M:
                if (height_tl > 5) {
                    height_tl -= 1;
                } else {
                    runOnUiThread(new Runnable() {
                        @Override
                        public void run() {
                            setResultToToast("最低高度为5米");
                        }
                    });
                }

                heig_text.setText(height_tl + "");
                break;

            case R.id.height_P:
                height_tl += 1;
                heig_text.setText(height_tl + "");
                break;

            case R.id.hot_M:
                if (radius_tl > 5) {
                    radius_tl -= 1;
                } else {
                    runOnUiThread(new Runnable() {
                        @Override
                        public void run() {
                            setResultToToast("最小半径为5米");
                        }
                    });
                }
                radius_text.setText(radius_tl + "");
                break;

            case R.id.hot_P:
                radius_tl += 1;
                radius_text.setText(radius_tl + "");
                break;

            case R.id.camera_M:
                pitch_tl -= 1;
                camera_text.setText(pitch_tl + "");
                break;

            case R.id.camera_P:
                pitch_tl += 1;
                camera_text.setText(pitch_tl + "");
                break;

            case R.id.flight_M:
                yaw_tl -= 1;
                flight_text.setText(yaw_tl + "");
                break;

            case R.id.flight_P:
                yaw_tl += 1;
                flight_text.setText(yaw_tl + "");
                break;

            case R.id.btn_confirm:
                Log.i("dongcaacty", "onClick: camera_action_type_tl" + camera_action_type_tl);

                // 存值
                TimeLineAction timeLineAction = new TimeLineAction(new Waypoint(droneGDLocationLat_tl, droneGDLocationLng_tl, height_tl), hotset_tl, radius_tl, hot_height_tl, camera_action_type_tl, pitch_tl, yaw_tl);
                timeLineActionList.add(timeLineAction);

                timeLineActionList.set(markerNum, timeLineAction);

                heightList.set(markerNum, (int) height_tl);
                hotpointList.set(markerNum, hotset_tl);
                radiusList.set(markerNum, radius_tl);
                hotheightList.set(markerNum, hot_height_tl);
                actionList.set(markerNum, camera_action_type_tl);
                gimbalangleList.set(markerNum, pitch_tl);
                yawangleList.set(markerNum, yaw_tl);

                Log.i("dongmarkerNum", "onClick: markerNum: " + markerNum);

                // 初始化
                height_tl = 50;
                hotset_tl = false;
                radius_tl = 5;
                hot_height_tl = 50;
                camera_action_type_tl = 4;
                pitch_tl = 0;
                yaw_tl = 0;

//                hot_set.clearCheck();
                hot_set.check(R.id.no_hot);
                camera_type.check(R.id.no);
//                camera_type.clearCheck( );

                wpTlRl.setVisibility(View.GONE);
                break;

            case R.id.btn_cancel:

                // 初始化
                height_tl = 50;
                hotset_tl = false;
                radius_tl = 5;
                hot_height_tl = 50;
                camera_action_type_tl = 4;
                pitch_tl = 0;
                yaw_tl = 0;

                hot_set.check(R.id.no_hot);
                camera_type.check(R.id.no);

                wpTlRl.setVisibility(View.GONE);
                break;


            case R.id.tracking_drawer_control_ib:

                if (mPushInfoSd.isOpened()) {
                    Log.i("tracking_drawer", "isOpened: animateClose");
                    //                    setResultToToast("tracking_drawer, isOpened?: animateClose");
                    mPushInfoSd.animateClose();
                } else {
                    Log.i("tracking_drawer", "isClosed: animateOpen");
                    //                    setResultToToast("tracking_drawer, isClosed?: animateOpen");
                    mPushInfoSd.animateOpen();
                }
                break;

            case R.id.simulator_stop_iv:

                if (mFlightController != null) {
                    mFlightController.getVirtualStickModeEnabled(new CommonCallbacks.CompletionCallbackWith<Boolean>() {
                        @Override
                        public void onSuccess(Boolean aBoolean) {
                            Log.i("getVirtualStick", "onSuccess: " + aBoolean);
                            if (!aBoolean) {

                                mFlightController.setVirtualStickModeEnabled(true, new CommonCallbacks.CompletionCallback() {
                                    @Override
                                    public void onResult(DJIError djiError) {
                                        if (djiError != null) {
                                            setResultToToast(djiError.getDescription());
                                        } else {
                                            isSimulator = true;
                                            runOnUiThread(new Runnable() {
                                                @Override
                                                public void run() {
                                                    disableSimulatorIv.setImageResource(R.mipmap.aircraft_mode);
                                                    //                                                                    setResultToToast("虚拟摇杆开启");
                                                }
                                            });
                                        }
                                    }
                                });
                            } else {
                                mFlightController.setVirtualStickModeEnabled(false, new CommonCallbacks.CompletionCallback() {
                                    @Override
                                    public void onResult(DJIError djiError) {
                                        if (djiError != null) {
                                            setResultToToast(djiError.getDescription());
                                        } else {
                                            isSimulator = false;
                                            //                                        setResultToToast("虚拟摇杆关闭");
                                            runOnUiThread(new Runnable() {
                                                @Override
                                                public void run() {
                                                    setResultToToast("simulator_stop_iv, 虚拟摇杆关闭");
                                                    disableSimulatorIv.setImageResource(R.mipmap.rc_mode);
                                                }
                                            });
                                        }
                                    }
                                });
                            }
                        }

                        @Override
                        public void onFailure(DJIError djiError) {

                        }
                    });
                }

                break;
            case R.id.speedPara_iv:
                final AlertDialog alertDialog_speed = new AlertDialog.Builder(this).create();
                alertDialog_speed.show();

                Window window_speed = alertDialog_speed.getWindow();
                //实现提示窗体透明的代码
                WindowManager.LayoutParams attributes_speed = window_speed.getAttributes();
                attributes_speed.alpha = 0.6f;
                window_speed.setAttributes(attributes_speed);
                window_speed.setContentView(R.layout.speedpara_dialog);

                Button btn_confirm_speed = window_speed.findViewById(R.id.btn_confirm);
                Button btn_cancel_speed = window_speed.findViewById(R.id.btn_cancel);

                RadioGroup speedParaRg = window_speed.findViewById(R.id.speedPara_rg);

                speedParaRg.setOnCheckedChangeListener(new RadioGroup.OnCheckedChangeListener() {
                    @Override
                    public void onCheckedChanged(RadioGroup group, int checkedId) {
                        switch (group.getCheckedRadioButtonId()) {
                            case R.id.speedPara_3_rbtn:
                                oneSpeedPara = 0.0009f;
                                twoSpeedPara = 0.0008f;
                                threeSpeedPara = 0.00041f;
                                fourSpeedPara = 0.00011f;
                                runOnUiThread(new Runnable() {
                                    @Override
                                    public void run() {
                                        speedParaTv.setText(Float.toString(fourSpeedPara));
                                    }
                                });
                                //Toast.makeText(PrintSettingActivity.this, orientation.toString(), Toast.LENGTH_SHORT).show();
                                break;
                            case R.id.speedPara_4_rbtn:
                                oneSpeedPara = 0.0009f;
                                twoSpeedPara = 0.0008f;
                                threeSpeedPara = 0.00042f;
                                fourSpeedPara = 0.00012f;
                                runOnUiThread(new Runnable() {
                                    @Override
                                    public void run() {
                                        speedParaTv.setText(Float.toString(fourSpeedPara));
                                    }
                                });
                                //Toast.makeText(PrintSettingActivity.this, orientation.toString(), Toast.LENGTH_SHORT).show();
                                break;
                            case R.id.speedPara_5_rbtn:
                                oneSpeedPara = 0.0009f;
                                twoSpeedPara = 0.0008f;
                                threeSpeedPara = 0.00045f;
                                fourSpeedPara = 0.00013f;
                                runOnUiThread(new Runnable() {
                                    @Override
                                    public void run() {
                                        speedParaTv.setText(Float.toString(fourSpeedPara));
                                    }
                                });
                                //Toast.makeText(PrintSettingActivity.this, orientation.toString(), Toast.LENGTH_SHORT).show();
                                break;
                            default:
                                break;
                        }
                    }
                });

                btn_confirm_speed.setOnClickListener(new View.OnClickListener() {
                    @Override
                    public void onClick(View view) {
                        alertDialog_speed.dismiss();
                    }
                });

                btn_cancel_speed.setOnClickListener(new View.OnClickListener() {
                    @Override
                    public void onClick(View view) {
                        alertDialog_speed.dismiss();
                    }
                });
                break;
        }
    }

    @Override
    public void onCheckedChanged(RadioGroup group, int checkedId) {
        Log.i("dongcheckedid", "onCheckedChanged: " + group.getCheckedRadioButtonId());
        switch (checkedId) {
            case R.id.is_hot:
                Log.i("dongcheckedid", "onCheckedChanged: " + checkedId);
                hotset_tl = true;
                ishot_layout.setVisibility(View.VISIBLE);

                break;
            case R.id.no_hot:
                Log.i("dongcheckedid", "onCheckedChanged: " + checkedId);
                hotset_tl = false;
                ishot_layout.setVisibility(View.GONE);
                break;
            case R.id.photo:
                camera_action_type_tl = 1;
                Log.i("camera_action_type", "onCheckedChanged: " + camera_action_type_tl);
                break;
            case R.id.video_yes:
                camera_action_type_tl = 2;
                break;
            case R.id.video_no:
                camera_action_type_tl = 3;
                break;
            case R.id.no:
                camera_action_type_tl = 4;
                break;
        }

    }

    @Override
    protected void onResume() {
        super.onResume();

        //        double decimation = 4;
        //        double sigma = 0.0;
        //        int nthreads = 8;
        //        String tagFamily = "tag36h11";
        //        //        double tagsize = 0.005184;
        //        double tagsize = 0.04;
        //        double fx = 5423.612353784232;
        //        double fy = 5519.056520316598;
        //        double cx = 1315.1967405060602;
        //        double cy = 838.5860437396254;
        //        ApriltagR.apriltagInit(tagFamily, 2, decimation, sigma, nthreads, tagsize, fx, fy, cx, cy);

        initFlightController();
        initPreviewer();


        handlerThreadForAprilTags = new HandlerThread("AprilTags");
        handlerThreadForAprilTags.start();
        handlerForAprilTags = new Handler(handlerThreadForAprilTags.getLooper());

        handlerThreadForTarmac = new HandlerThread("tarmac");
        handlerThreadForTarmac.start();
        handlerForTarmac = new Handler(handlerThreadForTarmac.getLooper());
    }

    @Override
    protected void onPause() {

        super.onPause();
    }

    @Override
    protected void onDestroy() {
        if (mCodecManager != null) {
            mCodecManager.cleanSurface();
            mCodecManager.destroyCodec();
        }
        unregisterReceiver(mReceiver);
        tagFlag = false;
        if (tagThread != null) {
            if (tagThread.isAlive()) {
                tagThread.interrupt();
            }
        }
        isDetection = false;
        detectIv.setImageResource(R.mipmap.detect_close_u);
        // 关闭虚拟摇杆
        if (mFlightController != null) {
            mFlightController.setVirtualStickModeEnabled(false, new CommonCallbacks.CompletionCallback() {
                @Override
                public void onResult(DJIError djiError) {
                    if (djiError != null) {
                        setResultToToast(djiError.getDescription());
                    } else {
                        //                                                setResultToToast("虚拟摇杆关闭");
                    }
                }
            });
        }
        //        removeListener();
        super.onDestroy();
    }

    /**
     * 登录大疆账号
     */
    private void loginAccount() {
        UserAccountManager.getInstance().logIntoDJIUserAccount(this, new CommonCallbacks.CompletionCallbackWith<UserAccountState>() {
            @Override
            public void onSuccess(final UserAccountState userAccountState) {
                Toast.makeText(MainActivity.this, "Login Success", Toast.LENGTH_LONG);
            }

            @Override
            public void onFailure(DJIError error) {
                Toast.makeText(MainActivity.this, "Login Error:" + error.getDescription(), Toast.LENGTH_LONG);
            }
        });

    }

    private void setResultToToast(final String string) {
        MainActivity.this.runOnUiThread(new Runnable() {
            @Override
            public void run() {
                Toast.makeText(MainActivity.this, string, Toast.LENGTH_SHORT).show();
            }
        });
    }

    /**
     * 学习一下Handler和HandlerThread
     *
     * @param r
     */
    protected synchronized void runInBackgroundForAprilTags(final Runnable r) {
        if (handlerForAprilTags != null) {
            handlerForAprilTags.post(r);
        }
    }

    protected synchronized void runInBackgroundForTarmac(final Runnable r) {
        if (handlerForTarmac != null) {
            handlerForTarmac.post(r);
        }
    }

    /**
     * Push AprilTags Status to TextView
     *
     * @param string
     */
    private void setResultToText(final String string, int id) {
        //        if (mPushInfoTv == null) {
        //            setResultToToast("Push info tv has not be init...");
        //        }
        if (id == 10) {
            MainActivity.this.runOnUiThread(new Runnable() {
                @Override
                public void run() {
                    bPushInfoTv.setText(string);
                }
            });
        }
        if (id == 1) {
            MainActivity.this.runOnUiThread(new Runnable() {
                @Override
                public void run() {
                    ltPushInfoTv.setText(string);
                }
            });
        }
        if (id == 2) {
            MainActivity.this.runOnUiThread(new Runnable() {
                @Override
                public void run() {
                    rtPushInfoTv.setText(string);
                }
            });
        }
        if (id == 4) {
            MainActivity.this.runOnUiThread(new Runnable() {
                @Override
                public void run() {
                    lbPushInfoTv.setText(string);
                }
            });
        }
        if (id == 3) {
            MainActivity.this.runOnUiThread(new Runnable() {
                @Override
                public void run() {
                    rbPushInfoTv.setText(string);
                }
            });
        }
        if (id == 0) {
            MainActivity.this.runOnUiThread(new Runnable() {
                @Override
                public void run() {
                    cPushInfoTv.setText(string);
                }
            });
        }
    }

    /**
     * 显示在左下角的识别结果列表数据
     */
    private void resultData() {
        resultItemArrayList = new ArrayList<>();
        RecognitionResultItem recognitionResultItem1 = new RecognitionResultItem("等待识别...", "", "", "", "", "", "", "", "");
        resultItemArrayList.add(recognitionResultItem1);
        for (int i = 0; i < 4; i++) {
            RecognitionResultItem recognitionResultItem = new RecognitionResultItem("", "", "", "", "", "", "", "", "");
            resultItemArrayList.add(recognitionResultItem);
        }
    }

    private void initPreviewer() {

        BaseProduct product = DemoApplication.getProductInstance();
        Log.i("initPreviewer", "DemoApplication.getProductInstance()");

        if (product == null || !product.isConnected()) {

        } else {
            if (null != fpvWidget) {
                fpvWidget.setSurfaceTextureListener(this);
                Log.i("initPreviewer", "fpvWidget.setSurfaceTextureListener");
            }
        }
    }

    @Override
    public void onSurfaceTextureAvailable(SurfaceTexture surface, int width, int height) {
        Log.e(TAG, "onSurfaceTextureAvailable");
        //        Log.i("dongSTA", "onSurfaceTextureAvailable: width:" + width + " height:" + height);
        if (mCodecManager == null) {
            mCodecManager = new DJICodecManager(this, surface, width, height);
        }
        captureBitmap = Bitmap.createBitmap(width, height, Bitmap.Config.ARGB_8888);
    }

    @Override
    public void onSurfaceTextureSizeChanged(SurfaceTexture surface, int width, int height) {
        Log.e(TAG, "onSurfaceTextureSizeChanged");
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
        //        Log.i(TAG, "onSurfaceTextureUpdated: 11111111111111");
        synchronized (this) {
            //            fpvWidget.getBitmap(captureBitmap);
        }
    }

    /**
     * 坐标转换-转为大疆坐标系
     * GCJ02=>WGS84   火星坐标系=>地球坐标系（精确）
     *
     * @param gcjLat
     * @param gcjLon
     * @return
     */
    public static LatLng getDJILatLng(double gcjLat, double gcjLon) {
        double initDelta = 0.01;
        double threshold = 0.000000001;
        double dLat = initDelta, dLon = initDelta;
        double mLat = gcjLat - dLat, mLon = gcjLon - dLon;
        double pLat = gcjLat + dLat, pLon = gcjLon + dLon;
        double wgsLat, wgsLon, i = 0;
        while (true) {
            wgsLat = (mLat + pLat) / 2;
            wgsLon = (mLon + pLon) / 2;
            double[] tmp = wgs2GCJ(wgsLat, wgsLon);
            dLat = tmp[0] - gcjLat;
            dLon = tmp[1] - gcjLon;
            if ((Math.abs(dLat) < threshold) && (Math.abs(dLon) < threshold))
                break;

            if (dLat > 0)
                pLat = wgsLat;
            else
                mLat = wgsLat;
            if (dLon > 0)
                pLon = wgsLon;
            else
                mLon = wgsLon;

            if (++i > 10000)
                break;
        }
        return new LatLng(wgsLat, wgsLon);
    }

    /**
     * WGS84=》GCJ02   地球坐标系=>火星坐标系
     *
     * @param wgLat
     * @param wgLon
     * @return
     */
    private static double[] wgs2GCJ(double wgLat, double wgLon) {
        double[] latlon = new double[2];
        //        if (outOfChina(wgLat, wgLon)) {
        //            latlon[0] = wgLat;
        //            latlon[1] = wgLon;
        //            return latlon;
        //        }
        double[] deltaD = delta(wgLat, wgLon);
        latlon[0] = wgLat + deltaD[0];
        latlon[1] = wgLon + deltaD[1];
        return latlon;
    }

    /**
     * 转换函数
     *
     * @param wgLat
     * @param wgLon
     * @return
     */
    private static double[] delta(double wgLat, double wgLon) {
        double[] latlng = new double[2];
        double dLat = transformLat(wgLon - 105.0, wgLat - 35.0);
        double dLon = transformLon(wgLon - 105.0, wgLat - 35.0);
        double radLat = wgLat / 180.0 * PI;
        double magic = Math.sin(radLat);
        magic = 1 - OFFSET * magic * magic;
        double sqrtMagic = Math.sqrt(magic);
        dLat = (dLat * 180.0) / ((AXIS * (1 - OFFSET)) / (magic * sqrtMagic) * PI);
        dLon = (dLon * 180.0) / (AXIS / sqrtMagic * Math.cos(radLat) * PI);
        latlng[0] = dLat;
        latlng[1] = dLon;
        return latlng;
    }

    /**
     * 转换纬度
     *
     * @param x
     * @param y
     * @return
     */
    private static double transformLat(double x, double y) {
        double ret = -100.0 + 2.0 * x + 3.0 * y + 0.2 * y * y + 0.1 * x * y + 0.2 * Math.sqrt(Math.abs(x));
        ret += (20.0 * Math.sin(6.0 * x * PI) + 20.0 * Math.sin(2.0 * x * PI)) * 2.0 / 3.0;
        ret += (20.0 * Math.sin(y * PI) + 40.0 * Math.sin(y / 3.0 * PI)) * 2.0 / 3.0;
        ret += (160.0 * Math.sin(y / 12.0 * PI) + 320 * Math.sin(y * PI / 30.0)) * 2.0 / 3.0;
        return ret;
    }

    /**
     * 转换经度
     *
     * @param x
     * @param y
     * @return
     */
    private static double transformLon(double x, double y) {
        double ret = 300.0 + x + 2.0 * y + 0.1 * x * x + 0.1 * x * y + 0.1 * Math.sqrt(Math.abs(x));
        ret += (20.0 * Math.sin(6.0 * x * PI) + 20.0 * Math.sin(2.0 * x * PI)) * 2.0 / 3.0;
        ret += (20.0 * Math.sin(x * PI) + 40.0 * Math.sin(x / 3.0 * PI)) * 2.0 / 3.0;
        ret += (150.0 * Math.sin(x / 12.0 * PI) + 300.0 * Math.sin(x / 30.0 * PI)) * 2.0 / 3.0;
        return ret;
    }

    /****************************TimeLine***************************/
    private void startTimeline() {
        if (MissionControl.getInstance().scheduledCount() > 0) {
            MissionControl.getInstance().startTimeline();
        } else {
            setResultToToast("Init the timeline first by clicking the Init button");
        }
    }

    private void pauseTimeline() {
        MissionControl.getInstance().pauseTimeline();
    }

    private void resumeTimeline() {
        MissionControl.getInstance().resumeTimeline();
    }

    private void cleanTimelineDataAndLog() {
        if (missionControl.scheduledCount() > 0) {
            missionControl.unscheduleEverything();
            missionControl.removeAllListeners();
        }
    }

    private void stopTimeline() {
        MissionControl.getInstance().stopTimeline();
    }

    private void initTimeline() {
        if (!GeneralUtils.checkGpsCoordinate(homeLatitude, homeLongitude)) {
            setResultToToast("No home point!!!");
            return;
        }

        // hot_num记录第几个动作是环绕
        hot_num.clear();
        hot_num.add(0);
        for (int i = 0; i < hotpointList.size(); i++) {
            if (hotpointList.get(i)) {
                hot_num.add(i);
//                Log.i("AddTimelineActivity", "for initTestingWaypointMission: hot_num: " + i);
            }
        }
        hot_num.add(hotpointList.size());

        record_num.clear();
        record_num.add(0);
        for (int i = 0; i < actionList.size(); i++) {
            if (actionList.get(i) == 2) {
                record_num.add(i);
                //                Log.i("AddTimelineActivity", "for initTestingWaypointMission: hot_num: " + i);
            }
        }

        // 绘制无人机轨迹
        initTimeLine = true;

        // 把第一个waypoint的高度给起飞点
        heightList.set(0, heightList.get(1));

        waypointList.clear();
        // 航点飞行模式下的第一个点设定为起飞点经纬度，高度为从后台获取的第一个航点的高度
        waypointList.add(new Waypoint(homeLatitude, homeLongitude, waypointList_GD.get(0).altitude));

        for (int i = 0; i < waypointList_GD.size(); i++) { // 遍历所有航点
            // 先将航点的地图经纬度转换为真实经纬度（高度为相对高度，不必转换），再创建新的WayPoint航点对象
            GPSTransformUtil.Gps gcjToGps84 = gpsTransformUtil.gcj_To_Gps84(waypointList_GD.get(i).coordinate.getLatitude(), waypointList_GD.get(i).coordinate.getLongitude());
            Waypoint mWaypoint = new Waypoint(gcjToGps84.getWgLat(), gcjToGps84.getWgLon(), heightList.get(i + 1));
            Log.i("donghomepoint", "onSuccess: gcjToGps84.getWgLat(): " + gcjToGps84.getWgLat() + "  gcjToGps84.getWgLon(): " + gcjToGps84.getWgLon());
            waypointList.add(mWaypoint);

            points.add(new LatLng(waypointList_GD.get(i).coordinate.getLatitude(), waypointList_GD.get(i).coordinate.getLongitude()));
        }

        // 航点飞行模式下的最后一个点设定为起飞点经纬度，高度为从后台获取的第一个航点的高度
        waypointList.add(new Waypoint(homeLatitude, homeLongitude, 9.5f));

        Waypoint waypoint = new Waypoint(homeLatitude, homeLongitude, 9.5f);
        TimeLineAction timeLineAction = new TimeLineAction(waypoint, false, 5, 5, 4, (int) gimbalPitch, (int) yaw);
        timeLineActionList.add(timeLineAction);
        heightList.add((int) 9.4);
        hotpointList.add(false);
        radiusList.add(5);
        hotheightList.add(5);
        actionList.add(4);
        gimbalangleList.add((int) gimbalangleList.get(markerNum));
        yawangleList.add((int) yawangleList.get(markerNum));


        for (int i = 0; i < hotpointList.size(); i++) {
            Log.i("AddTimelineActivity", "forhotpointList initTimeline: " + hotpointList.get(i));
        }
        Log.i("AddTimelineActivity", "initTestingWaypointMission: waypointList.size: " + waypointList.size());
        Log.i("AddTimelineActivity", "initTestingWaypointMission: timeLineActionList.size: " + timeLineActionList.size() +
                " hotpointList.size: " + hotpointList.size() +
                " radiusList.size: " + radiusList.size() +
                " heightList.size: " + heightList.size() +
                " actionList.size: " + actionList.size() +
                " angleList.size: " + yawangleList.size());

//        points.add(point);
        polylineOptions.addAll(points);
        polyline = aMap.addPolyline(polylineOptions);
        polylineList.add(polyline);

        List<TimelineElement> elements = new ArrayList<>();

        missionControl = MissionControl.getInstance();
        final TimelineEvent preEvent = null;
        MissionControl.Listener listener = new MissionControl.Listener() {
            @Override
            public void onEvent(@Nullable TimelineElement element, TimelineEvent event, DJIError error) {
                updateTimelineStatus(element, event, error);
            }
        };

        elements.add(new TakeOffAction());

//        Attitude attitude = new Attitude(-45, Rotation.NO_ROTATION, Rotation.NO_ROTATION);
//        GimbalAttitudeAction gimbalAction = new GimbalAttitudeAction(attitude);
//        gimbalAction.setCompletionTime(2);
//        elements.add(gimbalAction);

        // 如果所有点都没有环绕飞行动作，则hot_num.size() = 2，共循环1次，即只有一次航点飞行，不进行环绕飞行的设置
        // 如果有n个点有环绕飞行动作，则hot_num.size() = n+2，共循环n+1次，即有n+1次航点飞行，有n次环绕飞行
        // 所以，整个飞行任务被分解为了2n+1段
        // k的数值代表第几次出现环绕飞行
        for (int k = 0; k < hot_num.size() - 1; k++) {

            TimelineElement waypointMission = TimelineMission.elementFromWaypointMission(initTestingWaypointMission(k));
            Log.i("AddTimelineActivity", "waypointMission:checkValidity: " + waypointMission.checkValidity());
            elements.add(waypointMission);
            addWaypointReachedTrigger(waypointMission);

            if (k < hot_num.size() - 2) {
                Log.i("AddTimelineActivity", "initTestinghotpointMission: hotpointMission ");
                Log.i("dongtimelinehot", "initTimeline: k:" + hot_num.get(k + 1));
                HotpointMission hotpointMission = new HotpointMission();
                hotpointMission.setHotpoint(new LocationCoordinate2D(waypointList.get(hot_num.get(k + 1)).coordinate.getLatitude(), waypointList.get(hot_num.get(k + 1)).coordinate.getLongitude()));
                hotpointMission.setAltitude(waypointList.get(hot_num.get(k + 1)).altitude);
                hotpointMission.setRadius(radiusList.get(hot_num.get(k + 1)));
                hotpointMission.setAngularVelocity(10);

                HotpointStartPoint startPoint = HotpointStartPoint.NORTH;
                hotpointMission.setStartPoint(startPoint);
                HotpointHeading heading = HotpointHeading.TOWARDS_HOT_POINT;
                hotpointMission.setHeading(heading);
                elements.add(new HotpointAction(hotpointMission, 360));
//                TimelineElement h = new HotpointAction(hotpointMission, 360);
//                Log.i("AddTimelineActivity", "hotpointMission:checkValidity: " + h.checkValidity());
            }
        }

//        attitude = new Attitude(-90, Rotation.NO_ROTATION, Rotation.NO_ROTATION);
//        gimbalAction = new GimbalAttitudeAction(attitude);
//        gimbalAction.setCompletionTime(2);
//        elements.add(gimbalAction);

        addAircraftLandedTrigger(missionControl);
        addBatteryPowerLevelTrigger(missionControl);

        if (missionControl.scheduledCount() > 0) {
            missionControl.unscheduleEverything();
            missionControl.removeAllListeners();
        }

        missionControl.scheduleElements(elements);
        missionControl.addListener(listener);
    }

    int finishedcount = 0;

    private void updateTimelineStatus(@Nullable TimelineElement element, TimelineEvent event, DJIError error) {

        if (element == preElement && event == preEvent && error == preError) {
            return;
        }

        if (element != null) {
            if (element instanceof TimelineMission) {
                setResultToToast(((TimelineMission) element).getMissionObject().getClass().getSimpleName() + " event is " + event.toString() + " " + (error == null ? "" : error.getDescription()));
                Log.i("dongpreEventSTOPPED", "element != null if " + ((TimelineMission) element).getMissionObject().getClass().getSimpleName() + "event is " + event.toString() + " " + (error == null ? "" : "Failed:" + error.getDescription()));
            } else {
                setResultToToast(element.getClass().getSimpleName() + " event is " + event.toString() + " " + (error == null ? "" : error.getDescription()));
                Log.i("dongpreEventSTOPPED", "element != null else " + element.getClass().getSimpleName() + "event is " + event.toString() + " " + (error == null ? "" : "Failed:" + error.getDescription()));
            }
        } else {
            Log.i("dongpreEventSTOPPED", "element == null else " + "Timeline Event is " + event.toString() + " " + (error == null ? "" : "Failed:" + error.getDescription()));
            setResultToToast("Timeline Event is " + event.toString() + " " + (error == null ? "" : "Failed:" + error.getDescription()));
        }

        preEvent = event;
        preElement = element;
        preError = error;


        if (preEvent == TimelineEvent.STOPPED) {
            Log.i("dongpreEventSTOPPED", "if updateTimelineStatus: preEvent == " + TimelineEvent.STOPPED);
            setResultToToast("updateTimelineStatus: preEvent == " + TimelineEvent.STOPPED);
        } else {
            Log.i("dongpreEventSTOPPED", " else updateTimelineStatus: preEvent == " + preEvent);
            if (preEvent == TimelineEvent.FINISHED) {
                finishedcount++;
            } else {
                finishedcount = 0;
            }
        }

        // 停止成功 执行降落
        if (finishedcount == 2) {

            if (mFlightController != null) {
                mFlightController.setVirtualStickModeEnabled(true, new CommonCallbacks.CompletionCallback() {
                    @Override
                    public void onResult(DJIError djiError) {
                        if (djiError != null) {
                            setResultToToast(djiError.getDescription());
                        } else {
                            isSimulator = true;
                            runOnUiThread(new Runnable() {
                                @Override
                                public void run() {
                                    disableSimulatorIv.setImageResource(R.mipmap.aircraft_mode);
                                    //                                                                    setResultToToast("虚拟摇杆开启");
                                }
                            });
                        }
                    }
                });
            }

            if (!tagFlag) {
                Log.i("dongtagFlag", "onClick: tagRunnable");
                tagFlag = true;
                isDetection = true;
                tagThread = new Thread(tagRunnable);
                tagThread.start();
                detectIv.setImageResource(R.mipmap.detect_open);
                recognizerForAprilTags = new ApriltagsDetectAPIModel();

                detectIv.setVisibility(View.VISIBLE);
                mPushDrawerIb.setVisibility(View.VISIBLE);
                speedParaIv.setVisibility(View.VISIBLE);
                atsrell.setVisibility(View.VISIBLE);
                atinfoll.setVisibility(View.VISIBLE);
                speedparall.setVisibility(View.VISIBLE);

                timelinell.setVisibility(View.INVISIBLE);
            }

        }
    }

    private WaypointMission initTestingWaypointMission(int k) {
        if (!GeneralUtils.checkGpsCoordinate(homeLatitude, homeLongitude)) {
            setResultToToast("未设定起飞点!");
            return null;
        }

        WaypointMission.Builder waypointMissionBuilder = new WaypointMission.Builder().autoFlightSpeed(mSpeed)
                .maxFlightSpeed(10f)
                .setExitMissionOnRCSignalLostEnabled(false) // 当飞行器和遥控器断连时，是否停止执行任务
                .finishedAction(WaypointMissionFinishedAction.NO_ACTION)
                .flightPathMode(WaypointMissionFlightPathMode.NORMAL)
                .gotoFirstWaypointMode(WaypointMissionGotoWaypointMode.SAFELY)
                .headingMode(WaypointMissionHeadingMode.AUTO)
                .repeatTimes(1);
        List<Waypoint> waypoints = new LinkedList<>();
        // hot_num.get(0) = 0，故循环变量p从0开始，hot_num.get(1)为下一次环绕飞行在动作列表中的位置
        // 故p以下一次环绕飞行动作前的动作个数为结束
        for (int p = hot_num.get(k); p <= hot_num.get(k + 1); p++) {
            // 先将第一个点（即起飞点的位置、第一点的高度）设定为航点飞行第一点
            Log.i("AddTimelineActivity", " p: " + p);
            Waypoint waypoint = new Waypoint(waypointList.get(p).coordinate.getLatitude(), waypointList.get(p).coordinate.getLongitude(), waypointList.get(p).altitude);
            if (p != hot_num.get(k)) { // 如果符合条件，说明进入了第二次循环，于是开始从第一个点开始设置动作
                TimeLineAction timeLineAction = timeLineActionList.get(p);
                int camera_action_type = timeLineAction.getAction();
                int gimbalangle = timeLineAction.getGimbalangle();
                int yawangle = timeLineAction.getYawangle();

                waypoint.addAction(new WaypointAction(WaypointActionType.GIMBAL_PITCH, gimbalangle));
                waypoint.addAction(new WaypointAction(WaypointActionType.ROTATE_AIRCRAFT, yawangle));
                Log.i("AddTimelineActivity", "initTestingWaypointMission: p: " + p + " camera_action_type: " + camera_action_type);
                switch (camera_action_type) { // 只有一个参数
                    case 1: // 拍摄一张照片
                        if (isRecordTimeLine) {
                            waypoint.addAction(new WaypointAction(WaypointActionType.STOP_RECORD, 3));
                        }
                        waypoint.addAction(new WaypointAction(WaypointActionType.START_TAKE_PHOTO, 3));
                        Log.i("AddTimelineActivity", "initTestingWaypointMission: START_TAKE_PHOTO");
                        if (isRecordTimeLine){
                            waypoint.addAction(new WaypointAction(WaypointActionType.START_RECORD, 3));
                        }
                        break;
                    case 2: // 开始录像
                        if (isRecordTimeLine) {
                            waypoint.addAction(new WaypointAction(WaypointActionType.STOP_RECORD, 3));
                        }
                        waypoint.addAction(new WaypointAction(WaypointActionType.START_RECORD, 3));
                        Log.i("AddTimelineActivity", "initTestingWaypointMission: START_RECORD");
                        isRecordTimeLine = true;
                        break;
                    case 3: // 结束录像
                        waypoint.addAction(new WaypointAction(WaypointActionType.STOP_RECORD, 3));
                        Log.i("AddTimelineActivity", "initTestingWaypointMission: STOP_RECORD");
                        isRecordTimeLine = false;
                        break;
                    case 4:
                        // 无动作
                        Log.i("AddTimelineActivity", "initTestingWaypointMission: 无动作");
                        break;
                }
            }
            waypoints.add(waypoint);
        }
        Log.i("AddTimelineActivity", "initTestingWaypointMission: waypoints.size: " + waypoints.size());
        waypointMissionBuilder.waypointList(waypoints).waypointCount(waypoints.size());
        return waypointMissionBuilder.build();
    }

    /**
     * Demo on WaypointReachedTrigger.  Once the expected waypoint is reached in the waypoint mission execution process,
     * this trigger's action will be called. If user has some special things to do for this waypoint, the code can be put
     * in this trigger action method.
     *
     * @param triggerTarget
     */
    private void addWaypointReachedTrigger(Triggerable triggerTarget) {
        int value = 1;
        WaypointReachedTrigger trigger = new WaypointReachedTrigger();
        trigger.setWaypointIndex(value);
        Log.i("dongWT", "addWaypointReachedTrigger: value: " + value);

//        trigger.setAction(new HotpointAction());
        addTrigger(trigger, triggerTarget, " at index " + value);
    }

    private void addTrigger(Trigger trigger, Triggerable triggerTarget, String additionalComment) {

        if (triggerTarget != null) {

            initTrigger(trigger);
            List<Trigger> triggers = triggerTarget.getTriggers();
            if (triggers == null) {
                triggers = new ArrayList<>();
            }

            triggers.add(trigger);
            triggerTarget.setTriggers(triggers);

            Log.i("dongWT", "addTrigger onCall: " + triggerTarget.getClass().getSimpleName() + " Trigger " + triggerTarget.getTriggers().size() + ") " + trigger.getClass().getSimpleName() + additionalComment);
            setResultToToast(triggerTarget.getClass().getSimpleName() + " Trigger " + triggerTarget.getTriggers().size() + ") " + trigger.getClass().getSimpleName() + additionalComment);
        }
    }

    private void initTrigger(final Trigger trigger) {
        trigger.addListener(triggerListener);
        trigger.setAction(new Trigger.Action() {
            @Override
            public void onCall() {
//                setResultToToast("Trigger " + trigger.getClass().getSimpleName() + " Action method onCall() is invoked");
                Log.i("dongWT", "initTrigger onCall: " + "Trigger " + trigger.getClass().getSimpleName() + " Action method onCall() is invoked");
            }
        });
    }

    private Trigger.Listener triggerListener = new Trigger.Listener() {
        @Override
        public void onEvent(Trigger trigger, TriggerEvent event, @Nullable DJIError error) {
            Log.i("dongWT", "triggerListener onCall: " + "Trigger " + trigger.getClass().getSimpleName() + " event is " + event.name() + (error == null ? " " : error.getDescription()));
//            setResultToToast("Trigger " + trigger.getClass().getSimpleName() + " event is " + event.name() + (error == null ? " " : error.getDescription()));
        }
    };

    /**
     * Demo on AircraftLandedTrigger. Once the aircraft is landed, this trigger action will be called if the timeline is
     * not finished yet.
     *
     * @param triggerTarget
     */
    private void addAircraftLandedTrigger(Triggerable triggerTarget) {
        AircraftLandedTrigger trigger = new AircraftLandedTrigger();
        addTrigger(trigger, triggerTarget, "");
    }

    /**
     * Demo on BatteryPowerLevelTrigger.  Once the batter remaining power is equal or less than the value,
     * the trigger's action will be called.
     *
     * @param triggerTarget which can be any action object or timeline object.
     */
    private void addBatteryPowerLevelTrigger(Triggerable triggerTarget) {
        float value = 20f;
        BatteryPowerLevelTrigger trigger = new BatteryPowerLevelTrigger();
        trigger.setPowerPercentageTriggerValue(value);
        addTrigger(trigger, triggerTarget, " at level " + value);
    }
}
